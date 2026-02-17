// PKI Interaction - Authorization ticket response.
// This code is used to decode the authorization ticket response, extract the certificate data and verify the signature.
// Created by Alessandro Giaccaglini - alessandro.giaccaglini@gmail.com

#include "Encoding.hpp"
#include "Setter.hpp"
#include "BitString.hpp"
#include "Seq.hpp"
#include "SequenceOf.hpp"
#include "SetOf.hpp"
#include "Utils.hpp"
#include "View.hpp"
#include "Getter.hpp"
#include <chrono>
#include <openssl/sha.h>
#include <openssl/pem.h>
#include <vector>
#include <openssl/obj_mac.h>
#include <openssl/err.h>
#include <openssl/bn.h>
#include <iostream>
#include <string>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <openssl/ec.h>
#include <openssl/evp.h>
#include <openssl/rand.h>
#include <openssl/hmac.h>
#include <openssl/kdf.h>
#include <ATResponse.h>
#include <INIReader.h>
#include <fstream>

extern "C"
{
#include "Ieee1609Dot2Data.h"
#include "InnerAtResponse.h"
#include "EtsiTs102941MessagesItss_EtsiTs102941Data.h"
}

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

#define AES_KEY_LENGTH 16
#define NONCE_LENGTH 12
#define AES_CCM_TAG_LENGTH 16
#define HMAC_TAG_LENGTH 32

ATResponse::~ATResponse ()
{

}

ATResponse::ATResponse ()
{
    dataResponse = NULL;
    length = 0;

    #if defined(__GNUC__) || defined(__clang__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-variable"
    #endif

    bool ephemeral = false;
    EC_KEY *m_ecKey = nullptr;
    EC_KEY *m_EPHecKey = nullptr;

    #if defined(__GNUC__) || defined(__clang__)
    #pragma GCC diagnostic pop
    #endif
}


std::string ATResponse::retrieveStringFromFile(const std::string& fileName) {
    std::ifstream fileIn(fileName, std::ios::binary);  // Open the file in binary mode
    std::string key;
    if (fileIn.is_open()) {
        size_t length;
        fileIn.read(reinterpret_cast<char*>(&length), sizeof(length));  // Read the length of the string
        key.resize(length);  // Resize the string
        fileIn.read(&key[0], length);  // Read the string into the buffer
        fileIn.close();
        std::cout << "Pre Shared Key retrieved: " << key << std::endl;
    } else {
        std::cerr << "Error opening file for reading." << std::endl;
    }
    return key;
}

void ATResponse::saveStringToFile(const std::string& key, const std::string& fileName) {
    std::ofstream fileOut(fileName, std::ios::binary);  // Open the file in binary mode
    if (fileOut.is_open()) {
        size_t length = key.size();
        fileOut.write(reinterpret_cast<const char*>(&length), sizeof(length));  // Write the length of the string
        fileOut.write(key.c_str(), length);  // Write the string
        fileOut.close();
        std::cout << "Pre Shared Key saved to binary file." << std::endl;
    } else {
        std::cerr << "Error opening file for writing." << std::endl;
    }
}

// Function to handle errors
void ATResponse::handleErrors()
{
  ERR_print_errors_fp(stderr);
  abort();
}

// Function to convert a hexadecimal string to a byte array
std::vector<unsigned char> ATResponse::hexStringToBytes(const std::string &hex)
{
  std::vector<unsigned char> bytes;
  for (unsigned int i = 0; i < hex.length(); i += 2)
  {
    std::string byteString = hex.substr(i, 2);
    unsigned char byte = static_cast<unsigned char>(strtol(byteString.c_str(), nullptr, 16));
    bytes.push_back(byte);
  }
  return bytes;
}

// Function to print the OpenSSL error
void ATResponse::print_openssl_error()
{
  char buffer[120];
  unsigned long error = ERR_get_error();
  ERR_error_string_n(error, buffer, sizeof(buffer));
  std::cerr << buffer << std::endl;
}


// Function to convert a byte array to a hexadecimal string
std::string ATResponse::to_hex_string(const unsigned char *data, size_t length)
{
  std::ostringstream oss;
  for (size_t i = 0; i < length; ++i)
  {
    oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(data[i]);
  }
  return oss.str();
}

// Function to convert a byte array to a hexadecimal string
std::string ATResponse::to_hex_string(const std::vector<unsigned char> &data)
{
  std::ostringstream oss;
  for (unsigned char byte : data)
  {
    oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
  }
  return oss.str();
}


// Function to read the content of a file
int ATResponse::readFileContent(const char *filename, unsigned char **dataResponse, uint32_t *length) {
    // Apertura del file in modalità lettura binaria
    FILE *file = fopen(filename, "rb");
    if (file == NULL) {
        perror("Errore nell'apertura del file");
        return 1;
    }

    // Determina la dimensione del file
    fseek(file, 0, SEEK_END);
    *length = ftell(file);
    rewind(file); // Riporta il puntatore all'inizio del file

    // Allocazione della memoria per il payload
    *dataResponse = (unsigned char *)malloc(*length);
    if (*dataResponse == NULL) {
        perror("Errore di allocazione memoria");
        fclose(file);
        return 1;
    }

    // Lettura del contenuto del file nel buffer
    size_t bytesRead = fread(*dataResponse, 1, *length, file);
    if (bytesRead != *length) {
        perror("Errore nella lettura del file");
        free(*dataResponse);
        fclose(file);
        return 1;
    }

    // Chiusura del file
    fclose(file);
    return 0; // Successo
}

// Function to load a compressed public key
EC_KEY* ATResponse::loadCompressedPublicKey(const std::string &compressedKey, int compression)
{
  // Crea l'oggetto EC_KEY per P-256
  EC_KEY *ec_key = EC_KEY_new_by_curve_name(NID_X9_62_prime256v1);
  if (!ec_key)
  {
    std::cerr << "Errore nella creazione di EC_KEY" << std::endl;
    return nullptr;
  }

  const EC_GROUP *group = EC_KEY_get0_group(ec_key);
  if (!group)
  {
    std::cerr << "Errore nel recupero del gruppo dalla curva" << std::endl;
    EC_KEY_free(ec_key);
    return nullptr;
  }

  std::vector<unsigned char> pk_data;
  // Aggiunge il prefisso per `compressed_y_0` o `compressed_y_1`
  if (compression == 2)
  {
    pk_data = {0x02}; // Prefisso per y pari
  }
  else if (compression == 3)
  {
    pk_data = {0x03}; // Prefisso per y dispari
  }

  // Aggiunge il contenuto di `compressedKey` dopo il prefisso
  pk_data.insert(pk_data.end(), compressedKey.begin(), compressedKey.end());

  // Verifica della lunghezza di `pk_data` ora dovrebbe essere 33 byte (1 byte di prefisso + 32 byte della chiave)
  if (pk_data.size() != 33)
  {
    std::cerr << "La chiave compressa con prefisso non ha la lunghezza corretta (33 byte)." << std::endl;
    return nullptr;
  }

  EC_POINT *point = EC_POINT_new(group);
  if (!point)
  {
    std::cerr << "Errore nella creazione di EC_POINT" << std::endl;
    EC_KEY_free(ec_key);
    return nullptr;
  }

  // Converte la chiave pubblica compressa
  if (!EC_POINT_oct2point(group, point, pk_data.data(), pk_data.size(), nullptr))
  {
    std::cerr << "Errore nella conversione della chiave pubblica compressa in EC_POINT" << std::endl;
    ERR_print_errors_fp(stderr); // Stampa l'errore dettagliato di OpenSSL
    EC_KEY_free(ec_key);
    EC_POINT_free(point);
    return nullptr;
  }

  if (!EC_KEY_set_public_key(ec_key, point))
  {
    std::cerr << "Errore nell'impostare la chiave pubblica su EC_KEY" << std::endl;
    EC_KEY_free(ec_key);
    EC_POINT_free(point);
    return nullptr;
  }


  EC_POINT_free(point);
  return ec_key;
}

// Function to compute the SHA-256 hash of a byte array
void ATResponse::computeSHA256(const std::vector<unsigned char> &data,
                   unsigned char hash[SHA256_DIGEST_LENGTH])
{
  SHA256_CTX sha256;
  SHA256_Init(&sha256);
  SHA256_Update(&sha256, data.data(), data.size());
  SHA256_Final(hash, &sha256);
}

// Function to concatenate two byte arrays
std::vector<unsigned char> ATResponse::concatenateHashes(const unsigned char hash1[SHA256_DIGEST_LENGTH], const unsigned char hash2[SHA256_DIGEST_LENGTH]) {
    std::vector<unsigned char> concatenatedHashes;
    concatenatedHashes.insert(concatenatedHashes.end(), hash1, hash1 + SHA256_DIGEST_LENGTH);
    concatenatedHashes.insert(concatenatedHashes.end(), hash2, hash2 + SHA256_DIGEST_LENGTH);
    return concatenatedHashes;
}

// Function to decrypt a message using AES-CCM
void ATResponse::decryptMessage(
    const std::vector<unsigned char> &encryptedMessage,
    const std::vector<unsigned char> &nonce,
    const unsigned char *presharedKey,
    std::vector<unsigned char> &decryptedMessage,
    const std::vector<unsigned char> &aesCcmTag)
{
    unsigned char aesKey[AES_KEY_LENGTH];
    std::memcpy(aesKey, presharedKey, AES_KEY_LENGTH);

    EVP_CIPHER_CTX *ctx = EVP_CIPHER_CTX_new();
    if (!ctx) handleErrors();


    if (EVP_DecryptInit_ex(ctx, EVP_aes_128_ccm(), nullptr, nullptr, nullptr) != 1) handleErrors();
    if (EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_CCM_SET_IVLEN, NONCE_LENGTH, nullptr) != 1) handleErrors();
    if (EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_CCM_SET_TAG, AES_CCM_TAG_LENGTH, (void*)aesCcmTag.data()) != 1) handleErrors();

    if (EVP_DecryptInit_ex(ctx, nullptr, nullptr, aesKey, nonce.data()) != 1) handleErrors();

    decryptedMessage.resize(encryptedMessage.size());
    int len;
    if (EVP_DecryptUpdate(ctx, decryptedMessage.data(), &len, encryptedMessage.data(), encryptedMessage.size()) != 1) {

        handleErrors();
    }

    int decryptedLen = len;
    if (EVP_DecryptFinal_ex(ctx, decryptedMessage.data() + len, &len) != 1) {
        handleErrors();
    }
    decryptedLen += len;
    decryptedMessage.resize(decryptedLen);

    EVP_CIPHER_CTX_free(ctx);
}

// Function to decrypt a message using AES-CCM
std::string ATResponse::doDecryption(std::string ciphertextWithTag_hex, std::string nonce_hex)
{
  
  std::vector<unsigned char> ciphertextWithTag(ciphertextWithTag_hex.begin(), ciphertextWithTag_hex.end());

  std::vector<unsigned char> ciphertext(ciphertextWithTag.begin(), ciphertextWithTag.end() - 16);
  std::vector<unsigned char> aesCcmTag(ciphertextWithTag.end() - 16, ciphertextWithTag.end());
  std::vector<unsigned char> nonce(nonce_hex.begin(), nonce_hex.end());
  if( m_aesKey == "")
  {
      m_aesKey = retrieveStringFromFile("pskAT.bin");
    //m_aesKey = "513d70d3a95d116d110317532b611ee5"; // TODO understand how put m_aesKey here, when you close the program, the value is lost
  }
  std::vector<unsigned char> psk = hexStringToBytes(m_aesKey);

  std::vector<unsigned char> decryptedMessage;

  decryptMessage(ciphertext, nonce, psk.data(), decryptedMessage, aesCcmTag);

  std::string decryptedPlaintext(decryptedMessage.begin(), decryptedMessage.end());


  return decryptedPlaintext;
}

// Function to load an EC_KEY object from a private key file
EC_KEY* ATResponse::loadECKeyFromFile(const std::string &private_key_file, const std::string &public_key_file)
{
  // Load the private key
  FILE *priv_file = fopen(private_key_file.c_str(), "r");
  if (!priv_file)
  {
    std::cerr << "Error opening file to load private key" << std::endl;
    return nullptr;
  }

  EC_KEY *ec_key = PEM_read_ECPrivateKey(priv_file, nullptr, nullptr, nullptr);
  if (!ec_key)
  {
    std::cerr << "Error reading private key from file" << std::endl;
    ERR_print_errors_fp(stderr);
    fclose(priv_file);
    return nullptr;
  }
  fclose(priv_file);

  // Load the public key
  FILE *pub_file = fopen(public_key_file.c_str(), "r");
  if (!pub_file)
  {
    std::cerr << "Error opening file to load public key" << std::endl;
    EC_KEY_free(ec_key);
    return nullptr;
  }

  EC_KEY *pub_key = PEM_read_EC_PUBKEY(pub_file, nullptr, nullptr, nullptr);
  if (!pub_key)
  {
    std::cerr << "Error reading public key from file" << std::endl;
    print_openssl_error();
    fclose(pub_file);
    EC_KEY_free(ec_key);
    return nullptr;
  }
  fclose(pub_file);

  // Extract the public key point and set it to the private key EC_KEY
  const EC_POINT *pub_key_point = EC_KEY_get0_public_key(pub_key);
  if (!pub_key_point)
  {
    std::cerr << "Error getting public key point" << std::endl;
    print_openssl_error();
    EC_KEY_free(ec_key);
    EC_KEY_free(pub_key);
    return nullptr;
  }

  if (EC_KEY_set_public_key(ec_key, pub_key_point) != 1)
  {
    std::cerr << "Error setting public key to the private key object" << std::endl;
    print_openssl_error();
    EC_KEY_free(ec_key);
    EC_KEY_free(pub_key);
    return nullptr;
  }

  EC_KEY_free(pub_key);
  return ec_key;
}


void ATResponse::readIniFile() {
    INIReader reader("./PKI_info.ini");



    if (reader.ParseError() < 0) {
        std::cout << "[ERR] Can't load 'PKI_info.ini'\n";
    }

    aaCert1 = reader.Get("ATinfo", "AAcert1", "");
    aaCert2 = reader.Get("ATinfo", "AAcert2", "");
    aaCert3 = reader.Get("ATinfo", "AAcert3", "");
    aaCert4 = reader.Get("ATinfo", "AAcert4", "");

    // Load RFC5480 keys (from EC section) so AT path can reuse the same ITS keys
    m_public_key_rfc   = reader.Get("ECinfo", "public_key_rfc",  "");
    m_private_key_rfc1 = reader.Get("ECinfo", "private_key_rfc1", "");
    m_private_key_rfc2 = reader.Get("ECinfo", "private_key_rfc2", "");

    //std::cout << "Config loaded from 'PKI_info.ini':  AAcertificate="
    //          << reader.Get("ATinfo", "AAcertificate", "UNKNOWN") <<  std::endl;

}


// Function to load an EC_KEY object from a RFC-5480 formatted private key
EC_KEY* ATResponse::loadECKeyFromRFC5480(const std::string &private_key_rfc, const std::string &public_key_rfc)
{
  EC_KEY *ec_key = nullptr;

  // Step 1: Convert private key hex string to binary DER format
  std::vector<unsigned char> derSK = hexStringToBytes(private_key_rfc);
  const unsigned char *priv_p = derSK.data();

  // Step 2: Parse private key DER (PKCS#8) to create EC_KEY
  EVP_PKEY *pkey = d2i_PrivateKey(EVP_PKEY_EC, nullptr, &priv_p, derSK.size());
  if (!pkey)
  {
    std::cerr << "Error loading private key from PKCS#8 format" << std::endl;
    ERR_print_errors_fp(stderr);
    return nullptr;
  }
  ec_key = EVP_PKEY_get1_EC_KEY(pkey);
  EVP_PKEY_free(pkey);

  if (!ec_key)
  {
    std::cerr << "Error converting EVP_PKEY to EC_KEY" << std::endl;
    return nullptr;
  }

  // Step 3: Convert public key hex string to binary DER format
  std::vector<unsigned char> derPK = hexStringToBytes(public_key_rfc);
  const unsigned char *pub_p = derPK.data();

  // Step 4: Parse public key DER and assign it to EC_KEY
  EC_KEY *pub_key = d2i_EC_PUBKEY(nullptr, &pub_p, derPK.size());
  if (!pub_key)
  {
    std::cerr << "Error loading public key from RFC-5480 format" << std::endl;
    ERR_print_errors_fp(stderr);
    EC_KEY_free(ec_key);
    return nullptr;
  }

  // Step 5: Extract the public key point and set it to the private key EC_KEY
  const EC_POINT *pub_key_point = EC_KEY_get0_public_key(pub_key);
  if (!pub_key_point)
  {
    std::cerr << "Error getting public key point" << std::endl;
    ERR_print_errors_fp(stderr);
    EC_KEY_free(ec_key);
    EC_KEY_free(pub_key);
    return nullptr;
  }

  if (EC_KEY_set_public_key(ec_key, pub_key_point) != 1)
  {
    std::cerr << "Error setting public key to the private key object" << std::endl;
    ERR_print_errors_fp(stderr);
    EC_KEY_free(ec_key);
    EC_KEY_free(pub_key);
    return nullptr;
  }

  // Clean up and return
  EC_KEY_free(pub_key);
  return ec_key;
}

// Function to recover the EC key pair from the private key file
ATResponse::GNpublicKey ATResponse::recoverECKeyPair(bool ephemeral)
{
  std::string private_key_file = "";
  std::string public_key_file = "";
  EC_KEY *ec_key = nullptr;
  if (ephemeral)
  {
    private_key_file = "./pkiReqRes/ephSKEY.pem";
    public_key_file = "./pkiReqRes/ephPKEY.pem";
    ec_key = loadECKeyFromFile(private_key_file, public_key_file);
    if (!ec_key)
    {
      return {};
    }
  }
  else
  {
    // Load RFC keys from INI
    if (m_public_key_rfc.empty() && m_private_key_rfc1.empty() && m_private_key_rfc2.empty()) {
      readIniFile();
    }
    std::string private_key_rfc = m_private_key_rfc1 + m_private_key_rfc2;
    std::string public_key_rfc = m_public_key_rfc;
    if (private_key_rfc.empty() || public_key_rfc.empty()) {
      std::cerr << "[ERR] Missing RFC keys in PKI_info.ini (ECinfo.public_key_rfc / private_key_rfc1+2)" << std::endl;
      return {};
    }
    ec_key = loadECKeyFromRFC5480(private_key_rfc, public_key_rfc);
    if (!ec_key)
    {
      return {};
    }
  }

  if (!ephemeral)
    m_ecKey = EC_KEY_dup(ec_key);
  else
    m_EPHecKey = EC_KEY_dup(ec_key);

  // Get the public key in hex form
  const EC_POINT *pub_key_point = EC_KEY_get0_public_key(ec_key);
  if (!pub_key_point)
  {
    std::cerr << "Error getting public key" << std::endl;
    print_openssl_error();
    EC_KEY_free(ec_key);
    return {};
  }

  BN_CTX *ctx = BN_CTX_new();
  if (!ctx)
  {
    std::cerr << "Error creating BN_CTX" << std::endl;
    print_openssl_error();
    EC_KEY_free(ec_key);
    return {};
  }

  char *pub_key_hex = EC_POINT_point2hex(EC_KEY_get0_group(ec_key), pub_key_point,
                                         POINT_CONVERSION_COMPRESSED, ctx);
  if (!pub_key_hex)
  {
    std::cerr << "Error converting public key to hex" << std::endl;
    print_openssl_error();
    BN_CTX_free(ctx);
    EC_KEY_free(ec_key);
    return {};
  }

  // Remove prefix from the PK
  std::string pub_key_hex_str(pub_key_hex);
  std::string prefix = pub_key_hex_str.substr(0, 2);
  if (prefix == "02")
    prefix = "compressed_y_0";
  else if (prefix == "03")
    prefix = "compressed_y_1";

  pub_key_hex_str = pub_key_hex_str.substr(2);

  public_key.prefix = prefix;
  public_key.pk = pub_key_hex_str;

  EC_KEY_free(ec_key);

  return public_key;
}

// Function to verify the signature
bool ATResponse::signatureVerification(const std::string &tbsData_hex, GNecdsaNistP256 &rValue, const std::string &sValue, GNecdsaNistP256 verifyKeyIndicator)
{

  int compression = 0;
  std::string publicKeyEA = "";
  if (!verifyKeyIndicator.p256_x_only.empty())
  {
    publicKeyEA = verifyKeyIndicator.p256_x_only;
  }
  else if (!verifyKeyIndicator.p256_compressed_y_0.empty())
  {
    publicKeyEA = verifyKeyIndicator.p256_compressed_y_0;
    compression = 2;
  }
  else if (!verifyKeyIndicator.p256_compressed_y_1.empty())
  {
    publicKeyEA = verifyKeyIndicator.p256_compressed_y_1;
    compression = 3;
  }

  EC_KEY *EAPublicKey = loadCompressedPublicKey(publicKeyEA, compression);

  if (!EAPublicKey)
  {
    std::cerr << "Failed to load the public key!" << std::endl;
    return {};
  }
  
  // Usa il certificato AA letto da PKI_info.ini (AAcert1..4 concatenati)
  if (aaCert1.empty() || aaCert2.empty() || aaCert3.empty() || aaCert4.empty()) {
    // fallback: prova a leggere dal file ini se non già caricato
    readIniFile();
  }
  std::string signIdentifierSelf = aaCert1 + aaCert2 + aaCert3 + aaCert4;

  std::vector<unsigned char> tbsData_bytes(tbsData_hex.begin(), tbsData_hex.end());
  std::vector<unsigned char> signIDbytes = hexStringToBytes(signIdentifierSelf);


  // Compute SHA-256 hash
  unsigned char tbsData_hash[SHA256_DIGEST_LENGTH];
  computeSHA256(tbsData_bytes, tbsData_hash);


  unsigned char signIDself_hash[SHA256_DIGEST_LENGTH];
  computeSHA256(signIDbytes, signIDself_hash);

   
  // Concatenate the hashes
  std::vector<unsigned char> concatenatedHashes = concatenateHashes(tbsData_hash, signIDself_hash);

  // Compute SHA-256 hash of the concatenated hashes
  unsigned char final_hash[SHA256_DIGEST_LENGTH];
  computeSHA256(concatenatedHashes, final_hash);
  
  std::string r_hex;
  // Here put the hexadecimal string related rSig, sSig (inside outer signature in CAM) and pk (inside verifyKeyIndicator in certificate)
  if (!rValue.p256_x_only.empty())
  {
    r_hex = rValue.p256_x_only;
  }
  else if (!rValue.p256_compressed_y_0.empty())
  {
    r_hex = rValue.p256_compressed_y_0;
  }
  else if (!rValue.p256_compressed_y_1.empty())
  {
    r_hex = rValue.p256_compressed_y_1;
  }
  std::string s_hex = sValue;

  std::vector<unsigned char> r_bytes(r_hex.begin(), r_hex.end());
  std::vector<unsigned char> s_bytes(s_hex.begin(), s_hex.end());

  // Create the ECDSA_SIG object
    ECDSA_SIG* signature = ECDSA_SIG_new();
    if (!signature) {
        std::cerr << "Error creating ECDSA_SIG object" << std::endl;
        print_openssl_error();
        EC_KEY_free(EAPublicKey);
        return 1;
    }

    // Convert r and s in bignum objects 
    BIGNUM* r = BN_bin2bn(r_bytes.data(), r_bytes.size(), nullptr);
    BIGNUM* s = BN_bin2bn(s_bytes.data(), s_bytes.size(), nullptr);

    if (!r || !s) {
        std::cerr << "Error converting r or s" << std::endl;
        print_openssl_error();
        ECDSA_SIG_free(signature);
        EC_KEY_free(EAPublicKey);
        if (r) BN_free(r);
        if (s) BN_free(s);
        return 1;
    }

    // Setting signature through r and s values
    if (!ECDSA_SIG_set0(signature, r, s)) {
        std::cerr << "Error setting r and s in signature" << std::endl;
        print_openssl_error();
        ECDSA_SIG_free(signature);
        EC_KEY_free(EAPublicKey);
        return 1;
    }

    
    // Verify the signature
    int verify_status = ECDSA_do_verify(final_hash, SHA256_DIGEST_LENGTH, signature, EAPublicKey);
    if (verify_status == 1) {
        std::cout << "AT Signature is valid" << std::endl;
    } else if (verify_status == 0) {
        std::cout << "AT Signature is invalid" << std::endl;
    } else {
        std::cerr << "Error verifying signature" << std::endl;
        print_openssl_error();
    }

    // Clean up
    ECDSA_SIG_free(signature);
    EC_KEY_free(EAPublicKey);

    return verify_status;
}

ATResponse::GNcertificateDC ATResponse::getATResponse()
{

    readIniFile();
    //--decode certificate part--
    std::string AAcertificate = aaCert1 + aaCert2 + aaCert3 + aaCert4;
  std::vector<unsigned char> binaryCert = hexStringToBytes(AAcertificate);

  // Compute SHA-256 hash
  unsigned char certificate_hash[SHA256_DIGEST_LENGTH];
  computeSHA256(binaryCert, certificate_hash);

  std::string certContent(binaryCert.begin(), binaryCert.end());
  bool getValue_ok;

  // Decode the certificate content to Ieee1609Dot2Data structure
  asn1cpp::Seq<CertificateBase> certData_decoded;
  certData_decoded = asn1cpp::oer::decode(certContent, CertificateBase);

  GNcertificateDC newCert;
  newCert.version = asn1cpp::getField(certData_decoded->version, long);
  newCert.type = asn1cpp::getField(certData_decoded->type, long);
  newCert.issuer = asn1cpp::getField(certData_decoded->issuer.choice.sha256AndDigest, std::string);

  if (asn1cpp::getField(certData_decoded->toBeSigned.id.present, CertificateId_PR) == CertificateId_PR_none)
    newCert.tbs.id = 0;
  else if (asn1cpp::getField(certData_decoded->toBeSigned.id.present, CertificateId_PR) == CertificateId_PR_name)
    newCert.tbs.name = asn1cpp::getField(certData_decoded->toBeSigned.id.choice.name, std::string);

  newCert.tbs.cracaId = asn1cpp::getField(certData_decoded->toBeSigned.cracaId, std::string);
  newCert.tbs.crlSeries = asn1cpp::getField(certData_decoded->toBeSigned.crlSeries, uint16_t);
  newCert.tbs.validityPeriod_start = asn1cpp::getField(certData_decoded->toBeSigned.validityPeriod.start, uint32_t);
  if (asn1cpp::getField(certData_decoded->toBeSigned.validityPeriod.duration.present, Duration_PR) == Duration_PR_hours)
    newCert.tbs.validityPeriod_duration = asn1cpp::getField(certData_decoded->toBeSigned.validityPeriod.duration.choice.hours, long);
  int size2 = asn1cpp::sequenceof::getSize(certData_decoded->toBeSigned.appPermissions);
  for (int j = 0; j < size2; j++)
  {
    auto appPermDecoded = asn1cpp::sequenceof::getSeq(certData_decoded->toBeSigned.appPermissions, PsidSsp, j, &getValue_ok);
    GNpsidSsp newServ;
    newServ.psid = asn1cpp::getField(appPermDecoded->psid, unsigned long);
    auto servicePermission = asn1cpp::getSeqOpt(appPermDecoded->ssp, ServiceSpecificPermissions, &getValue_ok);
    if (asn1cpp::getField(servicePermission->present, ServiceSpecificPermissions_PR) == ServiceSpecificPermissions_PR_bitmapSsp)
    {
      newServ.bitmapSsp = asn1cpp::getField(servicePermission->choice.bitmapSsp, std::string);
    }
    newCert.tbs.appPermissions.push_back(newServ);
  }

  newCert.tbs.symAlgEnc = asn1cpp::getField(certData_decoded->toBeSigned.encryptionKey->supportedSymmAlg, long);
  if (asn1cpp::getField(certData_decoded->toBeSigned.encryptionKey->publicKey.present, BasePublicEncryptionKey_PR) == BasePublicEncryptionKey_PR_eciesNistP256)
    {
      switch (asn1cpp::getField(certData_decoded->toBeSigned.encryptionKey->publicKey.choice.eciesNistP256.present, EccP256CurvePoint_PR))
      {
      case EccP256CurvePoint_PR_x_only:
        newCert.tbs.encPublicKey.p256_x_only = asn1cpp::getField(certData_decoded->toBeSigned.encryptionKey->publicKey.choice.eciesNistP256.choice.x_only, std::string);
        break;
      case EccP256CurvePoint_PR_fill:
        newCert.tbs.encPublicKey.p256_fill = "NULL";
        break;
      case EccP256CurvePoint_PR_compressed_y_0:
        newCert.tbs.encPublicKey.p256_compressed_y_0 = asn1cpp::getField(certData_decoded->toBeSigned.encryptionKey->publicKey.choice.eciesNistP256.choice.compressed_y_0, std::string);
        break;
      case EccP256CurvePoint_PR_compressed_y_1:
        newCert.tbs.encPublicKey.p256_compressed_y_1 = asn1cpp::getField(certData_decoded->toBeSigned.encryptionKey->publicKey.choice.eciesNistP256.choice.compressed_y_1, std::string);
        break;
      case EccP256CurvePoint_PR_uncompressedP256:
        newCert.tbs.encPublicKey.p256_uncompressed_x = asn1cpp::getField(certData_decoded->toBeSigned.encryptionKey->publicKey.choice.eciesNistP256.choice.uncompressedP256.x, std::string);
        newCert.tbs.encPublicKey.p256_uncompressed_y = asn1cpp::getField(certData_decoded->toBeSigned.encryptionKey->publicKey.choice.eciesNistP256.choice.uncompressedP256.y, std::string);
        break;
      default:
        break;
      }
    }


  // Filling last certificate field, verifyKeyIndicator.
  if (asn1cpp::getField(certData_decoded->toBeSigned.verifyKeyIndicator.present, VerificationKeyIndicator_PR) == VerificationKeyIndicator_PR_verificationKey)
  {
    if (asn1cpp::getField(certData_decoded->toBeSigned.verifyKeyIndicator.choice.verificationKey.present, PublicVerificationKey_PR) == PublicVerificationKey_PR_ecdsaNistP256)
    {
      switch (asn1cpp::getField(certData_decoded->toBeSigned.verifyKeyIndicator.choice.verificationKey.choice.ecdsaNistP256.present, EccP256CurvePoint_PR))
      {
      case EccP256CurvePoint_PR_x_only:
        newCert.tbs.verifyKeyIndicator.p256_x_only = asn1cpp::getField(certData_decoded->toBeSigned.verifyKeyIndicator.choice.verificationKey.choice.ecdsaNistP256.choice.x_only, std::string);
        break;
      case EccP256CurvePoint_PR_fill:
        newCert.tbs.verifyKeyIndicator.p256_fill = "NULL";
        break;
      case EccP256CurvePoint_PR_compressed_y_0:
        newCert.tbs.verifyKeyIndicator.p256_compressed_y_0 = asn1cpp::getField(certData_decoded->toBeSigned.verifyKeyIndicator.choice.verificationKey.choice.ecdsaNistP256.choice.compressed_y_0, std::string);
        break;
      case EccP256CurvePoint_PR_compressed_y_1:
        newCert.tbs.verifyKeyIndicator.p256_compressed_y_1 = asn1cpp::getField(certData_decoded->toBeSigned.verifyKeyIndicator.choice.verificationKey.choice.ecdsaNistP256.choice.compressed_y_1, std::string);
        break;
      case EccP256CurvePoint_PR_uncompressedP256:
        newCert.tbs.verifyKeyIndicator.p256_uncompressed_x = asn1cpp::getField(certData_decoded->toBeSigned.verifyKeyIndicator.choice.verificationKey.choice.ecdsaNistP256.choice.uncompressedP256.x, std::string);
        newCert.tbs.verifyKeyIndicator.p256_uncompressed_y = asn1cpp::getField(certData_decoded->toBeSigned.verifyKeyIndicator.choice.verificationKey.choice.ecdsaNistP256.choice.uncompressedP256.y, std::string);
        break;
      default:
        break;
      }
    }
  }
  // Signature part inside certificate signer. In this case the signature is of type Brain384. This code will note save anything. But for the moment is not important the EA signature.
  auto signcertData_decoded = asn1cpp::getSeqOpt(certData_decoded->signature, Signature, &getValue_ok);
  // auto present = asn1cpp::getField(signcertData_decoded->present, Signature_PR);
  if (asn1cpp::getField(signcertData_decoded->present, Signature_PR) == Signature_PR_ecdsaNistP256Signature)
  {
    auto present4 = asn1cpp::getField(signcertData_decoded->choice.ecdsaNistP256Signature.rSig.present, EccP256CurvePoint_PR);
    switch (present4)
    {
    case EccP256CurvePoint_PR_x_only:
      newCert.rSig.p256_x_only = asn1cpp::getField(signcertData_decoded->choice.ecdsaNistP256Signature.rSig.choice.x_only, std::string);
      break;
    case EccP256CurvePoint_PR_fill:
      newCert.rSig.p256_fill = "NULL";
      break;
    case EccP256CurvePoint_PR_compressed_y_0:
      newCert.rSig.p256_compressed_y_0 = asn1cpp::getField(signcertData_decoded->choice.ecdsaNistP256Signature.rSig.choice.compressed_y_0, std::string);
      break;
    case EccP256CurvePoint_PR_compressed_y_1:
      newCert.rSig.p256_compressed_y_1 = asn1cpp::getField(signcertData_decoded->choice.ecdsaNistP256Signature.rSig.choice.compressed_y_1, std::string);
      break;
    case EccP256CurvePoint_PR_uncompressedP256:
      newCert.rSig.p256_uncompressed_x = asn1cpp::getField(signcertData_decoded->choice.ecdsaNistP256Signature.rSig.choice.uncompressedP256.x, std::string);
      newCert.rSig.p256_uncompressed_y = asn1cpp::getField(signcertData_decoded->choice.ecdsaNistP256Signature.rSig.choice.uncompressedP256.y, std::string);
      break;
    default:
      break;
    }
    newCert.signature_sSig = asn1cpp::getField(signcertData_decoded->choice.ecdsaNistP256Signature.sSig, std::string);
  }

  //----------------------

  

  if (readFileContent("./pkiReqRes/responseAT.bin", &dataResponse, &length) != 0) {
    std::cout << "[ERR] Error reading file" << std::endl;
  }

  public_key = recoverECKeyPair(ephemeral);

  //std::string packetContent(dataResponse, dataResponse + length); Check if the other is correct, otherwise use this.
  std::string packetContent((char *)dataResponse, (int)length);
  
  free(dataResponse);

  cPacket encPacket;

  asn1cpp::Seq<Ieee1609Dot2Data> ieeeData_decoded;

  ieeeData_decoded = asn1cpp::oer::decode(packetContent, Ieee1609Dot2Data);
  encPacket.m_protocolversion = asn1cpp::getField(ieeeData_decoded->protocolVersion, long);

  auto contentDecoded = asn1cpp::getSeqOpt(ieeeData_decoded->content, Ieee1609Dot2Content, &getValue_ok);

  // check the present, here is always signed data
  auto present1 = asn1cpp::getField(contentDecoded->present, Ieee1609Dot2Content_PR);
  if (present1 == Ieee1609Dot2Content_PR_encryptedData)
  {
    auto encDataDec = asn1cpp::getSeq(contentDecoded->choice.encryptedData, EncryptedData, &getValue_ok);
    // recipients part

    int size = asn1cpp::sequenceof::getSize(encDataDec->recipients);
    for (int i = 0; i < size; i++)
    {
      // Filling all certificate fields
      auto recipient = asn1cpp::sequenceof::getSeq(encDataDec->recipients, RecipientInfo, i);
      auto present3 = asn1cpp::getField(recipient->present, RecipientInfo_PR);
      if (present3 == RecipientInfo_PR_pskRecipInfo)
      {
        encPacket.content.encrData.recipient = asn1cpp::getField(recipient->choice.pskRecipInfo, std::string);
      }
    }
    // cipher part
    auto present6 = asn1cpp::getField(encDataDec->ciphertext.present, SymmetricCiphertext_PR);
    if (present6 == SymmetricCiphertext_PR_aes128ccm)
    {
      encPacket.content.encrData.ciphertext = asn1cpp::getField(encDataDec->ciphertext.choice.aes128ccm.ccmCiphertext, std::string);
      encPacket.content.encrData.nonce = asn1cpp::getField(encDataDec->ciphertext.choice.aes128ccm.nonce, std::string);
    }
  }

  encPacket.content.unsecuredData = doDecryption(encPacket.content.encrData.ciphertext,encPacket.content.encrData.nonce);
 if (encPacket.content.unsecuredData.empty())
  {
    std::cerr << "Error decrypting the message" << std::endl;
    return {};
  }

  asn1cpp::Seq<Ieee1609Dot2Data> signedDataDecoded;
  signedDataDecoded = asn1cpp::oer::decode(encPacket.content.unsecuredData, Ieee1609Dot2Data);

  cPacket sPack;
  sPack.m_protocolversion = asn1cpp::getField(signedDataDecoded->protocolVersion, long);
  // bool getValue_ok2;
  auto contentDecoded2 = asn1cpp::getSeqOpt(signedDataDecoded->content, Ieee1609Dot2Content, &getValue_ok);
  // check the present, here is always signed data
  auto present7 = asn1cpp::getField(contentDecoded2->present, Ieee1609Dot2Content_PR);
  if (present7 == Ieee1609Dot2Content_PR_signedData)
  {
    auto signDec = asn1cpp::getSeqOpt(contentDecoded2->choice.signedData, SignedData, &getValue_ok);
    // First signed data field, HASH ID
    sPack.content.signData.hashID = asn1cpp::getField(signDec->hashId, long);
    auto tbsDecoded = asn1cpp::getSeqOpt(signDec->tbsData, ToBeSignedData, &getValue_ok);
    auto payload_decoded = asn1cpp::getSeqOpt(tbsDecoded->payload, SignedDataPayload, &getValue_ok);
    auto dataContainerDecoded = asn1cpp::getSeqOpt(payload_decoded->data, Ieee1609Dot2Data, &getValue_ok);
    sPack.content.signData.tbsdata.protocolversion = asn1cpp::getField(dataContainerDecoded->protocolVersion, long);
    auto contentContainerDecoded = asn1cpp::getSeqOpt(dataContainerDecoded->content, Ieee1609Dot2Content);
    auto present8 = asn1cpp::getField(contentContainerDecoded->present, Ieee1609Dot2Content_PR);
    if (present8 == Ieee1609Dot2Content_PR_unsecuredData)
    {
      sPack.content.signData.tbsdata.unsecuredData = asn1cpp::getField(contentContainerDecoded->choice.unsecuredData, std::string);
    }
    sPack.content.signData.tbsdata.header_psid = asn1cpp::getField(tbsDecoded->headerInfo.psid, unsigned long);
    sPack.content.signData.tbsdata.header_generationTime = asn1cpp::getField(tbsDecoded->headerInfo.generationTime, uint64_t);
    auto present3 = asn1cpp::getField(signDec->signer.present, SignerIdentifier_PR);
    if (present3 == SignerIdentifier_PR_digest)
    {
      sPack.content.signData.signer_digest = asn1cpp::getField(signDec->signer.choice.digest, std::string);
    }

    auto present9 = asn1cpp::getField(signDec->signature.present, Signature_PR);

    if (present9 == Signature_PR_ecdsaNistP256Signature)
    {
      auto present10 = asn1cpp::getField(signDec->signature.choice.ecdsaNistP256Signature.rSig.present, EccP256CurvePoint_PR);
      switch (present10)
      {
      case EccP256CurvePoint_PR_x_only:
        sPack.content.signData.rSig.p256_x_only = asn1cpp::getField(signDec->signature.choice.ecdsaNistP256Signature.rSig.choice.x_only, std::string);
        break;
      case EccP256CurvePoint_PR_fill:
        sPack.content.signData.rSig.p256_fill = "NULL";
        break;
      case EccP256CurvePoint_PR_compressed_y_0:
        sPack.content.signData.rSig.p256_compressed_y_0 = asn1cpp::getField(signDec->signature.choice.ecdsaNistP256Signature.rSig.choice.compressed_y_0, std::string);
        break;
      case EccP256CurvePoint_PR_compressed_y_1:
        sPack.content.signData.rSig.p256_compressed_y_1 = asn1cpp::getField(signDec->signature.choice.ecdsaNistP256Signature.rSig.choice.compressed_y_1, std::string);
        break;
      case EccP256CurvePoint_PR_uncompressedP256:
        sPack.content.signData.rSig.p256_uncompressed_x = asn1cpp::getField(signDec->signature.choice.ecdsaNistP256Signature.rSig.choice.uncompressedP256.x, std::string);
        sPack.content.signData.rSig.p256_uncompressed_y = asn1cpp::getField(signDec->signature.choice.ecdsaNistP256Signature.rSig.choice.uncompressedP256.y, std::string);
        break;
      default:
        break;
      }
      sPack.content.signData.signature_sSig = asn1cpp::getField(signDec->signature.choice.ecdsaNistP256Signature.sSig, std::string);
    }

    std::string tbs_hex = asn1cpp::oer::encode(tbsDecoded);
    signValidation = signatureVerification(tbs_hex, sPack.content.signData.rSig, sPack.content.signData.signature_sSig, newCert.tbs.verifyKeyIndicator);
  }

  if (signValidation)
  {
    asn1cpp::Seq<EtsiTs102941MessagesItss_EtsiTs102941Data> etsiData;
    etsiData = asn1cpp::oer::decode(sPack.content.signData.tbsdata.unsecuredData, EtsiTs102941MessagesItss_EtsiTs102941Data);
    // int etsiVersion = asn1cpp::getField(etsiData->version, int);
    auto etsiContent = asn1cpp::getSeq(etsiData->content, EtsiTs102941MessagesItss_EtsiTs102941DataContent, &getValue_ok);
    auto pres = asn1cpp::getField(etsiContent->present, EtsiTs102941MessagesItss_EtsiTs102941DataContent_PR);
    if (pres == EtsiTs102941MessagesItss_EtsiTs102941DataContent_PR_authorizationResponse)
    {
      auto res = asn1cpp::getSeq(etsiContent->choice.authorizationResponse, InnerAtResponse, &getValue_ok);

      response ATres;
      ATres.requestHash = asn1cpp::getField(res->requestHash, std::string);
      ATres.response_code = asn1cpp::getField(res->responseCode, long);
      if ( ATres.response_code != 0){
        std::cout << "Response code is not 0, request denied" << std::endl;
        return {};
      }
      bool getValue_ok3;
      auto certDecoded = asn1cpp::getSeqOpt(res->certificate, CertificateBase, &getValue_ok3);
      ATres.certificate.version = asn1cpp::getField(certDecoded->version, long);
      ATres.certificate.type = asn1cpp::getField(certDecoded->type, long);
      auto present11 = asn1cpp::getField(certDecoded->issuer.present, IssuerIdentifierSec_PR);
      if (present11 == IssuerIdentifierSec_PR_sha256AndDigest)
      {
        ATres.certificate.issuer = asn1cpp::getField(certDecoded->issuer.choice.sha256AndDigest, std::string);
      }

      if (asn1cpp::getField(certDecoded->toBeSigned.id.present, CertificateId_PR) == CertificateId_PR_none)
      {
        ATres.certificate.tbs.id = 0;
      } else if (asn1cpp::getField(certDecoded->toBeSigned.id.present, CertificateId_PR) == CertificateId_PR_name){
          ATres.certificate.tbs.name = asn1cpp::getField(certDecoded->toBeSigned.id.choice.name, std::string);
      }
      ATres.certificate.tbs.cracaId = asn1cpp::getField(certDecoded->toBeSigned.cracaId, std::string);
      ATres.certificate.tbs.crlSeries = asn1cpp::getField(certDecoded->toBeSigned.crlSeries, uint16_t);
      ATres.certificate.tbs.validityPeriod_start = asn1cpp::getField(certDecoded->toBeSigned.validityPeriod.start, uint32_t);
      if (asn1cpp::getField(certDecoded->toBeSigned.validityPeriod.duration.present, Duration_PR) == Duration_PR_hours)
      {
        ATres.certificate.tbs.validityPeriod_duration = asn1cpp::getField(certDecoded->toBeSigned.validityPeriod.duration.choice.hours, long);
      }
      int size2 = asn1cpp::sequenceof::getSize(certDecoded->toBeSigned.appPermissions);
      for (int j = 0; j < size2; j++)
      {
        auto appPermDecoded = asn1cpp::sequenceof::getSeq(certDecoded->toBeSigned.appPermissions, PsidSsp, j, &getValue_ok);
        GNpsidSsp newServ;
        newServ.psid = asn1cpp::getField(appPermDecoded->psid, unsigned long);
        auto servicePermission = asn1cpp::getSeqOpt(appPermDecoded->ssp, ServiceSpecificPermissions, &getValue_ok);
        if (asn1cpp::getField(servicePermission->present, ServiceSpecificPermissions_PR) == ServiceSpecificPermissions_PR_bitmapSsp)
        {
          newServ.bitmapSsp = asn1cpp::getField(servicePermission->choice.bitmapSsp, std::string);
        }
        ATres.certificate.tbs.appPermissions.push_back(newServ);
      }
      // Filling last certificate field, verifyKeyIndicator.
      if (asn1cpp::getField(certDecoded->toBeSigned.verifyKeyIndicator.present, VerificationKeyIndicator_PR) == VerificationKeyIndicator_PR_verificationKey)
      {
        if (asn1cpp::getField(certDecoded->toBeSigned.verifyKeyIndicator.choice.verificationKey.present, PublicVerificationKey_PR) == PublicVerificationKey_PR_ecdsaNistP256)
        {
          switch (
              asn1cpp::getField(certDecoded->toBeSigned.verifyKeyIndicator.choice.verificationKey.choice.ecdsaNistP256.present, EccP256CurvePoint_PR))
          {
          case EccP256CurvePoint_PR_x_only:
            ATres.certificate.tbs.verifyKeyIndicator.presentVerKey = 1;
            ATres.certificate.tbs.verifyKeyIndicator.p256_x_only = asn1cpp::getField(certDecoded->toBeSigned.verifyKeyIndicator.choice.verificationKey.choice.ecdsaNistP256.choice.x_only, std::string);
            break;
          case EccP256CurvePoint_PR_fill:
              ATres.certificate.tbs.verifyKeyIndicator.presentVerKey = 2;
            ATres.certificate.tbs.verifyKeyIndicator.p256_fill = "NULL";
            break;
          case EccP256CurvePoint_PR_compressed_y_0:
              ATres.certificate.tbs.verifyKeyIndicator.presentVerKey = 3;
            ATres.certificate.tbs.verifyKeyIndicator.p256_compressed_y_0 = asn1cpp::getField(certDecoded->toBeSigned.verifyKeyIndicator.choice.verificationKey.choice.ecdsaNistP256.choice.compressed_y_0, std::string);
            break;
          case EccP256CurvePoint_PR_compressed_y_1:
              ATres.certificate.tbs.verifyKeyIndicator.presentVerKey = 4;
            ATres.certificate.tbs.verifyKeyIndicator.p256_compressed_y_1 = asn1cpp::getField(certDecoded->toBeSigned.verifyKeyIndicator.choice.verificationKey.choice.ecdsaNistP256.choice.compressed_y_1, std::string);
            break;
          case EccP256CurvePoint_PR_uncompressedP256:
              ATres.certificate.tbs.verifyKeyIndicator.presentVerKey = 5;
            ATres.certificate.tbs.verifyKeyIndicator.p256_uncompressed_x = asn1cpp::getField(certDecoded->toBeSigned.verifyKeyIndicator.choice.verificationKey.choice.ecdsaNistP256.choice.uncompressedP256.x, std::string);
            ATres.certificate.tbs.verifyKeyIndicator.p256_uncompressed_y = asn1cpp::getField(certDecoded->toBeSigned.verifyKeyIndicator.choice.verificationKey.choice.ecdsaNistP256.choice.uncompressedP256.y, std::string);
            break;
          default:
            break;
          }
        }
      }
      // Signature part inside certificate signer
      auto signCertDecoded = asn1cpp::getSeqOpt(certDecoded->signature, Signature, &getValue_ok);
      if (asn1cpp::getField(signCertDecoded->present, Signature_PR) == Signature_PR_ecdsaNistP256Signature)
      {
        auto present12 = asn1cpp::getField(signCertDecoded->choice.ecdsaNistP256Signature.rSig.present, EccP256CurvePoint_PR);
        switch (present12)
        {
        case EccP256CurvePoint_PR_x_only:
            ATres.certificate.rSig.presentRSig = 1;
          ATres.certificate.rSig.p256_x_only = asn1cpp::getField(signCertDecoded->choice.ecdsaNistP256Signature.rSig.choice.x_only, std::string);
          break;
        case EccP256CurvePoint_PR_fill:
            ATres.certificate.rSig.presentRSig = 2;
          ATres.certificate.rSig.p256_fill = "NULL";
          break;
        case EccP256CurvePoint_PR_compressed_y_0:
            ATres.certificate.rSig.presentRSig = 3;
          ATres.certificate.rSig.p256_compressed_y_0 = asn1cpp::getField(signCertDecoded->choice.ecdsaNistP256Signature.rSig.choice.compressed_y_0, std::string);
          break;
        case EccP256CurvePoint_PR_compressed_y_1:
            ATres.certificate.rSig.presentRSig = 4;
          ATres.certificate.rSig.p256_compressed_y_1 = asn1cpp::getField(signCertDecoded->choice.ecdsaNistP256Signature.rSig.choice.compressed_y_1, std::string);
          break;
        case EccP256CurvePoint_PR_uncompressedP256:
            ATres.certificate.rSig.presentRSig = 5;
          ATres.certificate.rSig.p256_uncompressed_x = asn1cpp::getField(signCertDecoded->choice.ecdsaNistP256Signature.rSig.choice.uncompressedP256.x, std::string);
          ATres.certificate.rSig.p256_uncompressed_y = asn1cpp::getField(signCertDecoded->choice.ecdsaNistP256Signature.rSig.choice.uncompressedP256.y, std::string);
          break;
        default:
          break;
        }
        ATres.certificate.signature_sSig = asn1cpp::getField(signCertDecoded->choice.ecdsaNistP256Signature.sSig, std::string);
      }
      
      std::string at_hex = asn1cpp::oer::encode(certDecoded);
      std::vector<unsigned char> at_bytes(at_hex.begin(), at_hex.end());
      std::cout << "[INFO] AT: " << to_hex_string(at_bytes) << std::endl;
      return ATres.certificate;
    }
  } else std::cout << "[ERR] Error - signature not valid" << std::endl;
  return {};
}

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif