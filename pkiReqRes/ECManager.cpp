// PKI Interaction - Enrollment Credential request
// This code is used to generate an Enrollment Credential request message, allows OScar to request a certificate from the PKI.
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
#include <cstdint>
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
#include <openssl/pkcs12.h>
#include <fstream>
#include <ECManager.h>
#include <sys/stat.h>
#include <HTTPRequest.hpp>
#include <INIReader.h>

extern "C"
{
#include "Ieee1609Dot2Data.h"
#include "InnerEcRequest.h"
#include "EtsiTs102941MessagesItss_EtsiTs102941Data.h"
}

// Constants
#define AES_KEY_LENGTH 16
#define NONCE_LENGTH 12
#define AES_CCM_TAG_LENGTH 16

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

ECManager::~ECManager ()
{

}

ECManager::ECManager ()
{
    ephemeral = false;
    m_ecKey = nullptr;
    m_EPHecKey = nullptr;
    request_result = "";
    signedData_result = "";
    encode_result = "";
    m_protocolversion = 1;
    m_version = 3;
    m_recipientID = "D41845A1F71C356A"; // 8 bytes contraint
    m_hashId = HashAlgorithm_sha256;
    m_psid = 623;
    m_itsID = "4472697665580108";
    m_certFormat = 1;
    m_hours = 168; // 7 days
    m_bitmapSspEA = "01C0";

}



// Function to convert a hex string to a byte array
std::vector<unsigned char> ECManager::hexStringToBytes(const std::string &hex)
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

// handleErrors() is a function to print the error stack from OpenSSL
void ECManager::handleErrors()
{
  ERR_print_errors_fp(stderr);
  abort();
}

// Function to convert a byte array to a hex string
std::string ECManager::to_hex_string(const unsigned char *data, size_t length)
{
  std::ostringstream oss;
  for (size_t i = 0; i < length; ++i)
  {
    oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(data[i]);
  }
  return oss.str();
}

// Function to convert a byte array to a hex string
std::string ECManager::to_hex_string(const std::vector<unsigned char> &data)
{
  std::ostringstream oss;
  for (unsigned char byte : data)
  {
    oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
  }
  return oss.str();
}

// Function to load a compressed public key from a hex string
EVP_PKEY* ECManager::loadCompressedPublicKey(const std::string &compressedKey, int compression)
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

  EVP_PKEY *evp_pkey = EVP_PKEY_new();
  if (!EVP_PKEY_assign_EC_KEY(evp_pkey, ec_key))
  {
    std::cerr << "Errore nell'assegnare EC_KEY a EVP_PKEY" << std::endl;
    EVP_PKEY_free(evp_pkey);
    EC_KEY_free(ec_key);
    EC_POINT_free(point);
    return nullptr;
  }

  EC_POINT_free(point);
  return evp_pkey;
}

// Function to compute the SHA-256 hash of a byte array
void ECManager::computeSHA256(const std::vector<unsigned char> &data,
                   unsigned char hash[SHA256_DIGEST_LENGTH])
{
  SHA256_CTX sha256;
  SHA256_Init(&sha256);
  SHA256_Update(&sha256, data.data(), data.size());
  SHA256_Final(hash, &sha256);
}

// Function to concatenate two byte arrays
std::vector<unsigned char> ECManager::concatenateHashes(const unsigned char hash1[SHA256_DIGEST_LENGTH], const unsigned char hash2[SHA256_DIGEST_LENGTH]) {
    std::vector<unsigned char> concatenatedHashes;
    concatenatedHashes.insert(concatenatedHashes.end(), hash1, hash1 + SHA256_DIGEST_LENGTH);
    concatenatedHashes.insert(concatenatedHashes.end(), hash2, hash2 + SHA256_DIGEST_LENGTH);
    return concatenatedHashes;
}


// Function to derive a key using KDF2
void ECManager::deriveKeyWithKDF2(const unsigned char* sharedSecret, size_t secretLen,
                       const unsigned char* P1, size_t P1_len,
                       unsigned char* derivedKey, size_t derivedKeyLen) {
    // size_t hBits = SHA256_DIGEST_LENGTH * 8; // SHA-256 produces 256 bits (32 bytes) - currently unused
    // size_t cThreshold = (derivedKeyLen * 8 + hBits - 1) / hBits;  // How many iterations are needed - currently unused

    size_t offset = 0;
    unsigned int counter = 1;
    std::vector<unsigned char> hashInput(secretLen + P1_len + 4);  // Buffer: Z + counter + P
    std::memcpy(hashInput.data(), sharedSecret, secretLen);  // Copia Z in hashInput

    while (offset < derivedKeyLen) {
        // Inserisci counter come 32-bit MSB (big endian) dopo sharedSecret
        hashInput[secretLen + 0] = (counter >> 24) & 0xFF;
        hashInput[secretLen + 1] = (counter >> 16) & 0xFF;
        hashInput[secretLen + 2] = (counter >> 8) & 0xFF;
        hashInput[secretLen + 3] = counter & 0xFF;

        // Aggiungi P1 al buffer
        std::memcpy(hashInput.data() + secretLen + 4, P1, P1_len);

        // Calcola l'hash
        unsigned char hash[SHA256_DIGEST_LENGTH];
        computeSHA256(hashInput,hash);

        // Copia l'output dell'hash nel buffer finale
        size_t copyLen = std::min(derivedKeyLen - offset, (size_t)SHA256_DIGEST_LENGTH);
        std::memcpy(derivedKey + offset, hash, copyLen);
        offset += copyLen;
        counter++;
    }
}


std::string ECManager::retrieveStringFromFile(const std::string& fileName) {
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

void ECManager::saveStringToFile(const std::string& key, const std::string& fileName) {
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

// Function to encrypt a message using ECIES
std::vector<unsigned char> ECManager::encryptMessage(
    std::vector<unsigned char> &plaintext,
    EVP_PKEY *receiverPublicKey,
    std::vector<unsigned char> &encryptedKey,
    std::vector<unsigned char> &ephemeralPublicKey,
    std::vector<unsigned char> &eciesTag,
    std::vector<unsigned char> &nonce,
    std::vector<unsigned char> &aesCcmTag,
     const unsigned char *p1)
{

unsigned char aesKey[AES_KEY_LENGTH];
    if (RAND_bytes(aesKey, AES_KEY_LENGTH) != 1) {
        handleErrors();
    }

    std::cout << "[INFO] Shared key: " << to_hex_string(aesKey,16) << std::endl;

    ecRes.setAesKey(to_hex_string(aesKey,16));
    std::string fileName = "pskEC.bin";
    saveStringToFile(to_hex_string(aesKey,16), fileName);

    nonce.resize(NONCE_LENGTH);
    if (RAND_bytes(nonce.data(), NONCE_LENGTH) != 1) {
        handleErrors();
    }

    // Step 1: Encrypt the plaintext with AES-128-CCM
    std::vector<unsigned char> ciphertext(plaintext.size());
    int len;
    EVP_CIPHER_CTX *ctx = EVP_CIPHER_CTX_new();
    if (!ctx) handleErrors();

    if (EVP_EncryptInit_ex(ctx, EVP_aes_128_ccm(), nullptr, nullptr, nullptr) != 1) handleErrors();
    if (EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_CCM_SET_IVLEN, NONCE_LENGTH, nullptr) != 1) handleErrors();
    if (EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_CCM_SET_TAG, AES_CCM_TAG_LENGTH, nullptr) != 1) handleErrors();
    if (EVP_EncryptInit_ex(ctx, nullptr, nullptr, aesKey, nonce.data()) != 1) handleErrors();
    if (EVP_EncryptUpdate(ctx, nullptr, &len, nullptr, plaintext.size()) != 1) handleErrors();
    if (EVP_EncryptUpdate(ctx, ciphertext.data(), &len, plaintext.data(), plaintext.size()) != 1) handleErrors();

    int ciphertext_len = len;

    if (EVP_EncryptFinal_ex(ctx, ciphertext.data() + len, &len) != 1) handleErrors();
    ciphertext_len += len;

    aesCcmTag.resize(AES_CCM_TAG_LENGTH);
    if (EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_CCM_GET_TAG, AES_CCM_TAG_LENGTH, aesCcmTag.data()) != 1) handleErrors();

    EVP_CIPHER_CTX_free(ctx);
    ciphertext.resize(ciphertext_len);

    // Step 2a: Generate ephemeral EC key pair
    EVP_PKEY_CTX *pctx = EVP_PKEY_CTX_new_id(EVP_PKEY_EC, nullptr);
    if (!pctx) handleErrors();
    if (EVP_PKEY_keygen_init(pctx) != 1) handleErrors();
    if (EVP_PKEY_CTX_set_ec_paramgen_curve_nid(pctx, NID_X9_62_prime256v1) != 1) handleErrors();

    EVP_PKEY *ephemeralKey = nullptr;
    if (EVP_PKEY_keygen(pctx, &ephemeralKey) != 1) handleErrors();
    EVP_PKEY_CTX_free(pctx);

    // Step 2b: Derive shared secret
    EVP_PKEY_CTX *deriveCtx = EVP_PKEY_CTX_new(ephemeralKey, nullptr);
    if (!deriveCtx) handleErrors();
    if (EVP_PKEY_derive_init(deriveCtx) != 1) handleErrors();
    if (EVP_PKEY_CTX_set_ecdh_cofactor_mode(deriveCtx, 1) != 1) handleErrors(); // Abilita moltiplicazione cofattore
    if (EVP_PKEY_derive_set_peer(deriveCtx, receiverPublicKey) != 1) handleErrors();

    size_t secretLen;
    if (EVP_PKEY_derive(deriveCtx, nullptr, &secretLen) != 1) handleErrors();

    std::vector<unsigned char> sharedSecret(secretLen);
    if (EVP_PKEY_derive(deriveCtx, sharedSecret.data(), &secretLen) != 1) handleErrors();
    EVP_PKEY_CTX_free(deriveCtx);

     // Step 2c: Derive keys ke and km
    unsigned char derivedKey[48];  // ke (16 bytes) + km (32 bytes)
    deriveKeyWithKDF2(sharedSecret.data(), secretLen, p1, SHA256_DIGEST_LENGTH, derivedKey, sizeof(derivedKey));
    


    unsigned char ke[16], km[32];
    memcpy(ke, derivedKey, 16);        // ke is 16 bytes
    memcpy(km, derivedKey + 16, 32);   // km is 32 bytes
    

    // Step 2d: Encrypt AES key with XOR
    encryptedKey.resize(AES_KEY_LENGTH);
    for (size_t i = 0; i < AES_KEY_LENGTH; i++) {
        encryptedKey[i] = aesKey[i] ^ ke[i];
    }

    // Step 3: HMAC computation to achieve integrity

    eciesTag.resize(32);  // Initialize to 32 bytes initially
    HMAC_CTX* hctx = HMAC_CTX_new();
    if (!hctx) handleErrors();

    if (HMAC_Init_ex(hctx, km, 32, EVP_sha256(), nullptr) != 1) handleErrors();
    if (HMAC_Update(hctx, encryptedKey.data(), encryptedKey.size()) != 1) handleErrors();
    unsigned int hmacLen = 0;
    if (HMAC_Final(hctx, eciesTag.data(), &hmacLen) != 1) handleErrors();

    HMAC_CTX_free(hctx);

    // Reduce tag length to 16 bytes
    eciesTag.resize(16);

    int keylen = i2o_ECPublicKey(EVP_PKEY_get1_EC_KEY(ephemeralKey), nullptr);
    ephemeralPublicKey.resize(keylen);
    unsigned char *pubKeyPtr = ephemeralPublicKey.data();
    i2o_ECPublicKey(EVP_PKEY_get1_EC_KEY(ephemeralKey), &pubKeyPtr);

    EVP_PKEY_free(ephemeralKey);
    return ciphertext;
}

ECManager::encData ECManager::doEncryption(std::string message, GNecdsaNistP256 encPkEA, const unsigned char *p1)
{
  // Choose the public key based on the encoding
  int compression = 0;
  std::string publicKeyEA = "";
  if (!encPkEA.p256_x_only.empty())
  {
    publicKeyEA = encPkEA.p256_x_only;
  }
  else if (!encPkEA.p256_compressed_y_0.empty())
  {
    publicKeyEA = encPkEA.p256_compressed_y_0;
    compression = 2;
  }
  else if (!encPkEA.p256_compressed_y_1.empty())
  {
    publicKeyEA = encPkEA.p256_compressed_y_1;
    compression = 3;
  }

  EVP_PKEY *receiverPublicKey = loadCompressedPublicKey(publicKeyEA, compression);

  if (!receiverPublicKey)
  {
    std::cerr << "Failed to load the public key!" << std::endl;
    return {};
  }

  std::vector<unsigned char> plaintext(message.begin(), message.end());

  encData data;
  std::vector<unsigned char> aesCcmTag;
  // Encrypt the message
  std::vector<unsigned char> ciphertext = encryptMessage(plaintext, receiverPublicKey, data.encryptedKey, data.ephemeralPublicKey, data.eciesTag, data.nonce, aesCcmTag, p1);

  // Store the data in the struct
  data.ephemeralPublicKey.erase(data.ephemeralPublicKey.begin());

  // Extract the x and y values from the ephemeral public key (it is in uncompressed form)
  data.x_value.insert(data.x_value.end(), data.ephemeralPublicKey.begin(), data.ephemeralPublicKey.begin() + 32);
  data.y_value.insert(data.y_value.end(), data.ephemeralPublicKey.begin() + 32, data.ephemeralPublicKey.end());

  data.ciphertextWithTag.insert(data.ciphertextWithTag.end(), ciphertext.begin(), ciphertext.end());
  data.ciphertextWithTag.insert(data.ciphertextWithTag.end(), aesCcmTag.begin(), aesCcmTag.end());

  EVP_PKEY_free(receiverPublicKey);

  return data;
}


// Function to print the error stack from OpenSSL
void ECManager::print_openssl_error()
{
  char buffer[120];
  unsigned long error = ERR_get_error();
  ERR_error_string_n(error, buffer, sizeof(buffer));
  std::cerr << buffer << std::endl;
}

// Function to load an EC key pair from a file
EC_KEY* ECManager::loadECKeyFromFile(const std::string &private_key_file, const std::string &public_key_file)
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

// Function to load an EC key pair from a RFC-5480 formatted string
EC_KEY* ECManager::loadECKeyFromRFC5480(const std::string &private_key_rfc, const std::string &public_key_rfc)
{
  EC_KEY *ec_key = nullptr;

  // Step 1: Convert private key hex string to binary DER format
  std::vector<unsigned char> derSK = hexStringToBytes(private_key_rfc);
  const unsigned char *priv_p = derSK.data();

  // Step 2: Parse private key DER (PKCS#8) to create EC_KEY
  PKCS8_PRIV_KEY_INFO *p8info = d2i_PKCS8_PRIV_KEY_INFO(nullptr, &priv_p, derSK.size());
  if (!p8info)
  {
    std::cerr << "Error loading private key from PKCS#8 format" << std::endl;
    ERR_print_errors_fp(stderr);
    return nullptr;
  }
  EVP_PKEY *pkey = EVP_PKCS82PKEY(p8info);
  PKCS8_PRIV_KEY_INFO_free(p8info);
  if (!pkey)
  {
    std::cerr << "Error converting PKCS#8 structure to EVP_PKEY" << std::endl;
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
ECManager::GNpublicKey ECManager::recoverECKeyPair(bool ephemeral)
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
    iniEC iniData = readIniFile();
    std::string public_key_rfc = iniData.pk_rfc;
    std::string private_key_rfc = iniData.sk_rfc1 + iniData.sk_rfc2;
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

// Function to sign a hash with a private key
ECDSA_SIG* ECManager::signHash(const unsigned char *hash, EC_KEY *ec_key)
{
  ECDSA_SIG *signature = ECDSA_do_sign(hash, SHA256_DIGEST_LENGTH, ec_key);
  if (!signature)
  {
    std::cerr << "Error signing hash" << std::endl;
    print_openssl_error();
  }
  return signature;
}

// Function to create a signature material
ECManager::GNsignMaterial ECManager::signatureCreation(const std::string &tbsData_hex, bool ephemeral)
{

  GNsignMaterial signMaterial;
  std::string signIdentifierSelf = "";

  std::vector<unsigned char> tbsData_bytes(tbsData_hex.begin(), tbsData_hex.end());
  //std::vector<unsigned char> tbsData_bytes = hexStringToBytes(tbsData_hex);
  std::vector<unsigned char> signIDbytes = hexStringToBytes(signIdentifierSelf);

  unsigned char tbsData_hash[SHA256_DIGEST_LENGTH];
  computeSHA256(tbsData_bytes, tbsData_hash);

  unsigned char signIDself_hash[SHA256_DIGEST_LENGTH];
  computeSHA256(signIDbytes, signIDself_hash);


  std::vector<unsigned char> concatenatedHashes = concatenateHashes(tbsData_hash, signIDself_hash);

  unsigned char final_hash[SHA256_DIGEST_LENGTH];
  computeSHA256(concatenatedHashes, final_hash);
  /*
  std::cout << "Final result: ";
    for (int i = 0; i < SHA256_DIGEST_LENGTH; i++) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(final_hash[i]);
    }
    std::cout << std::endl;*/

  EC_KEY *ec_key = nullptr;

  if (!ephemeral)
    ec_key = EC_KEY_dup(m_ecKey);
  else
    ec_key = EC_KEY_dup(m_EPHecKey);

  // Sign the final hash
  ECDSA_SIG *signature = signHash(final_hash, ec_key);
  if (!signature)
  {
    EC_KEY_free(ec_key);
  }

  // Extract r and s from the signature
  const BIGNUM *r;
  const BIGNUM *s;
  ECDSA_SIG_get0(signature, &r, &s);

  // Convert r and s to hex strings
  char *r_hex = BN_bn2hex(r);
  char *s_hex = BN_bn2hex(s);

  if (!r_hex || !s_hex)
  {
    std::cerr << "Error: Failed to convert r or s to hexadecimal." << std::endl;
    ECDSA_SIG_free(signature);
    EC_KEY_free(ec_key);
    return {};
  }

  auto pad_hex_string = [](const char *hex_str) -> std::string
  {
    std::string padded_hex(hex_str);
    if (padded_hex.length() < 64)
    {
      std::stringstream ss;
      ss << std::setfill('0') << std::setw(64) << padded_hex;
      padded_hex = ss.str();
    }
    return padded_hex;
  };

  std::string r_padded_hex = pad_hex_string(r_hex);
  std::string s_padded_hex = pad_hex_string(s_hex);

  signMaterial.r = r_padded_hex;
  signMaterial.s = s_padded_hex;

  // Clean up
  OPENSSL_free(r_hex);
  OPENSSL_free(s_hex);
  ECDSA_SIG_free(signature);
  EC_KEY_free(ec_key);

  return signMaterial;
}

// get the current timestamp in microseconds
uint64_t ECManager::getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());
    uint64_t microseconds_since_epoch = static_cast<uint64_t>(duration.count());

    const uint64_t seconds_per_year = 365 * 24 * 60 * 60;
    const uint64_t leap_seconds = 8 * 24 * 60 * 60;
    const uint64_t epoch_difference_seconds = (34 * seconds_per_year) + leap_seconds;
    const uint64_t epoch_difference = epoch_difference_seconds * 1'000'000ULL;

    const uint64_t offset_microseconds = 5 * 1'000'000ULL;

    return (microseconds_since_epoch - epoch_difference) + offset_microseconds;
}


void ECManager::regeneratePEM() {
    // Create a new EC_KEY object with the specified curve
    EC_KEY* ec_key = EC_KEY_new_by_curve_name(NID_X9_62_prime256v1);
    if (!ec_key) {
        std::cerr << "Error creating EC_KEY object" << std::endl;
        print_openssl_error();
        return;
    }

    // Generate the key pair
    if (EC_KEY_generate_key(ec_key) != 1) {
        std::cerr << "Error generating EC key pair" << std::endl;
        print_openssl_error();
        EC_KEY_free(ec_key);
        return;
    }

    // Save the private key to a PEM file
    FILE* priv_file = fopen("./pkiReqRes/ephSKEY.pem", "w");
    if (!priv_file) {
        std::cerr << "Error opening file for writing private key" << std::endl;
        EC_KEY_free(ec_key);
        return;
    }

    if (PEM_write_ECPrivateKey(priv_file, ec_key, NULL, NULL, 0, NULL, NULL) != 1) {
        std::cerr << "Error writing private key to PEM file" << std::endl;
        print_openssl_error();
        fclose(priv_file);
        EC_KEY_free(ec_key);
        return;
    }

    fclose(priv_file);

    // Save the public key to a PEM file
    FILE* pub_file = fopen("./pkiReqRes/ephPKEY.pem", "w");
    if (!pub_file) {
        std::cerr << "Error opening file for writing public key" << std::endl;
        EC_KEY_free(ec_key);
        return ;
    }

    if (PEM_write_EC_PUBKEY(pub_file, ec_key) != 1) {
        std::cerr << "Error writing public key to PEM file" << std::endl;
        print_openssl_error();
        fclose(pub_file);
        EC_KEY_free(ec_key);
        return ;
    }

    fclose(pub_file);

    // Clean up
    EC_KEY_free(ec_key);

    std::cout << "Key pair generated and saved to ephSKEY.pem and ephPKEY.pem" << std::endl;

}


// get the current timestamp in seconds
uint32_t ECManager::getCurrentTimestamp32() {
    auto now = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
    uint64_t seconds_since_epoch = static_cast<uint64_t>(duration.count());

    const uint64_t seconds_per_year = 365 * 24 * 60 * 60;
    const uint64_t leap_seconds = 8 * 24 * 60 * 60;
    const uint64_t epoch_difference_seconds = (34 * seconds_per_year) + leap_seconds;

    uint64_t tai_seconds_since_2004 = seconds_since_epoch - epoch_difference_seconds;

    return static_cast<uint32_t>(tai_seconds_since_2004);
}

ECManager::iniEC ECManager::readIniFile() {
    INIReader reader("./PKI_info.ini");

    iniEC iniData;

    if (reader.ParseError() < 0) {
        std::cout << "[ERR] Can't load 'PKI_info.ini'\n";
    }

    iniData.eaCert1 = reader.Get("ECinfo", "eaCert1", "UNKNOWN");
    iniData.eaCert2 = reader.Get("ECinfo", "eaCert2", "UNKNOWN");
    iniData.eaCert3 = reader.Get("ECinfo", "eaCert3", "UNKNOWN");
    iniData.pk_rfc = reader.Get("ECinfo", "public_key_rfc", "UNKNOWN");
    iniData.sk_rfc1 = reader.Get("ECinfo", "private_key_rfc1", "UNKNOWN");
    iniData.sk_rfc2 = reader.Get("ECinfo", "private_key_rfc2", "UNKNOWN");
    iniData.itsID = reader.Get("ECinfo", "itsID", "UNKNOWN");
    iniData.recipientID = reader.Get("ECinfo", "recipientID", "UNKNOWN");
    iniData.bitmapSspEA = reader.Get("ECinfo", "bitmapEA", "UNKNOWN");

    /*std::cout << "Config loaded from 'PKI_info.ini': eaCertificate="
              << reader.Get("ECinfo", "eaCertificate", "UNKNOWN") << ", itsID="
              << reader.Get("ECinfo", "itsID", "UNKNOWN") << ", recipientID="
              << reader.Get("ECinfo", "recipientID", "UNKNOWN") << ", bitmapEA="
              << reader.Get("ECinfo", "bitmapEA", "UNKNOWN") <<  std::endl;*/
    return iniData;
}

bool ECManager::isFileNotEmpty() {
    std::string filePath = "./pkiReqRes/responseEC.bin";
    struct stat fileStat;
    if (stat(filePath.c_str(), &fileStat) != 0) {
        return false; // File does not exist
    }
    return fileStat.st_size > 0;
}

void ECManager::createRequest()
{
  //--decode certificate part--
  iniEC iniData = readIniFile();
  std::string EAcertificate = iniData.eaCert1 + iniData.eaCert2 + iniData.eaCert3;
  std::vector<unsigned char> binaryCert = hexStringToBytes(EAcertificate);

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

  // Recover the EC key pair
  public_key = recoverECKeyPair(ephemeral);
  ephemeral = true;
  EPHpublic_key = recoverECKeyPair(ephemeral);


  uint64_t m_generationTime = getCurrentTimestamp();

  // ---------- INNER EC REQUEST  -------------

  auto InnerRequest = asn1cpp::makeSeq(InnerEcRequest);
  std::vector<unsigned char> ITS_S_ID = hexStringToBytes(iniData.itsID);
  std::string itsID(ITS_S_ID.begin(), ITS_S_ID.end());
  asn1cpp::setField(InnerRequest->itsId, itsID);
  asn1cpp::setField(InnerRequest->certificateFormat, m_certFormat);
  // Set the verification key
  asn1cpp::setField(InnerRequest->publicKeys.verificationKey.present, PublicVerificationKey_PR_ecdsaNistP256);
  if (EPHpublic_key.prefix == "compressed_y_0")
  {
    asn1cpp::setField(InnerRequest->publicKeys.verificationKey.choice.ecdsaNistP256.present, EccP256CurvePoint_PR_compressed_y_0);
    std::vector<unsigned char> pk_bytes = hexStringToBytes(EPHpublic_key.pk);
    std::string pk_string(pk_bytes.begin(), pk_bytes.end());
    asn1cpp::setField(InnerRequest->publicKeys.verificationKey.choice.ecdsaNistP256.choice.compressed_y_0, pk_string);
  }
  else if (EPHpublic_key.prefix == "compressed_y_1")
  {
    asn1cpp::setField(InnerRequest->publicKeys.verificationKey.choice.ecdsaNistP256.present, EccP256CurvePoint_PR_compressed_y_1);
    std::vector<unsigned char> pk_bytes = hexStringToBytes(EPHpublic_key.pk);
    std::string pk_string(pk_bytes.begin(), pk_bytes.end());
    asn1cpp::setField(InnerRequest->publicKeys.verificationKey.choice.ecdsaNistP256.choice.compressed_y_1, pk_string);
  }
    
    auto appPermission = asn1cpp::makeSeq (SequenceOfPsidSsp);
    auto psid1 = asn1cpp::makeSeq (PsidSsp);
    asn1cpp::setField (psid1->psid, m_psid);
    auto servicePermission1 = asn1cpp::makeSeq (ServiceSpecificPermissions);
    asn1cpp::setField (servicePermission1->present, ServiceSpecificPermissions_PR_bitmapSsp);
    std::vector<unsigned char> eaSsp = hexStringToBytes (iniData.bitmapSspEA);
    std::string eassp_hex(eaSsp.begin(), eaSsp.end());
    asn1cpp::setField (servicePermission1->choice.bitmapSsp, eassp_hex);
    asn1cpp::setField (psid1->ssp, servicePermission1);
    asn1cpp::sequenceof::pushList (*appPermission, psid1);
    
  asn1cpp::setField (InnerRequest->requestedSubjectAttributes.appPermissions, appPermission); 
  request_result = asn1cpp::oer::encode(InnerRequest);

  // ---------- EtsiTs1030971Data-Signed ----------------
  auto ieeeData = asn1cpp::makeSeq(Ieee1609Dot2Data);
  asn1cpp::setField(ieeeData->protocolVersion, m_version);
  auto contentContainer1 = asn1cpp::makeSeq(Ieee1609Dot2Content);
  asn1cpp::setField(contentContainer1->present, Ieee1609Dot2Content_PR_signedData);
  auto signData = asn1cpp::makeSeq(SignedData);
  asn1cpp::setField(signData->hashId, m_hashId);
  auto tbs = asn1cpp::makeSeq(ToBeSignedData);
  auto signPayload = asn1cpp::makeSeq(SignedDataPayload);
  auto dataPayload2 = asn1cpp::makeSeq(Ieee1609Dot2Data);
  asn1cpp::setField(dataPayload2->protocolVersion, m_version);
  auto dataContentPayload2 = asn1cpp::makeSeq(Ieee1609Dot2Content);
  // ---------- EtsiTs102941Data ----------------
  asn1cpp::setField(dataContentPayload2->present, Ieee1609Dot2Content_PR_unsecuredData);
    auto dataPayload102 = asn1cpp::makeSeq(EtsiTs102941MessagesItss_EtsiTs102941Data);
    asn1cpp::setField(dataPayload102->version, Version_v1);
    auto dataContentPayload102 = asn1cpp::makeSeq(EtsiTs102941MessagesItss_EtsiTs102941DataContent);
    // ---------- EtsiTs103097Data-Signed (InnerECManagerSignedForPOP) ----------------
    asn1cpp::setField(dataContentPayload102->present, EtsiTs102941MessagesItss_EtsiTs102941DataContent_PR_enrolmentRequest);
    
        auto dataPayload =  asn1cpp::makeSeq(Ieee1609Dot2Data);
        asn1cpp::setField(dataPayload->protocolVersion, m_version);
        auto dataContentPayload = asn1cpp::makeSeq(Ieee1609Dot2Content);
        // ---------- EtsiTs103097Data-Signed (InnerECManagerSignedForPOP) ----------------
        asn1cpp::setField(dataContentPayload->present, Ieee1609Dot2Content_PR_signedData);
        auto signDataInner = asn1cpp::makeSeq(SignedData);
        asn1cpp::setField(signDataInner->hashId, m_hashId);
        auto tbsInner = asn1cpp::makeSeq(ToBeSignedData);
        // ---------- Payload InnerECrequest ----------------
        auto signPayloadInner = asn1cpp::makeSeq(SignedDataPayload);
        auto dataPayloadInner = asn1cpp::makeSeq(Ieee1609Dot2Data);
        asn1cpp::setField(dataPayloadInner->protocolVersion, m_version);
        auto dataContentPayloadInner = asn1cpp::makeSeq(Ieee1609Dot2Content);
        asn1cpp::setField(dataContentPayloadInner->present, Ieee1609Dot2Content_PR_unsecuredData);
        asn1cpp::setField(dataContentPayloadInner->choice.unsecuredData, request_result);
        asn1cpp::setField(dataPayloadInner->content, dataContentPayloadInner);
        // ---------- Payload InnerECrequest ----------------
        asn1cpp::setField(signPayloadInner->data, dataPayloadInner);
        asn1cpp::setField(tbsInner->payload, signPayloadInner);
        asn1cpp::setField(tbsInner->headerInfo.psid, m_psid);
        asn1cpp::setField(tbsInner->headerInfo.generationTime, m_generationTime);
        asn1cpp::setField(signDataInner->tbsData, tbsInner);
        std::string tbs_hex = asn1cpp::oer::encode(tbsInner);
        GNsignMaterial sign_material = signatureCreation(tbs_hex, ephemeral);
        asn1cpp::setField(signDataInner->signer.present, SignerIdentifier_PR_self);
        asn1cpp::setField(signDataInner->signer.choice.self, NULL);
        auto signatureContentInner = asn1cpp::makeSeq(Signature);
        asn1cpp::setField(signatureContentInner->present, Signature_PR_ecdsaNistP256Signature);
        asn1cpp::setField(signatureContentInner->choice.ecdsaNistP256Signature.rSig.present, EccP256CurvePoint_PR_x_only);
        std::vector<unsigned char> R_bytes = hexStringToBytes(sign_material.r);
        std::string r_string(R_bytes.begin(), R_bytes.end());
        asn1cpp::setField(signatureContentInner->choice.ecdsaNistP256Signature.rSig.choice.x_only, r_string);
        std::vector<unsigned char> S_bytes = hexStringToBytes(sign_material.s);
        std::string s_string(S_bytes.begin(), S_bytes.end());
        asn1cpp::setField(signatureContentInner->choice.ecdsaNistP256Signature.sSig, s_string);
        asn1cpp::setField(signDataInner->signature, signatureContentInner);
        // ---------- EtsiTs103097Data-Signed (InnerECManagerSignedForPOP) ----------------
        asn1cpp::setField(dataContentPayload->choice.signedData, signDataInner);
        asn1cpp::setField(dataPayload->content, dataContentPayload);

    asn1cpp::setField(dataContentPayload102->choice.enrolmentRequest, dataPayload);
    //asn1cpp::setField(dataContentPayload102->content.choice.enrolmentRequest.content, dataContentPayload102);
  asn1cpp::setField(dataPayload102->content, dataContentPayload102);
  std::string pop_request = asn1cpp::oer::encode(dataPayload102);
  asn1cpp::setField(dataContentPayload2->choice.unsecuredData, pop_request);
  asn1cpp::setField(dataPayload2->content, dataContentPayload2);
  asn1cpp::setField(signPayload->data, dataPayload2);
  asn1cpp::setField(tbs->payload, signPayload);
  asn1cpp::setField(tbs->headerInfo.psid, m_psid);
  asn1cpp::setField(tbs->headerInfo.generationTime, m_generationTime);
  asn1cpp::setField(signData->tbsData, tbs);
  // ---------- EtsiTs102941Data ----------------
  std::string tbs_hexOuter = asn1cpp::oer::encode(tbs);
  ephemeral = false;
  GNsignMaterial sign_materialOuter = signatureCreation(tbs_hexOuter, ephemeral);
  asn1cpp::setField(signData->signer.present, SignerIdentifier_PR_self);
  asn1cpp::setField(signData->signer.choice.self, NULL);
  auto signatureContent = asn1cpp::makeSeq(Signature);
  asn1cpp::setField(signatureContent->present, Signature_PR_ecdsaNistP256Signature);
  asn1cpp::setField(signatureContent->choice.ecdsaNistP256Signature.rSig.present, EccP256CurvePoint_PR_x_only);
  std::vector<unsigned char> R_bytes2 = hexStringToBytes(sign_materialOuter.r);
  std::string r_string2(R_bytes2.begin(), R_bytes2.end());
  asn1cpp::setField(signatureContent->choice.ecdsaNistP256Signature.rSig.choice.x_only, r_string2);
  std::vector<unsigned char> S_bytes2 = hexStringToBytes(sign_materialOuter.s);
  std::string s_string2(S_bytes2.begin(), S_bytes2.end());
  asn1cpp::setField(signatureContent->choice.ecdsaNistP256Signature.sSig, s_string2);
  asn1cpp::setField(signData->signature, signatureContent);
  asn1cpp::setField(contentContainer1->choice.signedData, signData);
  asn1cpp::setField(ieeeData->content, contentContainer1);
  // ---------- EtsiTs1030971Data-Signed ----------------
  
  signedData_result = asn1cpp::oer::encode(ieeeData);


  encData dataEnc = doEncryption(signedData_result, newCert.tbs.encPublicKey, certificate_hash);

  // ---------- DATA ENCRYPTED ENCODING PART ----------------

  auto ieeeData2 = asn1cpp::makeSeq(Ieee1609Dot2Data);
  asn1cpp::setField(ieeeData2->protocolVersion, m_version);
  auto contentContainer = asn1cpp::makeSeq(Ieee1609Dot2Content);
  asn1cpp::setField(contentContainer->present, Ieee1609Dot2Content_PR_encryptedData);


  auto recipientsSeq = asn1cpp::makeSeq(SequenceOfRecipientInfo);
  auto recipInfo = asn1cpp::makeSeq(RecipientInfo);
  asn1cpp::setField(recipInfo->present, RecipientInfo_PR_certRecipInfo);
  std::vector<unsigned char> recipient = hexStringToBytes(iniData.recipientID);
  std::string recID(recipient.begin(), recipient.end());
  asn1cpp::setField(recipInfo->choice.certRecipInfo.recipientId, recID);
  asn1cpp::setField(recipInfo->choice.certRecipInfo.encKey.present, EncryptedDataEncryptionKey_PR_eciesNistP256);

  std::string encKey(dataEnc.encryptedKey.begin(), dataEnc.encryptedKey.end());
  asn1cpp::setField(recipInfo->choice.certRecipInfo.encKey.choice.eciesNistP256.c, encKey);

  std::string eciesTag(dataEnc.eciesTag.begin(), dataEnc.eciesTag.end());
  asn1cpp::setField(recipInfo->choice.certRecipInfo.encKey.choice.eciesNistP256.t, eciesTag);

  asn1cpp::setField(recipInfo->choice.certRecipInfo.encKey.choice.eciesNistP256.v.present, EccP256CurvePoint_PR_uncompressedP256);
  std::string x_value(dataEnc.x_value.begin(), dataEnc.x_value.end());
  std::string y_value(dataEnc.y_value.begin(), dataEnc.y_value.end());
  asn1cpp::setField(recipInfo->choice.certRecipInfo.encKey.choice.eciesNistP256.v.choice.uncompressedP256.x, x_value);
  asn1cpp::setField(recipInfo->choice.certRecipInfo.encKey.choice.eciesNistP256.v.choice.uncompressedP256.y, y_value);
  asn1cpp::sequenceof::pushList(*recipientsSeq, recipInfo);
  asn1cpp::setField(contentContainer->choice.encryptedData.recipients, recipientsSeq);

  // ciphertext part, put value that came from the encoding part
  asn1cpp::setField(contentContainer->choice.encryptedData.ciphertext.present, SymmetricCiphertext_PR_aes128ccm);
  std::string nonce(dataEnc.nonce.begin(), dataEnc.nonce.end());
  asn1cpp::setField(contentContainer->choice.encryptedData.ciphertext.choice.aes128ccm.nonce, nonce);
  std::string ciphertextWithTag(dataEnc.ciphertextWithTag.begin(), dataEnc.ciphertextWithTag.end());
  asn1cpp::setField(contentContainer->choice.encryptedData.ciphertext.choice.aes128ccm.ccmCiphertext, ciphertextWithTag);

  asn1cpp::setField(ieeeData2->content, contentContainer);

  encode_result = asn1cpp::oer::encode(ieeeData2);

  // Saving the binary file for the request
  std::ofstream binaryFile("requestEC.bin", std::ios::binary);
  if (binaryFile.is_open())
  {
    binaryFile.write(reinterpret_cast<const char *>(encode_result.data()), encode_result.size());
    binaryFile.close();
    //std::cout << "[INFO] Request saved as requestEC.bin" << std::endl;
  }
  else
  {
    std::cerr << "[ERR] Failed to save binary file" << std::endl;
  }

  // Calculating request ID (16-byte SHA256 hash of the payload)
  
  std::vector<unsigned char> reqID(encode_result.begin(), encode_result.end());
  unsigned char hash[SHA256_DIGEST_LENGTH];
  computeSHA256(reqID, hash);

  // Outputting the first 16 bytes of the SHA256 hash as the request ID
  std::cout << "[INFO] Request ID: ";
  for (int i = 0; i < 16; ++i)
  {
    printf("%02x", hash[i]);
  }
  std::cout << std::endl;

}

void ECManager::sendPOST() {
    try {
        // File path
        const std::string filePath = "./requestEC.bin";
        const std::string responseFilePath = "./pkiReqRes/responseEC.bin";


        // Read the file
        std::ifstream file(filePath, std::ios::binary);
        if (!file) {
            throw std::runtime_error("Error opening file: " + filePath);
        }

        // Check the file size
        file.seekg(0, std::ios::end);
        const size_t fileSize = file.tellg();
        file.seekg(0, std::ios::beg);

        // Read the file into a buffer
        std::vector<char> fileBuffer(fileSize);
        file.read(fileBuffer.data(), fileSize);
        file.close();

        // Convert the buffer to a string
        const std::string body(fileBuffer.begin(), fileBuffer.end());

        // Send the POST request
        http::Request request{"http://0.atos-ea.l0.c-its-pki.eu/"};
        const auto response = request.send(
                "POST",
                body, // Body
                {{"Content-Type", "application/x-its-request"}} // Header fields
        );


        // Save the response to a file
        std::ofstream responseFile(responseFilePath, std::ios::binary);
        if (!responseFile) {
            throw std::runtime_error("Error opening response file: " + responseFilePath);
        }
        responseFile.write(reinterpret_cast<const char*>(response.body.data()), response.body.size());
        responseFile.close();

        // Print the response file path
        //std::cout << "Response saved in: " << responseFilePath << '\n';

    } catch (const std::exception& e) {
        std::cerr << "Request failed, error: " << e.what() << '\n';
    }
}


bool ECManager::manageRequest() {

    if (isFileNotEmpty()) {
        EC = ecRes.getECResponse();
        if (EC.version == 0 && EC.issuer.empty()) {
            return false;
        }

        // conversion of duration in years to seconds
        if (EC.tbs.validityPeriod_start <= getCurrentTimestamp32() &&
            getCurrentTimestamp32() <= EC.tbs.validityPeriod_start + EC.tbs.validityPeriod_duration * 31557600) {
            std::cout << "[INFO] EC is valid" << std::endl;
            return true;
        } else {
            std::cout << "[INFO] EC is not valid" << std::endl;
            regeneratePEM();
            createRequest();
            sendPOST();
            EC = ecRes.getECResponse();
            if (EC.version == 0 && EC.issuer.empty()) {
                std::cerr << "[ERR] Error - EC response is empty, or response code is 0" << std::endl;
                return false;
            }
            return true;
        }
    } else {
        std::cout << "[INFO] File EC is empty" << std::endl;
        regeneratePEM();
        createRequest();
        sendPOST();
        EC = ecRes.getECResponse();
        if (EC.version == 0 && EC.issuer.empty()) {
            std::cerr << "[ERR] Error - EC response is empty, or response code is 0" << std::endl;
            return false;
        }
        return true;
    }
}

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif