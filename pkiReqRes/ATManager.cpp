// PKI Interaction - Authorization Ticket request.
// This file contains the implementation of the Authorization Ticket request.
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
#include <fstream>
#include <ATManager.h>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <sys/stat.h>
#include <HTTPRequest.hpp>
#include <INIReader.h>



extern "C" {
    #include "Ieee1609Dot2Data.h"
    #include "SharedAtRequest.h"
    #include "InnerAtRequest.h"
    #include "EtsiTs102941MessagesItss_EtsiTs102941Data.h"
    #include "EcSignature.h"
    
}

#define AES_KEY_LENGTH 16
#define NONCE_LENGTH 12
#define AES_CCM_TAG_LENGTH 16
#define HMAC_TAG_LENGTH 32


ATManager::~ATManager ()
{

}

ATManager::ATManager (std::atomic<bool> *terminatorFlagPtr)
{
    m_terminatorFlagPtr = terminatorFlagPtr;
    ephemeral = false;
    m_ecKey = nullptr;
    m_EPHecKey = nullptr;
    request_result = "";
    signedData_result = "";
    encode_result = "";
    m_protocolversion = 3;
    m_recipientID = "A8AA9CAB63B783EE"; // 8 bytes contraint, hashid8 of AA certificate
    m_hashId = HashAlgorithm_sha256;
    m_psid = 623;
    m_certFormat = 1;
    m_eaId = "D41845A1F71C356A"; // 8 bytes contraint, hashid8 of EA certificate
    m_hours = 168; // 7 days
    m_CAM = 36;
    m_DENM = 37;
    m_CPM = 639;
    m_bitmapSspCAM = "01FFFC";
    m_bitmapSspDENM = "01FFFFFF";
    m_bitmapSspCPM = "01";

}





std::string ATManager::retrieveStringFromFile(const std::string& fileName) {
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

void ATManager::saveStringToFile(const std::string& key, const std::string& fileName) {
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


// Function to generate a random HMAC key
std::vector<unsigned char> ATManager::generateHMACKey(size_t length) {
    std::vector<unsigned char> hmacKey(length);
    if (RAND_bytes(hmacKey.data(), length) != 1) {
        throw std::runtime_error("Errore durante la generazione della chiave HMAC");
    }
    return hmacKey;
}

// Function to compute the HMAC tag
std::vector<unsigned char> ATManager::computeHMACTag(const std::vector<unsigned char>& hmacKey, const std::vector<unsigned char>& verificationKey) {
      
   
    unsigned char hmacTag[EVP_MAX_MD_SIZE];
    unsigned int hmacLen;

    HMAC_CTX* hmacCtx = HMAC_CTX_new();
    if (!hmacCtx) throw std::runtime_error("Errore durante l'inizializzazione del contesto HMAC");
    
    if (HMAC_Init_ex(hmacCtx, hmacKey.data(), hmacKey.size(), EVP_sha256(), nullptr) != 1) {
        throw std::runtime_error("Errore durante l'inizializzazione di HMAC");
    }    
    if (HMAC_Update(hmacCtx, verificationKey.data(), verificationKey.size()) != 1) {
        throw std::runtime_error("Errore durante l'aggiornamento di HMAC");
    }    
    if (HMAC_Final(hmacCtx, hmacTag, &hmacLen) != 1) {
        throw std::runtime_error("Errore durante la finalizzazione di HMAC");
    }    
    HMAC_CTX_free(hmacCtx);
    
   
    std::vector<unsigned char> keyTag(hmacTag, hmacTag + 16);
    return keyTag;
}

// Function to convert a hexadecimal string to a byte array
std::vector<unsigned char> ATManager::hexStringToBytes(const std::string &hex)
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


// Function to handle OpenSSL errors
void ATManager::handleErrors()
{
  ERR_print_errors_fp(stderr);
  abort();
}

// Function to convert a byte array to a hexadecimal string
std::string ATManager::to_hex_string(const unsigned char *data, size_t length)
{
  std::ostringstream oss;
  for (size_t i = 0; i < length; ++i)
  {
    oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(data[i]);
  }
  return oss.str();
}

std::string ATManager::toHexString(const std::string& input) {
    std::ostringstream oss;
    for (unsigned char byte : input) {
        oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
    }
    return oss.str();
}

// Function to convert a byte vector to a hexadecimal string
std::string ATManager::to_hex_string(const std::vector<unsigned char> &data)
{
  std::ostringstream oss;
  for (unsigned char byte : data)
  {
    oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
  }
  return oss.str();
}

// Function to load a compressed public key
EVP_PKEY* ATManager::loadCompressedPublicKey(const std::string &compressedKey, int compression)
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
void ATManager::computeSHA256(const std::vector<unsigned char> &data,
                   unsigned char hash[SHA256_DIGEST_LENGTH])
{
  SHA256_CTX sha256;
  SHA256_Init(&sha256);
  SHA256_Update(&sha256, data.data(), data.size());
  SHA256_Final(hash, &sha256);
}

// Function to concatenate two byte arrays
std::vector<unsigned char> ATManager::concatenateHashes(const unsigned char hash1[SHA256_DIGEST_LENGTH], const unsigned char hash2[SHA256_DIGEST_LENGTH]) {
    std::vector<unsigned char> concatenatedHashes;
    concatenatedHashes.insert(concatenatedHashes.end(), hash1, hash1 + SHA256_DIGEST_LENGTH);
    concatenatedHashes.insert(concatenatedHashes.end(), hash2, hash2 + SHA256_DIGEST_LENGTH);
    return concatenatedHashes;
}

// Function to derive a key using KDF2
void ATManager::deriveKeyWithKDF2(const unsigned char* sharedSecret, size_t secretLen,
                       const unsigned char* P1, size_t P1_len,
                       unsigned char* derivedKey, size_t derivedKeyLen) {
    size_t hBits = SHA256_DIGEST_LENGTH * 8; // SHA-256 produces 256 bits (32 bytes)
    size_t cThreshold = (derivedKeyLen * 8 + hBits - 1) / hBits;  // Quante iterazioni necessarie

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


// Function to encrypt a message using ECIES
std::vector<unsigned char> ATManager::encryptMessage(
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
    atRes.setAesKey(to_hex_string(aesKey,16));
    std::string fileName = "pskAT.bin";
    saveStringToFile(to_hex_string(aesKey,16), fileName);

    nonce.resize(NONCE_LENGTH);
    if (RAND_bytes(nonce.data(), NONCE_LENGTH) != 1) {
        handleErrors();
    }

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

// Function to encrypt a message using ECIES
ATManager::encData ATManager::doEncryption(std::string message, GNecdsaNistP256 encPkEA, const unsigned char *p1)
{

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

  std::vector<unsigned char> ciphertext = encryptMessage(plaintext, receiverPublicKey, data.encryptedKey, data.ephemeralPublicKey, data.eciesTag, data.nonce, aesCcmTag, p1);

  data.ephemeralPublicKey.erase(data.ephemeralPublicKey.begin());

  data.x_value.insert(data.x_value.end(), data.ephemeralPublicKey.begin(), data.ephemeralPublicKey.begin() + 32);
  data.y_value.insert(data.y_value.end(), data.ephemeralPublicKey.begin() + 32, data.ephemeralPublicKey.end());

  data.ciphertextWithTag.insert(data.ciphertextWithTag.end(), ciphertext.begin(), ciphertext.end());
  data.ciphertextWithTag.insert(data.ciphertextWithTag.end(), aesCcmTag.begin(), aesCcmTag.end());

  EVP_PKEY_free(receiverPublicKey);

  return data;
}

// Function to print an OpenSSL error
void ATManager::print_openssl_error()
{
  char buffer[120];
  unsigned long error = ERR_get_error();
  ERR_error_string_n(error, buffer, sizeof(buffer));
  std::cerr << buffer << std::endl;
}

// Function to load an EC key pair from a file
EC_KEY* ATManager::loadECKeyFromFile(const std::string &private_key_file, const std::string &public_key_file)
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

// Function to recover an EC key pair from a file
ATManager::GNpublicKey ATManager::recoverECKeyPair(bool ephemeral)
{
  std::string private_key_file = "";
  std::string public_key_file = "";
  EC_KEY *ec_key = nullptr;
  if (ephemeral)
  {
    private_key_file = "./pkiReqRes/ephSKEY2.pem";
    public_key_file = "./pkiReqRes/ephPKEY2.pem";
    ec_key = loadECKeyFromFile(private_key_file, public_key_file);
    if (!ec_key)
    {
      return {};
    }
  }
  else
  {
    private_key_file = "./pkiReqRes/ephSKEY.pem";
    public_key_file = "./pkiReqRes/ephPKEY.pem";
    ec_key = loadECKeyFromFile(private_key_file, public_key_file);
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
ECDSA_SIG* ATManager::signHash(const unsigned char *hash, EC_KEY *ec_key)
{
  ECDSA_SIG *signature = ECDSA_do_sign(hash, SHA256_DIGEST_LENGTH, ec_key);
  if (!signature)
  {
    std::cerr << "Error signing hash" << std::endl;
    print_openssl_error();
  }
  return signature;
}

// Function to create a signature
ATManager::GNsignMaterial ATManager::signatureCreation(const std::string &tbsData_hex, bool ephemeral, const std::string &signer_hex )
{

  GNsignMaterial signMaterial;

  std::vector<unsigned char> tbsData_bytes(tbsData_hex.begin(), tbsData_hex.end());
  
  std::vector<unsigned char> signIDbytes = hexStringToBytes(signer_hex);

  unsigned char tbsData_hash[SHA256_DIGEST_LENGTH];
  computeSHA256(tbsData_bytes, tbsData_hash);


  unsigned char signIDself_hash[SHA256_DIGEST_LENGTH];
  computeSHA256(signIDbytes, signIDself_hash);


  std::vector<unsigned char> concatenatedHashes = concatenateHashes(tbsData_hash, signIDself_hash);

  unsigned char final_hash[SHA256_DIGEST_LENGTH];
  computeSHA256(concatenatedHashes, final_hash);


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
uint64_t ATManager::getCurrentTimestamp() {
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

// Function to get the current timestamp in seconds
uint32_t ATManager::getCurrentTimestamp32() {
    auto now = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
    uint64_t seconds_since_epoch = static_cast<uint64_t>(duration.count());

    const uint64_t seconds_per_year = 365 * 24 * 60 * 60;
    const uint64_t leap_seconds = 8 * 24 * 60 * 60;
    const uint64_t epoch_difference_seconds = (34 * seconds_per_year) + leap_seconds;

    uint64_t tai_seconds_since_2004 = seconds_since_epoch - epoch_difference_seconds;

    return static_cast<uint32_t>(tai_seconds_since_2004);
}

ATManager::iniAT ATManager::readIniFile() {
    INIReader reader("./PKI_info.ini");

    iniAT iniData;

    if (reader.ParseError() < 0) {
        std::cout << "[ERR] Can't load 'PKI_info.ini'\n";
    }

    iniData.aaCert1 = reader.Get("ATinfo", "AAcert1", "UNKNOWN");
    iniData.aaCert2 = reader.Get("ATinfo", "AAcert2", "UNKNOWN");
    iniData.aaCert3 = reader.Get("ATinfo", "AAcert3", "UNKNOWN");
    iniData.aaCert4 = reader.Get("ATinfo", "AAcert4", "UNKNOWN");
    iniData.recipientAA = reader.Get("ATinfo", "recipientAA", "UNKNOWN");
    iniData.bitmapCAM = reader.Get("ATinfo", "bitmapCAM", "UNKNOWN");
    iniData.bitmapDENM = reader.Get("ATinfo", "bitmapDENM", "UNKNOWN");
    iniData.eaIDstring = reader.Get("ATinfo", "eaIDstring", "UNKNOWN");

    /*std::cout << "Config loaded from 'PKI_info.ini': recipientAA="
              << reader.Get("ATinfo", "recipientAA", "UNKNOWN") << ", AAcertificate="
              << reader.Get("ATinfo", "AAcertificate", "UNKNOWN") << ", bitmapCAM="
              << reader.Get("ATinfo", "bitmapCAM", "UNKNOWN") << ", bitmapDENM="
              << reader.Get("ATinfo", "bitmapDENM", "UNKNOWN") << ", eaIDstring="
              << reader.Get("ATinfo", "eaIDstring", "UNKNOWN") <<  std::endl;*/
    return iniData;
}

void ATManager::regeneratePEM() {
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
    FILE* priv_file = fopen("./pkiReqRes/ephSKEY2.pem", "w");
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
    FILE* pub_file = fopen("./pkiReqRes/ephPKEY2.pem", "w");
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

    std::cout << "Key pair generated and saved to ephSKEY2.pem and ephPKEY2.pem" << std::endl;

}

void ATManager::createRequest () {

    if(m_ECHex=="" || m_ECHex.empty()) {
        std::cerr << "[ERROR] Critical error: m_ECHex is empty. Cannot create any request.\n";
        m_terminatorFlagPtr = true;
        return;
    }

    iniAT iniData = readIniFile();

//--decode certificate part--
    std::string AAcertificate = iniData.aaCert1 + iniData.aaCert2 + iniData.aaCert3 + iniData.aaCert4;
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
  auto present = asn1cpp::getField(signcertData_decoded->present, Signature_PR);
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
  ephemeral = true;
  EPHpublic_key = recoverECKeyPair(ephemeral);
  ephemeral = false;
  public_key = recoverECKeyPair(ephemeral);

  uint64_t m_generationTime = getCurrentTimestamp();


// ---------- EtsiTs1030971Data-Signed ----------------
  auto ieeeData = asn1cpp::makeSeq(Ieee1609Dot2Data);
  asn1cpp::setField(ieeeData->protocolVersion, m_protocolversion);
  auto contentContainer1 = asn1cpp::makeSeq(Ieee1609Dot2Content);
  asn1cpp::setField(contentContainer1->present, Ieee1609Dot2Content_PR_signedData);
  auto signData = asn1cpp::makeSeq(SignedData);
  asn1cpp::setField(signData->hashId, m_hashId);
  auto tbs = asn1cpp::makeSeq(ToBeSignedData);
  auto signPayload = asn1cpp::makeSeq(SignedDataPayload);
  auto dataPayload2 = asn1cpp::makeSeq(Ieee1609Dot2Data);
  asn1cpp::setField(dataPayload2->protocolVersion, m_protocolversion);
  auto dataContentPayload2 = asn1cpp::makeSeq(Ieee1609Dot2Content);
  // ---------- EtsiTs102941Data ----------------
  asn1cpp::setField(dataContentPayload2->present, Ieee1609Dot2Content_PR_unsecuredData);
    auto dataPayload102 = asn1cpp::makeSeq(EtsiTs102941MessagesItss_EtsiTs102941Data);
    asn1cpp::setField(dataPayload102->version, Version_v1);
    auto dataContentPayload102 = asn1cpp::makeSeq(EtsiTs102941MessagesItss_EtsiTs102941DataContent);
    // ---------- EtsiTs103097Data-Signed ----------------
    asn1cpp::setField(dataContentPayload102->present, EtsiTs102941MessagesItss_EtsiTs102941DataContent_PR_authorizationRequest);
    
    // ---------- AT request-------------
    auto InnerRequest = asn1cpp::makeSeq (InnerAtRequest);
    asn1cpp::setField (InnerRequest->publicKeys.verificationKey.present, PublicVerificationKey_PR_ecdsaNistP256);
    if (EPHpublic_key.prefix == "compressed_y_0")
            {
              asn1cpp::setField (InnerRequest->publicKeys.verificationKey.choice.ecdsaNistP256.present, EccP256CurvePoint_PR_compressed_y_0);
              std::vector<unsigned char> pk_bytes = hexStringToBytes (EPHpublic_key.pk);
              std::string pk_string(pk_bytes.begin(), pk_bytes.end());
              asn1cpp::setField (InnerRequest->publicKeys.verificationKey.choice.ecdsaNistP256.choice.compressed_y_0, pk_string);
            }
    else if (EPHpublic_key.prefix == "compressed_y_1")
            {
              asn1cpp::setField (InnerRequest->publicKeys.verificationKey.choice.ecdsaNistP256.present, EccP256CurvePoint_PR_compressed_y_1);
              std::vector<unsigned char> pk_bytes = hexStringToBytes (EPHpublic_key.pk);
              std::string pk_string(pk_bytes.begin(), pk_bytes.end());
              asn1cpp::setField (InnerRequest->publicKeys.verificationKey.choice.ecdsaNistP256.choice.compressed_y_1, pk_string);
            }
    std::vector<unsigned char> hmacKey = generateHMACKey();
    std::string m_hmacKey(hmacKey.begin(), hmacKey.end());
      auto verificationKey = asn1cpp::makeSeq (PublicVerificationKey);
      asn1cpp::setField (verificationKey->present, PublicVerificationKey_PR_ecdsaNistP256);
      if (EPHpublic_key.prefix == "compressed_y_0")
            {
              asn1cpp::setField (verificationKey->choice.ecdsaNistP256.present, EccP256CurvePoint_PR_compressed_y_0);
              std::vector<unsigned char> pk_bytes = hexStringToBytes (EPHpublic_key.pk);
              std::string pk_string(pk_bytes.begin(), pk_bytes.end());
              asn1cpp::setField (verificationKey->choice.ecdsaNistP256.choice.compressed_y_0, pk_string);
            }
      else if (EPHpublic_key.prefix == "compressed_y_1")
            {
              asn1cpp::setField (verificationKey->choice.ecdsaNistP256.present, EccP256CurvePoint_PR_compressed_y_1);
              std::vector<unsigned char> pk_bytes = hexStringToBytes (EPHpublic_key.pk);
              std::string pk_string(pk_bytes.begin(), pk_bytes.end());
              asn1cpp::setField (verificationKey->choice.ecdsaNistP256.choice.compressed_y_1, pk_string);
            }
      
      std::string verkey = asn1cpp::oer::encode(verificationKey);
      std::vector<unsigned char> ver_key(verkey.begin(), verkey.end());

    std::vector<unsigned char> keyTag = computeHMACTag(hmacKey, ver_key);

    std::string m_keyTag(keyTag.begin(), keyTag.end());
    asn1cpp::setField (InnerRequest->hmacKey, m_hmacKey ); 
      std::vector<unsigned char> ea_id = hexStringToBytes(iniData.eaIDstring);
      std::string ea_hex(ea_id.begin(), ea_id.end());
      asn1cpp::setField (InnerRequest->sharedAtRequest.eaId, ea_hex ); 
      asn1cpp::setField (InnerRequest->sharedAtRequest.keyTag, m_keyTag ); 
      asn1cpp::setField (InnerRequest->sharedAtRequest.certificateFormat, m_certFormat );
        auto validity = asn1cpp::makeSeq (ValidityPeriod);
        uint32_t m_start = getCurrentTimestamp32();
        asn1cpp::setField (validity->start, m_start);
        asn1cpp::setField (validity->duration.present, Duration_PR_hours);
        asn1cpp::setField (validity->duration.choice.hours, m_hours);
        asn1cpp::setField (InnerRequest->sharedAtRequest.requestedSubjectAttributes.validityPeriod, validity); 
        auto appPermission = asn1cpp::makeSeq (SequenceOfPsidSsp);
        auto psid1 = asn1cpp::makeSeq (PsidSsp);
        asn1cpp::setField (psid1->psid, m_CAM);
        auto servicePermission1 = asn1cpp::makeSeq (ServiceSpecificPermissions);
        asn1cpp::setField (servicePermission1->present, ServiceSpecificPermissions_PR_bitmapSsp);
        std::vector<unsigned char> camSsp = hexStringToBytes (iniData.bitmapCAM);
        std::string cam_hex(camSsp.begin(), camSsp.end());
        asn1cpp::setField (servicePermission1->choice.bitmapSsp, cam_hex);
        asn1cpp::setField (psid1->ssp, servicePermission1);
        asn1cpp::sequenceof::pushList (*appPermission, psid1);
        auto psid2 = asn1cpp::makeSeq (PsidSsp);
        asn1cpp::setField (psid2->psid, m_DENM);
        auto servicePermission2 = asn1cpp::makeSeq (ServiceSpecificPermissions);
        asn1cpp::setField (servicePermission2->present, ServiceSpecificPermissions_PR_bitmapSsp);
        std::vector<unsigned char> denmSsp = hexStringToBytes (iniData.bitmapDENM);
        std::string denm_hex(denmSsp.begin(), denmSsp.end());
        asn1cpp::setField (servicePermission2->choice.bitmapSsp, denm_hex);
        asn1cpp::setField (psid2->ssp, servicePermission2);
        asn1cpp::sequenceof::pushList (*appPermission, psid2);
        /*auto psid3 = asn1cpp::makeSeq (PsidSsp);
        asn1cpp::setField (psid3->psid, m_CPM);
        auto servicePermission3 = asn1cpp::makeSeq (ServiceSpecificPermissions);
        asn1cpp::setField (servicePermission3->present, ServiceSpecificPermissions_PR_bitmapSsp);
        std::vector<unsigned char> cpmSsp = hexStringToBytes (m_bitmapSspCPM);
        std::string cpm_hex(cpmSsp.begin(), cpmSsp.end());
        asn1cpp::setField (servicePermission3->choice.bitmapSsp, cpm_hex);
        asn1cpp::setField (psid3->ssp, servicePermission3);
        asn1cpp::sequenceof::pushList (*appPermission, psid3);*/
      asn1cpp::setField (InnerRequest->sharedAtRequest.requestedSubjectAttributes.appPermissions, appPermission); 
    auto sharedAT = asn1cpp::makeSeq (SharedAtRequest);
    asn1cpp::setField (sharedAT->eaId, ea_hex ); 
    asn1cpp::setField (sharedAT->keyTag, m_keyTag ); 
    asn1cpp::setField (sharedAT->certificateFormat, m_certFormat );
    asn1cpp::setField (sharedAT->requestedSubjectAttributes.validityPeriod, validity );
    asn1cpp::setField (sharedAT->requestedSubjectAttributes.appPermissions, appPermission );
    std::string shared_hex = asn1cpp::oer::encode (sharedAT); 
    std::vector<unsigned char> shared_bytes(shared_hex.begin(), shared_hex.end());
    //std::vector<unsigned char> shared_bytes = hexStringToBytes(shared_hex);
    unsigned char sharedAtRequest_hash[SHA256_DIGEST_LENGTH];
    computeSHA256(shared_bytes, sharedAtRequest_hash);
    std::string m_sharedAT(sharedAtRequest_hash, sharedAtRequest_hash + SHA256_DIGEST_LENGTH);
    asn1cpp::setField (InnerRequest->ecSignature.present, EcSignature_PR_ecSignature);
    asn1cpp::setField (InnerRequest->ecSignature.choice.ecSignature.protocolVersion, m_protocolversion);
    auto contentInner = asn1cpp::makeSeq (Ieee1609Dot2Content);
    asn1cpp::setField (contentInner->present, Ieee1609Dot2Content_PR_signedData);
    auto signData3 = asn1cpp::makeSeq (SignedData);
    asn1cpp::setField (signData3->hashId, m_hashId);
    auto tbs3 = asn1cpp::makeSeq (ToBeSignedData);
    auto signPayload3 = asn1cpp::makeSeq (SignedDataPayload);
    auto hashData = asn1cpp::makeSeq (HashedData);
    asn1cpp::setField (hashData->present, HashedData_PR_sha256HashedData);
    asn1cpp::setField (hashData->choice.sha256HashedData, m_sharedAT);  
    asn1cpp::setField (signPayload3->extDataHash, hashData);
    asn1cpp::setField (tbs3->payload, signPayload3);
    asn1cpp::setField (tbs3->headerInfo.psid, m_psid);
    asn1cpp::setField (tbs3->headerInfo.generationTime, m_generationTime); 
    asn1cpp::setField (signData3->tbsData, tbs3);
    std::string tbs_hex = asn1cpp::oer::encode (tbs3);
    std::string ec_hex = m_ECHex;
    GNsignMaterial sign_material = signatureCreation (tbs_hex, ephemeral, ec_hex);
    // generate the digest of the certificate, so calculate the hash of certificate and take only the last 8 bytes
    std::vector<unsigned char> digest_bytes = hexStringToBytes (ec_hex);
    unsigned char digest_hash[SHA256_DIGEST_LENGTH];
    computeSHA256 (digest_bytes, digest_hash);
    std::string ec_h8 = "";
    for (int i = 24; i < 32; i++)
    {
        std::stringstream ss;
        ss << std::hex << std::setw(2) << std::setfill('0') << (int) digest_hash[i];
        ec_h8 += ss.str();
    }
    std::vector<unsigned char> ec_bytes = hexStringToBytes (ec_h8);
    std::string m_digest(ec_bytes.begin(), ec_bytes.end());
    asn1cpp::setField (signData3->signer.present, SignerIdentifier_PR_digest);
    asn1cpp::setField (signData3->signer.choice.digest, m_digest);
    auto signatureContentInner = asn1cpp::makeSeq (Signature);
    asn1cpp::setField (signatureContentInner->present, Signature_PR_ecdsaNistP256Signature);
    asn1cpp::setField (signatureContentInner->choice.ecdsaNistP256Signature.rSig.present, EccP256CurvePoint_PR_x_only);      
    std::vector<unsigned char> R_bytes = hexStringToBytes (sign_material.r);
    std::string r_string(R_bytes.begin(), R_bytes.end());
    asn1cpp::setField (signatureContentInner->choice.ecdsaNistP256Signature.rSig.choice.x_only, r_string);
    std::vector<unsigned char> S_bytes = hexStringToBytes (sign_material.s);
    std::string s_string(S_bytes.begin(), S_bytes.end());
    asn1cpp::setField (signatureContentInner->choice.ecdsaNistP256Signature.sSig, s_string);
    asn1cpp::setField (signData3->signature, signatureContentInner);
    asn1cpp::setField (contentInner->choice.signedData, signData3);
    asn1cpp::setField (InnerRequest->ecSignature.choice.ecSignature.content, contentInner );

  asn1cpp::setField(dataContentPayload102->choice.authorizationRequest, InnerRequest);
  asn1cpp::setField(dataPayload102->content, dataContentPayload102);
  std::string at_request = asn1cpp::oer::encode(dataPayload102);
  asn1cpp::setField(dataContentPayload2->choice.unsecuredData, at_request);
  asn1cpp::setField(dataPayload2->content, dataContentPayload2);
  asn1cpp::setField(signPayload->data, dataPayload2);
  asn1cpp::setField(tbs->payload, signPayload);
  asn1cpp::setField(tbs->headerInfo.psid, m_psid);
  asn1cpp::setField(tbs->headerInfo.generationTime, m_generationTime);
  asn1cpp::setField(signData->tbsData, tbs);
  // ---------- EtsiTs102941Data ----------------
  std::string tbs_hexOuter = asn1cpp::oer::encode(tbs);
  ephemeral = true;
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

// ---------- DATA ENCRYPTED ENCODING PART ----------------

encData dataEnc = doEncryption(signedData_result, newCert.tbs.encPublicKey, certificate_hash);

auto ieeeData2 = asn1cpp::makeSeq (Ieee1609Dot2Data);
asn1cpp::setField (ieeeData2->protocolVersion, m_protocolversion);
auto contentContainer = asn1cpp::makeSeq (Ieee1609Dot2Content);
asn1cpp::setField (contentContainer->present, Ieee1609Dot2Content_PR_encryptedData);

//recipient parts with sequence of, put many info.
auto recipientsSeq = asn1cpp::makeSeq (SequenceOfRecipientInfo);
auto recipInfo = asn1cpp::makeSeq (RecipientInfo);
asn1cpp::setField (recipInfo->present, RecipientInfo_PR_certRecipInfo );
std::vector<unsigned char> recipient = hexStringToBytes(iniData.recipientAA);
std::string recID(recipient.begin(), recipient.end());
asn1cpp::setField (recipInfo->choice.certRecipInfo.recipientId, recID );
asn1cpp::setField (recipInfo->choice.certRecipInfo.encKey.present, EncryptedDataEncryptionKey_PR_eciesNistP256 );

std::string encKey(dataEnc.encryptedKey.begin(), dataEnc.encryptedKey.end());
asn1cpp::setField (recipInfo->choice.certRecipInfo.encKey.choice.eciesNistP256.c, encKey);

std::string eciesTag(dataEnc.eciesTag.begin(), dataEnc.eciesTag.end());
asn1cpp::setField (recipInfo->choice.certRecipInfo.encKey.choice.eciesNistP256.t, eciesTag);

asn1cpp::setField (recipInfo->choice.certRecipInfo.encKey.choice.eciesNistP256.v.present, EccP256CurvePoint_PR_uncompressedP256 );
std::string x_value(dataEnc.x_value.begin(), dataEnc.x_value.end());
std::string y_value(dataEnc.y_value.begin(), dataEnc.y_value.end());
asn1cpp::setField (recipInfo->choice.certRecipInfo.encKey.choice.eciesNistP256.v.choice.uncompressedP256.x, x_value );
asn1cpp::setField (recipInfo->choice.certRecipInfo.encKey.choice.eciesNistP256.v.choice.uncompressedP256.y, y_value );
asn1cpp::sequenceof::pushList (*recipientsSeq, recipInfo);
asn1cpp::setField (contentContainer->choice.encryptedData.recipients, recipientsSeq);

//ciphertext part, put value that came from the encoding part
asn1cpp::setField (contentContainer->choice.encryptedData.ciphertext.present,SymmetricCiphertext_PR_aes128ccm );
std::string nonce(dataEnc.nonce.begin(), dataEnc.nonce.end());
asn1cpp::setField (contentContainer->choice.encryptedData.ciphertext.choice.aes128ccm.nonce, nonce );
std::string ciphertextWithTag(dataEnc.ciphertextWithTag.begin(), dataEnc.ciphertextWithTag.end());
asn1cpp::setField (contentContainer->choice.encryptedData.ciphertext.choice.aes128ccm.ccmCiphertext, ciphertextWithTag );

asn1cpp::setField (ieeeData2->content, contentContainer);

encode_result = asn1cpp::oer::encode(ieeeData2);

  // Saving the binary file for the request
  std::ofstream binaryFile("requestAA.bin", std::ios::binary);
  if (binaryFile.is_open())
  {
    binaryFile.write(reinterpret_cast<const char *>(encode_result.data()), encode_result.size());
    binaryFile.close();
    //std::cout << "[INFO] Request saved as requestAA.bin" << std::endl;
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
  std::cout << "[INFO] AA Request ID: ";
  for (int i = 0; i < 16; ++i)
  {
    printf("%02x", hash[i]);
  }
  std::cout << std::endl;

}

bool ATManager::isFileNotEmpty() {
    std::string filePath = "./pkiReqRes/responseAT.bin";
    struct stat fileStat;
    if (stat(filePath.c_str(), &fileStat) != 0) {
        return false; // File does not exist
    }
    return fileStat.st_size > 0;
}

void ATManager::sendPOST() {
        try {
            // File path
            const std::string filePath = "./requestAA.bin";
            const std::string responseFilePath = "./pkiReqRes/responseAT.bin";


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
            http::Request request{"http://0.atos-aa.l0.c-its-pki.eu/"};
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

void ATManager::updateAT() {

    m_auhtTicket.m_version = atResCert.version;
    m_auhtTicket.m_type = atResCert.type;

    m_auhtTicket.m_issuer = atResCert.issuer;

    m_auhtTicket.m_id_none = atResCert.tbs.id;
    m_auhtTicket.m_cracaId = atResCert.tbs.cracaId;
    m_auhtTicket.m_crlSeries = atResCert.tbs.crlSeries;
    m_auhtTicket.m_validityPeriod_start = atResCert.tbs.validityPeriod_start;
    m_auhtTicket.m_validityPeriod_duration = atResCert.tbs.validityPeriod_duration;
    m_auhtTicket.m_psid=atResCert.tbs.appPermissions[0].psid;
    m_auhtTicket.m_bitmapSsp = atResCert.tbs.appPermissions[0].bitmapSsp;
    m_auhtTicket.m_psid2 = atResCert.tbs.appPermissions[1].psid;
    m_auhtTicket.m_bitmapSsp2 = atResCert.tbs.appPermissions[1].bitmapSsp;
    m_auhtTicket.presentVerKey = atResCert.tbs.verifyKeyIndicator.presentVerKey;
    switch (m_auhtTicket.presentVerKey) {
        case 1:
            m_auhtTicket.verifykeyindicator = atResCert.tbs.verifyKeyIndicator.p256_x_only;
           break;
        case 3:
            m_auhtTicket.verifykeyindicator = atResCert.tbs.verifyKeyIndicator.p256_compressed_y_0;
            break;
        case 4:
            m_auhtTicket.verifykeyindicator = atResCert.tbs.verifyKeyIndicator.p256_compressed_y_1;
            break;
        default:
            break;
    }
    m_auhtTicket.presentSignature = atResCert.rSig.presentRSig;
    switch (m_auhtTicket.presentSignature) {
        case 1:
            m_auhtTicket.m_rSig = atResCert.rSig.p256_x_only;
            break;
        case 3:
            m_auhtTicket.m_rSig =atResCert.rSig.p256_compressed_y_0;
            break;
        case 4:
            m_auhtTicket.m_rSig =atResCert.rSig.p256_compressed_y_1;
            break;
        default:
            break;
    }
    m_auhtTicket.m_Ssig = atResCert.signature_sSig;
    std::cout << "[INFO] AT updated" << std::endl;

}

/*
void ATManager::updateAT() {
    AT tmp;
    tmp.m_version = atResCert.version;
    tmp.m_type = atResCert.type;
    std::copy(atResCert.issuer.begin(), atResCert.issuer.end(), tmp.m_issuer);
    //std::copy(atResCert.tbs.name.begin(), atResCert.tbs.name.end(), tmp.m_id_name);
    tmp.m_id_none = atResCert.tbs.id;
    std::copy(atResCert.tbs.cracaId.begin(), atResCert.tbs.cracaId.end(), tmp.m_cracaId);
    tmp.m_crlSeries = atResCert.tbs.crlSeries;
    tmp.m_validityPeriod_start = atResCert.tbs.validityPeriod_start;
    tmp.m_validityPeriod_duration = atResCert.tbs.validityPeriod_duration;
    tmp.m_psid=atResCert.tbs.appPermissions[0].psid;
    std::copy(atResCert.tbs.appPermissions[0].bitmapSsp.begin(), atResCert.tbs.appPermissions[0].bitmapSsp.end(), tmp.m_bitmapSsp);
    tmp.m_psid2 = atResCert.tbs.appPermissions[1].psid;
    std::copy(atResCert.tbs.appPermissions[1].bitmapSsp.begin(), atResCert.tbs.appPermissions[1].bitmapSsp.end(), tmp.m_bitmapSsp2);
    tmp.presentVerKey = atResCert.tbs.verifyKeyIndicator.presentVerKey;
    switch (tmp.presentVerKey) {
        case 1:
            std::copy(atResCert.tbs.verifyKeyIndicator.p256_x_only.begin(), atResCert.tbs.verifyKeyIndicator.p256_x_only.end(), tmp.verifykeyindicator);
            break;
        case 3:
            std::copy(atResCert.tbs.verifyKeyIndicator.p256_compressed_y_0.begin(), atResCert.tbs.verifyKeyIndicator.p256_compressed_y_0.end(), tmp.verifykeyindicator);
            break;
        case 4:
            std::copy(atResCert.tbs.verifyKeyIndicator.p256_compressed_y_1.begin(), atResCert.tbs.verifyKeyIndicator.p256_compressed_y_1.end(), tmp.verifykeyindicator);
            break;
        default:
            break;
    }
    tmp.presentSignature = atResCert.rSig.presentRSig;
    switch (tmp.presentSignature) {
        case 1:
            std::copy(atResCert.rSig.p256_x_only.begin(), atResCert.rSig.p256_x_only.end(), tmp.m_rSig);
            break;
        case 3:
            std::copy(atResCert.rSig.p256_compressed_y_0.begin(), atResCert.rSig.p256_compressed_y_0.end(), tmp.m_rSig);
            break;
        case 4:
            std::copy(atResCert.rSig.p256_compressed_y_1.begin(), atResCert.rSig.p256_compressed_y_1.end(), tmp.m_rSig);
            break;
        default:
            break;
    }
    std::copy(atResCert.signature_sSig.begin(), atResCert.signature_sSig.end(), tmp.m_Ssig);
    m_auhtTicket.store(tmp);

    std::cout << "[INFO] AT updated" << std::endl;
}
 */

bool ATManager::manageRequest() {
    m_terminatorFlagPtr=false;
    if (isFileNotEmpty()) {

        //std::cout << "[INFO] File AT is not empty" << std::endl;
        atResCert = atRes.getATResponse();
        if (atResCert.version == 0 && atResCert.issuer.empty()){
            m_terminatorFlagPtr=true;
            return false;
        }

        if (atResCert.tbs.validityPeriod_start <= getCurrentTimestamp32() && getCurrentTimestamp32()
        <= atResCert.tbs.validityPeriod_start + atResCert.tbs.validityPeriod_duration * 3600) {

            if (atResCert.tbs.validityPeriod_start + atResCert.tbs.validityPeriod_duration * 3600 - getCurrentTimestamp32() < 1800) {
                std::cout << "[INFO] AT is valid but less than 30 minutes" << std::endl;
                regeneratePEM();
                createRequest();
                if(m_terminatorFlagPtr){
                    return false;
                }
                sendPOST();
                atResCert = atRes.getATResponse();
                if (atResCert.version == 0 && atResCert.issuer.empty()){
                    m_terminatorFlagPtr=true;
                    return false;
                }
                updateAT();
                return true;
            } else {  // if the AT is still valid, wait for 15 minutes
                std::cout << "[INFO] AT is still valid" << std::endl;
                //if all is valid, update the struct m_authTicket with the value of atResCert
                updateAT();
                return true;
            }

        } else {
            std::cout << "[INFO] AT is not valid" << std::endl;
            regeneratePEM();
            createRequest();
            if(m_terminatorFlagPtr){
                std::cout << "[INFO] Terminator flag is set" << std::endl;
                return false;
            }
            sendPOST();
            atResCert = atRes.getATResponse();
            if (atResCert.version == 0 && atResCert.issuer.empty()){
                m_terminatorFlagPtr=true;
                return false;
            }
            updateAT();
            return true;
        }
    } else {
        std::cout << "[INFO] File AT is empty" << std::endl;
        regeneratePEM();
        createRequest();
        if(m_terminatorFlagPtr){
            return false;
        }
        sendPOST();
        atResCert = atRes.getATResponse();
        if (atResCert.version == 0 && atResCert.issuer.empty()){
            m_terminatorFlagPtr=true;
            return false;
        }
        updateAT();
        return true;
    }
}

//TODO part to use thread and check periodically the AT
/* Fix when will be used threads

void ATManager::atCheckThread(){
    while(!m_terminatorFlagPtr){
        std::cout << "[INFO] Checking AT" << std::endl;
    if (isFileNotEmpty()) {  // check if the file is empty or not exist
        std::cout << "[INFO] File AT is not empty" << std::endl;
        atResCert = atRes.getATResponse();    // if not empty, get the AT response
        if (atResCert.version == 0 && atResCert.issuer.empty()){
            m_terminatorFlagPtr=true;
            return;
        }
        if (atResCert.tbs.validityPeriod_start <= getCurrentTimestamp32() && getCurrentTimestamp32()
                <= atResCert.tbs.validityPeriod_start + atResCert.tbs.validityPeriod_duration) { // check if the AT is valid
            std::cout << "[INFO] AT is valid" << std::endl;
            // check the threshold of the AT validity, if it is less than 30 minutes, send a new request
            if (atResCert.tbs.validityPeriod_start + atResCert.tbs.validityPeriod_duration - getCurrentTimestamp32() < 1800) {
                createRequest();
                if(m_terminatorFlagPtr){
                    return;
                }
                sendPOST();
                atResCert = atRes.getATResponse();
                if (atResCert.version == 0 && atResCert.issuer.empty()){
                    m_terminatorFlagPtr=true;
                    return;
                }
                updateAT();
            } else {  // if the AT is still valid, wait for 15 minutes
                std::cout << "[INFO] AT is still valid" << std::endl;
                //if all is valid, update the struct m_authTicket with the value of atResCert
                updateAT();
            }
        } else { // if the AT is not valid, send a new request
            std::cout << "[INFO] AT is not valid" << std::endl;
            createRequest();
            if(m_terminatorFlagPtr){
                return;
            }
            sendPOST();
            atResCert = atRes.getATResponse();
            if (atResCert.version == 0 && atResCert.issuer.empty()){
                m_terminatorFlagPtr=true;
                return;
            }
            updateAT();
        }
    } else { // if the file is empty, send a new request
        createRequest();
        if(m_terminatorFlagPtr){
            return;
        }
        sendPOST();
        atResCert = atRes.getATResponse();
        if (atResCert.version == 0 && atResCert.issuer.empty()){
            m_terminatorFlagPtr=true;
            return;
        }
        updateAT();
    }
    // TODO exit from the function and terminate the thread if terminate flag is set
    std::cout << "[INFO] Waiting for 15 minutes" << std::endl;
    std::this_thread::sleep_for(std::chrono::minutes(15));
    }
}

void ATManager::terminate(){
    if( m_checkThread != nullptr){
        m_checkThread->join();
        delete m_checkThread;
    }
}


//run thread
void ATManager::run(){
    m_checkThread = new std::thread(&ATManager::atCheckThread, this);
}

*/





