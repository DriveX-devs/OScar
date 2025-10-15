// Created by Alessandro on 19/12/2024.

#ifndef EC_MANAGER_H
#define EC_MANAGER_H

#include <string>
#include <vector>
#include <ECresponse.h>

class ECManager {
        public:

    typedef struct encryptionData
    {
        std::string recipient;
        std::string nonce;
        std::string ciphertext;

    } eData;

    typedef struct TobeSigned
    {
        long protocolversion;
        std::string unsecuredData;
        unsigned long header_psid;
        uint64_t header_generationTime;
    } tbsDataSigned;

    typedef struct curvePoint
    {
        std::string p256_x_only;
        std::string p256_fill;
        std::string p256_compressed_y_0;
        std::string p256_compressed_y_1;
        std::string p256_uncompressed_x;
        std::string p256_uncompressed_y;
    } GNecdsaNistP256;

    typedef struct signedData
    {
        long hashID;
        tbsDataSigned tbsdata;
        std::string signer_digest;
        GNecdsaNistP256 rSig;
        std::string signature_sSig;
    } sData;

    typedef struct contentData
    {
        sData signData;
        eData encrData;
        std::string unsecuredData;
    } contData;

    typedef struct cipherPacket
    {
        long m_protocolversion;
        contData content;
    } cPacket;

    typedef struct _servicePermissions
    {
        unsigned long psid;
        std::string bitmapSsp;
    } GNpsidSsp;

    typedef struct _tbsCertificateDataContainer
    {
        int id; // Actually is a struct with different type, but we consider the case "none" with value 0.
        std::string name;
        std::string cracaId;
        uint16_t crlSeries;
        uint32_t validityPeriod_start;
        long validityPeriod_duration;
        std::vector<GNpsidSsp> appPermissions;
        long symAlgEnc;
        GNecdsaNistP256 encPublicKey;
        GNecdsaNistP256 verifyKeyIndicator; // Ignore other fields beacuse there is always ecdsaNistP256
    } GNtbsCertDC;

    typedef struct _certificateDataContainer
    {
        long version;
        long type;
        std::string issuer;
        GNtbsCertDC tbs;
        GNecdsaNistP256 rSig;
        std::string signature_sSig;
    } GNcertificateDC;

    typedef struct _publicKey
    {
        std::string prefix;
        std::string pk;
    } GNpublicKey;

    typedef struct _encryptionData
    {
        std::vector<unsigned char> ciphertextWithTag;
        std::vector<unsigned char> encryptedKey;
        std::vector<unsigned char> ephemeralPublicKey;
        std::vector<unsigned char> x_value;
        std::vector<unsigned char> y_value;
        std::vector<unsigned char> eciesTag;
        std::vector<unsigned char> nonce;
    } encData;

    typedef struct _signatureMaterial
    {
        std::string r;
        std::string s;
    } GNsignMaterial;

    typedef struct _iniINFOEC{
        std::string recipientID;
        std::string eaCert1;
        std::string eaCert2;
        std::string eaCert3;
        std::string pk_rfc;
        std::string sk_rfc1;
        std::string sk_rfc2;
        std::string itsID;
        std::string bitmapSspEA;
    } iniEC;


        ECManager(); // Costruttore
        virtual ~ECManager(); // Distruttore


        bool manageRequest();

        std::string getECBytes() {return ecRes.getECBytes();}


        private:

        std::vector<unsigned char> hexStringToBytes(const std::string &hex);
        void handleErrors();
        void saveStringToFile(const std::string& key, const std::string& fileName);
        std::string retrieveStringFromFile(const std::string& fileName);
        std::string to_hex_string(const unsigned char *data, size_t length);
        std::string to_hex_string(const std::vector<unsigned char> &data);
        EVP_PKEY *loadCompressedPublicKey(const std::string &compressedKey, int compression);
        void computeSHA256(const std::vector<unsigned char> &data, unsigned char hash[SHA256_DIGEST_LENGTH]);
        std::vector<unsigned char> concatenateHashes(const unsigned char hash1[SHA256_DIGEST_LENGTH], const unsigned char hash2[SHA256_DIGEST_LENGTH]);
        void deriveKeyWithKDF2(const unsigned char* sharedSecret, size_t secretLen, const unsigned char* P1, size_t P1_len, unsigned char* derivedKey, size_t derivedKeyLen);
        std::vector<unsigned char> encryptMessage(std::vector<unsigned char> &plaintext,EVP_PKEY *receiverPublicKey, std::vector<unsigned char> &encryptedKey,std::vector<unsigned char> &ephemeralPublicKey,std::vector<unsigned char> &eciesTag, std::vector<unsigned char> &nonce, std::vector<unsigned char> &aesCcmTag, const unsigned char *p1);
        encData doEncryption(std::string message, GNecdsaNistP256 encPkEA, const unsigned char *p1);
        void print_openssl_error();
        EC_KEY *loadECKeyFromFile(const std::string &private_key_file, const std::string &public_key_file);
        EC_KEY *loadECKeyFromRFC5480(const std::string &private_key_rfc, const std::string &public_key_rfc);
        GNpublicKey recoverECKeyPair(bool ephemeral);
        ECDSA_SIG *signHash(const unsigned char *hash, EC_KEY *ec_key);
        GNsignMaterial signatureCreation(const std::string &tbsData_hex, bool ephemeral);
        uint64_t getCurrentTimestamp();
        uint32_t getCurrentTimestamp32();
        void createRequest();
        void sendPOST();
        void regeneratePEM();
        bool isFileNotEmpty();
        iniEC readIniFile();

        ECResponse ecRes;
        ECResponse::GNcertificateDC EC;

        GNpublicKey public_key;
        GNpublicKey EPHpublic_key;
        bool ephemeral;
        EC_KEY *m_ecKey ;
        EC_KEY *m_EPHecKey;
        bool validSignature;
        std::string request_result;
        std::string signedData_result ;
        std::string encode_result;
        long m_protocolversion ;
        long m_version = 3;
        std::string m_recipientID; // 8 bytes contraint
        long m_hashId ;
        unsigned long m_psid ;
        std::string m_itsID ;
        long m_certFormat;
        uint16_t m_hours ; // 7 days
        std::string m_bitmapSspEA;
};

#endif // EC_MANAGER_H
