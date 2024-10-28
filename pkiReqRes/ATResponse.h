// Created by Alessandro on 19/12/2024.

#ifndef AT_RESPONSE_H
#define AT_RESPONSE_H

#include <string>
#include <vector>

class ATResponse {
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
        int presentRSig;
        int presentVerKey;
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

    typedef struct innerResponse
    {
        std::string requestHash;
        long response_code;
        GNcertificateDC certificate;
    } response;

    typedef struct _publicKey
    {
        std::string prefix;
        std::string pk;
    } GNpublicKey;

    typedef struct _signatureMaterial
    {
        std::string r;
        std::string s;
    } GNsignMaterial;




    ATResponse();
    virtual ~ATResponse();

    ATResponse::GNcertificateDC getATResponse();
    void setAesKey(std::string aesKey) {m_aesKey = aesKey;}

private:

    std::vector<unsigned char> hexStringToBytes(const std::string &hex);
    void handleErrors();
    void saveStringToFile(const std::string& key, const std::string& fileName);
    std::string retrieveStringFromFile(const std::string& fileName);
    std::string to_hex_string(const unsigned char *data, size_t length);
    std::string to_hex_string(const std::vector<unsigned char> &data);
    void computeSHA256(const std::vector<unsigned char> &data, unsigned char hash[SHA256_DIGEST_LENGTH]);
    std::vector<unsigned char> concatenateHashes(const unsigned char hash1[SHA256_DIGEST_LENGTH], const unsigned char hash2[SHA256_DIGEST_LENGTH]);
    int readFileContent(const char *filename, unsigned char **dataResponse, uint32_t *length);
    void print_openssl_error();
    EC_KEY *loadECKeyFromFile(const std::string &private_key_file, const std::string &public_key_file);
    EC_KEY *loadECKeyFromRFC5480(const std::string &private_key_rfc, const std::string &public_key_rfc);
    GNpublicKey recoverECKeyPair(bool ephemeral);
    EC_KEY* loadCompressedPublicKey(const std::string &compressedKey, int compression);
    void decryptMessage(const std::vector<unsigned char> &encryptedMessage,const std::vector<unsigned char> &nonce, const unsigned char *presharedKey, std::vector<unsigned char> &decryptedMessage, const std::vector<unsigned char> &aesCcmTag);
    std::string doDecryption(std::string ciphertextWithTag_hex, std::string nonce_hex);
    bool signatureVerification(const std::string &tbsData_hex, GNecdsaNistP256 &rValue, const std::string &sValue, GNecdsaNistP256 verifyKeyIndicator);
    void readIniFile();

    unsigned char *dataResponse;
    uint32_t length;

    bool err_key;

    std::string aaCert1;
    std::string aaCert2;
    std::string aaCert3;
    std::string aaCert4;
    bool signValidation;
    GNpublicKey public_key;
    GNpublicKey EPHpublic_key;
    bool ephemeral;
    EC_KEY *m_ecKey ;
    EC_KEY *m_EPHecKey ;

    std::string m_aesKey="";
};

#endif // AT_RESPONSE_H
