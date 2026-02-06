// Created by Alessandro on 19/12/2024.

#ifndef AT_MANAGER_H
#define AT_MANAGER_H

#include <string>
#include <vector>
#include <atomic>
#include <condition_variable>
#include <thread>
#include <ATResponse.h>

class ATManager {
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

    long getVersion(){ return m_auhtTicket.m_version; }
    long getType(){ return m_auhtTicket.m_type; }
    long getValidityPeriod_duration(){ return m_auhtTicket.m_validityPeriod_duration; }
    int getId_none(){ return m_auhtTicket.m_id_none; }
    int getPresentVerKey(){ return m_auhtTicket.presentVerKey; }
    int getPresentSignature(){ return m_auhtTicket.presentSignature; }
    uint16_t getCrlSeries(){ return m_auhtTicket.m_crlSeries; }
    uint32_t getValidityPeriod_start(){ return m_auhtTicket.m_validityPeriod_start; }
    unsigned long getPsid(){ return m_auhtTicket.m_psid; }
    unsigned long getPsid2(){ return m_auhtTicket.m_psid2; }
    std::string getCracaId(){ return m_auhtTicket.m_cracaId; }
    std::string getIssuer(){ return m_auhtTicket.m_issuer; }
    std::string getBitmapSsp(){ return m_auhtTicket.m_bitmapSsp; }
    std::string getBitmapSsp2(){ return m_auhtTicket.m_bitmapSsp2; }
    std::string getVerifykeyindicator(){ return m_auhtTicket.verifykeyindicator; }
    std::string getRSig(){ return m_auhtTicket.m_rSig; }
    std::string getSSig(){ return m_auhtTicket.m_Ssig; }

    /*
    long getVersion(){ return m_auhtTicket.load().m_version; }
    long getType(){ return m_auhtTicket.load().m_type; }
    long getValidityPeriod_duration(){ return m_auhtTicket.load().m_validityPeriod_duration; }
    int getId_none(){ return m_auhtTicket.load().m_id_none; }
    int getPresentVerKey(){ return m_auhtTicket.load().presentVerKey; }
    int getPresentSignature(){ return m_auhtTicket.load().presentSignature; }
    uint16_t getCrlSeries(){ return m_auhtTicket.load().m_crlSeries; }
    uint32_t getValidityPeriod_start(){ return m_auhtTicket.load().m_validityPeriod_start; }
    unsigned long getPsid(){ return m_auhtTicket.load().m_psid; }
    unsigned long getPsid2(){ return m_auhtTicket.load().m_psid2; }
    std::string getId_name(){ return m_auhtTicket.load().m_id_name; }
    std::string getCracaId(){ return m_auhtTicket.load().m_cracaId; }
    std::string getIssuer(){ return m_auhtTicket.load().m_issuer; }
    std::string getBitmapSsp(){ return m_auhtTicket.load().m_bitmapSsp; }
    std::string getBitmapSsp2(){ return m_auhtTicket.load().m_bitmapSsp2; }
    std::string getVerifykeyindicator(){ return m_auhtTicket.load().verifykeyindicator; }
    std::string getRSig(){ return m_auhtTicket.load().m_rSig; }
    std::string getSSig(){ return m_auhtTicket.load().m_Ssig; }
     */

    typedef struct _iniINFOAT{
        std::string recipientAA;
        std::string aaCert1;
        std::string aaCert2;
        std::string aaCert3;
        std::string aaCert4;
        std::string bitmapCAM;
        std::string bitmapDENM;
        std::string eaIDstring;
    } iniAT;





    ATManager(std::atomic<bool> *terminatorFlagPtr); // Costruttore
    virtual ~ATManager(); // Distruttore

    void createRequest();
    bool manageRequest();
    //void run();
    //void terminate();

    void setECHex(std::string ecBytes) {m_ECHex=ecBytes;}

private:

    std::vector<unsigned char> hexStringToBytes(const std::string &hex);
    void handleErrors();
    void saveStringToFile(const std::string& key, const std::string& fileName);
    std::string retrieveStringFromFile(const std::string& fileName);
    std::string toHexString(const std::string& input);
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
    GNpublicKey recoverECKeyPair(bool ephemeral);
    ECDSA_SIG *signHash(const unsigned char *hash, EC_KEY *ec_key);
    GNsignMaterial signatureCreation(const std::string &tbsData_hex, bool ephemeral, const std::string &signer_hex = "" );
    uint64_t getCurrentTimestamp();
    uint32_t getCurrentTimestamp32();
    std::vector<unsigned char> generateHMACKey(size_t length = 32);
    std::vector<unsigned char> computeHMACTag(const std::vector<unsigned char>& hmacKey, const std::vector<unsigned char>& verificationKey);
    //void atCheckThread();
    void sendPOST();
    void regeneratePEM();
    void updateAT();
    bool isFileNotEmpty();
    iniAT readIniFile();


    typedef struct _authorizationTicket
    {
        long m_version,
                m_type,
                m_validityPeriod_duration;
        int m_id_none,
                presentVerKey,
                presentSignature;
        uint16_t m_crlSeries;
        uint32_t m_validityPeriod_start;
        unsigned long m_psid,
                m_psid2;
        std::string  m_cracaId,
                m_issuer,
                m_bitmapSsp,
                m_bitmapSsp2,
                verifykeyindicator,
                m_rSig,
                m_Ssig;
    } AT;

    /*
    typedef struct _authorizationTicket
    {
        long m_version,
                m_type,
                m_validityPeriod_duration;
        int m_id_none,
                presentVerKey,
                presentSignature;
        uint16_t m_crlSeries;
        uint32_t m_validityPeriod_start;
        unsigned long m_psid,
                        m_psid2;
        char m_id_name[100],
                m_cracaId[100],
                m_issuer[100],
                m_bitmapSsp[100],
                m_bitmapSsp2[100],
                verifykeyindicator[100],
                m_rSig[100],
                m_Ssig[100];
    } AT;
    */

    AT m_auhtTicket;

    //std::atomic<AT> m_auhtTicket;

    ATResponse atRes;
    ATResponse::GNcertificateDC atResCert;

    bool m_terminatorFlagPtr;

    std::thread* m_checkThread;

    GNpublicKey public_key;
    GNpublicKey EPHpublic_key;
    bool ephemeral;
    EC_KEY *m_ecKey ;
    EC_KEY *m_EPHecKey ;
    bool validSignature;
    std::string request_result;
    std::string signedData_result;
    std::string encode_result;
    long m_protocolversion ;
    std::string m_recipientID ; // 8 bytes contraint, hashid8 of AA certificate
    long m_hashId ;
    unsigned long m_psid ;
    long m_certFormat ;
    std::string m_eaId ; // 8 bytes contraint, hashid8 of EA certificate
    uint16_t m_hours ; // 7 days
    unsigned long m_CAM ;
    unsigned long m_DENM;
    unsigned long m_CPM ;
    std::string m_bitmapSspCAM ;
    std::string m_bitmapSspDENM ;
    std::string m_bitmapSspCPM ;

    std::string m_ECHex="";
};

#endif // AT_MANAGER_H
