#ifndef SECURITY_H
#define SECURITY_H


#include <string>
#include <openssl/ecdsa.h>
#include <openssl/sha.h>
#include <openssl/err.h>
#include <openssl/bn.h>
#include <openssl/pem.h>
#include "asn_utils.h"
#include "utils.h"
#include "gn_utils.h"
//#include "btpdatarequest.h"
#include <openssl/obj_mac.h>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <stdexcept>
#include <ATManager.h>

extern "C" {
#include "CAM.h"
}

/**
 * @brief The Security class.
 *
 * This class is responsible for handling the security aspects of the GeoNet protocol.
 * It provides methods to create and extract secure packets.
 */
class Security
{
public:

  typedef enum {
    SECURITY_OK,
    SECURITY_VERIFICATION_FAILED,
  } Security_error_t;


  // App Permission field
  typedef struct EccP256CurvePoint{
    std::string p256_x_only;
    std::string p256_fill;
    std::string p256_compressed_y_0;
    std::string p256_compressed_y_1;
    std::string p256_uncompressed_x;
    std::string p256_uncompressed_y;
  }GNecdsaNistP256;

  // Service Permission field
  typedef struct _servicePermissions{
    unsigned long psid;
    std::string bitmapSsp;
  }GNpsidSsp;

  // ToBeSignedCertificate Container, Certificate field
  typedef struct _tbsCertificateDataContainer{
    int id; // Actually is a struct with different type, but we consider the case "none" with value 0.
    std::string cracaId;
    uint16_t  crlSeries;
    uint32_t  validityPeriod_start;
    long validityPeriod_duration;
    std::vector<GNpsidSsp> appPermissions;
    GNecdsaNistP256 verifyKeyIndicator; // Ignore other fields beacuse there is always ecdsaNistP256
  }GNtbsCertDC;

  // Signature Container, Signed Data and Certificate field
  typedef struct _signatureDataContainer{
    GNecdsaNistP256 rSig;
    std::string sSig;
  }GNsgtrDC;

  // Certificate list
  typedef struct _certificateDataContainer{
    long version;
    long type;
    std::string issuer;
    GNtbsCertDC tbs;
    GNsgtrDC signature;
  }GNcertificateDC;

  // Signer Container, Signed Data field
  typedef struct _signerIdContainer{
    std::string digest;
    std::vector<GNcertificateDC> certificate;
  }GNsgrC;

  // tbsData Container, Signed Data field
  typedef struct tbsDataContainer{
    long protocol_version;
    std::string unsecureData;
    unsigned long headerInfo_psid;
    uint64_t headerInfo_generetionTime;
  }GNtbsDC;

  // Signed Data Container, Secure Data Packet field
  typedef struct _signedDataContainer{
    long hashId;
    GNtbsDC tbsData;
    GNsgrC signerId;
    GNsgtrDC signature;
  }GNsignDC;

  typedef struct _dataContent{
    std::string unsecuredData;
    GNsignDC signData;
    bool encryptedData; // it is not a bool, for the moment not implemented, never used
    std::string signedCertificateRequest;
    std::string signedX509CertificateRequest;
  }GNcontent;


  // IeeeData Container
  typedef struct _secureDataPacket{
    long protocol_version;
    GNcontent content;
  }GNsecDP;

  typedef struct _publicKey{
    std::string prefix;
    std::string pk;
  }GNpublicKey;

  typedef struct _signatureMaterial{
    std::string r;
    std::string s;
  }GNsignMaterial;

  GNDataRequest_t createSecurePacket(GNDataRequest_t dataRequest, bool &isCertificate);
  Security_error_t extractSecurePacket(GNDataIndication_t &dataIndication, bool &isCertificate);


  /**
   * @brief Construct a new Security object.
   *
   * Default constructor for the Security class.
   */
  Security();
  virtual ~Security();

    void setATmanager(ATManager *atm){m_atmanager = atm;};
    void setMessageType(int type){m_messageType = type;}



private:

  std::string to_hex(const std::string& input);

  std::vector<unsigned char> hexStringToBytes(const std::string& hex);
  void computeSHA256(const std::vector<unsigned char>& data, unsigned char hash[SHA256_DIGEST_LENGTH]);
  std::vector<unsigned char> concatenateHashes(const unsigned char hash1[SHA256_DIGEST_LENGTH], const unsigned char hash2[SHA256_DIGEST_LENGTH]);
  void print_openssl_error();
  // GNpublicKey generateECKeyPair();
  ECDSA_SIG* signHash(const unsigned char* hash, EC_KEY* ec_key);
  GNsignMaterial signatureCreation( const std::string& tbsData_hex,  const std::string& certificate_hex);
  bool signatureVerification( const std::string& tbsData_hex,  const std::string& certificate_hex, const GNsgtrDC& signatureRS, const std::string& verifyKeyIndicator);
  void mapCleaner();
  EC_KEY* loadECKeyFromFile(const std::string &private_key_file, const std::string &public_key_file);
  void recoverECKeyPair();
  uint64_t getCurrentTimestamp();

  //EventId m_eventCleaner;

  EC_KEY *m_ecKey;
  int64_t m_timestampLastCertificate = 0;
  std::map<uint64_t, std::pair<std::string,std::string>> m_receivedCertificates;
  std::string m_certificate;
  GNpublicKey publicKey;
  bool validSignature;

  ATManager *m_atmanager;


  // Ieee1609Dot2Data fields
  int m_messageType;

  long m_protocolVersion;
  long m_hashId;
  unsigned long m_psid;
  uint64_t m_generationTime;
  std::string m_digest;
  
};
#endif // SECURITY_H
