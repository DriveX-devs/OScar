// Secure Header
// Here is created the structure of the secure packet, and the functions to sign and verify the signature.
// Created by Alessandro Giaccaglini - alessandro.giaccaglini@gmail.com

#include "security.h"
#include "SequenceOf.hpp"
#include "Seq.hpp"
#include "Getter.hpp"
#include "Setter.hpp"
#include <openssl/sha.h>
#include <vector>
#include <openssl/obj_mac.h>
#include <openssl/err.h>
#include <chrono>
#include <openssl/bn.h>
#include <iostream>
#include <string>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <openssl/ec.h>

extern "C" {
#include "CAM.h"
#include "Ieee1609Dot2Data.h"
#include "Ieee1609Dot2Content.h"
#include "EtsiTs103097Data.h"
#include "CertificateBase.h"
#
}

Security::~Security ()
{
    if (m_ecKey != nullptr)
    {
        EC_KEY_free(m_ecKey);
    }
}

Security::Security ()
{
    //m_eventCleaner = Simulator::Schedule(MilliSeconds (1000),&Security::mapCleaner,this);
    m_atmanager = nullptr;
    m_ecKey = nullptr;
    m_protocolVersion = 3;
    m_hashId = HashAlgorithm_sha256;
    m_generationTime = 0;
    m_digest = "";
    m_psid = 0;

}

void Security::mapCleaner ()
{
    uint64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    for (auto it = m_receivedCertificates.begin(); it != m_receivedCertificates.end(); ) {
        if (it->first < now - 5000) {
            it = m_receivedCertificates.erase(it);
        } else {
            ++it;
        }
    }
}


std::vector<unsigned char>
Security::hexStringToBytes (const std::string &hex)
{
    std::vector<unsigned char> bytes;
    for (unsigned int i = 0; i < hex.length (); i += 2)
    {
        std::string byteString = hex.substr (i, 2);
        unsigned char byte = static_cast<unsigned char> (strtol (byteString.c_str (), nullptr, 16));
        bytes.push_back (byte);
    }
    return bytes;
}


void
Security::computeSHA256 (const std::vector<unsigned char> &data,
                         unsigned char hash[SHA256_DIGEST_LENGTH])
{
    SHA256_CTX sha256;
    SHA256_Init (&sha256);
    SHA256_Update (&sha256, data.data (), data.size ());
    SHA256_Final (hash, &sha256);
}

std::vector<unsigned char>
Security::concatenateHashes (const unsigned char hash1[SHA256_DIGEST_LENGTH],
                             const unsigned char hash2[SHA256_DIGEST_LENGTH])
{
    std::vector<unsigned char> concatenatedHashes;
    concatenatedHashes.insert (concatenatedHashes.end (), hash1, hash1 + SHA256_DIGEST_LENGTH);
    concatenatedHashes.insert (concatenatedHashes.end (), hash2, hash2 + SHA256_DIGEST_LENGTH);
    return concatenatedHashes;
}

void
Security::print_openssl_error ()
{
    char buffer[120];
    unsigned long error = ERR_get_error ();
    ERR_error_string_n (error, buffer, sizeof (buffer));
    std::cerr << buffer << std::endl;
}
/*
Security::GNpublicKey
Security::generateECKeyPair ()
{

    EC_KEY *ec_key = EC_KEY_new_by_curve_name (NID_X9_62_prime256v1);
    if (!ec_key)
    {
        std::cerr << "Error creating EC_KEY object" << std::endl;
        print_openssl_error ();
        return {};
    }

    if (!EC_KEY_generate_key (ec_key))
    {
        std::cerr << "Error generating EC key pair" << std::endl;
        print_openssl_error ();
        EC_KEY_free (ec_key);
        return{};
    }


    m_ecKey = EC_KEY_dup(ec_key);

    // Get the public key in hex form
    const EC_POINT *pub_key_point = EC_KEY_get0_public_key (ec_key);
    if (!pub_key_point)
    {
        std::cerr << "Error getting public key" << std::endl;
        print_openssl_error ();
        EC_KEY_free (ec_key);
        return {};
    }

    BN_CTX *ctx = BN_CTX_new ();
    if (!ctx)
    {
        std::cerr << "Error creating BN_CTX" << std::endl;
        print_openssl_error ();
        EC_KEY_free (ec_key);
        return {};
    }

    char *pub_key_hex = EC_POINT_point2hex (EC_KEY_get0_group (ec_key), pub_key_point,
                                            POINT_CONVERSION_COMPRESSED, ctx);
    if (!pub_key_hex)
    {
        std::cerr << "Error converting public key to hex" << std::endl;
        print_openssl_error ();
        BN_CTX_free (ctx);
        EC_KEY_free (ec_key);
        return {};
    }

    // Remove prefix from the PK
    std::string pub_key_hex_str (pub_key_hex);
    std::string prefix = pub_key_hex_str.substr (0, 2);
    if (prefix == "02")
        prefix = "compressed_y_0";
    else if (prefix == "03")
        prefix = "compressed_y_1";

    pub_key_hex_str = pub_key_hex_str.substr (2);

    publicKey.prefix = prefix;
    publicKey.pk = pub_key_hex_str;

    EC_KEY_free(ec_key);

    return publicKey;
}
*/
// Function to sign a hash with a private key
ECDSA_SIG *
Security::signHash (const unsigned char *hash, EC_KEY *ec_key)
{
    ECDSA_SIG *signature = ECDSA_do_sign (hash, SHA256_DIGEST_LENGTH, ec_key);
    if (!signature)
    {
        std::cerr << "Error signing hash" << std::endl;
        print_openssl_error ();
    }
    return signature;
}

// Function to load an EC key pair from a file
EC_KEY* Security::loadECKeyFromFile(const std::string &private_key_file, const std::string &public_key_file)
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
void Security::recoverECKeyPair()
{
    std::string private_key_file = "";
    std::string public_key_file = "";
    private_key_file = "./pkiReqRes/ephSKEY.pem";
    public_key_file = "./pkiReqRes/ephPKEY.pem";
    EC_KEY *ec_key = nullptr;
    ec_key = loadECKeyFromFile(private_key_file, public_key_file);
    if (!ec_key)
    {
        return;
    }
    if (m_ecKey)
    {
        EC_KEY_free(m_ecKey);
    }
    m_ecKey = EC_KEY_dup(ec_key);

    EC_KEY_free(ec_key);
}

Security::GNsignMaterial
Security::signatureCreation (const std::string& tbsData_hex, const std::string& certificate_hex)
{

    GNsignMaterial signMaterial;

    std::vector<unsigned char> tbsData_bytes = hexStringToBytes (tbsData_hex);
    std::vector<unsigned char> certificate_bytes = hexStringToBytes (certificate_hex);

    unsigned char tbsData_hash[SHA256_DIGEST_LENGTH];
    computeSHA256 (tbsData_bytes, tbsData_hash);

    unsigned char certificate_hash[SHA256_DIGEST_LENGTH];
    computeSHA256 (certificate_bytes, certificate_hash);

    std::vector<unsigned char> concatenatedHashes =
            concatenateHashes (tbsData_hash, certificate_hash);

    unsigned char final_hash[SHA256_DIGEST_LENGTH];
    computeSHA256 (concatenatedHashes, final_hash);
    recoverECKeyPair(); // try to move this otherwise it will read file every time
    EC_KEY *ec_key = EC_KEY_dup (m_ecKey);

    // Sign the final hash
    ECDSA_SIG *signature = signHash (final_hash, ec_key);
    if (!signature)
    {
        EC_KEY_free (ec_key);
    }

    // Extract r and s from the signature
    const BIGNUM *r;
    const BIGNUM *s;
    ECDSA_SIG_get0 (signature, &r, &s);


    // Convert r and s to hex strings
    char *r_hex = BN_bn2hex (r);
    char *s_hex = BN_bn2hex (s);

    if (!r_hex || !s_hex) {
        std::cerr << "Error: Failed to convert r or s to hexadecimal." << std::endl;
        ECDSA_SIG_free(signature);
        EC_KEY_free(ec_key);
        return {};
    }


    auto pad_hex_string = [](const char* hex_str) -> std::string {
        std::string padded_hex(hex_str);
        if (padded_hex.length() < 64) {
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
    OPENSSL_free (r_hex);
    OPENSSL_free (s_hex);
    ECDSA_SIG_free (signature);
    EC_KEY_free (ec_key);

    return signMaterial;
}

bool
Security::signatureVerification (const std::string& tbsData_hex, const std::string& certificate_hex, const GNsgtrDC& signatureRS,const std::string& verifyKeyIndicator)
{

    // Convert hex string to bytes
    std::vector<unsigned char> tbsData_bytes = hexStringToBytes (tbsData_hex);
    std::vector<unsigned char> certificate_bytes = hexStringToBytes (certificate_hex);

    // Compute SHA-256 hash
    unsigned char tbsData_hash[SHA256_DIGEST_LENGTH];
    computeSHA256 (tbsData_bytes, tbsData_hash);

    unsigned char certificate_hash[SHA256_DIGEST_LENGTH];
    computeSHA256 (certificate_bytes, certificate_hash);

    // Concatenate the hashes
    std::vector<unsigned char> concatenatedHashes =
            concatenateHashes (tbsData_hash, certificate_hash);

    // Compute SHA-256 hash of the concatenated hashes
    unsigned char final_hash[SHA256_DIGEST_LENGTH];
    computeSHA256 (concatenatedHashes, final_hash);

    std::string r_hex;
    // Here put the hexadecimal string related rSig, sSig (inside outer signature in CAM) and pk (inside verifyKeyIndicator in certificate)
    if (!signatureRS.rSig.p256_x_only.empty()) {
        r_hex = signatureRS.rSig.p256_x_only;
    } else if (!signatureRS.rSig.p256_compressed_y_0.empty()) {
        r_hex = signatureRS.rSig.p256_compressed_y_0;
    } else if (!signatureRS.rSig.p256_compressed_y_1.empty()) {
        r_hex = signatureRS.rSig.p256_compressed_y_1;
    }
    std::string s_hex = signatureRS.sSig;

    // Convert hex strings to bytes

    std::vector<unsigned char> r_bytes(r_hex.begin(), r_hex.end());
    std::vector<unsigned char> s_bytes(s_hex.begin(), s_hex.end());
    std::vector<unsigned char> pk_bytes(verifyKeyIndicator.begin(), verifyKeyIndicator.end());


    // Ensure pk_bytes size is correct for compressed public key
    if (pk_bytes.size () != 33)
    {
        std::cerr << "Error: Public key size is incorrect, expected 33 bytes for compressed key"
                  << std::endl;
    }

    // Create the public key object
    EC_KEY *ec_key = EC_KEY_new_by_curve_name (NID_X9_62_prime256v1);
    if (!ec_key)
    {
        std::cerr << "Error creating EC_KEY object" << std::endl;
        print_openssl_error ();
    }

    EC_POINT *pub_key_point = EC_POINT_new (EC_KEY_get0_group (ec_key));
    if (!pub_key_point)
    {
        std::cerr << "Error creating EC_POINT object" << std::endl;
        print_openssl_error ();
        EC_KEY_free (ec_key);
    }

    // Convert the public key from octet string (compressed form)
    if (!EC_POINT_oct2point (EC_KEY_get0_group (ec_key), pub_key_point, pk_bytes.data (),
                             pk_bytes.size (), nullptr))
    {
        std::cerr << "Error converting public key" << std::endl;
        print_openssl_error ();
        EC_POINT_free (pub_key_point);
        EC_KEY_free (ec_key);
    }

    // Set the public key
    if (!EC_KEY_set_public_key (ec_key, pub_key_point))
    {
        std::cerr << "Error setting public key" << std::endl;
        print_openssl_error ();
        EC_POINT_free (pub_key_point);
        EC_KEY_free (ec_key);
    }

    // Create the ECDSA_SIG object
    ECDSA_SIG *signature = ECDSA_SIG_new ();
    if (!signature)
    {
        std::cerr << "Error creating ECDSA_SIG object" << std::endl;
        print_openssl_error ();
        EC_POINT_free (pub_key_point);
        EC_KEY_free (ec_key);
    }

    // Convert r and s in bignum objects
    BIGNUM *r = BN_bin2bn (r_bytes.data (), r_bytes.size (), nullptr);
    BIGNUM *s = BN_bin2bn (s_bytes.data (), s_bytes.size (), nullptr);

    if (!r || !s)
    {
        std::cerr << "Error converting r or s" << std::endl;
        print_openssl_error ();
        ECDSA_SIG_free (signature);
        EC_POINT_free (pub_key_point);
        EC_KEY_free (ec_key);
        if (r)
            BN_free (r);
        if (s)
            BN_free (s);
    }

    // Setting signature through r and s values
    if (!ECDSA_SIG_set0 (signature, r, s))
    {
        std::cerr << "Error setting r and s in signature" << std::endl;
        print_openssl_error ();
        ECDSA_SIG_free (signature);
        EC_POINT_free (pub_key_point);
        EC_KEY_free (ec_key);
    }

    // Verify the signature
    int verify_status = ECDSA_do_verify (final_hash, SHA256_DIGEST_LENGTH, signature, ec_key);
    if (verify_status == 1)
    {
        validSignature = true;
    }
    else if (verify_status == 0)
    {
        validSignature = false;
    }
    else
    {
        std::cerr << "Error verifying signature" << std::endl;
        print_openssl_error ();
    }

    // Clean up
    ECDSA_SIG_free (signature);
    EC_POINT_free (pub_key_point);
    EC_KEY_free (ec_key);

    return validSignature;
}

uint64_t Security::getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());
    uint64_t microseconds_since_epoch = static_cast<uint64_t>(duration.count());


    const uint64_t seconds_per_year = 365 * 24 * 60 * 60;
    const uint64_t leap_seconds = 8 * 24 * 60 * 60;
    const uint64_t epoch_difference_seconds = (34 * seconds_per_year) + leap_seconds;
    const uint64_t epoch_difference = epoch_difference_seconds * 1'000'000ULL;

    return microseconds_since_epoch - epoch_difference;
}

std::string Security::to_hex(const std::string& input) {
    std::ostringstream oss;
    for (unsigned char byte : input) {
        oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
    }
    return oss.str();
}


GNDataRequest_t
Security::createSecurePacket (GNDataRequest_t dataRequest, bool &isCertificate)
{

    auto ieeeData = asn1cpp::makeSeq (Ieee1609Dot2Data);

    // IeeeData, protocol version
    asn1cpp::setField (ieeeData->protocolVersion, m_protocolVersion);

    // IeeeContent
    auto ieeeContent = asn1cpp::makeSeq (Ieee1609Dot2Content);
    asn1cpp::setField (ieeeContent->present, Ieee1609Dot2Content_PR_signedData);

    // SignedData part
    auto signData = asn1cpp::makeSeq (SignedData);

    // HashID field
    asn1cpp::setField (signData->hashId, m_hashId);

    // TobeSigned field
    auto tbs = asn1cpp::makeSeq (ToBeSignedData);
    auto signPayload = asn1cpp::makeSeq (SignedDataPayload);
    auto dataPayload = asn1cpp::makeSeq (Ieee1609Dot2Data);
    asn1cpp::setField (dataPayload->protocolVersion, m_protocolVersion);
    auto dataContentPayload = asn1cpp::makeSeq (Ieee1609Dot2Content);
    asn1cpp::setField (dataContentPayload->present, Ieee1609Dot2Content_PR_unsecuredData);

    const uint8_t *buffer = dataRequest.data.getBufferAlloc();
    std::string packetContent((char *) buffer, (int) dataRequest.data.getBufferSize());

    asn1cpp::setField (dataContentPayload->choice.unsecuredData,   packetContent);    asn1cpp::setField (dataPayload->content, dataContentPayload);
    asn1cpp::setField (signPayload->data, dataPayload);
    asn1cpp::setField (tbs->payload, signPayload);
    switch(m_messageType){
        case 1:
            m_psid = 36;
            asn1cpp::setField (tbs->headerInfo.psid, m_psid);
            break;
        case 2:
            m_psid = 639; // not supported yet by AA
            asn1cpp::setField (tbs->headerInfo.psid, m_psid);
            break;
        case 3:
            m_psid = 638; // not supported yet by AA, not sure if 639 is correct, VAM is VRU Awareness basic service?
            asn1cpp::setField (tbs->headerInfo.psid, m_psid);
            break;
        default:
            break;
    }
    asn1cpp::setField (tbs->headerInfo.psid, m_psid);
    m_generationTime = getCurrentTimestamp();
    asn1cpp::setField (tbs->headerInfo.generationTime,  m_generationTime);
    asn1cpp::setField (signData->tbsData, tbs);
    

    // For each second it will send a signer part with certificate, otherwise it will send digest.
    uint64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    if (now - m_timestampLastCertificate >= 1000 ||  m_timestampLastCertificate == 0)
    {

        m_timestampLastCertificate = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();

        //GNpublicKey public_key = generateECKeyPair ();
        isCertificate = true;
        asn1cpp::setField (signData->signer.present, SignerIdentifier_PR_certificate);
        // Signed Data, Signer part with always 1 Certificate in the Sequence.
        auto certList = asn1cpp::makeSeq (SequenceOfCertificate);
        auto certificate = asn1cpp::makeSeq (CertificateBase);
        asn1cpp::setField (certificate->version, m_atmanager->getVersion());
        asn1cpp::setField (certificate->type, m_atmanager->getType());
        asn1cpp::setField (certificate->issuer.present, IssuerIdentifierSec_PR_sha256AndDigest);
        std::vector<unsigned char> issuerBytes = hexStringToBytes (to_hex(m_atmanager->getIssuer()));
        std::string issuerString(issuerBytes.begin(), issuerBytes.end());
        asn1cpp::setField (certificate->issuer.choice.sha256AndDigest, issuerString);

        asn1cpp::setField (certificate->toBeSigned.id.present, CertificateId_PR_none);
        asn1cpp::setField (certificate->toBeSigned.id.choice.none, m_atmanager->getId_none());
        std::vector<unsigned char> craca = hexStringToBytes (to_hex(m_atmanager->getCracaId()));
        std::string m_cracaID(craca.begin(), craca.end());
        asn1cpp::setField (certificate->toBeSigned.cracaId, m_cracaID);
        asn1cpp::setField (certificate->toBeSigned.crlSeries, m_atmanager->getCrlSeries());
        asn1cpp::setField (certificate->toBeSigned.validityPeriod.start, m_atmanager->getValidityPeriod_start());
        asn1cpp::setField (certificate->toBeSigned.validityPeriod.duration.present, Duration_PR_hours);
        asn1cpp::setField (certificate->toBeSigned.validityPeriod.duration.choice.hours, m_atmanager->getValidityPeriod_duration());
        auto appPermission = asn1cpp::makeSeq (SequenceOfPsidSsp);

        // Always two items in SequenceofPsisSsp, App permission
        auto psid1 = asn1cpp::makeSeq (PsidSsp);
        asn1cpp::setField (psid1->psid, m_atmanager->getPsid());
        auto servicePermission1 = asn1cpp::makeSeq (ServiceSpecificPermissions);
        asn1cpp::setField (servicePermission1->present, ServiceSpecificPermissions_PR_bitmapSsp);
        std::vector<unsigned char> bitmap1 = hexStringToBytes (to_hex(m_atmanager->getBitmapSsp()));
        std::string m_bitmap1(bitmap1.begin(), bitmap1.end());
        asn1cpp::setField (servicePermission1->choice.bitmapSsp, m_bitmap1);
        asn1cpp::setField (psid1->ssp, servicePermission1);
        asn1cpp::sequenceof::pushList (*appPermission, psid1);

        auto psid2 = asn1cpp::makeSeq (PsidSsp);
        asn1cpp::setField (psid2->psid, m_atmanager->getPsid2());
        auto servicePermission2 = asn1cpp::makeSeq (ServiceSpecificPermissions);
        asn1cpp::setField (servicePermission2->present, ServiceSpecificPermissions_PR_bitmapSsp);
        std::vector<unsigned char> bitmap2 = hexStringToBytes (to_hex(m_atmanager->getBitmapSsp2()));
        std::string m_bitmap2(bitmap2.begin(), bitmap2.end());
        asn1cpp::setField (servicePermission2->choice.bitmapSsp, m_bitmap2);
        asn1cpp::setField (psid2->ssp, servicePermission2);
        asn1cpp::sequenceof::pushList (*appPermission, psid2);
        asn1cpp::setField (certificate->toBeSigned.appPermissions, appPermission);


        asn1cpp::setField (certificate->toBeSigned.verifyKeyIndicator.present, VerificationKeyIndicator_PR_verificationKey);
        asn1cpp::setField (certificate->toBeSigned.verifyKeyIndicator.choice.verificationKey.present, PublicVerificationKey_PR_ecdsaNistP256);
        std::vector<unsigned char> verKey;
        std::string m_verKey;
        switch(m_atmanager->getPresentVerKey()){
            case 1:
                asn1cpp::setField (certificate->toBeSigned.verifyKeyIndicator.choice.verificationKey.choice.ecdsaNistP256.present, EccP256CurvePoint_PR_x_only);
                verKey = hexStringToBytes (to_hex(m_atmanager->getVerifykeyindicator()));
                m_verKey = std::string(verKey.begin(), verKey.end());
                asn1cpp::setField (certificate->toBeSigned.verifyKeyIndicator.choice.verificationKey.choice.ecdsaNistP256.choice.x_only, m_verKey);
                break;
            case 3:
                asn1cpp::setField (certificate->toBeSigned.verifyKeyIndicator.choice.verificationKey.choice.ecdsaNistP256.present, EccP256CurvePoint_PR_compressed_y_0);
                verKey = hexStringToBytes (to_hex(m_atmanager->getVerifykeyindicator()));
                m_verKey = std::string(verKey.begin(), verKey.end());
                asn1cpp::setField (certificate->toBeSigned.verifyKeyIndicator.choice.verificationKey.choice.ecdsaNistP256.choice.compressed_y_0,m_verKey);
                break;
            case 4:
                asn1cpp::setField (certificate->toBeSigned.verifyKeyIndicator.choice.verificationKey.choice.ecdsaNistP256.present, EccP256CurvePoint_PR_compressed_y_1);
                verKey = hexStringToBytes (to_hex(m_atmanager->getVerifykeyindicator()));
                m_verKey = std::string(verKey.begin(), verKey.end());
                asn1cpp::setField (certificate->toBeSigned.verifyKeyIndicator.choice.verificationKey.choice.ecdsaNistP256.choice.compressed_y_1, m_verKey);
                break;
            default:
                break;
        }
        auto signatureCert = asn1cpp::makeSeq (Signature);
        asn1cpp::setField (signatureCert->present, Signature_PR_ecdsaNistP256Signature);
      std::vector<unsigned char> rsignature;
        std::string m_rsign;
        switch (m_atmanager->getPresentSignature())
        {
            case 1:
                asn1cpp::setField (signatureCert->choice.ecdsaNistP256Signature.rSig.present, EccP256CurvePoint_PR_x_only);
                rsignature = hexStringToBytes (to_hex(m_atmanager->getRSig()));
                m_rsign = std::string(rsignature.begin(), rsignature.end());
                asn1cpp::setField (signatureCert->choice.ecdsaNistP256Signature.rSig.choice.x_only, m_rsign);
                break;
            case 3:
                asn1cpp::setField (signatureCert->choice.ecdsaNistP256Signature.rSig.present, EccP256CurvePoint_PR_compressed_y_0);
                rsignature = hexStringToBytes (to_hex(m_atmanager->getRSig()));
                m_rsign = std::string(rsignature.begin(), rsignature.end());
                asn1cpp::setField (signatureCert->choice.ecdsaNistP256Signature.rSig.choice.compressed_y_0, m_rsign);
                break;
            case 4:
                asn1cpp::setField (signatureCert->choice.ecdsaNistP256Signature.rSig.present, EccP256CurvePoint_PR_compressed_y_1);
                rsignature = hexStringToBytes (to_hex(m_atmanager->getRSig()));
                m_rsign = std::string(rsignature.begin(), rsignature.end());
                asn1cpp::setField (signatureCert->choice.ecdsaNistP256Signature.rSig.choice.compressed_y_1, m_rsign);
                break;
            default:
                break;
        }
        std::vector<unsigned char> ssignature = hexStringToBytes (to_hex(m_atmanager->getSSig()));
        std::string m_ssign(ssignature.begin(), ssignature.end());
        asn1cpp::setField (signatureCert->choice.ecdsaNistP256Signature.sSig, m_ssign);

        asn1cpp::setField (certificate->signature, signatureCert);

        asn1cpp::sequenceof::pushList (*certList, certificate);
        asn1cpp::setField (signData->signer.choice.certificate, certList);

        std::string certHex = asn1cpp::oer::encode (certificate);
        m_certificate = certHex;

        // generate the digest of the certificate, so calculate the hash of certificate and take only the last 8 bytes
        std::vector<unsigned char> cer_bytes = hexStringToBytes(certHex);
        unsigned char c_hash[SHA256_DIGEST_LENGTH];
        computeSHA256(cer_bytes, c_hash);
        m_digest.clear();
        for (int i = 24; i < 32; i++)
        {
            std::stringstream ss;
            ss << std::hex << std::setw(2) << std::setfill('0') << (int)c_hash[i];
            m_digest += ss.str();
        }
    }
    else
    {

        // Signed Data, Signer field: version with only digest, other wireshark message with certificate option
        asn1cpp::setField (signData->signer.present, SignerIdentifier_PR_digest);
        std::vector<unsigned char> dig_bytes = hexStringToBytes(m_digest);
        std::string m_dig(dig_bytes.begin(), dig_bytes.end());
        asn1cpp::setField(signData->signer.choice.digest, m_dig);
        isCertificate = false;
    }

    std::string tbs_hex = asn1cpp::oer::encode (tbs);
    GNsignMaterial sign_material = signatureCreation (tbs_hex, m_certificate);

    // Signed Data, Signature part,
    auto signatureContent = asn1cpp::makeSeq (Signature);
    asn1cpp::setField (signatureContent->present, Signature_PR_ecdsaNistP256Signature);
    asn1cpp::setField (signatureContent->choice.ecdsaNistP256Signature.rSig.present, EccP256CurvePoint_PR_x_only);
    std::vector<unsigned char> R_bytes = hexStringToBytes (sign_material.r);
    std::string r_string(R_bytes.begin(), R_bytes.end());
    asn1cpp::setField (signatureContent->choice.ecdsaNistP256Signature.rSig.choice.x_only, r_string);
    std::vector<unsigned char> S_bytes = hexStringToBytes (sign_material.s);
    std::string s_string(S_bytes.begin(), S_bytes.end());
    asn1cpp::setField (signatureContent->choice.ecdsaNistP256Signature.sSig, s_string);
    asn1cpp::setField (signData->signature, signatureContent);

    asn1cpp::setField (ieeeContent->choice.signedData, signData);
    asn1cpp::setField (ieeeData->content, ieeeContent);

    // data encode
    std::string encode_result = asn1cpp::oer::encode (ieeeData);

    if(encode_result.empty()){
        std::cout << "Error encoding data" << std::endl;
        if(isCertificate) std::cout << "Certificate problem" << std::endl;
            else std::cout << "Digest problem" << std::endl;
    }

    packetBuffer pktbuf(encode_result.c_str (), static_cast<unsigned int>(encode_result.size ()));
    dataRequest.data = pktbuf;
    delete[] buffer;
    return dataRequest;
}

Security::Security_error_t
Security::extractSecurePacket (GNDataIndication_t &dataIndication, bool &isCertificate) {

    asn1cpp::Seq<Ieee1609Dot2Data> ieeeData_decoded;

    std::vector<unsigned char> bufferCopy(dataIndication.data, dataIndication.data + dataIndication.lenght);
    uint32_t sizeB = dataIndication.lenght;

    std::string packetContent((char *) bufferCopy.data(), sizeB);

    ieeeData_decoded = asn1cpp::oer::decode(packetContent, Ieee1609Dot2Data);

    GNsecDP secureDataPacket;


    secureDataPacket.protocol_version = asn1cpp::getField (ieeeData_decoded->protocolVersion, long);
    // boolean value for getSeq, getSeqOpt
    bool getValue_ok;

    // content of Ieee1609Dot2Data
    auto contentDecoded = asn1cpp::getSeqOpt (ieeeData_decoded->content, Ieee1609Dot2Content, &getValue_ok);

    // check the present, here is always signed data
    auto present1 = asn1cpp::getField (contentDecoded->present, Ieee1609Dot2Content_PR);
    if (present1 == Ieee1609Dot2Content_PR_signedData) {
        auto signedDataDecoded = asn1cpp::getSeqOpt (contentDecoded->choice.signedData, SignedData, &getValue_ok);

        // First signed data field, HASH ID
        secureDataPacket.content.signData.hashId = asn1cpp::getField (signedDataDecoded->hashId, long);
        // Second signed data field, TBSDATA, inside there is another Ieee1609Dot2Data container with unsercuredData present.
        auto tbsDecoded =  asn1cpp::getSeqOpt (signedDataDecoded->tbsData, ToBeSignedData, &getValue_ok);
        auto payload_decoded =  asn1cpp::getSeqOpt (tbsDecoded->payload, SignedDataPayload, &getValue_ok);
        auto dataContainerDecoded = asn1cpp::getSeqOpt (payload_decoded->data, Ieee1609Dot2Data, &getValue_ok);
        secureDataPacket.content.signData.tbsData.protocol_version = asn1cpp::getField (dataContainerDecoded->protocolVersion, long);
        auto contentContainerDecoded =  asn1cpp::getSeqOpt (dataContainerDecoded->content, Ieee1609Dot2Content);
        auto present2 = asn1cpp::getField (contentContainerDecoded->present, Ieee1609Dot2Content_PR);
        if (present2 == Ieee1609Dot2Content_PR_unsecuredData) {
            secureDataPacket.content.signData.tbsData.unsecureData = asn1cpp::getField (contentContainerDecoded->choice.unsecuredData, std::string);
        }
        // else if( present2 == ??) Is it needed? Never present
        secureDataPacket.content.signData.tbsData.headerInfo_psid = asn1cpp::getField (tbsDecoded->headerInfo.psid, unsigned long);
        secureDataPacket.content.signData.tbsData.headerInfo_generetionTime =asn1cpp::getField (tbsDecoded->headerInfo.generationTime, uint64_t);

        // Third signed data field, SIGNER, can be "digest" or "certificate"
        auto present3 = asn1cpp::getField (signedDataDecoded->signer.present, SignerIdentifier_PR);

        if (present3 == SignerIdentifier_PR_digest) {
            isCertificate = false;
            secureDataPacket.content.signData.signerId.digest =  asn1cpp::getField (signedDataDecoded->signer.choice.digest, std::string);
        } else if (present3 == SignerIdentifier_PR_certificate) {
            isCertificate = true;
            std::string verificationKey;
            //There is always only one certificate, but to be sure a for is implemented.
            int size = asn1cpp::sequenceof::getSize(signedDataDecoded->signer.choice.certificate);
            for (int i = 0; i < size; i++) {
                // Filling all certificate fields
                auto certDecoded = asn1cpp::sequenceof::getSeq (signedDataDecoded->signer.choice.certificate, CertificateBase, i);
                GNcertificateDC newCert;
                newCert.version = asn1cpp::getField (certDecoded->version, long);
                newCert.type = asn1cpp::getField (certDecoded->type, long);
                if (asn1cpp::getField (certDecoded->issuer.present, IssuerIdentifierSec_PR) == IssuerIdentifierSec_PR_sha256AndDigest) {
                    newCert.issuer = asn1cpp::getField (certDecoded->issuer.choice.sha256AndDigest, std::string);
                }

                if (asn1cpp::getField (certDecoded->toBeSigned.id.present, CertificateId_PR) == CertificateId_PR_none) {
                    //getField(certDecoded->toBeSigned.id.choice.none, long ); all types give error, so there is a string. Value is meaningless.
                    newCert.tbs.id = 0;
                }
                // else if (certDecoded->toBeSigned.id.present == name or binary or linkageData). Never present
                newCert.tbs.cracaId = asn1cpp::getField (certDecoded->toBeSigned.cracaId, std::string);
                newCert.tbs.crlSeries = asn1cpp::getField (certDecoded->toBeSigned.crlSeries, uint16_t);
                newCert.tbs.validityPeriod_start = asn1cpp::getField (certDecoded->toBeSigned.validityPeriod.start, uint32_t);
                if (asn1cpp::getField (certDecoded->toBeSigned.validityPeriod.duration.present, Duration_PR) == Duration_PR_hours) {
                    newCert.tbs.validityPeriod_duration = asn1cpp::getField (certDecoded->toBeSigned.validityPeriod.duration.choice.hours, long);
                }
                // else if (certDecoded->toBeSigned.validityPeriod.duration.present == minutes, seconds, etc...). Never present
                // Manage the list of APP PERMISSIONS. There will be always two items, but to be sure a for is implemented.

                int size2 = asn1cpp::sequenceof::getSize(certDecoded->toBeSigned.appPermissions);
                for (int j = 0; j < size2; j++) {
                    auto appPermDecoded = asn1cpp::sequenceof::getSeq (certDecoded->toBeSigned.appPermissions, PsidSsp, j, &getValue_ok);
                    GNpsidSsp newServ;
                    newServ.psid = asn1cpp::getField (appPermDecoded->psid, unsigned long);
                    auto servicePermission = asn1cpp::getSeqOpt (appPermDecoded->ssp, ServiceSpecificPermissions, &getValue_ok);
                    if (asn1cpp::getField (servicePermission->present, ServiceSpecificPermissions_PR) == ServiceSpecificPermissions_PR_bitmapSsp) {
                        newServ.bitmapSsp = asn1cpp::getField (servicePermission->choice.bitmapSsp, std::string);
                    } // else if( servicePermission->present == opaque or null). Never present
                    newCert.tbs.appPermissions.push_back(newServ);
                }
                // Filling last certificate field, verifyKeyIndicator.
                if (asn1cpp::getField (certDecoded->toBeSigned.verifyKeyIndicator.present, VerificationKeyIndicator_PR) ==VerificationKeyIndicator_PR_verificationKey) {
                    if (asn1cpp::getField (certDecoded->toBeSigned.verifyKeyIndicator.choice.verificationKey.present, PublicVerificationKey_PR) == PublicVerificationKey_PR_ecdsaNistP256) {

                        std::vector<unsigned char> prefix_y_0 = hexStringToBytes("02");
                        std::string y0_string(prefix_y_0.begin(), prefix_y_0.end());
                        std::vector<unsigned char> prefix_y_1 = hexStringToBytes("03");
                        std::string y1_string(prefix_y_1.begin(), prefix_y_1.end());

                        switch (
                                asn1cpp::getField (certDecoded->toBeSigned.verifyKeyIndicator.choice.verificationKey.choice.ecdsaNistP256.present,EccP256CurvePoint_PR)) {
                            case EccP256CurvePoint_PR_x_only:
                                newCert.tbs.verifyKeyIndicator.p256_x_only = asn1cpp::getField (certDecoded->toBeSigned.verifyKeyIndicator.choice.verificationKey.choice.ecdsaNistP256.choice.x_only, std::string);
                                break;
                            case EccP256CurvePoint_PR_fill:
                                newCert.tbs.verifyKeyIndicator.p256_fill = "NULL";
                                break;
                            case EccP256CurvePoint_PR_compressed_y_0:
                                newCert.tbs.verifyKeyIndicator.p256_compressed_y_0 = asn1cpp::getField (certDecoded->toBeSigned.verifyKeyIndicator.choice.verificationKey.choice.ecdsaNistP256.choice.compressed_y_0, std::string);
                                verificationKey = y0_string + newCert.tbs.verifyKeyIndicator.p256_compressed_y_0;
                                break;
                            case EccP256CurvePoint_PR_compressed_y_1:
                                newCert.tbs.verifyKeyIndicator.p256_compressed_y_1 = asn1cpp::getField (certDecoded->toBeSigned.verifyKeyIndicator.choice.verificationKey.choice.ecdsaNistP256.choice.compressed_y_1, std::string);
                                verificationKey = y1_string + newCert.tbs.verifyKeyIndicator.p256_compressed_y_1;
                                break;
                            case EccP256CurvePoint_PR_uncompressedP256:
                                newCert.tbs.verifyKeyIndicator.p256_uncompressed_x = asn1cpp::getField (certDecoded->toBeSigned.verifyKeyIndicator.choice.verificationKey.choice.ecdsaNistP256.choice.uncompressedP256.x, std::string);
                                newCert.tbs.verifyKeyIndicator.p256_uncompressed_y = asn1cpp::getField (certDecoded->toBeSigned.verifyKeyIndicator.choice.verificationKey.choice.ecdsaNistP256.choice.uncompressedP256.y, std::string);
                                break;
                            default:
                                break;
                        }

                    } // else if(certDecoded->toBeSigned.verifyKeyIndicator.choice.verificationKey.present == ecc..). Never present

                } // else if (certDecoded->toBeSigned.verifyKeyIndicator.present == VerificationKeyIndicator_PR_reconstructionValue). Never present

                // Signature part inside certificate signer
                auto signCertDecoded = asn1cpp::getSeqOpt (certDecoded->signature, Signature, &getValue_ok);
                if (asn1cpp::getField (signCertDecoded->present, Signature_PR) == Signature_PR_ecdsaNistP256Signature) {
                    auto present4 = asn1cpp::getField (signCertDecoded->choice.ecdsaNistP256Signature.rSig.present, EccP256CurvePoint_PR);
                    switch (present4) {
                        case EccP256CurvePoint_PR_x_only:
                            newCert.signature.rSig.p256_x_only = asn1cpp::getField (signCertDecoded->choice.ecdsaNistP256Signature.rSig.choice.x_only, std::string);
                            break;
                        case EccP256CurvePoint_PR_fill:
                            newCert.signature.rSig.p256_fill = "NULL";
                            break;
                        case EccP256CurvePoint_PR_compressed_y_0:
                            newCert.signature.rSig.p256_compressed_y_0 = asn1cpp::getField (signCertDecoded->choice.ecdsaNistP256Signature.rSig.choice.compressed_y_0, std::string);
                            break;
                        case EccP256CurvePoint_PR_compressed_y_1:
                            newCert.signature.rSig.p256_compressed_y_1 = asn1cpp::getField (signCertDecoded->choice.ecdsaNistP256Signature.rSig.choice.compressed_y_1, std::string);
                            break;
                        case EccP256CurvePoint_PR_uncompressedP256:
                            newCert.signature.rSig.p256_uncompressed_x = asn1cpp::getField (signCertDecoded->choice.ecdsaNistP256Signature.rSig.choice.uncompressedP256.x, std::string);
                            newCert.signature.rSig.p256_uncompressed_y = asn1cpp::getField (signCertDecoded->choice.ecdsaNistP256Signature.rSig.choice.uncompressedP256.y,std::string);
                            break;
                        default:
                            break;
                    }
                    newCert.signature.sSig = asn1cpp::getField (
                            signCertDecoded->choice.ecdsaNistP256Signature.sSig, std::string);
                } // else if(signCertDecoded->present == etc...). Never present
                secureDataPacket.content.signData.signerId.certificate.push_back(newCert);

                std::string certHex = asn1cpp::oer::encode (certDecoded);
                std::pair<std::string, std::string> pair = std::make_pair (verificationKey,certHex);
                uint64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
                m_receivedCertificates[timestamp] = pair;
                mapCleaner();

            }
        }

        //Signature part of signed data
        if (asn1cpp::getField (signedDataDecoded->signature.present, Signature_PR) ==
            Signature_PR_ecdsaNistP256Signature) {

            auto present5 = asn1cpp::getField ( signedDataDecoded->signature.choice.ecdsaNistP256Signature.rSig.present,EccP256CurvePoint_PR);
            switch (present5) {
                case EccP256CurvePoint_PR_x_only:
                    secureDataPacket.content.signData.signature.rSig.p256_x_only = asn1cpp::getField (signedDataDecoded->signature.choice.ecdsaNistP256Signature.rSig.choice.x_only, std::string);
                    break;
                case EccP256CurvePoint_PR_fill:
                    secureDataPacket.content.signData.signature.rSig.p256_fill = "NULL";
                    break;
                case EccP256CurvePoint_PR_compressed_y_0:
                    secureDataPacket.content.signData.signature.rSig.p256_compressed_y_0 = asn1cpp::getField (signedDataDecoded->signature.choice.ecdsaNistP256Signature.rSig.choice.compressed_y_0,  std::string);
                    break;
                case EccP256CurvePoint_PR_compressed_y_1:
                    secureDataPacket.content.signData.signature.rSig.p256_compressed_y_1 = asn1cpp::getField (signedDataDecoded->signature.choice.ecdsaNistP256Signature.rSig.choice.compressed_y_1, std::string);
                    break;
                case EccP256CurvePoint_PR_uncompressedP256:
                    secureDataPacket.content.signData.signature.rSig.p256_uncompressed_x = asn1cpp::getField (signedDataDecoded->signature.choice.ecdsaNistP256Signature.rSig.choice.uncompressedP256.x, std::string);
                    secureDataPacket.content.signData.signature.rSig.p256_uncompressed_y = asn1cpp::getField (signedDataDecoded->signature.choice.ecdsaNistP256Signature.rSig.choice.uncompressedP256.y, std::string);
                    break;
                default:
                    break;
            }
            secureDataPacket.content.signData.signature.sSig = asn1cpp::getField (signedDataDecoded->signature.choice.ecdsaNistP256Signature.sSig, std::string);
        }

        std::string tbs_hex = asn1cpp::oer::encode(tbsDecoded);
        if (m_receivedCertificates.empty()) {
            std::cerr << "[INFO] No certificate received" << std::endl;
            return SECURITY_VERIFICATION_FAILED;
        } else {
            //for every item in map do signature verification
            bool signValid = false;
            for (auto const &item: m_receivedCertificates) {
                if (signatureVerification(tbs_hex, item.second.second,secureDataPacket.content.signData.signature,item.second.first)) {
                    signValid = true;
                    break;
                }
            }
            if (!signValid) {
                return SECURITY_VERIFICATION_FAILED;
            }
        }
    } else if (present1 == Ieee1609Dot2Content_PR_unsecuredData) { // Is it needed? Never present
        secureDataPacket.content.unsecuredData = asn1cpp::getField (contentDecoded->choice.unsecuredData, std::string);
    }

    dataIndication.data = new unsigned char[secureDataPacket.content.signData.tbsData.unsecureData.size()];
    std::memcpy(dataIndication.data, secureDataPacket.content.signData.tbsData.unsecureData.data(), secureDataPacket.content.signData.tbsData.unsecureData.size());
    dataIndication.lenght = secureDataPacket.content.signData.tbsData.unsecureData.size();

    return SECURITY_OK;
}
