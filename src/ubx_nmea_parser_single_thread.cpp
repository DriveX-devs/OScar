// UBX and NMEA serial GNSS parser
// Designed for the ZED-F9R module, but compatible with any GNSS receiver (some data may not be available with non-ublox devices)
// This module allows OScar to directly retrieve positioning and other kinematics data, without the need of interfacing
// to the gpsd daemon or other external software.
// Copyright (c) 2024 Mauro Vittorio

#include "ubx_nmea_parser_single_thread.h"
#include "utils.h"
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <ctime>
#include <cstring>
#include <sstream>
#include <thread>
#include <utility>
#include <chrono>
#include <algorithm>
#include <iomanip>
#include <unistd.h>
#include <fstream>

using namespace std::chrono;

#define JSON_GET_NUM(json,key) (json[key].number_value())
#define JSON_GET_INT(json,key) (json[key].int_value())
#define JSON_GET_STR(json,key) (json[key].string_value())

/** printUbxMessage() prints a UBX message in the format index[byte] (used for debug purposes) */
void
UBXNMEAParserSingleThread::printUbxMessage(std::vector<uint8_t> msg) {
    std::cout << "message size: " << msg.size() << std::endl;
    for (long unsigned int i = 0; i < msg.size(); i++) {
        printf("%ld[%02X] ", i, msg[i]);
    }
}

/** printNmeaSentence() prints a NMEA sentence (printable chars only) */
void
UBXNMEAParserSingleThread::printNmeaSentence(std::string s) {
	if (!s.empty()) {
		for (long unsigned int i = 0; i < s.size(); i++) {
			if (isprint(s[i])) std::cout << s[i];
		}
		std::cout << std::endl;
	} else {
        std::cout << "The nmea sentence is empty." << std::endl;
    }
}

/** decimal_deg() converts latitude and longitude from the NMEA format to single values in degrees */
double
UBXNMEAParserSingleThread::decimal_deg(double value, char quadrant) {
    // Concatenate int part and dec part
    int degrees = floor(value/100);
    double minutes = value - (degrees*100);
    value = degrees + minutes/60;

    // Value is positive when the quadrant is 'N' or 'E'
    if(quadrant=='W' || quadrant=='S') {
        value = -value;
    }

    return value;
}

/** hexToSigned() performs a conversion from a uint8_t array of 1 to sizeof(T) bytes to a signed long value.
 *
 *  By exploiting the dimension of the type T (by default set to a int32_t, see the .h file), shifts every
 *  bytes to the left by multiples of 8, obtaining the whole number in a single sizeof(T)*8 bit variable.
 *  Then, in case the sign bit it set, fills the leading bits with 1s so that the cast will correctly output a
 *  negative signed number.
 *
 *  If it's provided with wrong input, raises an error and returns -1.
 *
 *  NOTE: This works only for T = integer signed types  */
// TODO: better comment why this function works like this
template <typename T> T
UBXNMEAParserSingleThread::hexToSigned(std::vector<uint8_t> data) {
    T value = 0;

    // Check if the function has been called with a proper type for the template T
    if constexpr (!std::is_same<T, int8_t>::value && !std::is_same<T, int16_t>::value && !std::is_same<T, int32_t>::value && !std::is_same<T, int64_t>::value) {
        std::cerr << "[ERROR] Invalid type for T in hexToSigned(). This is a (very bad) bug. Please report it to the developers.";
        return -1;
    }

    // Return an error in case a vector exceeding the size of T is provided (i.e., more bytes are provided than what T can store)
    if (data.empty() || data.size() > sizeof(T)) {
        std::cerr << "[ERROR] Invalid std::vector data size. Check UBX-ESF-RAW/UBX-NAV-ATT. ";
        return -1;
    }

    // Store all the bytes in a single value, by looping over the 'data' array and setting one byte at a time
    for(unsigned int i=0;i<data.size();i++) {
        value = (value << 8) | data[i];
    }

    // If the number if negative (the first if() checks if the sign bit - the highest bit - is set), fill the leading bits with 1s so that the
    // cast will correctly output a negative signed number
    if(value & (0x01 << (data.size()*8-1))) {
        for (unsigned int i=0;i<sizeof(T)-data.size();i++) {
            value |= (0xffULL << (data.size()+i)*8);
        }
    }

    return static_cast<T>(value);
}

/** clearBuffer() initializes or clears the atomic data structure
 * (see ubx_nmea_parser_single_thread.h for more information) */
void
UBXNMEAParserSingleThread::clearBuffer() {
	out_t tmp;

    // Human-readable dates (epoch time of last update)
	strcpy(tmp.ts_pos,"");
    strcpy(tmp.ts_pos_ubx,"");
    strcpy(tmp.ts_pos_nmea,"");
	strcpy(tmp.ts_acc,"");
	strcpy(tmp.ts_att,"");
	strcpy(tmp.ts_alt,"");
    strcpy(tmp.ts_comp_acc,"");
    strcpy(tmp.ts_comp_ang_rate,"");
	strcpy(tmp.ts_sog_cog_ubx,"");
	strcpy(tmp.ts_sog_cog_nmea,"");
	strcpy(tmp.fix_ubx,"");
	strcpy(tmp.fix_nmea,"");
    tmp.fix_nmea_rmc='0';

    // Raw and compensated accelerations + angular speeds (deg/s)
	tmp.raw_acc_x = 0;
	tmp.raw_acc_y = 0;
	tmp.raw_acc_z = 0;
	tmp.comp_acc_x = 0;
	tmp.comp_acc_y = 0;
	tmp.comp_acc_z = 0;
    tmp.comp_ang_rate_x = 0;
    tmp.comp_ang_rate_y = 0;
    tmp.comp_ang_rate_z = 0;

    // Latitude
	tmp.lat = 0;
    tmp.lat_ubx = 0;
    tmp.lat_nmea = 0;

    // Longitude
	tmp.lon = 0;
    tmp.lon_ubx = 0;
    tmp.lon_nmea = 0;

    // Altitude
	tmp.alt = 0;
    tmp.alt_ubx = 0;
    tmp.alt_nmea = 0;

    // Attitude
	tmp.roll = 0;
	tmp.pitch = 0;
	tmp.heading = 0;

    // Speed Over Ground
    tmp.sog = 0;
	tmp.sog_ubx = 0;
	tmp.sog_nmea = 0;

    // Coursr Over Ground
    tmp.cog = 0;
	tmp.cog_ubx = 0;
	tmp.cog_nmea = 0;

    // Last updates (microseconds)
	tmp.lu_pos = 0;
    tmp.lu_pos_ubx = 0;
    tmp.lu_pos_nmea = 0;
	tmp.lu_acc = 0;
	tmp.lu_att = 0;
	tmp.lu_alt = 0;
    tmp.lu_alt_ubx = 0;
    tmp.lu_alt_nmea = 0;
    tmp.lu_comp_acc = 0;
    tmp.lu_comp_ang_rate = 0;
    tmp.lu_sog_cog = 0;
	tmp.lu_sog_cog_ubx = 0;
	tmp.lu_sog_cog_nmea = 0;
    tmp.lu_fix_nmea_rmc = 0;

	m_outBuffer.store(tmp);
}

/** validateUbxMessage() calculates the UBX checksum using the 8-bit Fletcher algorithm. The calculation range excludes
 *  the first two bytes and the last two bytes of the message, that represent this value.
 *  see section 3.4 - UBX Checksum of the ZED-F9R Interface Description document for more information */
bool
UBXNMEAParserSingleThread::validateUbxMessage(std::vector<uint8_t> msg) {
    uint8_t CK_A = 0x00;
    uint8_t CK_B = 0x00;

    for (long unsigned int i = 2; i <= msg.size()-3; i++) { //checksum calculation range
            CK_A = CK_A + msg[i];
            CK_B = CK_B + CK_A;

            CK_A &= 0xFF;
            CK_B &= 0xFF;
    }

    return (msg[msg.size() -2] == CK_A && msg[msg.size()-1] == CK_B);
}

/** validateNmeaSentence() analyzes and validates a NMEA sentence, calculating its checksum character by applying the
 *  XOR operator between the starting dollar character '$' and the asterisk (excluded),
 *  comparing it with the checksum value extracted from the sentence read */
bool
UBXNMEAParserSingleThread::validateNmeaSentence(const std::string& nmeaMessage) {
    // Check that the message starts with '$' and contains '*'
    if (nmeaMessage.front() != '$' || nmeaMessage.find('*') == std::string::npos) {
        return false;  // Invalid format
    }

    // Find the position of the '*' character
    size_t asteriskPos = nmeaMessage.find('*');

    // Ensure there's enough space for the checksum after '*'
    // This basically checks it the checksum is there as expected
    if (asteriskPos + 2 >= nmeaMessage.length()) {
        return false;  // Invalid or missing checksum
    }

    // XOR all characters between '$' and '*' (excluded), as required by the NMEA standard
    unsigned char checksum = 0;
    for (size_t i = 1; i < asteriskPos; ++i) {
        checksum ^= static_cast<unsigned char>(nmeaMessage[i]);
    }

    // Convert checksum to hexadecimal string (two uppercase digits)
    char hexChecksum[3];
    snprintf(hexChecksum, sizeof(hexChecksum), "%02X", checksum);

    // Extract the provided checksum from the message (characters after '*')
    std::string providedChecksum = nmeaMessage.substr(asteriskPos + 1, 2);

    // Compare the calculated checksum with the provided one
    return providedChecksum == hexChecksum;
}

/** getPosition() retrieves and processes the latest position data from the atomic buffer,
 *  calculates the age of information in microseconds, then returns latitude and longitude
 *  as a std::pair, optionally returns the age of information if age_us is not a nullptr
 *  print_timestamp_and_age is a flag that enables the printing of the timestamp and age of
 *  information (i.e., how old is the information now with respect to when it was last parsed)
 *  for debug purposes
 *
 *  Note: use first() and second() to access the element of a pair - first() contains the lat, second() contains the lon */
std::pair<double,double>
UBXNMEAParserSingleThread::getPosition(long *age_us, bool print_timestamp_and_age) {
    if(m_parser_started==false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return std::pair<double,double>(0,0);
    }

	out_t tmp = m_outBuffer.load();

    // Obtains the current timestamp, extracts the us value from the time_point<> data type
    // and calculates the age of information by subtracting with the last update value
    auto now = time_point_cast<microseconds>(system_clock::now());
    auto epoch_ts = now.time_since_epoch().count(); // Convert the chrono time_point to microseconds in epoch time
    long local_age_us = epoch_ts - tmp.lu_pos;

    long validity_thr = getValidityThreshold(); // Get the threshold after which the data should be considered too old to be valid

    // Saves the value if --show-live-data debug option is enabled
    if (m_debug_age_info_rate) m_debug_age_info.age_pos = local_age_us;

    if (print_timestamp_and_age == true) std::cout << "[Position] - " <<  tmp.ts_sog_cog_ubx
                                                   << " Now: " << epoch_ts
                                                   << " TS position UBX: " << tmp.lu_pos_ubx
                                                   << " Age of information: " << local_age_us << " us"
                                                   << std::endl << std::endl;
    if (age_us != nullptr) *age_us = local_age_us;

    // Checks if the information is valid, flagging the state
    if (local_age_us >= validity_thr) {
        m_pos_valid.store(false);
    }
    else m_pos_valid.store(true);

	return std::pair<double,double>(tmp.lat,tmp.lon);
}

/** getPositionUbx() retrieves the latest UBX position data parsed, behaving in the same
 * way of getPosition(), but considering only the data coming from UBX messages */
 // TODO: evaluate whether the three functions getPosition(), getPositionUbx() and getPositionNmea() can be merged into a single one
std::pair<double,double>
UBXNMEAParserSingleThread::getPositionUbx(long *age_us, bool print_timestamp_and_age) {
    if(m_parser_started==false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return std::pair<double,double>(0,0);
    }

    out_t tmp = m_outBuffer.load();

    auto now = time_point_cast<microseconds>(system_clock::now());
    auto end = now.time_since_epoch().count();
    long local_age_us = end - tmp.lu_pos_ubx;

    long validity_thr = getValidityThreshold();

    if (m_debug_age_info_rate) m_debug_age_info.age_pos_ubx = local_age_us;

    if (print_timestamp_and_age == true) std::cout << "[Position(UBX)] - " <<  tmp.ts_pos_ubx
                                                   << "Age of information: " << local_age_us << " us"
                                                   << std::endl << std::endl;
    if (age_us != nullptr) *age_us = local_age_us;

    if (local_age_us >= validity_thr) {
        m_pos_valid_ubx.store(false);
    }
    else m_pos_valid_ubx.store(true);

    return std::pair<double,double>(tmp.lat_ubx,tmp.lon_ubx);
}

/** getPositionNmea() retrieves the latest UBX position data parsed, behaving in the same
 * way of getPosition(), but considering only the data coming from UBX messages */
std::pair<double,double>
UBXNMEAParserSingleThread::getPositionNmea(long *age_us, bool print_timestamp_and_age) {
    if(m_parser_started==false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return std::pair<double,double>(0,0);
    }

    out_t tmp = m_outBuffer.load();

    auto now = time_point_cast<microseconds>(system_clock::now());
    auto end = now.time_since_epoch().count();
    long local_age_us = end - tmp.lu_pos_nmea;

    long validity_thr = getValidityThreshold();

    if (m_debug_age_info_rate) m_debug_age_info.age_pos_nmea = local_age_us;

    if (print_timestamp_and_age == true) std::cout << "[Position(NMEA)] - " <<  tmp.ts_pos_nmea
                                                   << "Age of information: " << local_age_us << " us"
                                                   << std::endl << std::endl;
    if (age_us != nullptr) *age_us = local_age_us;

    if (local_age_us >= validity_thr) {
        m_pos_valid_nmea.store(false);
    }
    else m_pos_valid_nmea.store(true);

    return std::pair<double,double>(tmp.lat_nmea,tmp.lon_nmea);
}


/** parseNmeaGns() checks and parses a GNGNS NMEA sentence in order to get the latitude and longitude data
 *
 *  The function separates the sentence fields using the commas and extract the relevant data.
 *  After parsing, it produces the timestamp related to when the information was parsed and updates
 *  the atomic data buffer ("m_outBuffer").
 *  For more information see page 2.7.9.1 GNSS Fix Data on page 28 of ublox ZED-F9R Interface Description */
void
UBXNMEAParserSingleThread::parseNmeaGns(std::string nmea_response) {
    // The approach of using .load() and then .store() works only in a single-writer multiple-reader context!
    // This is, however, not a problem here as we expect to have a single function writing at a time on the m_outBuffer,
    // since the read from serial procedure is single threaded
    out_t out_nmea = m_outBuffer.load(); // Get the current data in the atomic buffer

    std::vector<std::string> fields; // Fields of the NMEA sentence
    std::stringstream ss(nmea_response);
    std::string field; // Current field

    // Separate the sentence by commas and store the fields of interest
    while (std::getline(ss, field, ',')) {
        fields.push_back(field);
    }

	// Latitude and Longitude substrings to be converted to double using std::stod() later
    std::string slat = fields[2]; // "s" = string --> "slat" = string latitude
    std::string slon = fields[4]; // "s" = string --> "slon" = string longitude
    std::string posMode = fields[6];
    std::string salt = fields[9]; // "s" = string --> "salt" = string altitude

    // cardinal points (N, S, W, E) - stored in fields 3 and 5 of GxGNS
	char cp_lat = fields[3].at(0);
    char cp_lon = fields[5].at(0);

    // Parse the fix mode
    // Unlike other NMEA sentences, GxGNS does not contain a single fix mode, but the fix mode for all constellations.
    // posMode will therefore contain as many characters as the number of constellations, each one representing the corresponding fix mode.
    // The first five characters are respectively for GPS, GLONASS, Galileo, BeiDou, and QZSS.
    // For example, RRRRN means that the fix mode is "RTK Fixed" for GPS, GLONASS, Galileo, and BeiDou, and "No Fix" for QZSS.

    // Here we want to parse a "global" fix mode, returning a single piece of information that best represents the current mode,
    // like if a single fix mode was parsed from GxRMC.

    // Therefore, if a fix was available from RMC shortly before this GNS sentence, check if the GNS fix mode contains
    // at least once the RMC fix mode character. In this case, "confirm" the fix already received from RMC, if it was
    // received within the last second
    auto fix_nmea_rmc_now = time_point_cast<microseconds>(system_clock::now()).time_since_epoch().count();
    if(out_nmea.fix_nmea_rmc != '0' && fix_nmea_rmc_now - out_nmea.lu_fix_nmea_rmc < 1000000 && posMode.find(out_nmea.fix_nmea_rmc) != std::string::npos) {
        switch(out_nmea.fix_nmea_rmc) {
            case 'N':
                strcpy(out_nmea.fix_nmea, "NoFix");
                m_2d_valid_fix = false;
                m_3d_valid_fix = false;
                break;
            case 'E':
                strcpy(out_nmea.fix_nmea, "Estimated/DeadReckoning");
                m_2d_valid_fix = false;
                m_3d_valid_fix = false;
                break;
            case 'A':
                strcpy(out_nmea.fix_nmea, "AutonomousGNSSFix");
                m_2d_valid_fix = true;
                m_3d_valid_fix = true;
                break;
            case 'D':
                strcpy(out_nmea.fix_nmea, "DGNSS");
                m_2d_valid_fix = true;
                m_3d_valid_fix = true;
                break;
            case 'F':
                strcpy(out_nmea.fix_nmea, "RTKFloat");
                m_2d_valid_fix = true;
                m_3d_valid_fix = true;
                break;
            case 'R':
                strcpy(out_nmea.fix_nmea, "RTKFixed");
                m_2d_valid_fix = true;
                m_3d_valid_fix = true;
                break;
            default:
                strcpy(out_nmea.fix_nmea, "Unknown/Invalid");
                m_2d_valid_fix = false;
                m_3d_valid_fix = false;
                break;
        }
    } else {
        // Otherwise, check the fix mode from GPS, Galileo, GLONASS, BeiDou (ignoring QZSS and other constellations for the time being)
        char fix_cumulative_char = '0';

        // If the first four characters are the same, take as fix mode the common one between GPS, Galileo, GLONASS, and BeiDou
        if (posMode[0] == posMode[1] && posMode[1] == posMode[2] && posMode[2] == posMode[3]) {
            fix_cumulative_char = posMode[0];
        } else {
            // Otherwise, take as reference the first non-'N' valid character between the first four, if any.
            // If no valid character is found, take as reference the GPS fix (i.e., initialize fix_cumulative_char with posMode[0])
            // TODO: for the time being, only (N,) E, A, D, F, R are supported. However, this could be extended to P, M, S even if we hardly ever encounter these modes
            fix_cumulative_char = posMode[0];
            for (int i = 0; i < 4; i++) {
                if (posMode[i] == 'E' || posMode[i] == 'A' || posMode[i] == 'D' || posMode[i] == 'F' || posMode[i] == 'R') {
                    fix_cumulative_char = posMode[i];
                    break;
                }
            }
        }

        switch(fix_cumulative_char) {
            case 'N':
                strcpy(out_nmea.fix_nmea, "NoFix");
                m_2d_valid_fix = false;
                m_3d_valid_fix = false;
                break;
            case 'E':
                strcpy(out_nmea.fix_nmea, "Estimated/DeadReckoning");
                m_2d_valid_fix = false;
                m_3d_valid_fix = false;
                break;
            case 'A':
                strcpy(out_nmea.fix_nmea, "AutonomousGNSSFix");
                m_2d_valid_fix = true;
                m_3d_valid_fix = true;
                break;
            case 'D':
                strcpy(out_nmea.fix_nmea, "DGNSS");
                m_2d_valid_fix = true;
                m_3d_valid_fix = true;
                break;
            case 'F':
                strcpy(out_nmea.fix_nmea, "RTKFloat");
                m_2d_valid_fix = true;
                m_3d_valid_fix = true;
                break;
            case 'R':
                strcpy(out_nmea.fix_nmea, "RTKFixed");
                m_2d_valid_fix = true;
                m_3d_valid_fix = true;
                break;
            default:
                strcpy(out_nmea.fix_nmea, "Unknown/Invalid");
                m_2d_valid_fix = false;
                m_3d_valid_fix = false;
                break;
        }
    }

    // Position parsing
    // If the position is not available, the NMEA sentence will contain empty fields (e.g., ",,")
    if (!slat.empty()) {
        out_nmea.lat = decimal_deg(std::stod(slat),cp_lat);
        out_nmea.lat_nmea = out_nmea.lat;
    } else {
        out_nmea.lat = Latitude_unavailable_serial_parser; // Latitude_unavailable
        out_nmea.lat_nmea = out_nmea.lat;
    }

	if (!slon.empty()) {
        out_nmea.lon = decimal_deg(std::stod(slon), cp_lon);
        out_nmea.lon_nmea = out_nmea.lon;
    } else {
        out_nmea.lon = Longitude_unavailable_serial_parser; // Longitude_unavailable
        out_nmea.lon_nmea = out_nmea.lon;
    }

	// Check if the altitude is negative and parse it accordingly using std::stod
    // TODO: check if we could avoid the if-else by giving directly the negative value as input to std::stod()
    // TODO: implement better error management in case a non-numeric string is found for the altitude field (currently, std::stod() would generate an exception)
	if (!salt.empty()) {
		if (salt[0] == '-') {
			salt = salt.erase(0,1);
			out_nmea.alt = std::stod(salt) * -1;
            out_nmea.alt_nmea = out_nmea.alt;
		}
		else {
            out_nmea.alt = std::stod(salt);
            out_nmea.alt_nmea = out_nmea.alt;
        }
	}
	else {
        out_nmea.alt = AltitudeValue_unavailable_serial_parser; // AltitudeValue_unavailable
        out_nmea.alt_nmea = out_nmea.alt;
    }

	// Produces and processes the current date-time timestamp (stores in a human-readable format when the information was parsed)
	std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	strcpy(out_nmea.ts_pos,std::ctime(&now));

	// Gets the update time with precision of microseconds
	auto update = time_point_cast<microseconds>(system_clock::now());

    // Compute the age of information
    if (m_debug_age_info_rate) {
        m_debug_age_info.age_pos = update.time_since_epoch().count() - out_nmea.lu_pos;
        m_debug_age_info.age_pos_nmea = update.time_since_epoch().count() - out_nmea.lu_pos_nmea;
        m_debug_age_info.age_alt = update.time_since_epoch().count() - out_nmea.lu_alt;
        m_debug_age_info.age_alt_nmea = update.time_since_epoch().count() - out_nmea.lu_alt_nmea;
    }

    // Converts time_point to microseconds
	out_nmea.lu_pos = update.time_since_epoch().count();
	out_nmea.lu_pos_nmea = out_nmea.lu_pos;
    out_nmea.lu_alt = update.time_since_epoch().count();
    out_nmea.lu_alt_nmea = out_nmea.lu_alt;

    // Validates data
    m_pos_valid = true;
    m_pos_valid_nmea = true;
    m_alt_valid = true;
    m_alt_valid_nmea = true;

    // Updates the buffer
	m_outBuffer.store(out_nmea);
}

/** parseNmeaGga() checks and parses a GNGGA NMEA sentence in order to get the latitude and longitude data
 *
 *  The function separates the sentence fields using the commaas and extract the data.
 *  After parsing, it produces the current information specific timestamp and updates the output buffer.
 *  For more information see page 2.7.5.1 Global Positioning System Fix Data
 *  on page 28 of ublox ZED-F9R Interface Description*/
void
UBXNMEAParserSingleThread::parseNmeaGga(std::string nmea_response) {
	out_t out_nmea = m_outBuffer.load();

    std::vector<std::string> fields;
    std::stringstream ss(nmea_response);
    std::string field;

    // Sentence separation into fields
    while (std::getline(ss, field, ',')) {
        fields.push_back(field);
    }

	// Latitude and Longitude substrings to be converted to double using std::stod() later
    std::string slat = fields[2];
    std::string slon = fields[4];
    std::string salt = fields[9];

    // cardinal points (N, S, W, E)
	char cp_lat = fields[3].at(0);
    char cp_lon = fields[5].at(0);

    // Fix value parsing and validation
    char fix = fields[6].at(0);
    if (fix == '0') {
        strcpy(out_nmea.fix_nmea, "NoFix");
        m_2d_valid_fix = false;
        m_3d_valid_fix = false;
    }
    else if (fix == '6') {
        strcpy(out_nmea.fix_nmea, "Estimated/DeadReckoning");
        m_2d_valid_fix = false;
        m_3d_valid_fix = false;
    }
    else if (fix == '1') {
        strcpy(out_nmea.fix_nmea, "AutonomousGNSSFix");
        m_2d_valid_fix = true;
        m_3d_valid_fix = true;
    }
    else if (fix == '2') {
        strcpy(out_nmea.fix_nmea,"DGNSS");
        m_2d_valid_fix = true;
        m_3d_valid_fix = true;
    }
    else if (fix == '5') {
        strcpy(out_nmea.fix_nmea, "RTKFloat");
        m_2d_valid_fix = true;
        m_3d_valid_fix = true;
    }
    else if (fix == '4') {
        strcpy(out_nmea.fix_nmea, "RTKFixed");
        m_2d_valid_fix = true;
        m_3d_valid_fix = true;
    }
    else {
        strcpy(out_nmea.fix_nmea, "Unknown/Invalid");
        m_2d_valid_fix = false;
        m_3d_valid_fix = false;
    }

    // Position data parsing
    if (!slat.empty()) {
        out_nmea.lat = decimal_deg(std::stod(slat),cp_lat);
        out_nmea.lat_nmea = out_nmea.lat;
    }
	else {
        out_nmea.lat = 900000001; // Latitude_unavailable
        out_nmea.lat_nmea = out_nmea.lat;
    }

	if (!slon.empty()) {
        out_nmea.lon = decimal_deg(std::stod(slon),cp_lon);
        out_nmea.lon_nmea = out_nmea.lon;
    }
	else {
        out_nmea.lon = 1800000001; //Longitude_unavailable
        out_nmea.lon_nmea = out_nmea.lon;
    }

	// Checks if the altitude is negative and parses accondingly using stod
	if (!salt.empty()) {
		if (salt[0] == '-') {
            salt = salt.erase(0,1);
            out_nmea.alt = std::stod(salt) * -1;
            out_nmea.alt_nmea = out_nmea.alt;
		}
		else {
            out_nmea.alt = std::stod(salt);
            out_nmea.alt_nmea = out_nmea.alt;
        }
	}
	else {
        out_nmea.alt = 800001; //AltitudeValue_unvailable
        out_nmea.alt_nmea = out_nmea.alt;
    }

	// Produces and processes the current date-time timestamp
	std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	strcpy(out_nmea.ts_pos,std::ctime(&now));

	// Gets the update time with precision of microseconds
	auto update = time_point_cast<microseconds>(system_clock::now()); // "update" is the "now" timestamp

    // Compute the period occurred between this update and the last update
    if (m_debug_age_info_rate) {
        m_debug_age_info.prd_pos = update.time_since_epoch().count() - out_nmea.lu_pos; // "lu" = "last update"
        m_debug_age_info.prd_pos_nmea = update.time_since_epoch().count() - out_nmea.lu_pos_nmea;
        m_debug_age_info.prd_alt = update.time_since_epoch().count() - out_nmea.lu_alt;
        m_debug_age_info.prd_alt_nmea = update.time_since_epoch().count() - out_nmea.lu_alt_nmea;
    }

    // Converts time_point to microseconds
	out_nmea.lu_pos = update.time_since_epoch().count(); // "lu" = "last update"
	out_nmea.lu_pos_nmea = out_nmea.lu_pos;
    out_nmea.lu_alt = update.time_since_epoch().count();
    out_nmea.lu_alt_nmea = out_nmea.lu_alt;

    // Set the flags to tell that this data is now valid and can be used
    m_pos_valid = true;
    m_pos_valid_nmea = true;
    m_alt_valid = true;
    m_alt_valid_nmea = true;

    // Updates the buffer
	m_outBuffer.store(out_nmea);
}

/** getAltitude() provides the altitude above sea level parsed from the GNS/GGA NMEA sentences and from the UBX-NAV-PVT message.
 *  The function loads the atomic data buffer then calculates and process the age of information based on user input. */
double
UBXNMEAParserSingleThread::getAltitude(long *age_us, bool print_timestamp_and_age) {
	if(m_parser_started == false) {
		std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
		return 0;
	}

    // Loads the atomic structure with the navigation/sensor data
    // This approach works only in a single-writer multiple-reader context, which is the case here
	out_t tmp = m_outBuffer.load();

    auto now = time_point_cast<microseconds>(system_clock::now());
    auto epoch_ts = now.time_since_epoch().count(); // Convert the chrono time_point to microseconds in epoch time
    long local_age_us = epoch_ts - tmp.lu_alt;

    // Stores age of information if debug option is enabled
    // TODO: check why it is stored instead of just being computed and returned
    if (m_debug_age_info_rate) m_debug_age_info.age_alt = local_age_us;

    // Checks the information validity
    if (local_age_us >= getValidityThreshold()) {
        m_alt_valid.store(false);
    }
    else m_alt_valid.store(true);

    if (print_timestamp_and_age == true) std::cout << "[Altitude timestamp] - " <<  tmp.ts_alt
                                                   << "Age of information: " << local_age_us << " us"
                                                   << std::endl << std::endl;
    if (age_us != nullptr) *age_us = local_age_us;

	return tmp.alt;
}

/** getAltitudeUbx() provides the latest altitude value parsed from the UBX-NAV-PVT message
 * This function behaves like getAltitude() but considers only the data coming from UBX messages.
 */
double
UBXNMEAParserSingleThread::getAltitudeUbx(long *age_us, bool print_timestamp_and_age) {
    if(m_parser_started == false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return 0;
    }

    out_t tmp = m_outBuffer.load();

    auto now = time_point_cast<microseconds>(system_clock::now());
    auto epoch_ts = now.time_since_epoch().count(); // Convert the chrono time_point to microseconds in epoch time
    long local_age_us = epoch_ts - tmp.lu_alt_ubx;

    if (m_debug_age_info_rate) m_debug_age_info.age_alt_ubx = local_age_us;

    if (local_age_us >= getValidityThreshold()) {
        m_alt_valid.store(false);
    }
    else m_alt_valid.store(true);

    if (print_timestamp_and_age == true) std::cout << "[Altitude(UBX) timestamp] - " <<  tmp.ts_alt
                                                   << "Age of information: " << local_age_us << " us"
                                                   << std::endl << std::endl;
    if (age_us != nullptr) *age_us = local_age_us;

    return tmp.alt_ubx;
}

/** getAltitudeNmea() provides the latest altitude NMEA value parsed from the GNS/GGA sentences
 * This function behaves like getAltitude() but considers only the data coming from NMEA messages.
 */
double
UBXNMEAParserSingleThread::getAltitudeNmea(long *age_us, bool print_timestamp_and_age) {
    if(m_parser_started == false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return 0;
    }

    out_t tmp = m_outBuffer.load();

    auto now = time_point_cast<microseconds>(system_clock::now());
    auto epoch_ts = now.time_since_epoch().count(); // Convert the chrono time_point to microseconds in epoch time
    long local_age_us = epoch_ts - tmp.lu_alt_nmea;

    if (m_debug_age_info_rate) m_debug_age_info.age_alt_nmea = local_age_us;

    if (local_age_us >= getValidityThreshold()) {
        m_alt_valid.store(false);
    }
    else m_alt_valid.store(true);

    if (print_timestamp_and_age == true) std::cout << "[Altitude(NMEA) timestamp] - " <<  tmp.ts_alt
                                                   << "Age of information: " << local_age_us << " us"
                                                   << std::endl << std::endl;
    if (age_us != nullptr) *age_us = local_age_us;

    return tmp.alt_nmea;
}

/** getAccelerations() provides the compensated accelerations (without the gravity acceleration) and
 * follows the same structure as getPosition() but with the use of std::tuple instead of std::pair
 * std::tuple is used as the function returns the acceleration along (x,y,z), considering the reference system of
 * the GNSS(+IMU) device
 * 
 *  Note: use std::get<index>(object) to access the elements of a tuple. */
std::tuple<double,double,double>
UBXNMEAParserSingleThread::getAccelerations(long *age_us, bool print_timestamp_and_age) {
    if(m_parser_started == false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return std::make_tuple(0,0,0);
    }

    // Loads the atomic structure with the navigation/sensor data
    // This approach works only in a single-writer multiple-reader context, which is the case here
	out_t tmp = m_outBuffer.load();

	auto now = time_point_cast<microseconds>(system_clock::now());
	auto epoch_ts = now.time_since_epoch().count(); // Convert the chrono time_point to microseconds in epoch time
    long local_age_us = epoch_ts - tmp.lu_comp_acc;

    // Stores age of information if the --show-live-data option is enabled
    if (m_debug_age_info_rate) m_debug_age_info.age_comp_acc = local_age_us;

    // Check the information validity
    if (local_age_us >= getValidityThreshold()) {
        m_comp_acc_valid.store(false);
    }
    else {
        m_comp_acc_valid.store(true);
    }

    if (print_timestamp_and_age == true) std::cout << "[Accelerations] - " <<  tmp.ts_comp_acc
                                                   << "Age of information: " << local_age_us << " us"
                                                   << std::endl << std::endl;

    if (age_us != nullptr) *age_us = local_age_us;

	return std::make_tuple(tmp.comp_acc_x,tmp.comp_acc_y,tmp.comp_acc_z);
}

/** getAngularRate() provide the angular accelerations from the three x, y and z axes.
 * The yaw rate will be the angular acceleration along the z axis.
*/
std::tuple<double,double,double>
UBXNMEAParserSingleThread::getAngularRates(long *age_us, bool print_timestamp_and_age) {
    if(m_parser_started == false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return std::make_tuple(0,0,0);
    }

    // Loads the atomic structure with the navigation/sensor data
    out_t tmp = m_outBuffer.load();

    auto now = time_point_cast<microseconds>(system_clock::now());
    auto epoch_ts = now.time_since_epoch().count(); // Convert the chrono time_point to microseconds in epoch time
    long local_age_us = epoch_ts - tmp.lu_comp_ang_rate;

    // Stores age of information if the --show-live-data option is enabled
    if (m_debug_age_info_rate) m_debug_age_info.age_comp_ang_rate = local_age_us;

    // Check the information validity
    if (local_age_us >= getValidityThreshold()) {
        m_comp_ang_rate_valid.store(false);
    }
    else {
        m_comp_ang_rate_valid.store(true);
    }

    if (print_timestamp_and_age == true) std::cout << "[Angular Rates] - " <<  tmp.ts_comp_ang_rate
                                                   << "Age of information: " << local_age_us << " us"
                                                   << std::endl << std::endl;

    // Store the age of information into age_us, if it was specified as a non-null pointer
    if (age_us != nullptr) *age_us = local_age_us;

    return std::make_tuple(tmp.comp_ang_rate_x,tmp.comp_ang_rate_y,tmp.comp_ang_rate_z);
}

/** getYawRate() provides the z axis angular acceleration value, without the gravity acceleration.
 * It calls getAngularRates and returns the z component.
*/
double
UBXNMEAParserSingleThread::getYawRate(long *age_us, bool print_timestamp_and_age) {
    if(m_parser_started == false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return 0;
    }

    return std::get<2>(getAngularRates(age_us, print_timestamp_and_age));
}

/** getLongitudinalAcceleration() provides the x axis longitudinal acceleration value.
 * It calls getAccelerations() and returns the x component.
*/
double
UBXNMEAParserSingleThread::getLongitudinalAcceleration(long *age_us, bool print_timestamp_and_age) {
    if(m_parser_started == false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return 0;
    }

    return std::get<0>(getAccelerations(age_us, print_timestamp_and_age));
}

/** getRawAccelerations() provide the three axes acceleration with the gravity acceleration included.
 * Therefore, on a flat surface a z component around 9.8 m/s^2 is expected.
*/
std::tuple<double,double,double>
UBXNMEAParserSingleThread::getRawAccelerations(long *age_us, bool print_timestamp_and_age) {
    if(m_parser_started==false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return std::make_tuple(0,0,0);
    }

    // Loads the atomic structure with the navigation/sensor data
    // This approach works only in a single-writer multiple-reader context, which is the case here
	out_t tmp = m_outBuffer.load();

    auto now = time_point_cast<microseconds>(system_clock::now());
    auto epoch_ts = now.time_since_epoch().count(); // Convert the chrono time_point to microseconds in epoch time
    long local_age_us = epoch_ts - tmp.lu_acc;

    // Stores age of information if the --show-live-data option is enabled
    if (m_debug_age_info_rate) m_debug_age_info.age_acc = local_age_us;

    // Check the information validity
    if (local_age_us >= getValidityThreshold()) {
        m_acc_valid.store(false);
    }
    else {
        m_acc_valid.store(true);
    }

    if (print_timestamp_and_age == true) std::cout << "[Raw Accelerations] - " <<  tmp.ts_acc
                                                   << "Age of information: " << local_age_us << " us"
                                                   << std::endl << std::endl;

    // Store the age of information into age_us, if it was specified as a non-null pointer
    if (age_us != nullptr) *age_us = local_age_us;

	return std::make_tuple(tmp.raw_acc_x,tmp.raw_acc_y,tmp.raw_acc_z);
}

/** parseEsfRaw() parses a UBX-ESF-RAW message which payload is composed by 4 reserved bytes
 *  plus a repeated group of 3x 8 bytes whose first 4 bytes in each group contain the relevant information
 *  about the acceleration in the x, y and z axes.
 *
 *  The function reads the first 4 bytes of the first 3 repeated group
 *  (3 bytes for acceleration value + 1 byte to identify the axis)
 *  but it can be extended in order to read also the gyroscope data
 *  and to include the sTag field too (see the for loop below).
 *
 *  In general, each group of 8 bytes is composed of 4 bytes with the current value, and 4 bytes for an unisgned
 *  sTag (sensor time tag), not parsed here; the 4 butes with the current value are in turn divided into 3 data bytes
 *  and a fourth data type byte (index 3 in esf_data when parsing), that indicated in this case the axis allong which
 *  the raw acceleration is measured.
 *
 *  (more details in the Interface description document of the ZED-F9R Module, 3.11.4 UBX-ESF-RAW ) */
void
UBXNMEAParserSingleThread::parseEsfRaw(std::vector<uint8_t> response) {
	// Skips the first 4 bytes of the payload (reserved)
    int offset = m_UBX_PAYLOAD_OFFSET + 4;

    int j = 0; // This index is reset every 8 bytes to isolate the repeated group
    std::vector<uint8_t> esf_data; // This "byte group vector" is used to temporarily stores each group of 3 bytes of interest

    // Loads the atomic structure with the navigation/sensor data
    // This approach works only in a single-writer multiple-reader context, which is the case here
    out_t out_esf = m_outBuffer.load();

    // TODO: extend this function to parse gyroscope data and sTag field
    for (int i = offset; i < offset + 24; i++) { // For the time being, ignores gyroscope data on the payload
    	esf_data.push_back(response[i]); j++; // Stores the ESF-RAW current byte

    	if (j == 8) {
            // After reading 8 bytes, a group has been fully read --> perform parsing

            /* Stops every 8 bytes to check and parse the data type
             * (16 (0x10),17 (0x11) or 18 (0x12) for XYZ acceleration values)*/
            if (esf_data[3] == 0x10) {
            	// Erases the unneeded bytes (byte 3 to identify the data type and bytes 4,5,6 and 7 (sTag field))
            	esf_data.erase(esf_data.begin()+3,esf_data.end());

            	// Converts the resulting array of 3 bytes to big endian by reversing it
                // TODO: we suppose here that OScar is run on a little-endian architecture; could be extended to support also execution on big-endian architectures (where reverse would not be needed)
            	std::reverse(esf_data.begin(),esf_data.end());

            	/* Produces the final signed value and scales it, dividing by 2^10 = 1024
            	 * as indicated in 3.2.7.5 Sensor data types (Integration manual) */
            	out_esf.raw_acc_x = static_cast<double>(hexToSigned(esf_data))/1024;

				esf_data.clear(); // Clears the current byte group vector for the next iteration
				j = 0;
			} else if (esf_data[3] == 0x11) {
                // TODO: improve code by writing esf_data.erase() and std::reverse() once for all cases, as these operations are in common to all the considered cases
            	esf_data.erase(esf_data.begin()+3,esf_data.end());
            	std::reverse(esf_data.begin(),esf_data.end());
            	out_esf.raw_acc_y = static_cast<double>(hexToSigned(esf_data))/1024;
            	esf_data.clear();
				j = 0;
			} else if (esf_data[3] == 0x12) {
            	esf_data.erase(esf_data.begin()+3,esf_data.end());
            	std::reverse(esf_data.begin(),esf_data.end());
            	out_esf.raw_acc_z = static_cast<double>(hexToSigned(esf_data))/1024;
            	esf_data.clear();
				j = 0;
			}
        }
    }

    // Produces and processes the current date-time timestamp (stores in a human-readable format when the information was parsed)
	std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	strcpy(out_esf.ts_acc,std::ctime(&now));

	// Gets the current update time with precision of microseconds
	auto update = time_point_cast<microseconds>(system_clock::now());

    // Compute the period occurred between this update and the last update
    if (m_debug_age_info_rate) {
        m_debug_age_info.prd_acc = update.time_since_epoch().count() - out_esf.lu_acc;
    }

    // Store the timestamp when this information was last retrieved
	out_esf.lu_acc = update.time_since_epoch().count();

    // Validates data
    m_acc_valid = true;

    // Updates the buffer
    m_outBuffer.store(out_esf);
}

/** getAttitude() provides the roll, pitch and raw angles extracted from the UBX-NAV-ATT Messages (see parseNavAtt()) */
std::tuple<double,double,double>
UBXNMEAParserSingleThread::getAttitude(long *age_us, bool print_timestamp_and_age) {
    if(m_parser_started==false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return std::make_tuple(0,0,0);
    }

	out_t tmp = m_outBuffer.load();

    auto now = time_point_cast<microseconds>(system_clock::now());
    auto end = now.time_since_epoch().count();
    long local_age_us = end - tmp.lu_att;

    if (m_debug_age_info_rate) m_debug_age_info.age_att = local_age_us;

    if (local_age_us >= getValidityThreshold()) {
        m_att_valid.store(false);
    }
    else m_att_valid.store(true);

    if (print_timestamp_and_age == true) std::cout << "[Attitude] - " <<  tmp.ts_att
                                                   << "Age of information: " << local_age_us << " us"
                                                   << std::endl << std::endl;
    if (age_us != nullptr) *age_us = local_age_us;

	return std::make_tuple(tmp.roll,tmp.pitch,tmp.heading);
}

/** Parses a UBX-NAV-ATT message, extrapolating precisely 12 bytes in group of 4
 *  that contain the three attitude angles of interest, operating from byte 14 to byte 25
 *  of the whole message, or from byte 8 to byte 19 of the payload.
 *
 *  (more details in 3.15.1 Interface description: UBX-NAV-ATT)
 *
 *  The function behaves similarly to parseEsfRaw(), using a temporary array
 *  to store, convert and scale the values. */
void
UBXNMEAParserSingleThread::parseNavAtt(std::vector<uint8_t> response) {
	const int ATT_DATA_START_INDEX = 8;
	const int ATT_DATA_END_INDEX = 20 + m_UBX_PAYLOAD_OFFSET;
	int offset = m_UBX_PAYLOAD_OFFSET + ATT_DATA_START_INDEX;
	std::vector<uint8_t> att_data;

	out_t out_att = m_outBuffer.load();

	// Reading 4*3 bytes from byte 8 to byte 19 of the payload
	for(int i = offset; i <= ATT_DATA_END_INDEX; i++) {
		if (i == 18) {

			// Converts to big endian
			std::reverse(att_data.begin(),att_data.end());

			// Converts to signed value and scales accordingly
			out_att.roll = static_cast<double>(hexToSigned(att_data)) * 0.00001;

			att_data.clear();
		}
		if (i == 22) {
			std::reverse(att_data.begin(),att_data.end());
			out_att.pitch = static_cast<double>(hexToSigned(att_data)) * 0.00001;
			att_data.clear();
		}
		if (i == 26) {
			std::reverse(att_data.begin(),att_data.end());
			out_att.heading = static_cast<double>(hexToSigned(att_data)) * 0.00001;
			att_data.clear();
			break;
		}
		att_data.push_back(response[i]);
	}

	// Produces and prints to struct the current date-time timestamp
	std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	strcpy(out_att.ts_att,std::ctime(&now));

	// Gets the update time with precision of microseconds
	auto update = time_point_cast<microseconds>(system_clock::now());

    // Updates age of information
    if (m_debug_age_info_rate) {
        m_debug_age_info.age_att = update.time_since_epoch().count() - out_att.lu_att;
    }

    // Retrieves the time since epoch in microseconds and updates the output struct
	out_att.lu_att = update.time_since_epoch().count();

    // Validates data
    m_att_valid = true;

    // Updates buffer
	m_outBuffer.store(out_att);
}

/** getSpeed() provides the latest speed value correctly parsed and validated. */
double
UBXNMEAParserSingleThread::getSpeed(long *age_us, bool print_timestamp_and_age) {
    if(m_parser_started==false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return 0;
    }

    out_t tmp = m_outBuffer.load();

    auto now = time_point_cast<microseconds>(system_clock::now());
    auto end = now.time_since_epoch().count();
    long local_age_us = end - tmp.lu_sog_cog;

    if (m_debug_age_info_rate) {
        m_debug_age_info.age_sog_cog = local_age_us;
    }

    if (local_age_us >= getValidityThreshold()) {
        m_sog_valid.store(false);
    }
    else m_sog_valid.store(true);

    if (print_timestamp_and_age == true) std::cout << "[Speed timestamp] - " <<  tmp.ts_sog_cog_nmea
                                                   << "Age of information: " << local_age_us << " us"
                                                   << std::endl << std::endl;
    if (age_us != nullptr) *age_us = local_age_us;

    return tmp.sog;
}

/** getSpeedUbx() provide the speed over ground value obtained from UBX-NAV-PVT message. */
double
UBXNMEAParserSingleThread::getSpeedUbx(long *age_us, bool print_timestamp_and_age) {
    if(m_parser_started==false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return 0;
    }

	out_t tmp = m_outBuffer.load();

    auto now = time_point_cast<microseconds>(system_clock::now());
    auto end = now.time_since_epoch().count();
    long local_age_us = end - tmp.lu_sog_cog_ubx;

    if (m_debug_age_info_rate) {
        m_debug_age_info.age_sog_cog = local_age_us;
        m_debug_age_info.age_sog_cog_ubx = local_age_us;
    }


    if (local_age_us >= getValidityThreshold()) {
        m_sog_valid_ubx.store(false);
    }
    else m_sog_valid_ubx.store(true);

    if (print_timestamp_and_age == true) std::cout << "[Speed timestamp - UBX] - " <<  tmp.ts_sog_cog_ubx
                                                   << "Age of information: " << local_age_us << " us"
                                                   << std::endl << std::endl;

    if (age_us != nullptr) *age_us = local_age_us;

    return tmp.sog_ubx;
}

/** getSpeedNmea() provide the speed over ground value obtained from the GXRMC sentence. */
double
UBXNMEAParserSingleThread::getSpeedNmea(long *age_us, bool print_timestamp_and_age) {
    if(m_parser_started==false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return 0;
    }

    out_t tmp = m_outBuffer.load();

    auto now = time_point_cast<microseconds>(system_clock::now());
    auto end = now.time_since_epoch().count();
    long local_age_us = end - tmp.lu_sog_cog_nmea;

    if (m_debug_age_info_rate) {
        m_debug_age_info.age_sog_cog = local_age_us;
        m_debug_age_info.age_sog_cog_nmea = local_age_us;
    }

    if (local_age_us >= getValidityThreshold()) {
        m_sog_valid_nmea.store(false);
    }
    else m_sog_valid_nmea.store(true);

    if (print_timestamp_and_age == true) std::cout << "[Speed timestamp - NMEA] - " <<  tmp.ts_sog_cog_nmea
                                                   << "Age of information: " << local_age_us << " us"
                                                   << std::endl << std::endl;
    if (age_us != nullptr) *age_us = local_age_us;

    return tmp.sog_nmea;
}

/** getCourseOverGroundNmea() provides the COG value obtained from the GXRMC sentence */
double
UBXNMEAParserSingleThread::getCourseOverGround(long *age_us, bool print_timestamp_and_age) {
    if(m_parser_started == false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return 0;
    }

    out_t tmp = m_outBuffer.load();
    auto now = time_point_cast<microseconds>(system_clock::now());
    auto end = now.time_since_epoch().count();
    long local_age_us = end - tmp.lu_sog_cog;

    if (m_debug_age_info_rate) {
        m_debug_age_info.age_sog_cog = local_age_us;
    }

    if (local_age_us >= getValidityThreshold()) {
        m_cog_valid.store(false);
    }
    else m_cog_valid.store(true);

    if (print_timestamp_and_age == true) std::cout << "[Course Over Ground timestamp] - " <<  tmp.ts_sog_cog_nmea
                                                   << "Age of information: " << local_age_us << " us"
                                                   << std::endl << std::endl;
    if (age_us != nullptr) *age_us = local_age_us;

    return tmp.cog;
}

/** getCourseOverGroundUbx() provides the COG value obtained from the UBX-NAV-PVT message */
double
UBXNMEAParserSingleThread::getCourseOverGroundUbx(long *age_us, bool print_timestamp_and_age) {
    if(m_parser_started == false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return 0;
    }

	out_t tmp = m_outBuffer.load();

    auto now = time_point_cast<microseconds>(system_clock::now());
    auto end = now.time_since_epoch().count();
    long local_age_us = end - tmp.lu_sog_cog_ubx;

    if (m_debug_age_info_rate) {
        m_debug_age_info.age_sog_cog = local_age_us;
        m_debug_age_info.age_sog_cog_ubx = local_age_us;
    }

    if (local_age_us >= getValidityThreshold()) {
        m_cog_valid_ubx.store(false);
    }
    else m_cog_valid_ubx.store(true);

    if (print_timestamp_and_age == true) std::cout << "[Course Over Ground timestamp - UBX] - " <<  tmp.ts_sog_cog_ubx
                                                   << "Age of information: " << local_age_us << " us"
                                                   << std::endl << std::endl;

    if (age_us != nullptr) *age_us = local_age_us;

	return tmp.cog_ubx;
}

/** getCourseOverGroundNmea() provides the COG value obtained from the GXRMC sentence */
double
UBXNMEAParserSingleThread::getCourseOverGroundNmea(long *age_us, bool print_timestamp_and_age) {
    if(m_parser_started == false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return 0;
    }

    out_t tmp = m_outBuffer.load();
    auto now = time_point_cast<microseconds>(system_clock::now());
    auto end = now.time_since_epoch().count();
    long local_age_us = end - tmp.lu_sog_cog_nmea;

    if (m_debug_age_info_rate) {
        m_debug_age_info.age_sog_cog = local_age_us;
        m_debug_age_info.age_sog_cog_nmea = local_age_us;
    }

    if (local_age_us >= getValidityThreshold()) {
        m_cog_valid_nmea.store(false);
    }
    else m_cog_valid_nmea.store(true);

    if (print_timestamp_and_age == true) std::cout << "[Course Over Ground timestamp - NMEA] - " <<  tmp.ts_sog_cog_nmea
                                                   << "Age of information: " << local_age_us << " us"
                                                   << std::endl << std::endl;
    if (age_us != nullptr) *age_us = local_age_us;

    return tmp.cog_nmea;
}

/** getFixMode() retrieves the fix mode from both UBX and NMEA (checking in this order)
 *  see parseNavStatus() and parseNmeaRmc() for details. */
std::string
UBXNMEAParserSingleThread::getFixMode() {
    if(m_parser_started==false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return "PARSER_NOT_STARTED";
    }

	out_t tmp = m_outBuffer.load();

    std::string fix_ubx(tmp.fix_ubx);

	// Checks if the fix mode has been already obtained from UBX
	if (fix_ubx.empty() == true) {
        std::string fix_nmea(tmp.fix_nmea);
        if (fix_nmea.empty()) fix_nmea = "Unavailable";
        return fix_nmea;
	}
	return fix_ubx;
}

/** getFixModeUbx() provides the latest fix value obtained from the UBX protocol (UBX-NAV-STATUS message) */
std::string
UBXNMEAParserSingleThread::getFixModeUbx() {
    if(m_parser_started==false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return "PARSER_NOT_STARTED";
    }

    out_t tmp = m_outBuffer.load();
    std::string fix_ubx(tmp.fix_ubx);
    if (fix_ubx.empty()) fix_ubx = "Unavailable";

    return fix_ubx;
}

/** getFixModeNmea() provides the latest fix value obtained from the NMEA protocol (GXRMC, GXGGA, GXGNS sentences) */
std::string
UBXNMEAParserSingleThread::getFixModeNmea() {
    if(m_parser_started==false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return "PARSER_NOT_STARTED";
    }

    out_t tmp = m_outBuffer.load();
    std::string fix_nmea(tmp.fix_nmea);
    if (fix_nmea.empty()) fix_nmea = "Unavailable";

    return fix_nmea;
}

/** getFixValidity2D() verifies if at least a 2D fix is present by checking the validity flags set by the parsing functions */
bool
UBXNMEAParserSingleThread::getFixValidity2D(bool print_error) {
    if (m_2d_valid_fix == false && print_error == true) {
        std::cerr << "No 2D Fix!" << std::endl;
        return false;
    }
    else return m_2d_valid_fix;
}

/** getFixValidity3D() verifies if at least a 3D fix is present by checking the validity flags set by the parsing functions */
bool
UBXNMEAParserSingleThread::getFixValidity3D(bool print_error) {
    if (m_3d_valid_fix == false && print_error == true) {
        std::cerr << "No 3D Fix!" << std::endl;
        return false;
    }
    else return m_3d_valid_fix;
}

/** getPositionValidity() verifies if the position value's age of information
 * is inside the acceptable validity range set by the user.
 *
 * Return values: 0 invalid - 1 UBX valid - 2 NMEA valid - 3 UBX/NMEA valid*/
std::atomic<int>
UBXNMEAParserSingleThread::getPositionValidity(bool print_error) {
    if (m_pos_valid == false && print_error == true) {
        std::cerr << "Error: Outdated position value (age > threshold)!" << std::endl;
        return 0;
    }
    if (m_pos_valid_ubx && !m_pos_valid_nmea) {
        return 1;
    }
    if (m_pos_valid_nmea && !m_pos_valid_ubx){
        return 2;
    }
    if (m_pos_valid_ubx && m_pos_valid_nmea) {
        return 3;
    }
    return 0;
}

bool
UBXNMEAParserSingleThread::getRawAccelerationsValidity(bool print_error) {
    if (m_acc_valid == false && print_error == true) {
        std::cerr << "Error: Outdated raw accelerations value (age > threshold)!" << std::endl;
        return false;
    } else return m_acc_valid;
}

bool
UBXNMEAParserSingleThread::getAttitudeValidity(bool print_error) {
    if (m_att_valid == false && print_error == true) {
        std::cerr << "Error: Outdated attitude value (age > threshold)!" << std::endl;
        return false;
    } else return m_att_valid;
}

bool
UBXNMEAParserSingleThread::getAccelerationsValidity(bool print_error) {
    if (m_comp_acc_valid == false && print_error == true) {
        std::cerr << "Error: Outdated acceleration value (age > threshold)!" << std::endl;
        return false;
    } else return m_comp_acc_valid;
}

bool
UBXNMEAParserSingleThread::getAltitudeValidity(bool print_error) {
    if (m_alt_valid == false && print_error == true) {
        std::cerr << "Error: Outdated altitude value (age > threshold)!" << std::endl;
        return false;
    } else return m_alt_valid;
}

bool
UBXNMEAParserSingleThread::getYawRateValidity(bool print_error) {
    if (m_comp_ang_rate_valid == false && print_error == true) {
        std::cerr << "Error: Outdated yaw rate value (age > threshold)!" << std::endl;
        return false;
    } else return m_comp_ang_rate_valid;
}

/** getSpeedValidity() verifies if the speed value's age of information
 * is inside the acceptable validity range set by the user.
 *
 * Return values: 0 invalid - 1 UBX valid - 2 NMEA valid - 3 UBX/NMEA valid */
int
UBXNMEAParserSingleThread::getSpeedValidity(bool print_error) {
    if (m_sog_valid == false && print_error == true) {
        std::cerr << "Error: Outdated UBX speed/course over ground value (age > threshold)!" << std::endl;
        return 0;
    }
    if (m_sog_valid_ubx && !m_sog_valid_nmea) {
        return 1;
    }
    if (m_sog_valid_nmea && !m_sog_valid_ubx) {
        return 2;
    }
    if (m_sog_valid_ubx && m_sog_valid_nmea) {
        return 3;
    }
    return 0;
}

/** getCourseOverGroundValidity() verifies if the course over ground value's age of information
 * is inside the acceptable validity range set by the user.
 *
 * Return values: 0 invalid - 1 UBX valid - 2 NMEA valid - 3 UBX/NMEA valid */
int
UBXNMEAParserSingleThread::getCourseOverGroundValidity(bool print_error) {
    if (m_cog_valid == false && print_error == true) {
        std::cerr << "Error: Outdated UBX speed/course over ground value (age > threshold)!" << std::endl;
        return 0;
    }
    if (m_cog_valid_ubx && !m_cog_valid_nmea) {
        return 1;
    }
    if (m_cog_valid_nmea && !m_cog_valid_ubx) {
        return 2;
    }
    if (m_cog_valid_ubx && m_cog_valid_nmea) {
        return 3;
    }
    return 0;
}

/** setValidityThreshold() sets the threshold value given by the user. */
bool
UBXNMEAParserSingleThread::setValidityThreshold(double threshold) {
    if (m_validity_threshold != 0) return true; // The threshold has already been set
    else {
        m_validity_threshold = threshold;
        return true;
    }
}

/** setDebugAgeInfo() sets the refresh rate of infomration printing when the debug option is enabled */
void
UBXNMEAParserSingleThread::setDebugAgeInfo(int rate) {
    if (m_debug_age_info_rate) std::cerr << "Error: debug option already enabled!" << '\n';
    else m_debug_age_info_rate = rate;
}

bool
UBXNMEAParserSingleThread::getDebugAgeInfo() {
    return m_debug_age_info_rate;
}

/** showDebugAgeInfo() displays on screen the debug information, refreshing at the user specified rate */
// TODO: make this function capture SIGINT in an intelligent way, so that "\033[?25h" is not skipped and the cursor is shown again
void UBXNMEAParserSingleThread::showDebugAgeInfo() {
    int line_counter = 0;

    if (!m_debug_age_info_rate) {
        std::cerr << "Error: age of information debug option disabled!" << '\n';
        return;
    }

    std::cout << "\033[?25l";  // Hide the cursor

    // Wait for OScar to be fully running
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    while (!m_terminatorFlagPtr->load()) { // Replace with a suitable termination condition
        out_t buf = m_outBuffer.load();

        // Clear previous output by moving up by the current line count
        for (int i = 0; i < line_counter; ++i) {
            std::cout << "\033[F\033[2K"; // Move up and clear line
        }

        std::cout << "\nDebug Information\n\n";
        line_counter = 3;

        std::cout << std::fixed << std::setprecision(12);
        std::cout << "Lat-Lon:                " << buf.lat << " | " << buf.lon << "\tAge[us]: " << m_debug_age_info.age_pos << '\n';
        std::cout << "Lat-Lon(UBX):           " << buf.lat_ubx << " | " << buf.lon_ubx << "\tAge[us]: " << m_debug_age_info.age_pos << '\n';
        std::cout << "Lat-Lon(NMEA):          " << buf.lat_nmea << " | " << buf.lon_nmea << "\tAge[us]: " << m_debug_age_info.age_pos << '\n' << '\n';
        line_counter += 4;

        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Speed and Heading:       " << buf.sog << "[m/s]" << " | " << buf.cog <<"[deg]" << "\t Age[us]: " << m_debug_age_info.age_sog_cog << '\n';
        std::cout << "Speed and Heading(UBX):  " << buf.sog_ubx << "[m/s]" << " | " << buf.cog_ubx <<"[deg]" << "\t Age[us]: " << m_debug_age_info.age_sog_cog << '\n';
        std::cout << "Speed and Heading(NMEA): " << buf.sog_nmea << "[m/s]" << " | " << buf.cog_nmea <<"[deg]" << "\t Age[us]: " << m_debug_age_info.age_sog_cog << '\n' << '\n';
        line_counter += 4;

        std::cout << "Longitudinal acc:        " << buf.comp_acc_x << " [m/s^2]" << "\tAge[us]: " << m_debug_age_info.age_comp_acc << '\n';
        std::cout << "Other accelerations:     " << "Y: " << buf.comp_acc_y << " | " << "Z: " << buf.comp_acc_z << " [m/s^2]" << "\tAge[us]: " << m_debug_age_info.age_comp_acc << '\n';
        std::cout << "Raw Accelerations:       " << "X: " << buf.raw_acc_x << " | " << "Y: " << buf.raw_acc_y << " | " << "Z: " << buf.raw_acc_z << " [m/s^2]" << "\tAge[us]: " << m_debug_age_info.age_acc << '\n' << '\n';
        line_counter += 4;

        std::cout << "Roll Pitch Yaw:          " << buf.roll << " | " << buf.pitch << " | " << buf.heading << " [deg]" << "\tAge[us]: " << m_debug_age_info.age_att << '\n';
        std::cout << "Yaw rate:                " << buf.comp_ang_rate_z << " [deg/s]" "\tAge[us]: " << m_debug_age_info.age_comp_ang_rate << '\n' << '\n';
        line_counter += 3;

        std::cout << "Altitude:                " << buf.alt << " [m]" << "\tAge[us]: " << m_debug_age_info.age_alt << '\n';
        std::cout << "Altitude(UBX):           " << buf.alt_ubx << " [m]" << "\tAge[us]: " << m_debug_age_info.age_alt << '\n';
        std::cout << "Altitude(NMEA):          " << buf.alt_nmea << " [m]" << "\tAge[us]: " << m_debug_age_info.age_alt << '\n';
        line_counter += 3;

        std::this_thread::sleep_for(std::chrono::milliseconds(m_debug_age_info_rate));
    }

    std::cout << "\033[?25h";  // Show the cursor again
}

double
UBXNMEAParserSingleThread::getValidityThreshold() {
    if (m_validity_threshold != 0) return m_validity_threshold * 1e6; // Microseconds
    else {
        std::cerr << "Error: data validity threshold has not been set!" << std::endl;
        return -1;
    }
}

/** parseNavPvt() extracts the speed over ground (sog, 4 bytes) and course over ground (cog, 4 bytes) values from the
 *  UBX-NAV-PVT message by reading from byte 60 to byte 68 (the two values are store one after the other)
 *
 *  See Interface description § 3.15.13 UBX-NAV-PVT. (page 100) */
void
UBXNMEAParserSingleThread::parseNavPvt(std::vector<uint8_t> response) {

	out_t out_pvt = m_outBuffer.load();

	/* Extract the 4 bytes arrays containing the speed over ground and the heading of motion in the UBX-NAV-PVT message
	 * Converts the arrays in little endian and retrieves the single signed value after scaling accordingly */

    // When we extract any value that is on more than one byte, we need to std::reverse as the data comes in big endian (network byte order)
    // and we assume that OScar is run on a little endian architecture
    // TODO: support both little endian and big endian architecture
    // Tip: this code can be used to check the current endianness of the system (remember that UBX messages are always big endian - network byte order)
    /* int num = 1;
    if(*(char *)&num == 1)
    {
        printf("\nLittle-Endian\n");
    } else {
        printf("Big-Endian\n");
    } */

    // Extract the speed (byte 60)
	std::vector<uint8_t> sog(response.begin() + m_UBX_PAYLOAD_OFFSET + 60, response.begin() + m_UBX_PAYLOAD_OFFSET + 64);
	std::reverse(sog.begin(),sog.end());
	out_pvt.sog = static_cast<double>(hexToSigned(sog)) * 0.001; // Converts from mm/s to m/s
    out_pvt.sog_ubx = out_pvt.sog;

    // Extract the course over ground (byte 64)
	std::vector<uint8_t> head_motion(response.begin() + m_UBX_PAYLOAD_OFFSET + 64, response.begin() + m_UBX_PAYLOAD_OFFSET + 68);
	std::reverse(head_motion.begin(),head_motion.end());
	out_pvt.cog = static_cast<double>(hexToSigned(head_motion)) * 0.00001;
    out_pvt.cog_ubx = out_pvt.cog;

    // Extract the latitude (byte 28)
    std::vector<uint8_t> lat(response.begin() + m_UBX_PAYLOAD_OFFSET + 28, response.begin() + m_UBX_PAYLOAD_OFFSET + 32);
    std::reverse(lat.begin(),lat.end());
    out_pvt.lat = static_cast<double>(hexToSigned(lat)) * 0.0000001;
    out_pvt.lat_ubx = out_pvt.lat;

    // Extract the longitude (byte 24)
    std::vector<uint8_t> lon(response.begin() + m_UBX_PAYLOAD_OFFSET + 24, response.begin() + m_UBX_PAYLOAD_OFFSET + 28);
    std::reverse(lon.begin(),lon.end());
    out_pvt.lon = static_cast<double>(hexToSigned(lon)) * 0.0000001;
    out_pvt.lon_ubx = out_pvt.lon;

    // Extract the altitude (byte 36)
    std::vector<uint8_t> alt(response.begin() + m_UBX_PAYLOAD_OFFSET + 36, response.begin() + m_UBX_PAYLOAD_OFFSET + 40);
    std::reverse(alt.begin(),alt.end());
    out_pvt.alt = static_cast<double>(hexToSigned(alt)) * 0.001; // Convert from mm to m
    out_pvt.alt_ubx = out_pvt.alt;

    // Extract the fix (byte 20-21)
    uint8_t fix_mode  = response[m_UBX_PAYLOAD_OFFSET + 20];
    uint8_t fix_flags = response[m_UBX_PAYLOAD_OFFSET + 21];

    // Verify fix validity (check if bit 0 in fix_flags is set to 1)
    // If it is set to 0, the fix is invalid
    if (!(fix_flags & 0x01)) {
        printf("Invalid fix: %02X\n",fix_flags);
        strcpy(out_pvt.fix_ubx, "Invalid");
        m_2d_valid_fix = false;
        m_3d_valid_fix = false;
        m_outBuffer.store(out_pvt);
        return;
    }

    if (fix_mode == 0x00) {
        strcpy(out_pvt.fix_ubx, "NoFix");
        m_2d_valid_fix = false;
        m_3d_valid_fix = false;
    }
    else if (fix_mode == 0x01) {
        strcpy(out_pvt.fix_ubx, "Estimated/DeadReckoning");
        m_2d_valid_fix = true;
        m_3d_valid_fix = true;
    }
    else if (fix_mode == 0x02) {
        strcpy(out_pvt.fix_ubx, "2D-Fix");
        m_2d_valid_fix = true;
        m_3d_valid_fix = false;
    }
    else if (fix_mode == 0x03) {
        strcpy(out_pvt.fix_ubx, "3D-Fix");
        m_2d_valid_fix = true;
        m_3d_valid_fix = true;
    }
    else if (fix_mode == 0x04) {
        strcpy(out_pvt.fix_ubx, "GPS+DeadReckoning");
        m_2d_valid_fix = true;
        m_3d_valid_fix = true;
    }
    else if (fix_mode == 0x05) {
        strcpy(out_pvt.fix_ubx, "TimeOnly");
        m_2d_valid_fix = false;
        m_3d_valid_fix = false;
    }
    else {
        strcpy(out_pvt.fix_ubx, "Unknown/Invalid");
        m_2d_valid_fix = false;
        m_3d_valid_fix = false;
    }

	// Gets the update time with precision of microseconds
	auto update = time_point_cast<microseconds>(system_clock::now());

    // Updates age of information
    if (m_debug_age_info_rate) {
        m_debug_age_info.age_sog_cog = update.time_since_epoch().count() - out_pvt.lu_sog_cog;
        m_debug_age_info.age_sog_cog_ubx = update.time_since_epoch().count() - out_pvt.lu_sog_cog_ubx;
        m_debug_age_info.age_pos = update.time_since_epoch().count() - out_pvt.lu_pos;
        m_debug_age_info.age_pos_ubx = update.time_since_epoch().count() - out_pvt.lu_pos_ubx;
        m_debug_age_info.age_alt = update.time_since_epoch().count() - out_pvt.lu_alt;
        m_debug_age_info.age_alt_ubx = update.time_since_epoch().count() - out_pvt.lu_alt_ubx;
    }

    // Retrieves the time since epoch in microseconds and updates the output struct
	out_pvt.lu_sog_cog = update.time_since_epoch().count();
	out_pvt.lu_sog_cog_ubx = out_pvt.lu_sog_cog;
	out_pvt.lu_pos = update.time_since_epoch().count();
	out_pvt.lu_pos_ubx = out_pvt.lu_pos;
    out_pvt.lu_alt = update.time_since_epoch().count();
    out_pvt.lu_alt_ubx = out_pvt.lu_alt;

    // Validates data
    m_sog_valid = true;
    m_sog_valid_ubx = true;
    m_cog_valid = true;
    m_cog_valid_ubx = true;
    m_pos_valid = true;
    m_pos_valid_ubx = true;
    m_alt_valid = true;
    m_alt_valid_ubx = true;

    // Updates the buffer
    m_outBuffer.store(out_pvt);
}

/** parseNmeaRmc() follows the same approach adopted in parseNmeaGns() by scanning the sentence, separating
 *  the fields using the commas encountered in order to retrieve the speed and the course over ground values
 *  along with the fix mode.
 *  Example $GNRMC,083559.00,A,4717.11437,N,00833.91522,E, 0.004,77.52, 091202,,, A ,V*57\r\n
	                         ^fix validity                sog^   cog^     fix mode^ */
void
UBXNMEAParserSingleThread::parseNmeaRmc(std::string nmea_response) {

    out_t out_nmea = m_outBuffer.load();

    std::vector<std::string> fields;
    std::stringstream ss(nmea_response);
    std::string field;

    while (std::getline(ss, field, ',')) {
        fields.push_back(field);
    }

    char fix = fields[12].at(0);
    char fix_validity = fields[2].at(0);
    std::string sog = fields[7];
    std::string cog = fields[8];

    // Check if speed and heading are negative and parses accordingly using stod
    if (!cog.empty()) {
        if (cog[0] == '-') {
            cog = cog.erase(0,1);
            out_nmea.cog = std::stod(cog) * -1;
            out_nmea.cog_nmea = out_nmea.cog;
        }
        else {
            out_nmea.cog = std::stod(cog);
            out_nmea.cog_nmea = out_nmea.cog;
        }
    }
    else {
        out_nmea.cog = 3601; // HeadingValue_unavailable
        out_nmea.cog_nmea = out_nmea.cog;
    }

    if (sog.empty() == false) {
        if (sog[0] == '-') {
            sog = sog.erase(0,1);
            out_nmea.sog = std::stod(sog) * -0.5144; // Conversion from knots to m/s
            out_nmea.sog_nmea = out_nmea.sog;
        }
        else {
            out_nmea.sog = std::stod(sog) * 0.5144;
            out_nmea.sog_nmea = out_nmea.sog;
        }
    }
    else {
        out_nmea.sog = 16383; // SpeedValue_unavailable
        out_nmea.sog_nmea = out_nmea.sog;
    }

    // Fix Validity Check
    // Possible status values: V = data invalid, A = data valid
    if (fix == 'N') {
        if (fix_validity != 'A') strcpy(out_nmea.fix_nmea, "NoFix (V)");
        else strcpy(out_nmea.fix_nmea, "NoFix");
        m_2d_valid_fix = false;
        m_3d_valid_fix = false;
    }
    else if (fix == 'E') {
        if (fix_validity != 'A') strcpy(out_nmea.fix_nmea, "Estimated/DeadReckoning (V)");
        else strcpy(out_nmea.fix_nmea, "Estimated/DeadReckoning");
        m_2d_valid_fix = false;
        m_3d_valid_fix = false;
    }
    else if (fix == 'A') {
        if (fix_validity != 'A') strcpy(out_nmea.fix_nmea, "AutonomousGNSSFix (V)");
        else strcpy(out_nmea.fix_nmea, "AutonomousGNSSFix");
        m_2d_valid_fix = true;
        m_3d_valid_fix = true;
    }
    else if (fix == 'D') {
        if (fix_validity != 'A') strcpy(out_nmea.fix_nmea,"DGNSS (V)");
        else strcpy(out_nmea.fix_nmea,"DGNSS");
        m_2d_valid_fix = true;
        m_3d_valid_fix = true;
    }
    else if (fix == 'F') {
        if (fix_validity != 'A') strcpy(out_nmea.fix_nmea, "RTKFloat (V)");
        else strcpy(out_nmea.fix_nmea, "RTKFloat");
        m_2d_valid_fix = true;
        m_3d_valid_fix = true;
    }
    else if (fix == 'R') {
        if (fix_validity != 'A') strcpy(out_nmea.fix_nmea, "RTKFixed (V)");
        else strcpy(out_nmea.fix_nmea, "RTKFixed");
        m_2d_valid_fix = true;
        m_3d_valid_fix = true;
    }
    else {
        if (fix_validity != 'A') strcpy(out_nmea.fix_nmea, "Unknown/Invalid (V)");
        else strcpy(out_nmea.fix_nmea, "Unknown/Invalid");
        m_2d_valid_fix = false;
        m_3d_valid_fix = false;
    }

    // Store the current letter of the fix status, obtained via the NMEA sentence
    out_nmea.fix_nmea_rmc = fix;
    out_nmea.lu_fix_nmea_rmc = time_point_cast<microseconds>(system_clock::now()).time_since_epoch().count();

	// Produces and prints the current date-time timestamp
	std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	strcpy(out_nmea.ts_sog_cog_nmea,std::ctime(&now));

	// Gets the update time with precision of microseconds
	auto update = time_point_cast<microseconds>(system_clock::now());

    if (m_debug_age_info_rate) {
        m_debug_age_info.age_sog_cog = update.time_since_epoch().count() - out_nmea.lu_sog_cog;
        m_debug_age_info.age_sog_cog_nmea = update.time_since_epoch().count() - out_nmea.lu_sog_cog_nmea;
    }

    // Converts time_point to microseconds
	out_nmea.lu_sog_cog = update.time_since_epoch().count();
	out_nmea.lu_sog_cog_nmea = out_nmea.lu_sog_cog;

    // Validates data
    m_sog_valid = true;
    m_sog_valid_nmea = true;
    m_cog_valid = true;
    m_cog_valid_nmea = true;

    // Updates the buffer
	m_outBuffer.store(out_nmea);
}

/** parseNavStatus() parses a UBX-NAV-STATUS message in order to get the Fix Mode of the receiver.
 * 
 *  Skips the first 4 bytes of the payload and processes the following two bytes
 *  See ZED-F9R interface description at 3.15 (page 109) for more information */
void
UBXNMEAParserSingleThread::parseNavStatus(std::vector<uint8_t> response) {
	uint8_t fix_mode  = response[m_UBX_PAYLOAD_OFFSET + 4];
	uint8_t fix_flags = response[m_UBX_PAYLOAD_OFFSET + 4 + 1];
	
	// Loads the global output buffer atomic struct
	out_t out_sts = m_outBuffer.load();

    // Verify fix validity
    // TODO: check if it makes sense to implement the same check on bit 0 as it is done for the UBX-NAV-PVT message
	if (fix_flags < 0xD0){
        strcpy(out_sts.fix_ubx, "Invalid");
        m_2d_valid_fix = false;
        m_3d_valid_fix = false;
        m_outBuffer.store(out_sts);
		return;
    }

    if (fix_mode == 0x00) {
        strcpy(out_sts.fix_ubx, "NoFix");
        m_2d_valid_fix = false;
        m_3d_valid_fix = false;
    }
    else if (fix_mode == 0x01) {
        strcpy(out_sts.fix_ubx, "Estimated/DeadReckoning");
        m_2d_valid_fix = true;
        m_3d_valid_fix = true;
    }
    else if (fix_mode == 0x02) {
        strcpy(out_sts.fix_ubx, "2D-Fix");
        m_2d_valid_fix = true;
        m_3d_valid_fix = false;
    }
    else if (fix_mode == 0x03) {
        strcpy(out_sts.fix_ubx, "3D-Fix");
        m_2d_valid_fix = true;
        m_3d_valid_fix = true;
    }
    else if (fix_mode == 0x04) {
        strcpy(out_sts.fix_ubx, "GPS+DeadReckoning");
        m_2d_valid_fix = true;
        m_3d_valid_fix = true;
    }
    else if (fix_mode == 0x05) {
        strcpy(out_sts.fix_ubx, "TimeOnly");
        m_2d_valid_fix = false;
        m_3d_valid_fix = false;
    }
    else {
        strcpy(out_sts.fix_ubx, "Unknown/Invalid");
        m_2d_valid_fix = false;
        m_3d_valid_fix = false;
    }
	
	// Updates the output buffer with new data
	m_outBuffer.store(out_sts);
}

/** parseEsfInf() parses a UBX-ESF-INS message in order to obtain measures about the vehicle dynamics.
 *
 *  In particular the function parses the compensated acceleration, without the gravity acceleration added
 *  to the x axis.
 *
 *  The fields of interest in the UBX-ESF-INS message from byte 12 (after the payload) to byte 36, containing
 *  the threee angular rates and the three gravity-free accelerations-
 *
 *  See ZED-F9R interface description at 3.11.2 UBX-ESF-INS for more information */
void
UBXNMEAParserSingleThread::parseEsfIns(std::vector<uint8_t> response) {
	out_t out_ins = m_outBuffer.load();

    // Compensated angular rates
    std::vector<uint8_t> comp_ang_rate_x(response.begin() + m_UBX_PAYLOAD_OFFSET + 12, response.begin() + m_UBX_PAYLOAD_OFFSET + 16);
    std::reverse(comp_ang_rate_x.begin(),comp_ang_rate_x.end());
    out_ins.comp_ang_rate_x = static_cast<double>(hexToSigned(comp_ang_rate_x)) * 0.001; // Accordingly scales to deg/s

    std::vector<uint8_t> comp_ang_rate_y(response.begin() + m_UBX_PAYLOAD_OFFSET + 16, response.begin() + m_UBX_PAYLOAD_OFFSET + 20);
    std::reverse(comp_ang_rate_y.begin(),comp_ang_rate_y.end());
    out_ins.comp_ang_rate_y = static_cast<double>(hexToSigned(comp_ang_rate_y)) * 0.001;

    std::vector<uint8_t> comp_ang_rate_z(response.begin() + m_UBX_PAYLOAD_OFFSET + 20, response.begin() + m_UBX_PAYLOAD_OFFSET + 24);
    std::reverse(comp_ang_rate_z.begin(),comp_ang_rate_z.end());
    out_ins.comp_ang_rate_z = static_cast<double>(hexToSigned(comp_ang_rate_z)) * 0.001;

    // Compensated accelerations
    std::vector<uint8_t> comp_acc_x(response.begin() + m_UBX_PAYLOAD_OFFSET + 24, response.begin() + m_UBX_PAYLOAD_OFFSET + 28);
	std::reverse(comp_acc_x.begin(),comp_acc_x.end());
	out_ins.comp_acc_x = static_cast<double>(hexToSigned(comp_acc_x)) * 0.01; // Accordingly scales to m/s^2

	std::vector<uint8_t> comp_acc_y(response.begin() + m_UBX_PAYLOAD_OFFSET + 28, response.begin() + m_UBX_PAYLOAD_OFFSET + 32);
	std::reverse(comp_acc_y.begin(),comp_acc_y.end());
	out_ins.comp_acc_y = static_cast<double>(hexToSigned(comp_acc_y)) * 0.01;

	std::vector<uint8_t> comp_acc_z(response.begin() + m_UBX_PAYLOAD_OFFSET + 32, response.begin() + m_UBX_PAYLOAD_OFFSET + 36);
	std::reverse(comp_acc_z.begin(),comp_acc_z.end());
	out_ins.comp_acc_z = static_cast<double>(hexToSigned(comp_acc_z)) * 0.01;

	// Produces and prints to struct the current date-time timestamp
	std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	strcpy(out_ins.ts_comp_acc,std::ctime(&now));
    strcpy(out_ins.ts_comp_ang_rate,std::ctime(&now));

    // Timestamps, age and last update (Same for accelerations and angular rate, but implemented as separate variables for scalability)
	// Gets the update time with precision of microseconds
	auto update = time_point_cast<microseconds>(system_clock::now());

    // Updates age of information
    if (m_debug_age_info_rate) {
        m_debug_age_info.age_comp_acc = update.time_since_epoch().count() - out_ins.lu_comp_acc;
        m_debug_age_info.age_comp_ang_rate = update.time_since_epoch().count() - out_ins.lu_comp_ang_rate;
    }

	// Retrieves the time since epoch in microseconds and updates the output struct
	out_ins.lu_comp_acc = update.time_since_epoch().count();
    out_ins.lu_comp_ang_rate = update.time_since_epoch().count();

    // Validates data
    m_comp_acc_valid = true;
    m_comp_ang_rate_valid = true;

    // Updates the buffer
	m_outBuffer.store(out_ins);
}

/** Reads from serial port, filtering for NMEA sentences and UBX messages and parsing accordingly.
 *
 *  Accounts for wrong inputs provided by the receiver and suspends the data parsing if the wrong
 *  bytes quantity is greater than the threshold provided by the user via CLI.
 *
 *  It handles overlapped messages by scanning for a new message's header while reading
 *  the current message and verifies every message calculating the checksum values before parsing. */
void
UBXNMEAParserSingleThread::readFromSerial() {

    std::vector<uint8_t> ubx_message, ubx_message_overlapped, wrong_input;
    std::string nmea_sentence;

    uint8_t byte = 0x00;
    uint8_t byte_previous = 0x00;
    bool started_ubx = false;
    bool started_ubx_overlapped = false;
    bool started_nmea = false;
    bool success = false;
    long unsigned int expectedLength = 0;
    long unsigned int expectedLengthOverlapped = 0;
    std::string expectedLengthStr;
    std::string expectedLengthStrOverlapped;

    // Variables useful for the trace file reproduction: not used in normal serial parser mode
    size_t ti = 0, tj = 0;
    std::string curr_data_str, curr_msg_type;
    uint64_t curr_tstamp, start_time=0, delta_time_us_real=0, start_time_us=0, delta_time_us_trace=0, variable_delta_us_factor=0;
    // Get current timestamp in microseconds
    uint64_t startup_time = get_timestamp_us();

    while (true) {
        if(!m_is_file_no_serial) {
            byte = m_serial.ReadChar(success);

            if (!success) {
                continue;
            }
        } else {
            // Read bytes from the trace in a timed manner
            if(ti==0 || tj>=curr_data_str.size()) {
                curr_data_str = JSON_GET_STR(m_trace[ti],"data");
                curr_msg_type = JSON_GET_STR(m_trace[ti],"type");
                curr_tstamp = JSON_GET_INT(m_trace[ti],"timestamp");

                if(!m_trace[ti].is_object()) {
                    std::cerr << "[ERROR] Unexpected JSON trace format. Terminating..." << std::endl;
                    m_terminatorFlagPtr->store(true);
                    break;
                }

                if(ti>0) {
                    delta_time_us_real = get_timestamp_us() - startup_time;
                    delta_time_us_trace = curr_tstamp - start_time;
                    variable_delta_us_factor = delta_time_us_trace - delta_time_us_real;

                    if(variable_delta_us_factor>0) {
                        std::this_thread::sleep_for(std::chrono::microseconds(variable_delta_us_factor));
                    }
                } else {
                    start_time = curr_tstamp;
                }

                ti++;

                tj=0;

                if(ti>=m_trace.size()) {
                    std::cout << "[INFO] Trace reproduction terminated. OScar will now terminate..." << std::endl;
                    m_terminatorFlagPtr->store(true);
                    break;
                }
            }

            if(curr_msg_type=="UBX" || curr_msg_type=="Unknown") {
                byte = str_to_byte(curr_data_str.substr(tj, 2));
                tj+=2;
            } else if(curr_msg_type=="NMEA") {
                byte = curr_data_str[tj];
                tj+=1;
            } else {
                std::cout << "[ERROR] Unexpected JSON trace format. Terminating..." << std::endl;
                m_terminatorFlagPtr->store(true);
                break;
            }
        }

        //printf("%c",byte); // For debug purposes

        // This array is cleared every time a correct UBX/NMEA message is received
        wrong_input.push_back(byte);
        if (wrong_input.size() >= m_WRONG_INPUT_THRESHOLD) {
            std::cerr << "Error. Wrong input detected. Size: " << wrong_input.size() << " . Terminating. Content: ";
            for (int i = 0; i < wrong_input.size(); i++) {
                printf("%02X-", wrong_input[i]);
            }
            printf("\n");

            printf("byte = %02X\n", byte);
            printf("byte_previous = %02X\n", byte_previous);
            printf("expectedLength = %lu\n", expectedLength);

            wrong_input.clear();
            m_terminatorFlagPtr->store(true);
        }

        // NMEA sentences reading and parsing
        if (started_ubx == false) {
            if (started_nmea == false) {
                if (byte == '$') {
                    nmea_sentence.push_back(byte);
                    wrong_input.clear();
                    started_nmea = true;

                    byte_previous = byte;
                    continue;
                }
            } else {
                if (byte != '$') {
                    nmea_sentence.push_back(byte);
                    wrong_input.clear();
                }
                // If another $ is found, start over
                else {
                    nmea_sentence.clear();
                    nmea_sentence.push_back(byte);
                    wrong_input.clear();

                    byte_previous = byte;
                    continue;
                }
                if (byte == '\n' && nmea_sentence.size() >= 9) { // 9 = min. valid sentence lenght e.g. $--XXX*hh
                    if (validateNmeaSentence(nmea_sentence)) {
                        if (nmea_sentence.compare(0, 6, "$GNGNS") == 0 ||
                            nmea_sentence.compare(0, 6, "$GPGNS") == 0) {
                            started_nmea = false;
                            byte_previous = byte;

                            parseNmeaGns(nmea_sentence);

                            nmea_sentence.clear();
                            continue;
                        }

                        if (nmea_sentence.compare(0, 6, "$GNRMC") == 0 ||
                            nmea_sentence.compare(0, 6, "$GPRMC") == 0) {
                            started_nmea = false;
                            byte_previous = byte;

                            parseNmeaRmc(nmea_sentence);

                            nmea_sentence.clear();
                            continue;
                        }
                        if (nmea_sentence.compare(0, 6, "$GNGGA") == 0 ||
                            nmea_sentence.compare(0, 6, "$GPGGA") == 0) {
                            started_nmea = false;
                            byte_previous = byte;

                            parseNmeaGga(nmea_sentence);

                            nmea_sentence.clear();
                            continue;
                        }

                        // Valid but unhandled NMEA sentence, ignore it and start over
                        started_nmea = false;
                        nmea_sentence.clear();

                        byte_previous = byte;
                        continue;

                    } else { // Invalid sentence
                        started_nmea = false;
                        nmea_sentence.clear();

                        byte_previous = byte;
                        continue;
                    }
                }
            }
        }

        // UBX reading logic
        if (started_nmea == false) {
            if (started_ubx == false) {
                if (byte_previous == m_UBX_HEADER[0] && byte == m_UBX_HEADER[1]) {
                    expectedLength = 0;
                    ubx_message.push_back(byte_previous);
                    ubx_message.push_back(byte);
                    wrong_input.clear();
                    started_ubx = true;

                    byte_previous = byte;
                    continue;
                }
            } else { // started_ubx is true

                // check if B5 62 is part of the current message or if it's a new message
                if (byte_previous == m_UBX_HEADER[0] && byte == m_UBX_HEADER[1]) {
                    // if length is nt reached -> surely it's an overlapping message
                    if (ubx_message.size() <= 4) {
                        std::cout << "length not reached!" << std::endl; //debug
                        //printf("byte: %02X byte_previous: %02X\n", byte, byte_previous);
                        //printUbxMessage(ubx_message);
                        ubx_message.clear();
                        ubx_message.push_back(byte_previous);
                        ubx_message.push_back(byte);
                        wrong_input.clear();

                        byte_previous = byte;
                        continue;
                    }
                    //if a B5 62 sequence is read in the middle of the message, start storing bytes also into the overlapped msg
                    expectedLengthOverlapped = 0;
                    ubx_message_overlapped.clear();
                    ubx_message_overlapped.push_back(byte_previous);
                    ubx_message_overlapped.push_back(byte);
                    ubx_message.push_back(byte);
                    wrong_input.clear();
                    started_ubx_overlapped = true;

                    byte_previous = byte;
                    continue;
                }

                ubx_message.push_back(byte);
                wrong_input.clear();

                // now use the length and checksum to see which message is valid and which is not
                if (ubx_message.size() == 6) {
                    sprintf(expectedLengthStr.data(), "%02X%02X", ubx_message[5], ubx_message[4]);
                    //since length is referred to payload only, add 8 bytes (6 bytes for header, class and length fields + 2 bytes for checksum field)
                    expectedLength = std::stol(expectedLengthStr, nullptr, 16) + 8;
                    expectedLengthStr.clear();
                }
                if (expectedLength > 0 && ubx_message.size() == expectedLength) {
                    //std::cout << "normal msg" << std::endl;
                    //printUbxMessage(ubx_message); //debug

                    if (started_ubx_overlapped == false) {
                        if (validateUbxMessage(ubx_message) == true) {

                            // Configuration messages
                            if (ubx_message[2] == 0x05 && ubx_message[3] == 0x00) {
                                std::cerr << "Configuration Error: CFG-ACK-NAK received. Terminating." << std::endl;
                                // *m_terminatorFlagPtr=true;
                            }

                            // Parse data messages
                            if (ubx_message[2] == 0x01 && ubx_message[3] == 0x03) {
                                // Parse, clear everything and start a new read
                                started_ubx_overlapped = false;
                                started_ubx = false;
                                byte_previous = byte;

                                parseNavStatus(ubx_message);

                                ubx_message.clear();
                                ubx_message_overlapped.clear();
                                continue;
                            }
                            if (ubx_message[2] == 0x10 && ubx_message[3] == 0x03) {
                                // Parse, clear everything and start a new read
                                started_ubx_overlapped = false;
                                started_ubx = false;
                                byte_previous = byte;

                                parseEsfRaw(ubx_message);

                                ubx_message.clear();
                                ubx_message_overlapped.clear();
                                continue;
                            }
                            if (ubx_message[2] == 0x10 && ubx_message[3] == 0x15) {
                                started_ubx_overlapped = false;
                                started_ubx = false;
                                byte_previous = byte;

                                parseEsfIns(ubx_message);

                                ubx_message.clear();
                                ubx_message_overlapped.clear();
                                continue;
                            }
                            if (ubx_message[2] == 0x01 && ubx_message[3] == 0x05) {
                                started_ubx_overlapped = false;
                                started_ubx = false;
                                byte_previous = byte;

                                parseNavAtt(ubx_message);

                                ubx_message.clear();
                                ubx_message_overlapped.clear();
                                continue;
                            }
                            if (ubx_message[2] == 0x01 && ubx_message[3] == 0x07) {
                                started_ubx_overlapped = false;
                                started_ubx = false;
                                byte_previous = byte;

                                parseNavPvt(ubx_message);

                                ubx_message.clear();
                                ubx_message_overlapped.clear();
                                continue;
                            }
                            // Valid but unhandled message
                            ubx_message_overlapped.clear();
                            started_ubx_overlapped = false;

                            //printUbxMessage(ubx_message);
                            ubx_message.clear();
                            started_ubx = false;

                            byte_previous = byte;
                            continue;

                        } else { // Invalid message
                            ubx_message_overlapped.clear();
                            started_ubx_overlapped = false;

                            //printUbxMessage(ubx_message);
                            ubx_message.clear();
                            started_ubx = false;

                            byte_previous = byte;
                            continue;
                        }
                    }
                }

                // Overlapped message checks section
                if (started_ubx_overlapped == true) {
                    ubx_message_overlapped.push_back(byte);
                    wrong_input.clear();

                    if (ubx_message_overlapped.size() == 6) {
                        sprintf(expectedLengthStrOverlapped.data(), "%02X%02X", ubx_message_overlapped[5],ubx_message_overlapped[4]);
                        expectedLengthOverlapped = std::stol(expectedLengthStrOverlapped, nullptr, 16) + 8; // 8 bytes for header and checksum
                        expectedLengthStrOverlapped.clear();
                    }

                    if (expectedLengthOverlapped > 0 && ubx_message_overlapped.size() == expectedLengthOverlapped) {
                        //std::cout << "overlapped msg" << std::endl;
                        //printUbxMessage(ubx_message_overlapped); //debug
                        if (validateUbxMessage(ubx_message_overlapped) == true) {
                            // Configuration messages
                            if (ubx_message_overlapped[2] == 0x05 && ubx_message_overlapped[3] == 0x00) {
                                std::cerr << "Configuration Error: CFG-ACK-NAK received. Terminating." << std::endl;
                                // *m_terminatorFlagPtr=true;
                            }

                            // Parse overlapped message
                            if (ubx_message_overlapped[2] == 0x01 && ubx_message_overlapped[3] == 0x03) {
                                started_ubx_overlapped = false;
                                started_ubx = false;
                                byte_previous = byte;

                                parseNavStatus(ubx_message_overlapped);

                                ubx_message.clear();
                                ubx_message_overlapped.clear();
                                continue;
                            }
                            if (ubx_message_overlapped[2] == 0x10 && ubx_message_overlapped[3] == 0x03) {
                                started_ubx_overlapped = false;
                                started_ubx = false;
                                byte_previous = byte;

                                parseEsfRaw(ubx_message_overlapped);

                                ubx_message.clear();
                                ubx_message_overlapped.clear();
                                continue;
                            }
                            if (ubx_message_overlapped[2] == 0x10 && ubx_message_overlapped[3] == 0x15) {
                                started_ubx_overlapped = false;
                                started_ubx = false;
                                byte_previous = byte;

                                parseEsfIns(ubx_message_overlapped);

                                ubx_message.clear();
                                ubx_message_overlapped.clear();
                                continue;

                            }
                            if (ubx_message_overlapped[2] == 0x01 && ubx_message_overlapped[3] == 0x05) {
                                started_ubx_overlapped = false;
                                started_ubx = false;
                                byte_previous = byte;

                                parseNavAtt(ubx_message_overlapped);

                                ubx_message.clear();
                                ubx_message_overlapped.clear();
                                continue;
                            }
                            if (ubx_message_overlapped[2] == 0x01 && ubx_message_overlapped[3] == 0x07) {
                                started_ubx_overlapped = false;
                                started_ubx = false;
                                byte_previous = byte;

                                parseNavPvt(ubx_message_overlapped);

                                ubx_message.clear();
                                ubx_message_overlapped.clear();
                                continue;
                            }

                            // Unhandled overlapped message, discard it and start a new read
                            ubx_message.clear();
                            started_ubx = false;

                            ubx_message_overlapped.clear();
                            //printUbxMessage(ubx_message_overlapped);
                            started_ubx_overlapped = false;

                            byte_previous = byte;
                            continue;
                        } else {
                            //invalid overlapped message, discard it and move on with the ubx_message
                            //std::cerr << "UBX Overlapped message: Invalid checksum" << std::endl;
                            ubx_message.clear();
                            started_ubx = false;

                            ubx_message_overlapped.clear();
                            //printUbxMessage(ubx_message_overlapped);
                            started_ubx_overlapped = false;

                            byte_previous = byte;
                            continue;
                        }
                    }
                }
            }
        }
        byte_previous = byte;
        //printf("byte = %02X\n",byte);
        //printf("byte_previous = %02X\n",byte_previous);
    }
}

/** readData() enables the necessary messages for data retrieving by sending a UBX-CFG-VALSET poll request,
 *  implementing also a CFG-VALSET poll request in order to disable the messages.
 *
 * It encapsulates "readFromSerial()" and executes it endlessly until the global atomic err_flag is raised.
 *
 * NOTE: This function is executed in a parallel thread (see main()) */
void
UBXNMEAParserSingleThread::readData() {
	
	// Initialization and preliminary operation on data buffer
    m_terminatorFlagPtr->store(false);
    clearBuffer();

    std::cout << "Serial data reader thread started." << std::endl;

	// Sends CFG-VALSET Command to enable messages
	// See more in interface description 3.10.5 UBX-CFG-VALSET S
	std::vector<uint8_t> ubx_valset_req_enable = {
		0xb5, 0x62, 0x06, 0x8a,  // Preamble + header
		0x27, 0x00, 		     // Length (PAYLOAD ONLY, 24 bytes) REMEMBER: little endian

		0x00, 				     // Version (0 = no transactions)
		0x01, 					 // 0x01 = update configuration in the RAM layer
		0x00, 0x00, 	         //Reserved

		//Messages KeyIds + value to be set (see chapter 5  of the sinterface description manual)
		0xb8, 0x00, 0x91, 0x20, 0x01, //CFG-MSGOUT-NMEA_ID_GNS_USB + enable value (0x01)
		0xae, 0x00, 0x91, 0x20, 0x01, //CFG-MSGOUT-NMEA_ID_RMC_USB
		0x17, 0x01, 0x91, 0x20, 0x01, //CFG-MSGOUT-UBX_ESF_INS_USB
		0xa2, 0x02, 0x91, 0x20, 0x01, //CFG-MSGOUT-UBX_ESF_RAW_USB
		0x22, 0x00, 0x91, 0x20, 0x01, //CFG-MSGOUT-UBX_NAV_ATT_USB
		0x09, 0x00, 0x91, 0x20, 0x01, //CFG-MSGOUT-UBX_NAV_PVT_USB
		0x1d, 0x00, 0x91, 0x20, 0x01, //CFG-MSGOUT-UBX_NAV_STATUS_USB

		0x00, 0x53 // Checksum
	};
	
	// Writes to serial port in order to enable the messages (skip this line if a trace is used instead of a real serial device)
    if(!m_is_file_no_serial) {
        m_serial.Write(reinterpret_cast<char *>(ubx_valset_req_enable.data()), ubx_valset_req_enable.size());
    }

	// Same as above but with the value 0x00 after the keyID
	std::vector<uint8_t> ubx_valset_req_disable = {
		0xb5, 0x62, 0x06, 0x8a, 0x27, 0x00, 0x00, 0x01, 0x00, 0x00,

		0xb8, 0x00, 0x91, 0x20, 0x00, //CFG-MSGOUT-NMEA_ID_GNS_USB + enable value (0x01)
		0xae, 0x00, 0x91, 0x20, 0x00, //CFG-MSGOUT-NMEA_ID_RMC_USB
		0x17, 0x01, 0x91, 0x20, 0x00, //CFG-MSGOUT-UBX_ESF_INS_USB
		0xa2, 0x02, 0x91, 0x20, 0x00, //CFG-MSGOUT-UBX_ESF_RAW_USB
		0x22, 0x00, 0x91, 0x20, 0x00, //CFG-MSGOUT-UBX_NAV_ATT_USB
		0x09, 0x00, 0x91, 0x20, 0x00, //CFG-MSGOUT-UBX_NAV_PVT_USB
		0x1d, 0x00, 0x91, 0x20, 0x00, //CFG-MSGOUT-UBX_NAV_STATUS_USB

		0xf9, 0xe3
	};

	// Disables the messages
    // TODO: why was this here? This is very likely not needed
    // if(!m_is_file_no_serial) {
	//      serialDev.Write(reinterpret_cast<char*>(ubx_valset_req_disable.data()), ubx_valset_req_disable.size());
    // }

	while (m_terminatorFlagPtr->load() == false && m_stopParserFlag.load() == false) {
		readFromSerial();
	}
}

/** startUBXNMEAParser() initializes and configures the serial interface, clears the atomic data buffer and
 *  encapsulates readData() to correctly start the serial reading and parsing process. */
int
UBXNMEAParserSingleThread::startUBXNMEAParser(std::string device, int baudrate, int data_bits, char parity, int stop_bits, std::atomic<bool> *terminatorFlagPtr, bool is_file_no_serial) {
    clearBuffer();

    if(terminatorFlagPtr==nullptr) {
        std::cerr << "Error: Invalid pointer to terminator flag. Terminating." << std::endl;
        return -1;
    }

    m_terminatorFlagPtr = terminatorFlagPtr;

    m_is_file_no_serial = is_file_no_serial;

    if(!m_is_file_no_serial) {
        // Serial interface handling and initializing
        m_serial.SetBaudRate(baudrate);
        m_serial.SetDataSize(data_bits);
        m_serial.SetParity(parity);
        m_serial.SetStopBits(stop_bits);
        m_serial.SetPortName(device);

        // ceSerial serial(device, baudrate, data_bits, parity, stop_bits);
        // Correct sample configuration for a U-blox ZED-F9R device: ("/dev/ttyACM0",115200,8,'N',1)
        std::cout << "Opening device " << m_serial.GetPort().c_str() << std::endl;
        std::cout << "Setting baudrate " << baudrate << std::endl;
        std::cout << "Setting data bits " << m_serial.GetDataSize() << std::endl;

        if (m_serial.Open() == 0) {
            std::cout << "Serial device opened successfully." << std::endl;
        } else {
            std::cerr << "Error: cannot open serial device." << std::endl;
            return -1;
        }
    } else {
        // Open the file specified by "device" and read the JSON using json11
        std::cout << "Opening trace file " << device << std::endl;
        std::ifstream json_trace_ifstream(device);
        std::string json_str((std::istreambuf_iterator<char>(json_trace_ifstream)), std::istreambuf_iterator<char>());
        std::string err="";
        json11::Json full_json_trace;
        full_json_trace = json11::Json::parse(json_str,err);

        json_trace_ifstream.close();

        if (!err.empty()) {
            std::cerr << "Error: cannot open JSON trace. More details: " << err << std::endl;
            return -1;
        }

        if(!full_json_trace.is_array()) {
            std::cerr << "Error: unsupported JSON trace format." << std::endl;
            return -1;
        }

        m_trace=full_json_trace.array_items();
    }

	// Start executing the parallel reading thread
	std::thread readerThread(&UBXNMEAParserSingleThread::readData,this);
    readerThread.detach();

    m_parser_started = true;

    return 1;
}

void
UBXNMEAParserSingleThread::stopUBXNMEAParser() {
    m_stopParserFlag = true;
    m_parser_started = false;
    m_serial.Close();
}

uint8_t UBXNMEAParserSingleThread::str_to_byte(const std::string &str) {
    if (str.length() != 2) {
        std::cerr << "str_to_byte: received size: " << str.size() << " - expected size: 2" << std::endl;
        throw std::invalid_argument("str_to_byte: invalid size of input string");
    }

    uint8_t final_val=0x00;
    for(int i=0;i<str.length();i++) {
        char curr_c = str[i];
        uint8_t curr_val= 0x00;

        if (curr_c <= '9' && curr_c >= '0') {
           curr_val = curr_c - '0';
        } else if (curr_c <= 'F' && curr_c >= 'A') {
           curr_val = curr_c - 'A' + 10;
        } else if (curr_c <= 'f' && curr_c >= 'a') {
           curr_val = curr_c - 'a' + 10;
        } else {
           throw std::invalid_argument("str_to_byte: invalid input string");
        }

        final_val=final_val*16+curr_val;
   }

    return final_val;
}
