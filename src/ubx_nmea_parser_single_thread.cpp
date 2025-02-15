// UBX and NMEA serial GNSS parser
// Designed for the ZED-F9R module, but compatible with any GNSS receiver (some data may not be available with non-ublox devices)
// This module allows OScar to directly retrieve positioning and other kinematics data, without the need of interfacing
// to the gpsd daemon or other external software.
// Copyright (c) 2024 Mauro Vittorio

#include "ubx_nmea_parser_single_thread.h"
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

using namespace std::chrono;

/* Prints a UBX message in the format index[byte] (for debug purposes) */
void
UBXNMEAParserSingleThread::printUbxMessage(std::vector<uint8_t> msg) {
    std::cout << "message size: " << msg.size() << std::endl;
    for (long unsigned int i = 0; i < msg.size(); i++) {
        printf("%ld[%02X] ", i, msg[i]);
    }
}

/* Prints a NMEA sentence (printable chars only) */
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

/* Converts latitude and longitude from NMEA format to single values in degrees */
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

/** Performs a conversion from a uint8_t array of 3 or 4 bytes to a signed long value.
 *
 *  By exploiting the dimension of a long value (32 bit), shifts every bytes to the left by
 *  multiples of 8, obtaining the whole number in a single 32 bit variable.
 *  Then calculates the two's complement to obtain the final signed value.
 *
 *  If it's provided with wrong input, raises a fatal error and triggers the global err_flag
 *  thus terminating the program
 *
 *  NOTE: This works only for 3 and 4-dimensioned arrays but it can be extended using
 *  a uint64_t return variable. */
// TODO: better comment why this function works like this
int32_t
UBXNMEAParserSingleThread::hexToSigned(std::vector<uint8_t> data) {
    int32_t value = 0;
    if (data.size() == 2) {
        value |= (data[0] << 8);
        value |= data[1];
        if (value & 0x80000000) {
            value = ~value + 1;
            return -static_cast<int32_t>(value);
        }
        return value;
    } else if (data.size() == 3) {
        value |= (data[0] << 16);
        value |= (data[1] << 8);
        value |= data[2];
        if (value & 0x80000000) {
            value = ~value + 1;
            return -static_cast<int32_t>(value);
        }
        return value;
    } else if (data.size() == 4) {
        value |= (data[0] << 24);
        value |= (data[1] << 16);
        value |= (data[2] << 8);
        value |= data[3];
        if (value & 0x80000000) {
            value = ~value + 1;
            return -static_cast<int32_t>(value);
        }
        return value;
    } else {
        std::cerr << "Fatal Error: Invalid std::vector data size. Check UBX-ESF-RAW/UBX-NAV-ATT. Terminating. ";
        //*m_terminatorFlagPtr = true; // Breaks the endless loop and terminates the program
        return -1;
    }
}

void
UBXNMEAParserSingleThread::clearBuffer() {
	out_t tmp;

    // Human-readable dates (epoch time of last update)
	strcpy(tmp.ts_pos,"");
    strcpy(tmp.ts_pos_ubx,"");
    strcpy(tmp.ts_pos_nmea,"");
	strcpy(tmp.ts_utc_time_ubx,"");
    strcpy(tmp.ts_utc_time_nmea,"");
	strcpy(tmp.ts_acc,"");
	strcpy(tmp.ts_att,"");
	strcpy(tmp.ts_alt,"");
    strcpy(tmp.ts_comp_acc,"");
    strcpy(tmp.ts_comp_ang_rate,"");
	strcpy(tmp.ts_sog_cog_ubx,"");
	strcpy(tmp.ts_sog_cog_nmea,"");
	strcpy(tmp.fix_ubx,"");
	strcpy(tmp.fix_nmea,"");

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

    // Latitude, longitude and altitude
    // TODO: better comment the division between _ubx and _nmea
    // "lat" contains the latest update between lat_ubx and lat_nmea
	tmp.lat = 0;
    tmp.lat_ubx = 0;
    tmp.lat_nmea = 0;
    // "lon" contains the latest update between lon_ubx and lon_nmea
	tmp.lon = 0;
    tmp.lon_ubx = 0;
    tmp.lon_nmea = 0;
    // "alt" contains the latest update between alt_ubx and alt_nmea
	tmp.alt = 0;
    tmp.alt_ubx = 0;
    tmp.alt_nmea = 0;

	tmp.roll = 0;
	tmp.pitch = 0;
	tmp.heading = 0;
    tmp.sog = 0;
	tmp.sog_ubx = 0;
	tmp.sog_nmea = 0;
    tmp.cog = 0;
	tmp.cog_ubx = 0;
	tmp.cog_nmea = 0;
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
	m_outBuffer.store(tmp);
	//printf("\n\nBuffer cleared\n\n");
}

/** Calculates the UBX checksum using the 8-bit Fletcher algorithm. The calculation range excludes
 *  the first two bytes and the last two bytes of the message, that represent this value.
 *  see section 3.4 - UBX Checksum of the ZED-F9R Interface Description document for more information */
bool
UBXNMEAParserSingleThread::validateUbxMessage(std::vector<uint8_t> msg) {
    uint8_t CK_A = 0x00;
    uint8_t CK_B = 0x00;

    for (long unsigned int i = 0; i < msg.size(); i++) {
        if (i >=2 && i <= msg.size() -3) { //checksum calculation range
            CK_A = CK_A + msg[i];
            CK_B = CK_B + CK_A;

            CK_A &= 0xFF;
            CK_B &= 0xFF;
        }
    }

    if (msg[msg.size() -2] == CK_A && msg[msg.size()-1] == CK_B) {
        return true;
    }
    else {
        //printf("\n\nINVALID MESSAGE: CK_A: %02X  CK_B: %02X\n\n", CK_A,CK_B);
        //printUbxMessage(msg);
        return false;
    }
}

/** Analyzes and validates a NMEA sentence, calculating its checksum character by applying the
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
    if (asteriskPos + 2 >= nmeaMessage.length()) {
        return false;  // Invalid or missing checksum
    }

    // XOR all characters between '$' and '*' (excluded)
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

std::string
UBXNMEAParserSingleThread::getUtcTimeUbx() {
    if(m_parser_started == false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return std::string("Error!");
    }

    out_t tmp = m_outBuffer.load();
    std::string utc_time(tmp.ts_utc_time_ubx);

    if (utc_time.empty() == false) {
        return utc_time;
    } else {
        return std::string("Error. UBX UTC time string is empty.");
    }
}


std::string
UBXNMEAParserSingleThread::getUtcTimeNmea() {
    if(m_parser_started == false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return std::string("Error!");
    }

    out_t tmp = m_outBuffer.load();
    std::string utc_time(tmp.ts_utc_time_nmea);

    if (utc_time.empty() == false) {
        return utc_time;
    } else {
        return std::string("Error. NMEA UTC time string is empty.");
    }
}

/** Retrieves and processes the specific position timestamp and data from the atomic buffer,
 *  calculates the age of information in microseconds, then returns latitude and longitude
 *  as a std::pair, optionally returns the age of information if print_timestamp_and_age is set
 *  to true when called.
 *
 *  Note: use first() and second() to access the element of a pair */
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
    auto end = now.time_since_epoch().count();
    long local_age_us = end - tmp.lu_pos;

    long validity_thr = getValidityThreshold();

    // Saves the value if -a debug option is enabled
    if (m_debug_age_info_rate) m_debug_age_info.age_pos = local_age_us;

    if (print_timestamp_and_age == true) std::cout << "[Position] - " <<  tmp.ts_sog_cog_ubx
                                                   << " Now: " << end
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
        m_pos_valid.store(false);
    }
    else m_pos_valid.store(true);

    return std::pair<double,double>(tmp.lat_ubx,tmp.lon_ubx);
}

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
        m_pos_valid.store(false);
    }
    else m_pos_valid.store(true);

    return std::pair<double,double>(tmp.lat_nmea,tmp.lon_nmea);
}



/** Checks and parses a GNGNS NMEA sentence in order to get the latitude and longitude data
 *
 *  The function scans the sentence by counting the commas encountered.
 *  The substring sought is found between commas 2 and 5.
 *  Example: $GNGNS,133033.40,   4503.87250,N,00739.68967,E   ,RRRR,19,0.86,250.6,47.2,1.4,0000,V*3B
 *
 *  After parsing, it produces the current information specific timestamp and updates the output buffer.
 *  For more information see page 2.7.9.1 GNSS Fix Data on page 28 of ublox ZED-F9R Interface Description*/
void
UBXNMEAParserSingleThread::parseNmeaGns(std::string nmea_response) {

    out_t out_nmea = m_outBuffer.load();

    std::vector<std::string> fields;
    std::stringstream ss(nmea_response);
    std::string field;

    // Separate the sentence by commas and store the fields of interest
    while (std::getline(ss, field, ',')) {
        fields.push_back(field);
    }

	// Latitude and Longitude substrings to be converted to double using std::stod() later
	std::string utc_time = "UTC Time: " + fields[1];
    std::string slat = fields[2];
    std::string slon = fields[4];
    std::string posMode = fields[6];
    std::string salt = fields[9];

    // cardinal points (N, S, W, E)
	char cp_lat = fields[3].at(0);
    char cp_lon = fields[5].at(0);

    /* debug
    std::cout << nmea_response;
    std::cout << utc_time << " Lat: " << slat << " Lon: " << slon
              << " Alt: " << salt << " cp lat: " << cp_lat << " cp lon: " << cp_lon << std::endl;
    */
    // Fix value parsing
    if (posMode == "NNNN") {
        strcpy(out_nmea.fix_nmea, "NoFix");
        m_2d_valid_fix = false;
        m_3d_valid_fix = false;
    }
    else if (posMode == "EEEE") {
        strcpy(out_nmea.fix_nmea, "Estimated/DeadReckoning");
        m_2d_valid_fix = false;
        m_3d_valid_fix = false;
    }
    else if (posMode == "AAAA") {
        strcpy(out_nmea.fix_nmea, "AutonomousGNSSFix");
        m_2d_valid_fix = true;
        m_3d_valid_fix = true;
    }
    else if (posMode == "DDDD") {
        strcpy(out_nmea.fix_nmea,"DGNSS");
        m_2d_valid_fix = true;
        m_3d_valid_fix = true;
    }
    else if (posMode == "FFFF") {
        strcpy(out_nmea.fix_nmea, "RTKFloat");
        m_2d_valid_fix = true;
        m_3d_valid_fix = true;
    }
    else if (posMode == "RRRR") {
        strcpy(out_nmea.fix_nmea, "RTKFixed");
        m_2d_valid_fix = true;
        m_3d_valid_fix = true;
    }
    else {
        strcpy(out_nmea.fix_nmea, "Unknown/Invalid");
        m_2d_valid_fix = false;
        m_3d_valid_fix = false;
    }

    // Posistion parsing
    if (!slat.empty()) {
        out_nmea.lat = decimal_deg(std::stod(slat),cp_lat);
        out_nmea.lat_nmea = out_nmea.lat;
    }
	else out_nmea.lat = 900000001; // Latitude_unavailable

	if (!slon.empty()) {
        out_nmea.lon = decimal_deg(std::stod(slon),cp_lon);
        out_nmea.lon_nmea = out_nmea.lon;
    }
	else out_nmea.lon = 1800000001; // Longitude_unavailable

	// Check if the altitude is negative and parses accondingly using stod
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
	else out_nmea.alt = 800001; // AltitudeValue_unavailable

	// Produces and processes the current date-time timestamp
	std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	strcpy(out_nmea.ts_pos,std::ctime(&now));

	// Uncomment this line to join the position timestamp and the UTC time string
	// strcat(out_nmea.ts_pos, utc_time.data());
	strcpy(out_nmea.ts_utc_time_nmea, utc_time.data());

	// Gets the update time with precision of microseconds
	auto update = time_point_cast<microseconds>(system_clock::now());

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
    m_alt_valid = true;

    // Updates the buffer
	m_outBuffer.store(out_nmea);
}

/** Checks and parses a GNGGA NMEA sentence
 *  Example: $GPGGA,092725.00,4717.11399,N,00833.91590,E,1,08,1.01,499.6,M,48.0,M,,*5B\r\n
 */
void
UBXNMEAParserSingleThread::parseNmeaGga(std::string nmea_response) {
	out_t out_nmea = m_outBuffer.load();

    std::vector<std::string> fields;
    std::stringstream ss(nmea_response);
    std::string field;

    while (std::getline(ss, field, ',')) {
        fields.push_back(field);
    }

	// Latitude and Longitude substrings to be converted to double using std::stod() later
	std::string utc_time = "UTC Time: " + fields[1];
    std::string slat = fields[2];
    std::string slon = fields[4];
    std::string salt = fields[9];

    // cardinal points (N, S, W, E)
	char cp_lat = fields[3].at(0);
    char cp_lon = fields[5].at(0);

	if (!slat.empty()) {
        out_nmea.lat = decimal_deg(std::stod(slat),cp_lat);
        out_nmea.lat_nmea = out_nmea.lat;
    }
	else out_nmea.lat = 900000001; // Latitude_unavailable

	if (!slon.empty()) {
        out_nmea.lon = decimal_deg(std::stod(slon),cp_lon);
        out_nmea.lon_nmea = out_nmea.lon;
    }
	else out_nmea.lon = 1800000001; //Longitude_unavailable

	// Check if the altitude is negative and parses accondingly using stod
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
	else out_nmea.alt = 800001; //AltitudeValue_unvailable

	// Produces and processes the current date-time timestamp
	std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	strcpy(out_nmea.ts_pos,std::ctime(&now));

	// Uncomment this line to join the position timestamp and the UTC time string
	// strcat(out_nmea.ts_pos, utc_time.data());
	strcpy(out_nmea.ts_utc_time_nmea, utc_time.data());

	// Gets the update time with precision of microseconds
	auto update = time_point_cast<microseconds>(system_clock::now());

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
    m_alt_valid = true;

    // Updates the buffer
	m_outBuffer.store(out_nmea);
}

/** getAltitude() provides the altitude above sea level
 *  taken from the GNS/GGA NMEA sentences
 */
double
UBXNMEAParserSingleThread::getAltitude(long *age_us, bool print_timestamp_and_age) {
	if(m_parser_started == false) {
		std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
		return 0;
	}

	out_t tmp = m_outBuffer.load();

    auto now = time_point_cast<microseconds>(system_clock::now());
    auto end = now.time_since_epoch().count();
    long local_age_us = end - tmp.lu_alt;

    if (m_debug_age_info_rate) m_debug_age_info.age_alt = local_age_us;

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

double
UBXNMEAParserSingleThread::getAltitudeUbx(long *age_us, bool print_timestamp_and_age) {
    if(m_parser_started == false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return 0;
    }

    out_t tmp = m_outBuffer.load();

    auto now = time_point_cast<microseconds>(system_clock::now());
    auto end = now.time_since_epoch().count();
    long local_age_us = end - tmp.lu_alt_ubx;

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

double
UBXNMEAParserSingleThread::getAltitudeNmea(long *age_us, bool print_timestamp_and_age) {
    if(m_parser_started == false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return 0;
    }

    out_t tmp = m_outBuffer.load();

    auto now = time_point_cast<microseconds>(system_clock::now());
    auto end = now.time_since_epoch().count();
    long local_age_us = end - tmp.lu_alt_nmea;

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

/** getAccelerations(), getAngularRates(), getYawRate(), getRawAccelerations() and getAttitude()
 *  follow the same structure as getPosition but with the use of
 *  std::tuple instead of std::pair
 * 
 *  Note: use std::get<index>(object) to access the elements of a tuple. */
std::tuple<double,double,double>
UBXNMEAParserSingleThread::getAccelerations(long *age_us, bool print_timestamp_and_age) {
    if(m_parser_started == false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return std::make_tuple(0,0,0);
    }

	out_t tmp = m_outBuffer.load();

	auto now = time_point_cast<microseconds>(system_clock::now());
	auto end = now.time_since_epoch().count();
    long local_age_us = end - tmp.lu_comp_acc;

    if (m_debug_age_info_rate) m_debug_age_info.age_comp_acc = local_age_us;

    if (local_age_us >= getValidityThreshold()) {
        m_comp_acc_valid.store(false);
    }
    else m_comp_acc_valid.store(true);

    if (print_timestamp_and_age == true) std::cout << "[Accelerations] - " <<  tmp.ts_comp_acc
                                                   << "Age of information: " << local_age_us << " us"
                                                   << std::endl << std::endl;
    if (age_us != nullptr) *age_us = local_age_us;

	return std::make_tuple(tmp.comp_acc_x,tmp.comp_acc_y,tmp.comp_acc_z);
}

std::tuple<double,double,double>
UBXNMEAParserSingleThread::getAngularRates(long *age_us, bool print_timestamp_and_age) {
    if(m_parser_started == false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return std::make_tuple(0,0,0);
    }

    out_t tmp = m_outBuffer.load();

    auto now = time_point_cast<microseconds>(system_clock::now());
    auto end = now.time_since_epoch().count();
    long local_age_us = end - tmp.lu_comp_ang_rate;

    if (m_debug_age_info_rate) m_debug_age_info.age_comp_ang_rate = local_age_us;

    if (local_age_us >= getValidityThreshold()) {
        m_comp_ang_rate_valid.store(false);
    }
    else m_comp_ang_rate_valid.store(true);

    if (print_timestamp_and_age == true) std::cout << "[Angular Rates] - " <<  tmp.ts_comp_ang_rate
                                                   << "Age of information: " << local_age_us << " us"
                                                   << std::endl << std::endl;
    if (age_us != nullptr) *age_us = local_age_us;

    return std::make_tuple(tmp.comp_ang_rate_x,tmp.comp_ang_rate_y,tmp.comp_ang_rate_z);
}

double
UBXNMEAParserSingleThread::getYawRate(long *age_us, bool print_timestamp_and_age) {
    if(m_parser_started == false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return 0;
    }

    out_t tmp = m_outBuffer.load();

    auto now = time_point_cast<microseconds>(system_clock::now());
    auto end = now.time_since_epoch().count();
    long local_age_us = end - tmp.lu_comp_ang_rate;

    if (m_debug_age_info_rate) m_debug_age_info.age_comp_ang_rate = local_age_us;

    if (local_age_us >= getValidityThreshold()) {
        m_comp_ang_rate_valid.store(false);
    }
    else m_comp_ang_rate_valid.store(true);

    if (print_timestamp_and_age == true) std::cout << "[Yaw Rate timestamp] - " <<  tmp.ts_comp_ang_rate
                                                   << "Age of information: " << local_age_us << " us"
                                                   << std::endl << std::endl;

    if (age_us != nullptr) *age_us = local_age_us;

    return tmp.comp_ang_rate_z;
}

double
UBXNMEAParserSingleThread::getLongitudinalAcceleration(long *age_us, bool print_timestamp_and_age) {
    if(m_parser_started == false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return 0;
    }

    out_t tmp = m_outBuffer.load();

    auto now = time_point_cast<microseconds>(system_clock::now());
    auto end = now.time_since_epoch().count();
    long local_age_us = end - tmp.lu_comp_acc;

    if (m_debug_age_info_rate) m_debug_age_info.age_comp_acc = local_age_us;

    if (local_age_us >= getValidityThreshold()) {
        m_comp_acc_valid.store(false);
    }
    else m_comp_acc_valid.store(true);

    if (print_timestamp_and_age == true) std::cout << "[Longitudinal Acceleration timestamp] - " <<  tmp.ts_comp_acc
                                                   << "Age of information: " << local_age_us << " us"
                                                   << std::endl << std::endl;
    if (age_us != nullptr) *age_us = local_age_us;

    return tmp.comp_acc_x;
}

std::tuple<double,double,double>
UBXNMEAParserSingleThread::getRawAccelerations(long *age_us, bool print_timestamp_and_age) {
    if(m_parser_started==false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return std::make_tuple(0,0,0);
    }

	out_t tmp = m_outBuffer.load();

    auto now = time_point_cast<microseconds>(system_clock::now());
    auto end = now.time_since_epoch().count();
    long local_age_us = end - tmp.lu_acc;

    if (m_debug_age_info_rate) m_debug_age_info.age_acc = local_age_us;

    if (local_age_us >= getValidityThreshold()) {
        m_acc_valid.store(false);
    }
    else m_acc_valid.store(true);

    if (print_timestamp_and_age == true) std::cout << "[Raw Accelerations] - " <<  tmp.ts_acc
                                                   << "Age of information: " << local_age_us << " us"
                                                   << std::endl << std::endl;

    if (age_us != nullptr) *age_us = local_age_us;

	return std::make_tuple(tmp.raw_acc_x,tmp.raw_acc_y,tmp.raw_acc_z);
}

/** Parses a UBX-ESF-RAW message which payload is composed by a 4 bytes of reserved space
 *  plus a repeated group of 8 bytes whose first 4 bytes contain the relevant information
 *  about the acceleratin in the x,y and z axes.
 *
 *  The function reads the first 4 bytes of the first 3 repeated group
 *  (3 bytes for acceleration value + 1 byte to identify the axis)
 *  but it can be extended in order to read also the gyroscope data
 *  and to include the sTag field too (see the for loop below)
 *
 *  (more details in 3.11.4 Interface description: UBX-ESF-RAW) */
void
UBXNMEAParserSingleThread::parseEsfRaw(std::vector<uint8_t> response) {

	// Skips the first 4 bytes of the payload (reserved)
    int offset = m_UBX_PAYLOAD_OFFSET + 4;

    int j = 0; // Resets every 8 bytes to isolate the repeated group
    std::vector<uint8_t> esf_data; // Temporarily stores the 3 bytes of interest

    out_t out_esf = m_outBuffer.load();

    for (int i = offset; i < offset + 24; i++) { // Ignores gyroscope data on the payload

    	esf_data.push_back(response[i]); j++; // Loads the ESF-RAW repeated group (data + sTag)

    	if (j == 8) {

            /* Stops every 8 bytes to check and parse the data type
             * (16 (0x10),17 (0x11) or 18 (0x12) for XYZ acceleration values)*/
            if (esf_data[3] == 0x10) {

            	// Erases the unneeded bytes (byte 3 to identify the data type and bytes 4,5,6 and 7 (sTag field))
            	esf_data.erase(esf_data.begin()+3,esf_data.end());

            	// Converts the resulting array of 3 bytes to big endian by reversing it
            	std::reverse(esf_data.begin(),esf_data.end());

            	/* Produces the final signed value and scales it, dividing by 2^10
            	 * as indicated in 3.2.7.5 Sensor data types (Integration manual) */
            	out_esf.raw_acc_x = static_cast<double>(hexToSigned(esf_data))/1024;

				esf_data.clear();
				j = 0;
			}
            if (esf_data[3] == 0x11) {
            	esf_data.erase(esf_data.begin()+3,esf_data.end());
            	std::reverse(esf_data.begin(),esf_data.end());
            	out_esf.raw_acc_y = static_cast<double>(hexToSigned(esf_data))/1024;
            	esf_data.clear();
				j = 0;
			}
            if (esf_data[3] == 0x12) {
            	esf_data.erase(esf_data.begin()+3,esf_data.end());
            	std::reverse(esf_data.begin(),esf_data.end());
            	out_esf.raw_acc_z = static_cast<double>(hexToSigned(esf_data))/1024;
            	esf_data.clear();
				j = 0;
			}
        }
    }

	// Produces and prints to output struct the current date-time specific timestamp
	std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	strcpy(out_esf.ts_acc,std::ctime(&now));

	// Gets the update time with precision of microseconds
	auto update = time_point_cast<microseconds>(system_clock::now());

    // Updates age of information
    if (m_debug_age_info_rate) {
        m_debug_age_info.age_acc = update.time_since_epoch().count() - out_esf.lu_acc;
    }

    // Retrieves the time since epoch in microseconds and updates the output struct
	out_esf.lu_acc = update.time_since_epoch().count();

    // Validates data
    m_acc_valid = true;

    // Updates the buffer
    m_outBuffer.store(out_esf);
}

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
        m_sog_cog_ubx_valid.store(false);
    }
    else m_sog_cog_ubx_valid.store(true);

    if (print_timestamp_and_age == true) std::cout << "[Speed timestamp - UBX] - " <<  tmp.ts_sog_cog_ubx
                                                   << "Age of information: " << local_age_us << " us"
                                                   << std::endl << std::endl;

    if (age_us != nullptr) *age_us = local_age_us;

    return tmp.sog_ubx;
}

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
        m_sog_cog_nmea_valid.store(false);
    }
    else m_sog_cog_nmea_valid.store(true);

    if (print_timestamp_and_age == true) std::cout << "[Speed timestamp - NMEA] - " <<  tmp.ts_sog_cog_nmea
                                                   << "Age of information: " << local_age_us << " us"
                                                   << std::endl << std::endl;
    if (age_us != nullptr) *age_us = local_age_us;

    return tmp.sog_nmea;
}

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
        m_sog_cog_ubx_valid.store(false);
    }
    else m_sog_cog_ubx_valid.store(true);

    if (print_timestamp_and_age == true) std::cout << "[Course Over Ground timestamp - UBX] - " <<  tmp.ts_sog_cog_ubx
                                                   << "Age of information: " << local_age_us << " us"
                                                   << std::endl << std::endl;

    if (age_us != nullptr) *age_us = local_age_us;

	return tmp.cog_ubx;
}

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
        m_sog_cog_nmea_valid.store(false);
    }
    else m_sog_cog_nmea_valid.store(true);

    if (print_timestamp_and_age == true) std::cout << "[Course Over Ground timestamp - NMEA] - " <<  tmp.ts_sog_cog_nmea
                                                   << "Age of information: " << local_age_us << " us"
                                                   << std::endl << std::endl;
    if (age_us != nullptr) *age_us = local_age_us;

    return tmp.cog_nmea;
}

/** Retrieves the fix mode both from UBX and NMEA (checking in this order)
 *  check parseNavStatus() and parseNmeaRmc() for details. */
std::string
UBXNMEAParserSingleThread::getFixMode() {
    if(m_parser_started==false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return "PARSER_NOT_STARTED";
    }

	out_t tmp = m_outBuffer.load();

    std::string fix_ubx(tmp.fix_ubx);

    /*
    // remove this (local testing purposes. REMOVE FOR FIELD TEST)
    std::string fix_ubx = "Fix mode: 3D-Fix [UBX]";
    std::string fix_nmea = "Fix Mode: RTK Fixed (R) [NMEA]";
    m_3d_valid_fix = true;
    m_2d_valid_fix = true;
    */

	// Checks if the fix mode has already been obtained from UBX
	if (fix_ubx.empty() == true) {
        std::string fix_nmea(tmp.fix_nmea);
        if (fix_nmea.empty()) fix_nmea = "Unavailable";
        return fix_nmea;
	}
	return fix_ubx;
}

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

bool
UBXNMEAParserSingleThread::getFixValidity2D(bool print_error) {
    if (m_2d_valid_fix == false && print_error == true) {
        std::cerr << "No 2D Fix!" << std::endl;
        return false;
    }
    else return m_2d_valid_fix;
}

bool
UBXNMEAParserSingleThread::getFixValidity3D(bool print_error) {
    if (m_3d_valid_fix == false && print_error == true) {
        std::cerr << "No 3D Fix!" << std::endl;
        return false;
    }
    else return m_3d_valid_fix;
}

std::atomic<bool>
UBXNMEAParserSingleThread::getPositionValidity(bool print_error) {
    if (m_pos_valid == false && print_error == true) {
        std::cerr << "Error: Outdated position value (age > threshold)!" << std::endl;
        return false;
    } else return m_pos_valid.load();
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

bool
UBXNMEAParserSingleThread::getSpeedAndCogValidity(bool print_error) {
    if (m_sog_cog_ubx_valid == false && print_error == true) {
        std::cerr << "Error: Outdated UBX speed/course over ground value (age > threshold)!" << std::endl;
        return false;
    }
    if (m_sog_cog_nmea_valid == false && print_error == true) {
        std::cerr << "Error: Outdated NMEA speed/course over ground value (age > threshold)!" << std::endl;
        return false;
    }
    if (m_sog_cog_ubx_valid == false) return m_sog_cog_nmea_valid;
    else return m_sog_cog_ubx_valid;
}

bool
UBXNMEAParserSingleThread::setValidityThreshold(double threshold) {
    if (m_validity_threshold != 0) return true; // The threshold has already been set
    else {
        m_validity_threshold = threshold;
        return true;
    }
}

void
UBXNMEAParserSingleThread::setDebugAgeInfo(int rate) {
    if (m_debug_age_info_rate) std::cerr << "Error: debug option already enabled!" << '\n';
    else m_debug_age_info_rate = rate;
}

bool
UBXNMEAParserSingleThread::getDebugAgeInfo() {
    return m_debug_age_info_rate;
}

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

/** Extracts the speed over ground (sog, 4 bytes) and course over ground (cog, 4 bytes) values from the
 *  UBX-NAV-PVT message by reading from byte 60 to byte 68 (the two values are store one after the other
 *
 *  See Interface description § 3.15.13 UBX-NAV-PVT. (page 100) */
void
UBXNMEAParserSingleThread::parseNavPvt(std::vector<uint8_t> response) {

	out_t out_pvt = m_outBuffer.load();

	/* Extract the 4 bytes arrays containing the speed over ground and the heading of motion in the UBX-NAV-PVT message
	 * Converts the arrays in little endian and retrieves the single signed value after scaling accordingly */
	std::vector<uint8_t> sog(response.begin() + m_UBX_PAYLOAD_OFFSET + 60, response.begin() + m_UBX_PAYLOAD_OFFSET + 64);
	std::reverse(sog.begin(),sog.end());
	out_pvt.sog = static_cast<double>(hexToSigned(sog)) * 0.001; // Converts from mm/s to m/s
    out_pvt.sog_ubx = out_pvt.sog;

	std::vector<uint8_t> head_motion(response.begin() + m_UBX_PAYLOAD_OFFSET + 64, response.begin() + m_UBX_PAYLOAD_OFFSET + 68);
	std::reverse(head_motion.begin(),head_motion.end());
	out_pvt.cog = static_cast<double>(hexToSigned(head_motion)) * 0.00001;
    out_pvt.cog_ubx = out_pvt.cog;

    std::vector<uint8_t> lat(response.begin() + m_UBX_PAYLOAD_OFFSET + 28, response.begin() + m_UBX_PAYLOAD_OFFSET + 32);
    std::reverse(lat.begin(),lat.end());
    out_pvt.lat = static_cast<double>(hexToSigned(lat)) * 0.0000001;
    out_pvt.lat_ubx = out_pvt.lat;

    std::vector<uint8_t> lon(response.begin() + m_UBX_PAYLOAD_OFFSET + 24, response.begin() + m_UBX_PAYLOAD_OFFSET + 28);
    std::reverse(lon.begin(),lon.end());
    out_pvt.lon = static_cast<double>(hexToSigned(lon)) * 0.0000001;
    out_pvt.lon_ubx = out_pvt.lon;

    std::vector<uint8_t> alt(response.begin() + m_UBX_PAYLOAD_OFFSET + 36, response.begin() + m_UBX_PAYLOAD_OFFSET + 40);
    std::reverse(alt.begin(),alt.end());
    out_pvt.alt = static_cast<double>(hexToSigned(alt)) * 0.001; // Convert from mm to m
    out_pvt.alt_ubx = out_pvt.alt;

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
    m_sog_cog_ubx_valid = true;
    m_pos_valid = true;
    m_alt_valid = true;

    // Updates the buffer
    m_outBuffer.store(out_pvt);
}

/** Follows the same approach adopted in parseNmeaGns() by scanning the sentence counting
 *  the commas encountered in order to retrieve the speed and the course over ground values
 *  Example $GNRMC,083559.00,A,4717.11437,N,00833.91522,E, 0.004,77.52, 091202,,, A ,V*57\r\n
	                            ^fix validity                sog^   cog^     fix mode^

*  along with the fix mode. */
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
    if (cog.empty() == false) {
        if (cog[0] == '-') {
            cog = cog.erase(0,1);
            out_nmea.cog = std::stod(cog) * -1;
            out_nmea.cog_nmea = out_nmea.cog;
        }
        else {
            out_nmea.cog_nmea = std::stod(cog);
            out_nmea.cog_nmea = out_nmea.cog;
        }
    }
    else out_nmea.cog_nmea = 3601; // HeadingValue_unavailable

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
    else out_nmea.sog_nmea = 16383; // SpeedValue_unavailable

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
    m_sog_cog_nmea_valid = true;

    // Updates the buffer
	m_outBuffer.store(out_nmea);
}

/** Parses a UBX-NAV-STATUS message in order to get the FIX MODE of the receiver
 * 
 *  Skips the first 4 bytes of the payload and processes the following two bytes
 *  See ZED-F9R interface decription at 3.15 (page 109) for details */
void
UBXNMEAParserSingleThread::parseNavStatus(std::vector<uint8_t> response) {
	uint8_t fix_mode  = response[m_UBX_PAYLOAD_OFFSET + 4];
	uint8_t fix_flags = response[m_UBX_PAYLOAD_OFFSET + 4 + 1];
	
	// Loads the global output buffer atomic struct
	out_t out_sts = m_outBuffer.load();

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

/** Parses a UBX-ESF-INS message in order to obtain measures about the vehicle dynamics.
 *  In particular the function parses the compesated acceleration */
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

/** Reads from serial port, filtering for NMEA sentences and UBX messages and parsing accordingly */
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

    while (true) {

        byte = m_serial.ReadChar(success);

        if (!success) {
            continue;
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

                                //todo: review this
                                // Calibration checks
                                // If there are uncalibrated measures, ignore them
                                /*
                                if (ubx_message[m_UBX_PAYLOAD_OFFSET + 1] != 0x07) {
                                    std::cout << "[INFO] Accelerometer uncalibrated!" << std::endl;
                                    break;
                                }
                                if (ubx_message[m_UBX_PAYLOAD_OFFSET + 1] != 0x3F) {
                                    std::cout << "[INFO] Accelerometer and gyroscope uncalibrated!" << std::endl;
                                    break;
                                }
                                */

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
                                //todo: review this
                                // Calibration checks
                                // If there are uncalibrated measures, ignore them
                                /*
                                if (ubx_message_overlapped[m_UBX_PAYLOAD_OFFSET + 1] != 0x07) {
                                    std::cout << "[INFO] Accelerometer uncalibrated!" << std::endl;
                                    break;
                                }
                                if (ubx_message_overlapped[m_UBX_PAYLOAD_OFFSET + 1] != 0x3F) {
                                    std::cout << "[INFO] Accelerometer and gyroscope uncalibrated!" << std::endl;
                                    break;
                                }
                                */

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
        // printf("byte_previous = %02X\n",byte_previous);
    }
}

/** Enables the necessary messages for data retrieving by sending a UBX-CFG-VALSET poll request.
 * Implements also a CFG-VALSET poll request in order to disable the messages.
 * Then encapsulates "readFromSerial()" and executes it endlessly until the global atomic err_flag is raised.
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
	
	// Writes to serial port in order to enable the messages
	m_serial.Write(reinterpret_cast<char*>(ubx_valset_req_enable.data()), ubx_valset_req_enable.size());

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
	//serialDev.Write(reinterpret_cast<char*>(ubx_valset_req_disable.data()), ubx_valset_req_disable.size());


	while (m_terminatorFlagPtr->load() == false && m_stopParserFlag.load() == false) {
		readFromSerial();
	}
}

int
UBXNMEAParserSingleThread::startUBXNMEAParser(std::string device, int baudrate, int data_bits, char parity, int stop_bits, std::atomic<bool> *terminatorFlagPtr) {
	// Serial interface handling and initializing
    m_serial.SetBaudRate(baudrate);
    m_serial.SetDataSize(data_bits);
    m_serial.SetParity(parity);
    m_serial.SetStopBits(stop_bits);
    m_serial.SetPortName(device);

    clearBuffer();

    if(terminatorFlagPtr==nullptr) {
        std::cerr << "Error: Invalid pointer to terminator flag. Terminating." << std::endl;
        return -1;
    }

    m_terminatorFlagPtr = terminatorFlagPtr;

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
