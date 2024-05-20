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
#include <thread>
#include <utility>
#include <chrono>
#include <algorithm>

using namespace std::chrono;

/* Prints a UBX message in the format index[byte] (for debug purposes) */
void
UBXNMEAParserSingleThread::printUbxMessage(std::vector<uint8_t> msg) {
	// Printing message (for debug purposes)
	std::cout << "message size: " << msg.size() << std::endl;
	std::cout << "{START}| ";
	for (long unsigned int i = 0; i < msg.size(); i++) {
		printf("%ld[%02X] ", i, msg[i]);
	}
	std::cout << " |{END}" << std::endl << std::endl;
}

/* Prints a NMEA sentence (printable chars only) */
void
UBXNMEAParserSingleThread::printNmeaSentence(std::string s) {
	if (!s.empty()) {
		for (long unsigned int i = 0; i < s.size(); i++) {
			if (isprint(s[i])) std::cout << s[i];
			// comment the line below to exclude non-printable characters from the sentence
			//else printf(" (%d) ", s[i]);
		}
		std::cout << std::endl;
	} else std::cout << "The nmea sentence is empty." << std::endl;
}

/* Converts latitude and longitude from NMEA format to single values in degrees */
double
UBXNMEAParserSingleThread::decimal_deg(double value, char quadrant) {
    //concatenate int part and dec part
    int degrees = floor(value/100);
    double minutes = value - (degrees*100);
    value = degrees + minutes/60;

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
long
UBXNMEAParserSingleThread::hexToSigned(std::vector<uint8_t> data) {
	if (data.size() == 3) {
		long value = (data[0] << 16) | (data[1] << 8) | (data[2]);
		long signMask = 1 << ((data.size()*2) * 4 - 1);
		long complementMask = signMask - 1;
		return -(value & signMask) | (value & complementMask);
	} else if (data.size() == 4) {
		long value = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | (data[3]);
		long signMask = 1 << ((data.size()*2) * 4 - 1);
		long complementMask = signMask - 1;
		return -(value & signMask) | (value & complementMask);
	} else {
		std::cerr << "Fatal Error: Invalid std::vector data size. Check ESF-RAW/NAV-ATT. Terminating. ";
		*m_terminatorFlagPtr=true; // Breaks the endless loop and terminates the program
		return -1;
	}
}

void
UBXNMEAParserSingleThread::clearBuffer() {
	out_t tmp;
	strcpy(tmp.ts_pos,"Buffer empty");
	strcpy(tmp.ts_utc_time,"Buffer empty");
	strcpy(tmp.ts_acc,"Buffer empty");
	strcpy(tmp.ts_att,"Buffer empty");
	strcpy(tmp.ts_alt,"Buffer empty");
	strcpy(tmp.ts_sog_cog_ubx,"Buffer empty");
	strcpy(tmp.ts_sog_cog_nmea,"Buffer empty");
	strcpy(tmp.fix_ubx,"Buffer empty");
	strcpy(tmp.fix_nmea,"Buffer empty");
	tmp.raw_acc_x = 0;
	tmp.raw_acc_y = 0;
	tmp.raw_acc_z = 0;
	tmp.comp_acc_x = 0;
	tmp.comp_acc_y = 0;
	tmp.comp_acc_z = 0;
	tmp.lat = 0;
	tmp.lon = 0;
	tmp.alt = 0;
	tmp.cp_lat = '\0';
	tmp.cp_lon = '\0';
	tmp.roll = 0;
	tmp.pitch = 0;
	tmp.heading = 0;
	tmp.sog_ubx = 0;
	tmp.sog_nmea = 0;
	tmp.cog_ubx = 0;
	tmp.cog_nmea = 0;
	tmp.lu_pos = 0;
	tmp.lu_acc = 0;
	tmp.lu_att = 0;
	tmp.lu_alt = 0;
	tmp.lu_sog_cog_ubx = 0;
	tmp.lu_sog_cog_nmea = 0;
	m_outBuffer.store(tmp);
	//printf("\n\nBuffer cleared\n\n");
}

void
UBXNMEAParserSingleThread::printBuffer() {
	out_t tmp = m_outBuffer.load();
	printf("[UTC TIME]  - %s\n", tmp.ts_utc_time);
	printf("[UBX]  - %s\n", tmp.fix_ubx);
	printf("[NMEA] - %s\n\n", tmp.fix_nmea);
	printf("[Position]\nLat: %.8f   deg   -   Lon: %.8f   deg  -   Altitude: %.2f   m\n\n",decimal_deg(tmp.lat,tmp.cp_lat),decimal_deg(tmp.lon,tmp.cp_lon),tmp.alt);
	printf("[Speed over ground]\n(UBX): %.3f m/s   -   (NMEA): %.3f m/s\n\n",tmp.sog_ubx,tmp.sog_nmea);
	printf("[Course over ground]\n(UBX): %.3f deg   -   (NMEA): %.3f deg\n\n",tmp.cog_ubx,tmp.cog_nmea);
	printf("[Accelerations (gravity-free)]\nX: %.3f  m/s^2  Y: %.3f  m/s^2  Z: %.3f  m/s^2\n\n",tmp.comp_acc_x,tmp.comp_acc_y,tmp.comp_acc_z);
	printf("[Raw accelerations]\nX: %.3f  m/s^2  Y: %.3f  m/s  Z: %.3f  m/s\n\n",tmp.raw_acc_x,tmp.raw_acc_y,tmp.raw_acc_z);
	printf("[Attitude]\nRoll: %.3f  deg  Pitch: %.3f  deg  Heading: %.3f  deg\n\n",tmp.roll,tmp.pitch,tmp.heading);
}

/** Retrives and processes the specific position timestamp, calculates
 *  the age of information in microseconds, then returns latitude and.
 *  longitude as a std::pair, optionally returns the age of info, if specified
 * 
 *  Note: use first() and second() to access the element of a pair */
std::pair<double,double>
UBXNMEAParserSingleThread::getPosition(double *age_us, bool print_timestamp) {
    if(m_parser_started==false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return std::make_pair(0,0);
    }

	out_t tmp = m_outBuffer.load();
	if (age_us != nullptr) {
		if (print_timestamp == true) std::cout << "[Position] " << tmp.ts_pos;

		// Gets current time with precision of microseconds
		auto now = time_point_cast<microseconds>(system_clock::now());

		// Converts time_point to microseconds
		auto end = now.time_since_epoch().count();

		*age_us = (double)end - tmp.lu_pos;
	}
	return std::make_pair(decimal_deg(tmp.lat,tmp.cp_lat),decimal_deg(tmp.lon,tmp.cp_lon));
}

/** Checks and parses a GNGNS NMEA sentence in order to get the latitude and longitude data
 *
 *  The function scans the sentence by counting the commas encountered.
 *  The substring sought is found between commas 2 and 5.
 *  Example: $GNGNS,133033.40,   4503.87250,N,00739.68967,E   ,RRRR,19,0.86,250.6,47.2,1.4,0000,V*3B
 *
 *  After parsing, it produces the current information specific timestamp and updates the output buffer. */
void
UBXNMEAParserSingleThread::parseNmeaGns(std::string nmea_response) {
	out_t out_nmea = m_outBuffer.load();

	int commas = 0;

	// Latitude and Longitude substrings to be converted to double using std::stod() later
	std::string utc_time = "UTC Time: ", slat, slon, salt;
	char cp_lat = '\0', cp_lon = '\0'; // cardinal points (N, S, W, E)

	for (long unsigned int i = 0; i < nmea_response.size(); i++) {
		if (nmea_response[i] == ',') {
			commas++;
		}
		// UTC time (hhmmss.ss)
		if (commas == 1){
			if (nmea_response[i+1] != ',') {
				utc_time += nmea_response[i+1];
			}
		}
		// Latitude string
		if (commas == 2){
			if (nmea_response[i+1] != ',') {
				slat += nmea_response[i+1];
			}
		}
		// Latitude cardinal point
		if (commas == 3) {
			if (cp_lat != '\0') continue;
			cp_lat = nmea_response[i+1];
		}
		// Longitude string
		if (commas == 4){
			if (nmea_response[i+1] != ',') {
				slon += nmea_response[i+1];
			}
		}
		// Longitude cardinal point
		if (commas == 5) {
			if (cp_lon != '\0') continue;
			cp_lon = nmea_response[i+1];
		}
		// Altitude above mean sea level
		if (commas == 9) {
			if(nmea_response[i+1] != ',') {
				salt += nmea_response[i+1];
			}
		}
	}

	if (slat.empty() == false) out_nmea.lat = std::stod(slat);
	else out_nmea.lat = 0;

	if (slon.empty() == false) out_nmea.lon = std::stod(slon);
	else out_nmea.lon = 0;

	// Check if the altitude is negative and parses accondingly using stod
	if (salt.empty() == false) {
		if (salt[0] == '-') {
			salt = salt.erase(0,1);
			out_nmea.alt = std::stod(salt) * -1;
		}
		else
		out_nmea.alt = std::stod(salt);
	}
	else out_nmea.alt = 0;

	out_nmea.cp_lat = cp_lat;
	out_nmea.cp_lon = cp_lon;

	// Produces and processes the current date-time timestamp
	std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	strcpy(out_nmea.ts_pos,std::ctime(&now));

	// Uncomment this line to join the position timestamp and the UTC time string
	// strcat(out_nmea.ts_pos, utc_time.data());
	strcpy(out_nmea.ts_utc_time, utc_time.data());

	// Gets the update time with precision of microseconds
	auto update = time_point_cast<microseconds>(system_clock::now());

	// Converts time_point to microseconds
	out_nmea.lu_pos = update.time_since_epoch().count();

	m_outBuffer.store(out_nmea);
}

/** Checks and parses a GNGGA NMEA sentence
 *  Example: $GPGGA,092725.00,4717.11399,N,00833.91590,E,1,08,1.01,499.6,M,48.0,M,,*5B\r\n
 */
void
UBXNMEAParserSingleThread::parseNmeaGga(std::string nmea_response) {
	out_t out_nmea = m_outBuffer.load();

	int commas = 0;

	// Latitude and Longitude substrings to be converted to double using std::stod() later
	std::string utc_time = "UTC Time: ", slat, slon, salt;
	char cp_lat = '\0', cp_lon = '\0'; // cardinal points (N, S, W, E)

	for (long unsigned int i = 0; i < nmea_response.size(); i++) {
		if (nmea_response[i] == ',') {
			commas++;
		}
		// UTC time (hhmmss.ss)
		if (commas == 1){
			if (nmea_response[i+1] != ',') {
				utc_time += nmea_response[i+1];
			}
		}
		// Latitude string
		if (commas == 2){
			if (nmea_response[i+1] != ',') {
				slat += nmea_response[i+1];
			}
		}
		// Latitude cardinal point
		if (commas == 3) {
			if (cp_lat != '\0') continue;
			cp_lat = nmea_response[i+1];
		}
		// Longitude string
		if (commas == 4){
			if (nmea_response[i+1] != ',') {
				slon += nmea_response[i+1];
			}
		}
		// Longitude cardinal point
		if (commas == 5) {
			if (cp_lon != '\0') continue;
			cp_lon = nmea_response[i+1];
		}
		// Altitude above mean sea level
		if (commas == 9) {
			if (nmea_response[i+1] != ',') {
				salt += nmea_response[i+1];
			}
		}
	}

	if (slat.empty() == false) out_nmea.lat = std::stod(slat);
	else out_nmea.lat = 0;

	if (slon.empty() == false) out_nmea.lon = std::stod(slon);
	else out_nmea.lon = 0;

	// Check if the altitude is negative and parses accondingly using stod
	if (salt.empty() == false) {
		if (salt[0] == '-') {
            salt = salt.erase(0,1);
            out_nmea.alt = std::stod(salt) * -1;
		}
		else
		out_nmea.alt = std::stod(salt);
	}
	else out_nmea.alt = 0;

	out_nmea.cp_lat = cp_lat;
	out_nmea.cp_lon = cp_lon;

	// Produces and processes the current date-time timestamp
	std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	strcpy(out_nmea.ts_pos,std::ctime(&now));

	// Uncomment this line to join the position timestamp and the UTC time string
	// strcat(out_nmea.ts_pos, utc_time.data());
	strcpy(out_nmea.ts_utc_time, utc_time.data());

	// Gets the update time with precision of microseconds
	auto update = time_point_cast<microseconds>(system_clock::now());

	// Converts time_point to microseconds
	out_nmea.lu_pos = update.time_since_epoch().count();

	m_outBuffer.store(out_nmea);
}

/** getAltitude() provides the altitude above sea level
 *  taken from the GNS/GGA NMEA sentences
 */
double
UBXNMEAParserSingleThread::getAltitude(double *age_us, bool print_timestamp) {
	if(m_parser_started == false) {
		std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
		return 0;
	}

	out_t tmp = m_outBuffer.load();

	if (age_us != nullptr) {
		if (print_timestamp == true) std::cout << "[Altitude] " << tmp.ts_alt;

		auto now = time_point_cast<microseconds>(system_clock::now());
		auto end = now.time_since_epoch().count();

		*age_us = (double)end - tmp.lu_alt;
	}
	return tmp.alt;
}

/** getAccelerations(), getRawAccelerations() and getAttitude()
 *  follow the same structure as getPosition but with the use of
 *  std::tuple instead of std::pair
 * 
 *  Note: use std::get<index>(object) to access the elements of a tuple. */
std::tuple<double,double,double>
UBXNMEAParserSingleThread::getAccelerations(double *age_us, bool print_timestamp) {
    if(m_parser_started==false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return std::make_tuple(0,0,0);
    }

	out_t tmp = m_outBuffer.load();
	if (age_us != nullptr) {
		if (print_timestamp == true) std::cout << "[Accelerations (gravity-free)] " << tmp.ts_comp_acc;

		auto now = time_point_cast<microseconds>(system_clock::now());
		auto end = now.time_since_epoch().count();

		*age_us = (double)end - tmp.lu_comp_acc;
	}
	return std::make_tuple(tmp.comp_acc_x,tmp.comp_acc_y,tmp.comp_acc_z);
}

std::tuple<double,double,double>
UBXNMEAParserSingleThread::getRawAccelerations(double *age_us, bool print_timestamp) {
    if(m_parser_started==false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return std::make_tuple(0,0,0);
    }

	out_t tmp = m_outBuffer.load();
	if (age_us != nullptr) {
		if (print_timestamp == true) std::cout << "[Raw Accelerations] " << tmp.ts_acc;

		auto now = time_point_cast<microseconds>(system_clock::now());
		auto end = now.time_since_epoch().count();

		*age_us = (double)end - tmp.lu_acc;
	}
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
            	out_esf.raw_acc_x = (double)hexToSigned(esf_data)/1024;

				esf_data.clear();
				j = 0;
			}
            if (esf_data[3] == 0x11) {
            	esf_data.erase(esf_data.begin()+3,esf_data.end());
            	std::reverse(esf_data.begin(),esf_data.end());
            	out_esf.raw_acc_y = (double)hexToSigned(esf_data)/1024;
            	esf_data.clear();
				j = 0;
			}
            if (esf_data[3] == 0x12) {
            	esf_data.erase(esf_data.begin()+3,esf_data.end());
            	std::reverse(esf_data.begin(),esf_data.end());
            	out_esf.raw_acc_z = (double)hexToSigned(esf_data)/1024;
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

	// Retrieves the time since epoch in microseconds and updates the output struct
	out_esf.lu_acc = update.time_since_epoch().count();

    m_outBuffer.store(out_esf);
}

std::tuple<double,double,double>
UBXNMEAParserSingleThread::getAttitude(double *age_us, bool print_timestamp) {
    if(m_parser_started==false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return std::make_tuple(0,0,0);
    }

	out_t tmp = m_outBuffer.load();
	if (age_us != nullptr) {
		if (print_timestamp == true) std::cout << "[Attitude] " << tmp.ts_att;

		auto now = time_point_cast<microseconds>(system_clock::now());
		auto end = now.time_since_epoch().count();

		*age_us = (double)end - tmp.lu_att;
	}
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
			out_att.roll = hexToSigned(att_data) * 0.00001;

			att_data.clear();
		}
		if (i == 22) {
			std::reverse(att_data.begin(),att_data.end());
			out_att.pitch = hexToSigned(att_data) * 0.00001;
			att_data.clear();
		}
		if (i == 26) {
			std::reverse(att_data.begin(),att_data.end());
			out_att.heading = hexToSigned(att_data) * 0.00001;
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

	// Retrieves the time since epoch in microseconds and updates the output struct
	out_att.lu_att = update.time_since_epoch().count();

	m_outBuffer.store(out_att);
}

/** The following two funcitions act similarly to the ones defined before
 *  but when specified, they will provide the latest value retrieved
 *  specifying if it is from UBX or NMEA (in this priority order) */
double
UBXNMEAParserSingleThread::getSpeed(double *age_us, bool print_timestamp_and_age) {
    if(m_parser_started==false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return 0;
    }

	out_t tmp = m_outBuffer.load();
	if (age_us != nullptr) {
		auto now = time_point_cast<microseconds>(system_clock::now());
		auto end = now.time_since_epoch().count();

		double age_ubx = end - tmp.lu_sog_cog_ubx;
		double age_nmea = end - tmp.lu_sog_cog_nmea;

		if (age_ubx <= age_nmea) {
			if (print_timestamp_and_age == true) std::cout << "[Speed - UBX]  " <<  tmp.ts_sog_cog_ubx
			                                     << "Age of information: " << age_ubx << " us"
			                                     << std::endl << std::endl;
			*age_us = age_ubx;
		} else {
			if (print_timestamp_and_age == true) std::cout << "[Speed - NMEA] " <<  tmp.ts_sog_cog_ubx
			                                     << "Age of information: " << age_nmea << " us"
			                                     << std::endl << std::endl;
			*age_us = age_nmea;
		}
	}
	if (tmp.sog_ubx != 0) return tmp.sog_ubx;
	return tmp.sog_nmea;
}

double
UBXNMEAParserSingleThread::getCourseOverGround(double *age_us, bool print_timestamp_and_age) {
    if(m_parser_started==false) {
        std::cerr << "Error: The parser has not been started. Call startUBXNMEAParser() first." << std::endl;
        return 0;
    }

	out_t tmp = m_outBuffer.load();
	if (age_us != nullptr) {
		auto now = time_point_cast<microseconds>(system_clock::now());
		auto end = now.time_since_epoch().count();

		double age_ubx = end - tmp.lu_sog_cog_ubx;
		double age_nmea = end - tmp.lu_sog_cog_nmea;

		if (age_ubx <= age_nmea) {
			if (print_timestamp_and_age == true) std::cout << "[Course Over Ground - UBX]  " <<  tmp.ts_sog_cog_ubx << "Age of Information: " << age_ubx << " us" << std::endl;;
			*age_us = age_ubx;
		} else {
			if (print_timestamp_and_age == true) std::cout << "[Course Over Ground - NMEA] " << tmp.ts_sog_cog_nmea << "Age of Information: " << age_nmea << " us" << std::endl;
			*age_us = age_nmea;
		}
	}
	if (tmp.cog_ubx != 0) return tmp.cog_ubx;
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
	std::string fix(tmp.fix_ubx);

	// Checks if the fix mode has been already obtained from UBX
	if (fix.empty() == true || strstr(fix.data(),"Buffer empty") != nullptr) {
		fix.clear();
		strcpy(fix.data(),tmp.fix_nmea);
		return fix;
	}

	return fix;
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
	out_pvt.sog_ubx = hexToSigned(sog) * 0.001; // Converts from mm/s to m/s

	std::vector<uint8_t> head_motion(response.begin() + m_UBX_PAYLOAD_OFFSET + 64, response.begin() + m_UBX_PAYLOAD_OFFSET + 68);
	std::reverse(head_motion.begin(),head_motion.end());
	out_pvt.cog_ubx = hexToSigned(head_motion) * 0.00001;

	// Produces and prints to struct the current date-time timestamp
	std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	strcpy(out_pvt.ts_sog_cog_ubx,std::ctime(&now));

	// Gets the update time with precision of microseconds
	auto update = time_point_cast<microseconds>(system_clock::now());

	// Retrieves the time since epoch in microseconds and updates the output struct
	out_pvt.lu_sog_cog_ubx = update.time_since_epoch().count();

	m_outBuffer.store(out_pvt);
}

/** Follows the same approach adopted in parseNmeaGns() by scanning the sentence counting
 *  the commas encountered in order to retrieve the speed and the course over ground values
 *  along with the fix mode. */
void
UBXNMEAParserSingleThread::parseNmeaRmc(std::string nmea_response) {
	/* Example $GNRMC,083559.00,A,4717.11437,N,00833.91522,E, 0.004,77.52, 091202,,, A ,V*57\r\n
	                                                         sog^   cog^     fix mode^        */
	out_t out_nmea = m_outBuffer.load();

	int commas = 0, j = 0;

	// Speed over ground, Course over ground, Fix mode
	std::string sog, cog;
	char fix = '\0';

	for (long unsigned int i = 0; i < nmea_response.size(); i++) {
		if (nmea_response[i] == ',') {
			commas++;
			j = 0;
		}
		if (commas == 7) {
			if (nmea_response[i+1] != ',') {
				sog += nmea_response[i+1];
				j++;
			}
		}
		if (commas == 8) {
			if (nmea_response[i+1] != ',') {
				cog += nmea_response[i+1];
				j++;
			}
		}
		if (commas == 12) {
			if (nmea_response[i+1] != ',') {
				fix = nmea_response[i+1];
				j++;
			}
		}
	}

	if (sog.empty() == false) out_nmea.sog_nmea =  std::stod(sog) * 0.5144; //Coversion from knots to m/s
	else out_nmea.sog_nmea = 0;

	if (cog.empty() == false) out_nmea.cog_nmea = std::stod(cog);
	else out_nmea.cog_nmea = 0;

	switch (fix) {
		case 'N':
			strcpy(out_nmea.fix_nmea,"Fix Mode: No Fix (N)");
			break;
		case 'E':
			strcpy(out_nmea.fix_nmea,"Fix Mode: Estimated/Dead Reckoning Fix (E)");
			break;
		case 'A':
			strcpy(out_nmea.fix_nmea,"Fix Mode: Autonomous GNSS Fix (A)");
			break;
		case 'D':
			strcpy(out_nmea.fix_nmea,"Fix Mode: Differential GNSS Fix (D)");
			break;
		case 'F':
			strcpy(out_nmea.fix_nmea,"Fix Mode: RTK Float (F)");
			break;
		case 'R':
			strcpy(out_nmea.fix_nmea,"Fix Mode: RTK Fixed (R)");
			break;
		default:
            strcpy(out_nmea.fix_nmea,"Fix Mode: Unknown (?)");
            break;
	}

	// Produces and prints the current date-time timestamp
	std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	strcpy(out_nmea.ts_sog_cog_nmea,std::ctime(&now));

	// Gets the update time with precision of microseconds
	auto update = time_point_cast<microseconds>(system_clock::now());

	// Converts time_point to microseconds
	out_nmea.lu_sog_cog_nmea = update.time_since_epoch().count();

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
        strcpy(out_sts.fix_ubx, "Invalid fix (outside limits) and/or NO RTK");
		m_outBuffer.store(out_sts);
		return;
    }

	switch (fix_mode) {
		case 0:
			strcpy(out_sts.fix_ubx,"No Fix Detected!");
			break;
		case 1:
			strcpy(out_sts.fix_ubx,"Fix mode: Dead reckoning only");
			break;
		case 2:
			strcpy(out_sts.fix_ubx,"Fix mode: 2D-Fix");
			break;
		case 3:
			strcpy(out_sts.fix_ubx,"Fix mode: 3D-Fix");
			break;
		case 4:
			strcpy(out_sts.fix_ubx,"Fix mode: GPS + dead reckoning combined");
			break;
		case 5:
			strcpy(out_sts.fix_ubx,"Fix mode: Time-only Fix");
			break;
		default:
			strcpy(out_sts.fix_ubx,"Unknown fix mode");
			break;
	}
	
	// Updates the output buffer with new data
	m_outBuffer.store(out_sts);
}

/** Parses a UBX-ESF-INS message in order to obtain measures about the vehicle dynamics.
 *  In particular the function parses the compesated acceleration */
void
UBXNMEAParserSingleThread::parseEsfIns(std::vector<uint8_t> response) {
	out_t out_ins = m_outBuffer.load();

	std::vector<uint8_t> comp_acc_x(response.begin() + m_UBX_PAYLOAD_OFFSET + 24, response.begin() + m_UBX_PAYLOAD_OFFSET + 27);
	std::reverse(comp_acc_x.begin(),comp_acc_x.end());
	out_ins.comp_acc_x = hexToSigned(comp_acc_x) * 0.01; // Accordingly scales to m/s^2

	std::vector<uint8_t> comp_acc_y(response.begin() + m_UBX_PAYLOAD_OFFSET + 28, response.begin() + m_UBX_PAYLOAD_OFFSET + 31);
	std::reverse(comp_acc_x.begin(),comp_acc_x.end());
	out_ins.comp_acc_y = hexToSigned(comp_acc_y) * 0.01;

	std::vector<uint8_t> comp_acc_z(response.begin() + m_UBX_PAYLOAD_OFFSET + 32, response.begin() + m_UBX_PAYLOAD_OFFSET + 35);
	std::reverse(comp_acc_x.begin(),comp_acc_x.end());
	out_ins.comp_acc_z = hexToSigned(comp_acc_z) * 0.01; // Accordingly scales to m/s^2

	// Produces and prints to struct the current date-time timestamp
	std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	strcpy(out_ins.ts_comp_acc,std::ctime(&now));

	// Gets the update time with precision of microseconds
	auto update = time_point_cast<microseconds>(system_clock::now());

	// Retrieves the time since epoch in microseconds and updates the output struct
	out_ins.lu_comp_acc = update.time_since_epoch().count();

	m_outBuffer.store(out_ins);
}

/** Reads from serial port, filtering for NMEA sentences and UBX messages and parsing accordingly */
void
UBXNMEAParserSingleThread::readFromSerial() {
	std::vector<uint8_t> ubx_message;
	std::string nmea_sentence;

	uint8_t byte;
	bool success = false;
	bool started_ubx = false;
	bool started_nmea = false;
	long unsigned int expectedLength = 0;
	std::string expectedLengthStr;

	while (true) {
		byte = m_serial.ReadChar(success);
        if(!success) {
            continue; // If the read operation fails, ignore the current value of "byte"
        }

        printf("%c",byte);

		// NMEA sentences reading and parsing
		if (started_nmea == false) {
			if (byte == '$') {
				started_nmea = true;
				nmea_sentence.push_back(byte);
			}
		}
		else {
			if (byte != '$') nmea_sentence.push_back(byte);
			if (byte == '\n') {
				if(strstr(nmea_sentence.data(),"GNGNS") != nullptr || strstr(nmea_sentence.data(),"GPGNS") != nullptr) parseNmeaGns(nmea_sentence);
				if(strstr(nmea_sentence.data(),"GNRMC") != nullptr || strstr(nmea_sentence.data(),"GPRMC") != nullptr) parseNmeaRmc(nmea_sentence);
				if(strstr(nmea_sentence.data(),"GNGGA") != nullptr || strstr(nmea_sentence.data(),"GPRMC") != nullptr) parseNmeaGga(nmea_sentence);
				nmea_sentence.clear();
				break;
			}
		}

		// UBX Messages reading and parsing
		if (started_ubx == false) {
			if (byte == m_UBX_HEADER[1]) {
				started_ubx = true;
				ubx_message.push_back(byte);
			} else if (byte == m_UBX_HEADER[0]) {
				ubx_message.push_back(byte);
			}
		} else {
			ubx_message.push_back(byte);
			if (ubx_message.size() == 6) {
				sprintf(expectedLengthStr.data(),"%02X%02X",ubx_message[5],ubx_message[4]);
				expectedLength = std::stol(expectedLengthStr,nullptr,16) + 8; // 8 bytes for header and checksum
				expectedLengthStr.clear();
			}
			if (expectedLength > 0 && ubx_message.size() == expectedLength) {

				// Configuration messages
				if (ubx_message[2] == 0x05 && ubx_message[3] == 0x00) {
					std::cerr << "Configuration Error: CFG-ACK-NAK received. Terminating." << std::endl;
                    *m_terminatorFlagPtr=true;
				}

				// Data messages
				if (ubx_message[2] == 0x01 && ubx_message[3] == 0x03) parseNavStatus(ubx_message);
				if (ubx_message[2] == 0x10 && ubx_message[3] == 0x03) parseEsfRaw(ubx_message);
				if (ubx_message[2] == 0x10 && ubx_message[3] == 0x15) parseEsfIns(ubx_message);
				if (ubx_message[2] == 0x01 && ubx_message[3] == 0x05) parseNavAtt(ubx_message);
				if (ubx_message[2] == 0x01 && ubx_message[3] == 0x07) parseNavPvt(ubx_message);

				//printUbxMessage(ubx_message);
				ubx_message.clear();
				break;
			}
		}
		if (expectedLength > 0 && ubx_message.size() == expectedLength) {
			break;
		}
	}
}

/** Enables the necessary messages for data retrieving by sending a UBX-CFG-VALSET poll request.
 * Implements also a CFG-VALSET poll request in order to disable the messages.
 * Then encapsultates "readFromSerial()" and executes it endlessly until the global atomic err_flag is raised.
 *
 * NOTE: This function is executed in a parallel thread (see main()) */
void
UBXNMEAParserSingleThread::readData() {
	
	// Initialization and preliminary operation on data buffer
	clearBuffer();
    m_terminatorFlagPtr->store(false);

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

    if(terminatorFlagPtr==nullptr) {
        std::cerr << "Error: Invalid pointer to terminator flag. Terminating." << std::endl;
        return -1;
    }

    m_terminatorFlagPtr=terminatorFlagPtr;

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