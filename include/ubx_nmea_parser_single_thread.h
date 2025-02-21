// UBX and NMEA serial GNSS parser
// Designed for the ZED-F9R module, but compatible with any GNSS receiver (some data may not be available with non-ublox devices)
// This module allows OScar to directly retrieve positioning and other kinematics data, without the need of interfacing
// to the gpsd daemon or other external software.
// Copyright (c) 2024 Mauro Vittorio

#ifndef UBXNMEAPARSERSINGLETHREAD_H
#define UBXNMEAPARSERSINGLETHREAD_H

#include "ceSerial.h"
#include "json11.h"

#include <atomic>
#include <vector>

// Values the serial parser should return to indicate that this information is not available
#define Latitude_unavailable_serial_parser 900000001
#define Longitude_unavailable_serial_parser 1800000001
#define AltitudeValue_unavailable_serial_parser 800001

class UBXNMEAParserSingleThread {
    public:
        UBXNMEAParserSingleThread() {
            m_parser_started = false;
            m_terminatorFlagPtr = nullptr;
            m_stopParserFlag = false;
        };
        ~UBXNMEAParserSingleThread() = default;

        static void printUbxMessage(std::vector<uint8_t> msg);
        static void printNmeaSentence(std::string s);

        // Getters
        std::pair<double,double> getPosition(long *age_us, bool print_timestamp_and_age);
        std::pair<double,double> getPositionUbx(long *age_us, bool print_timestamp_and_age);
        std::pair<double,double> getPositionNmea(long *age_us, bool print_timestamp_and_age);
        std::tuple<double,double,double> getAccelerations(long *age_us, bool print_timestamp_and_age);
        std::tuple<double,double,double> getAngularRates(long *age_us, bool print_timestamp_and_age);
        std::tuple<double,double,double> getRawAccelerations(long *age_us, bool print_timestamp_and_age);
        std::tuple<double,double,double> getAttitude(long *age_us, bool print_timestamp_and_age);
        double getSpeed(long *age_us, bool print_timestamp_and_age);
        double getSpeedUbx(long *age_us, bool print_timestamp_and_age);
        double getSpeedNmea(long *age_us, bool print_timestamp_and_age);
        double getCourseOverGround(long *age_us, bool print_timestamp_and_age);
        double getCourseOverGroundUbx(long *age_us, bool print_timestamp_and_age);
        double getCourseOverGroundNmea(long *age_us, bool print_timestamp_and_age);
        double getAltitude(long *age_us, bool print_timestamp_and_age);
        double getAltitudeUbx(long *age_us, bool print_timestamp_and_age);
        double getAltitudeNmea(long *age_us, bool print_timestamp_and_age);
        double getYawRate(long *age_us, bool print_timestamp_and_age);
        double getLongitudinalAcceleration(long *age_us, bool print_timestamp_and_age);
        std::string getFixMode();
        std::string getFixModeUbx();
        std::string getFixModeNmea();
        double getValidityThreshold();
        void showDebugAgeInfo();

        // Setters
        bool setValidityThreshold(double threshold);
        void setDebugAgeInfo(int rate);

        // Validity methods
        bool validateNmeaSentence(const std::string& nmeaMessage);
        bool validateUbxMessage(std::vector<uint8_t> msg);
        bool getFixValidity2D(bool print_error);
        bool getFixValidity3D(bool print_error);
        std::atomic<int> getPositionValidity(bool print_error);
        bool getRawAccelerationsValidity(bool print_error);
        bool getAttitudeValidity(bool print_error);
        bool getAccelerationsValidity(bool print_error);
        bool getAltitudeValidity(bool print_error);
        bool getYawRateValidity(bool print_error);
        int getSpeedValidity(bool print_error);
        int getCourseOverGroundValidity(bool print_error);

        bool getDebugAgeInfo();

        int startUBXNMEAParser(std::string device, int baudrate, int data_bits, char parity, int stop_bits, std::atomic<bool> *m_terminatorFlagPtr, bool is_file_no_serial);
        void stopUBXNMEAParser();

        void setWrongInputThreshold(int threshold) {m_WRONG_INPUT_THRESHOLD=threshold;}
    private:
        /* Atomic Buffer struct that contains gnss data: includes timestamps of the information parsed,
         * information value and last update in microseconds in order to calculate the age of information.
         *
         * For data that can be obtained both from UBX and NMEA protocols, <data> contains the most recent value obtained
         * from any of the protocols while <data_protocol> stores the most recent data value, for that protocol.
         *
         * Example:
         * lat: latest latitude value (obtained from the latest parsed UBX/NMEA message/sentence)
         * lat_ubx: latest latitude value from UBX
         * lat_nmea: latest latitude value from NMEA
         *
         * The parser provide specific function to get all of three data kind, for example:
         * getPosition() provides the latest position fetching from "lat" and "lon" variables.
         * getPositionUbx() provides the latest UBX parsed position data.
         * getPositionNmea() provides the latest NMEA parsed position data */
        typedef struct Output {
            // Human-readable date strings that contain the timestamp (epoch time) of the last updates for teach information
            // Currently, the dates are used just for printing when the "print_error" flag is set to true;
            // they are not used for any other purpose (for the time being)
            char ts_pos[100],
                    ts_pos_ubx[100],
                    ts_pos_nmea[100],
                    ts_acc[100],
                    ts_att[100], // Attitude
                    ts_alt[100],
                    ts_comp_acc[100], // Compensated acceleration (without "g")
                    ts_comp_ang_rate[100], // Compensated angular rate over the z-axis
                    ts_sog_cog_ubx[100], // Speed Over Ground (SOG), Course Over Ground (COG) - UBX
                    ts_sog_cog_nmea[100]; // Speed Over Ground (SOG), Course Over Ground (COG) - NMEA
            char fix_ubx[100],
                    fix_nmea[100];
            double lat, lon, alt,
                    lat_ubx, lon_ubx, alt_ubx,
                    lat_nmea, lon_nmea, alt_nmea;           // Latitude, longitude and altitude above sea level
            double raw_acc_x, raw_acc_y, raw_acc_z,         // Raw and compensated accelerations
                   comp_acc_x, comp_acc_y, comp_acc_z,
                   comp_ang_rate_x,                         // Compensated angular rates
                   comp_ang_rate_y,
                   comp_ang_rate_z;
            double roll, pitch, heading;				    // Attitude angles
            double sog, cog,
                    sog_ubx, sog_nmea,
                    cog_ubx, cog_nmea;					    // Speed over ground and course over ground
            long lu_pos, lu_pos_ubx, lu_pos_nmea,
                 lu_acc, lu_att,
                 lu_alt, lu_alt_ubx, lu_alt_nmea,
                 lu_comp_acc,
                 lu_comp_ang_rate,
                 lu_sog_cog,
                 lu_sog_cog_ubx,
                 lu_sog_cog_nmea;						    // Last updates on relevant information
        } out_t;

        typedef struct AgeInfo {
            long age_pos, age_pos_ubx, age_pos_nmea,
                age_acc, age_att,
                age_alt, age_alt_ubx, age_alt_nmea,
                age_comp_acc,
                age_comp_ang_rate,
                age_sog_cog,
                age_sog_cog_ubx,
                age_sog_cog_nmea,						    // Last updates on relevant information
                prd_pos, prd_pos_ubx, prd_pos_nmea,
                prd_acc, prd_att,
                prd_alt, prd_alt_ubx, prd_alt_nmea,
                prd_comp_acc,
                prd_comp_ang_rate,
                prd_sog_cog,
                prd_sog_cog_ubx,
                prd_sog_cog_nmea;						    // Last periodicity ("prd") of the information update
        } age_t;

        std::atomic<out_t> m_outBuffer;
        std::atomic<bool> *m_terminatorFlagPtr = nullptr; // Used in endless loops in order to terminate the program
        std::atomic<bool> m_stopParserFlag = false;
        bool m_parser_started=false;

        // Data validity flags and user-defined threshold
        std::atomic<bool> m_2d_valid_fix = false;        // UBX-NAV-STATUS - GXGNS - GXGGA - GXRMC
        std::atomic<bool> m_3d_valid_fix = false;        // UBX-NAV-STATUS - GXGNS - GXGGA - GXRMC
        std::atomic<bool> m_pos_valid = false;
        std::atomic<bool> m_pos_valid_ubx = false;       // UBX-NAV-PVT
        std::atomic<bool> m_pos_valid_nmea = false;      // GXGGA - GXGNS
        std::atomic<bool> m_acc_valid = false;           // UBX only (UBX-ESF-RAW)
        std::atomic<bool> m_att_valid = false;           // UBX only (UBX-NAV-ATT)
        std::atomic<bool> m_alt_valid = false;
        std::atomic<bool> m_alt_valid_ubx = false;       // UBX-NAV-PVT
        std::atomic<bool> m_alt_valid_nmea = false;      // GXGGA - GXGNS
        std::atomic<bool> m_comp_acc_valid = false;      // UBX only (UBX-ESF-INS)
        std::atomic<bool> m_comp_ang_rate_valid = false; // UBX only (UBX-ESF-INS)
        std::atomic<bool> m_sog_valid = false;
        std::atomic<bool> m_sog_valid_ubx = false;       // UBX-NAV-PVT
        std::atomic<bool> m_sog_valid_nmea = false;      // GXRMC
        std::atomic<bool> m_cog_valid = false;
        std::atomic<bool> m_cog_valid_ubx = false;       // UBX-NAV-PVT
        std::atomic<bool> m_cog_valid_nmea = false;      // GXRMC

        // CLI and debug flags
        double m_validity_threshold = 0;
        int m_debug_age_info_rate = 0;

        // Age of information struct
        age_t m_debug_age_info;

        ceSerial m_serial;
        std::vector<json11::Json> m_trace;

        const std::vector<uint8_t> m_UBX_HEADER = {0xb5, 0x62};
        int m_WRONG_INPUT_THRESHOLD = 1000;

        // Flag that is set to true when reading from a JSON trace (from TRACEN-X) instead of a real serial port
        bool m_is_file_no_serial = false;

        // UBX Header (2 bytes) + message class (1 byte) + message ID (1 byte) + message length (2 bytes)
        const uint8_t m_UBX_PAYLOAD_OFFSET = 6;

        // Mathematical and buffer operations
        static double decimal_deg(double value, char quadrant);
        template <typename T = int32_t> T hexToSigned(std::vector<uint8_t> data);
        static uint8_t str_to_byte(const std::string& str);
        void clearBuffer();

        // Parsers
        void parseNmeaGns(std::string nmea_response);
        void parseNmeaGga(std::string nmea_response);
        void parseEsfRaw(std::vector<uint8_t> response);
        void parseNavAtt(std::vector<uint8_t> response);
        void parseNavPvt(std::vector<uint8_t> response);
        void parseNmeaRmc(std::string nmea_response);
        void parseNavStatus(std::vector<uint8_t> response);
        void parseEsfIns(std::vector<uint8_t> response);

        // Serial reading functions
        void readFromSerial();
        void readData();
};

#endif // UBXNMEAPARSERSINGLETHREAD_H