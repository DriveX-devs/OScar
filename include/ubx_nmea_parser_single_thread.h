// UBX and NMEA serial GNSS parser
// Designed for the ZED-F9R module, but compatible with any GNSS receiver (some data may not be available with non-ublox devices)
// This module allows OScar to directly retrieve positioning and other kinematics data, without the need of interfacing
// to the gpsd daemon or other external software.
// Copyright (c) 2024 Mauro Vittorio

#ifndef UBXNMEAPARSERSINGLETHREAD_H
#define UBXNMEAPARSERSINGLETHREAD_H

#include "ceSerial.h"

#include <atomic>
#include <vector>

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

        std::pair<double,double> getPosition(double *age_us, bool print_timestamp);
        std::tuple<double,double,double> getAccelerations(double *age_us, bool print_timestamp);
        std::tuple<double,double,double> getRawAccelerations(double *age_us, bool print_timestamp);
        std::tuple<double,double,double> getAttitude(double *age_us, bool print_timestamp);
        double getSpeed(double *age_us, bool print_timestamp_and_age);
        double getCourseOverGroundUbx(double *age_us, bool print_timestamp_and_age);
        double getCourseOverGroundNmea(double *age_us, bool print_timestamp_and_age);
        double getAltitude(double *age_us, bool print_timestamp_and_age);
        std::string getFixMode();
        bool getFixValidity2D();
        bool getFixValidity3D();
        int startUBXNMEAParser(std::string device, int baudrate, int data_bits, char parity, int stop_bits, std::atomic<bool> *m_terminatorFlagPtr);
        void stopUBXNMEAParser();
    private:
        /* Buffer structure to be printed to the user */
        typedef struct Output {
            char ts_pos[100],
                    ts_utc_time[100],
                    ts_acc[100],
                    ts_att[100],
                    ts_alt[100],
                    ts_comp_acc[100],
                    ts_sog_cog_ubx[100],
                    ts_sog_cog_nmea[100];					// Timestamps
            char fix_ubx[100],
                    fix_nmea[100];
            char cp_lat, cp_lon;						    // Latitude and longitude cardinal points
            double lat, lon, alt;                           // Latitude, longitude and altitude above sea level
            double raw_acc_x, raw_acc_y, raw_acc_z,
                    comp_acc_x, comp_acc_y, comp_acc_z;     // Accelerations
            double roll, pitch, heading;				    // Attitude angles
            double sog_ubx, sog_nmea,
                    cog_ubx, cog_nmea;					    // Speed over ground and course over ground
            long lu_pos, lu_acc,
                 lu_att, lu_alt,
                 lu_comp_acc,
                 lu_sog_cog_ubx,
                 lu_sog_cog_nmea;						    // Last updates on relevant information
        } out_t;

        std::atomic<out_t> m_outBuffer;
        std::atomic<bool> *m_terminatorFlagPtr = nullptr; // Used in endless loops in order to terminate the program
        std::atomic<bool> m_stopParserFlag = false;
        bool m_parser_started=false;
        bool m_2d_valid_fix = false;
        bool m_3d_valid_fix = false;

        ceSerial m_serial;

        const std::vector<uint8_t> m_UBX_HEADER = {0xb5, 0x62};
        const int m_WRONG_INPUT_TRESHOLD = 1000;

        // UBX Header (2 bytes) + message class (1 byte) + message ID (1 byte) + message length (2 bytes)
        const uint8_t m_UBX_PAYLOAD_OFFSET = 6;

        // Mathematical and buffer operations
        static double decimal_deg(double value, char quadrant);
        long hexToSigned(std::vector<uint8_t> data);
        void clearBuffer();
        void printBuffer();

        // Parsers
        void parseNmeaGns(std::string nmea_response);
        void parseNmeaGga(std::string nmea_response);
        void parseEsfRaw(std::vector<uint8_t> response);
        void parseNavAtt(std::vector<uint8_t> response);
        void parseNavPvt(std::vector<uint8_t> response);
        void parseNmeaRmc(std::string nmea_response);
        void parseNavStatus(std::vector<uint8_t> response);
        void parseEsfIns(std::vector<uint8_t> response);

        void readFromSerial();
        void readData();
};

#endif // UBXNMEAPARSERSINGLETHREAD_H