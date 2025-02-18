/*
 * VRUdp integration changes:
 *
 * */

#include "gpsc.h"
#include "HeadingConfidence.h"
#include <stdexcept>
#include <iostream>
#include <math.h> 
#include <cfloat>
#include <utility>
#include <tuple>
#include <chrono>
#include "utmuts.h"

extern "C" {
    #include "VAM.h"
    #include "utmuts.h"
}

#define MIN_DIST_EQUAL_x_EPSILON 0.0001 // [m]
static const double PI_CONST = 3.1415926535897932384626433832795028841971693993751058209;
#define VRUDP_HEADING_UNAVAILABLE -DBL_MAX

#define DEG_2_RAD_BSR_2(degs) (degs*(M_PI/180.0))

#define GPSSTATUS(gpsdata) gpsdata.fix.status
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

void
VDPGPSClient::openConnection() {
	int rval;

	// Set some initial "unavailable" values for the Vehicle Width and Length
	m_vehicle_length=VDPValueConfidence<long,long>(VehicleLengthValue_unavailable,VehicleLengthConfidenceIndication_unavailable);
	m_vehicle_width=VehicleWidth_unavailable;

    if(m_use_gpsd==true) {
        if ((rval = gps_open(m_server.c_str(), std::to_string(m_port).c_str(), &m_gps_data)) != 0) {
            throw std::runtime_error("GNSS device open failed: " + std::string(gps_errstr(rval)));
        }

        (void)gps_stream(&m_gps_data, WATCH_ENABLE | WATCH_JSON, NULL);
        (void)gps_waiting(&m_gps_data, 5000000);
    } else {
        if(m_serialParserPtr==nullptr) {
            throw std::runtime_error("Cannot start the VDP GNSS Client. Invalid pointer to a serial parser object.");
        }
    }
}

void
VDPGPSClient::closeConnection() {
    if(m_use_gpsd==true) {
        (void) gps_stream(&m_gps_data, WATCH_DISABLE, NULL);
        (void) gps_close(&m_gps_data);
    }
}

// Serial parser performance log methods
std::pair<double,double> VDPGPSClient::getParserPosition() {
    return m_serialParserPtr->getPosition(nullptr,false);
}

std::pair<double,double> VDPGPSClient::getParserPositionUbx() {
    return m_serialParserPtr->getPositionUbx(nullptr,false);
}

std::pair<double,double> VDPGPSClient::getParserPositionNmea() {
    return m_serialParserPtr->getPositionNmea(nullptr,false);
}

std::tuple<double,double,double> VDPGPSClient::getParserAccelerations() {
    return m_serialParserPtr->getAccelerations(nullptr,false);
}

std::tuple<double,double,double> VDPGPSClient::getParserAngularRates() {
    return m_serialParserPtr->getAngularRates(nullptr,false);
}

std::tuple<double,double,double> VDPGPSClient::getParserRawAccelerations() {
    return m_serialParserPtr->getRawAccelerations(nullptr,false);
}

std::tuple<double,double,double> VDPGPSClient::getParserAttitude() {
    return m_serialParserPtr->getAttitude(nullptr,false);}

double VDPGPSClient::getParserSpeed() {
    return m_serialParserPtr->getSpeed(nullptr,false);
}

double VDPGPSClient::getParserSpeedUbx() {
    return m_serialParserPtr->getSpeedUbx(nullptr,false);
}

double VDPGPSClient::getParserSpeedNmea() {
    return m_serialParserPtr->getSpeedNmea(nullptr,false);
}

double VDPGPSClient::getParserCourseOverGround() {
    return m_serialParserPtr->getCourseOverGround(nullptr,false);
}

double VDPGPSClient::getParserCourseOverGroundUbx() {
    return m_serialParserPtr->getCourseOverGroundUbx(nullptr,false);
}

double VDPGPSClient::getParserCourseOverGroundNmea() {
    return m_serialParserPtr->getCourseOverGroundNmea(nullptr,false);
}

double VDPGPSClient::getParserAltitude() {
    return m_serialParserPtr->getAltitude(nullptr,false);
}

double VDPGPSClient::getParserYawRate() {
    return m_serialParserPtr->getYawRate(nullptr,false);
}

double VDPGPSClient::getParserLongitudinalAcceleration() {
    return m_serialParserPtr->getLongitudinalAcceleration(nullptr,false);
}

double VDPGPSClient::getParserValidityThreshold() {
    return m_serialParserPtr->getValidityThreshold();
}

std::string VDPGPSClient::getParserFixMode() {
    return m_serialParserPtr->getFixMode();
}

std::string VDPGPSClient::getParserFixModeUbx() {
    return m_serialParserPtr->getFixModeUbx();
}

std::string VDPGPSClient::getParserFixModeNmea() {
    return m_serialParserPtr->getFixModeNmea();
}

VDPValueConfidence<>
VDPGPSClient::getHeadingValue() {
    if(m_use_gpsd==true) {
        int rval;
        rval = gps_read(&m_gps_data, nullptr, 0);

        if (rval == -1) {
            throw std::runtime_error("Cannot read the heading from GNSS device: " + std::string(gps_errstr(rval)));
        } else {
            // Check if the mode is set and if a fix has been obtained
            if ((m_gps_data.set & MODE_SET) == MODE_SET) { //&& GPSSTATUS(m_gps_data)!=STATUS_NO_FIX) {
                if (m_gps_data.fix.mode == MODE_2D || m_gps_data.fix.mode == MODE_3D) {
                    if (static_cast<int>(m_gps_data.fix.track * DECI) < 0 ||
                        static_cast<int>(m_gps_data.fix.track * DECI) > 3601) {
                        return VDPValueConfidence<>(HeadingValue_unavailable, HeadingConfidence_unavailable);
                    } else {
                        return VDPValueConfidence<>(static_cast<int>(m_gps_data.fix.track * DECI),
                                                    HeadingConfidence_unavailable);
                    }
                }
            }
        }
    } else {
        // Check if a valid fix is present
        std::string fixModeUbx = m_serialParserPtr->getFixModeUbx();
        std::string fixModeNmea = m_serialParserPtr->getFixModeNmea();
        if (fixModeUbx != "Invalid" && fixModeUbx != "NoFix" && fixModeUbx != "Unknown/Invalid" &&
            fixModeNmea != "NoFix (V)" && fixModeNmea != "NoFix" &&
            fixModeNmea != "Unknown/Invalid (V)" && fixModeNmea != "Unknown/Invalid") {

                double heading;
                // Check if the heading value is present and valid in UBX, NMEA or both
                if (m_serialParserPtr->getCourseOverGroundValidity(false) > 0) {
                    heading = m_serialParserPtr->getCourseOverGround(nullptr,false);
                    if (static_cast<int>(heading * DECI) < 0 || static_cast<int>(heading * DECI) > 3601) {
                        return VDPValueConfidence<>(HeadingValue_unavailable, HeadingConfidence_unavailable);
                    } else {
                        return VDPValueConfidence<>(static_cast<int>(heading * DECI), HeadingConfidence_unavailable);
                    }
                }
                else {
                    return VDPValueConfidence<>(HeadingValue_unavailable, HeadingConfidence_unavailable);
                }
        }
    }
    return VDPValueConfidence<>(HeadingValue_unavailable, HeadingConfidence_unavailable);
}

double VDPGPSClient::getPedHeadingValue() {
    if(m_use_gpsd==true) {
        int rval;
        rval = gps_read(&m_gps_data, nullptr, 0);

        if (rval == -1) {
            throw std::runtime_error("Cannot read the heading from GNSS device: " + std::string(gps_errstr(rval)));
        } else {
            // Check if the mode is set and if a fix has been obtained
            if ((m_gps_data.set & MODE_SET) == MODE_SET) { //&& GPSSTATUS(m_gps_data)!=STATUS_NO_FIX) {
                if (m_gps_data.fix.mode == MODE_2D || m_gps_data.fix.mode == MODE_3D) {
                    if (static_cast<int>(m_gps_data.fix.track * DECI) < 0 ||
                        static_cast<int>(m_gps_data.fix.track * DECI) > 3601) {
                        return VRUDP_HEADING_UNAVAILABLE;
                    } else {
                        return m_gps_data.fix.track;
                    }
                }
            }
        }
    } else {
        std::string fixModeUbx = m_serialParserPtr->getFixModeUbx();
        std::string fixModeNmea = m_serialParserPtr->getFixModeNmea();
        if (fixModeUbx != "Invalid" && fixModeUbx != "NoFix" && fixModeUbx != "Unknown/Invalid" &&
            fixModeNmea != "NoFix (V)" && fixModeNmea != "NoFix" &&
            fixModeNmea != "Unknown/Invalid (V)" && fixModeNmea != "Unknown/Invalid") {

            double heading;
            // Check if the heading value is present and valid in UBX, NMEA or both
            if (m_serialParserPtr->getCourseOverGroundValidity(false) > 0) {
                heading = m_serialParserPtr->getCourseOverGround(nullptr,false);
                if (static_cast<int>(heading * DECI) < 0 || static_cast<int>(heading * DECI) > 3601) {
                    return VRUDP_HEADING_UNAVAILABLE;
                } else {
                    return heading;
                }
            }
            else {
                return VRUDP_HEADING_UNAVAILABLE;
            }
        }
    }
    return VRUDP_HEADING_UNAVAILABLE;
}

VDPValueConfidence<>
VDPGPSClient::getSpeedValue() {
    if(m_use_gpsd==true) {
        int rval;
        rval = gps_read(&m_gps_data, nullptr, 0);

        if (rval == -1) {
            throw std::runtime_error("Cannot read the speed from GNSS device: " + std::string(gps_errstr(rval)));
        } else {
            // Check if the mode is set and if a fix has been obtained
            if ((m_gps_data.set & MODE_SET) == MODE_SET) { // && GPSSTATUS(m_gps_data)!=STATUS_NO_FIX) {
                if (m_gps_data.fix.mode == MODE_2D || m_gps_data.fix.mode == MODE_3D) {
                    return VDPValueConfidence<>(m_gps_data.fix.speed * CENTI, SpeedConfidence_unavailable);
                } else return VDPValueConfidence<>(SpeedValue_unavailable, SpeedConfidence_unavailable);
            }
        }
    } else {
        // Check if a valid fix is present
        std::string fixModeUbx = m_serialParserPtr->getFixModeUbx();
        std::string fixModeNmea = m_serialParserPtr->getFixModeNmea();
        if (fixModeUbx != "Invalid" && fixModeUbx != "NoFix" && fixModeUbx != "Unknown/Invalid" &&
            fixModeNmea != "NoFix (V)" && fixModeNmea != "NoFix" &&
            fixModeNmea != "Unknown/Invalid (V)" && fixModeNmea != "Unknown/Invalid") {

            // Check if the heading value is present and valid in UBX, NMEA or both
            if (m_serialParserPtr->getSpeedValidity(false) > 0) {
                double speed = m_serialParserPtr->getSpeed(nullptr,false);
                return VDPValueConfidence<>(static_cast<int>(speed * CENTI), SpeedConfidence_unavailable);
            }
            else {
                return VDPValueConfidence<>(SpeedValue_unavailable, SpeedConfidence_unavailable);
            }
        }
    }
    return VDPValueConfidence<>(SpeedValue_unavailable, SpeedConfidence_unavailable);
}

double VDPGPSClient::getPedSpeedValue() {
    if(m_use_gpsd==true) {
        int rval;
        rval = gps_read(&m_gps_data, nullptr, 0);

        if (rval == -1) {
            throw std::runtime_error("Cannot read the speed from GNSS device: " + std::string(gps_errstr(rval)));
        } else {
            // Check if the mode is set and if a fix has been obtained
            if ((m_gps_data.set & MODE_SET) == MODE_SET) { // && GPSSTATUS(m_gps_data)!=STATUS_NO_FIX) {
                if (m_gps_data.fix.mode == MODE_2D || m_gps_data.fix.mode == MODE_3D) {
                    return m_gps_data.fix.speed;
                } else return -DBL_MAX;
            }
        }
    } else {
        // Check if a valid fix is present
        std::string fixModeUbx = m_serialParserPtr->getFixModeUbx();
        std::string fixModeNmea = m_serialParserPtr->getFixModeNmea();
        if (fixModeUbx != "Invalid" && fixModeUbx != "NoFix" && fixModeUbx != "Unknown/Invalid" &&
            fixModeNmea != "NoFix (V)" && fixModeNmea != "NoFix" &&
            fixModeNmea != "Unknown/Invalid (V)" && fixModeNmea != "Unknown/Invalid") {

            if (m_serialParserPtr->getSpeedValidity(false) > 0) {
                return m_serialParserPtr->getSpeed(nullptr, false);
            } else {
                return -DBL_MAX;
            }
        }
        return -DBL_MAX;
    }
    return -DBL_MAX;
}

VDPValueConfidence<>
VDPGPSClient::getAltitudeValue() {
    if(m_use_gpsd == true) {
        int rval;
        rval = gps_read(&m_gps_data, nullptr, 0);

        if (rval == -1) {
            throw std::runtime_error("Cannot read the altitude from GNSS device: " + std::string(gps_errstr(rval)));
        } else {
            // Check if the mode is set and if a fix has been obtained
            if ((m_gps_data.set & MODE_SET) == MODE_SET) { // && GPSSTATUS(m_gps_data)!=STATUS_NO_FIX) {
                if (m_gps_data.fix.mode == MODE_2D || m_gps_data.fix.mode == MODE_3D) {
                    return VDPValueConfidence<>(m_gps_data.fix.altitude * CENTI, AltitudeConfidence_unavailable);
                } else return VDPValueConfidence<>(AltitudeValue_unavailable, AltitudeConfidence_unavailable);
            }
        }


    } else {
        std::string fixModeUbx = m_serialParserPtr->getFixModeUbx();
        std::string fixModeNmea = m_serialParserPtr->getFixModeNmea();
        if (fixModeUbx != "Invalid" && fixModeUbx != "NoFix" && fixModeUbx != "Unknown/Invalid" &&
            fixModeNmea != "NoFix (V)" && fixModeNmea != "NoFix" &&
            fixModeNmea != "Unknown/Invalid (V)" && fixModeNmea != "Unknown/Invalid") {

            if (m_serialParserPtr->getAltitudeValidity(false) == true) {
                double altitude = m_serialParserPtr->getAltitude(nullptr, false);
                return VDPValueConfidence<>(static_cast<int>(altitude * CENTI), AltitudeConfidence_unavailable);
            } else {
                return VDPValueConfidence<>(AltitudeValue_unavailable, AltitudeConfidence_unavailable);
            }
        } else {
            return VDPValueConfidence<>(AltitudeValue_unavailable, AltitudeConfidence_unavailable);
        }
    }
    return VDPValueConfidence<>(AltitudeValue_unavailable, AltitudeConfidence_unavailable);
}

std::pair<long,long>
VDPGPSClient::getCurrentPosition() {
    if(m_use_gpsd==true) {
        int rval;
        rval = gps_read(&m_gps_data, nullptr, 0);

        if (rval == -1) {
            throw std::runtime_error("Cannot read the speed from GNSS device: " + std::string(gps_errstr(rval)));
        } else {
            // Check if the mode is set and if a fix has been obtained
            if ((m_gps_data.set & MODE_SET) == MODE_SET) { // && GPSSTATUS(m_gps_data)!=STATUS_NO_FIX) {
                if (m_gps_data.fix.mode == MODE_2D || m_gps_data.fix.mode == MODE_3D) {
                    if (!isnan(m_gps_data.fix.latitude) && !isnan(m_gps_data.fix.longitude)) {
                        return std::pair<long, long>(m_gps_data.fix.latitude * DOT_ONE_MICRO,
                                                     m_gps_data.fix.longitude * DOT_ONE_MICRO);
                    }
                }
            }
        }
        return std::pair<double, double>(Latitude_unavailable, Longitude_unavailable);
    } else {
        std::string fixModeUbx = m_serialParserPtr->getFixModeUbx();
        std::string fixModeNmea = m_serialParserPtr->getFixModeNmea();
        if (fixModeUbx != "Invalid" && fixModeUbx != "NoFix" && fixModeUbx != "Unknown/Invalid" &&
            fixModeNmea != "NoFix (V)" && fixModeNmea != "NoFix" &&
            fixModeNmea != "Unknown/Invalid (V)" && fixModeNmea != "Unknown/Invalid") {
            if (m_serialParserPtr->getPositionValidity(false) > 0) {
                std::pair<double, double> position = m_serialParserPtr->getPosition(nullptr, false);
                return std::pair<long, long>(position.first * DOT_ONE_MICRO, position.second * DOT_ONE_MICRO);
            } else {
                return std::pair<double, double>(Latitude_unavailable, Longitude_unavailable);
            }
        }
        return std::pair<double, double>(Latitude_unavailable, Longitude_unavailable);
    }
    return std::pair<double, double>(Latitude_unavailable, Longitude_unavailable);
}

VDPGPSClient::VRU_position_latlon_t
VDPGPSClient::getPedPosition() {
    VRU_position_latlon_t ped_pos{-DBL_MAX,-DBL_MAX,-DBL_MAX};
    if(m_use_gpsd==true) {
        int rval;
        rval = gps_read(&m_gps_data, nullptr, 0);

        if (rval == -1) {
            throw std::runtime_error("Cannot read the speed from GNSS device: " + std::string(gps_errstr(rval)));
        } else {
            // Check if the mode is set and if a fix has been obtained
            if ((m_gps_data.set & MODE_SET) == MODE_SET) { // && GPSSTATUS(m_gps_data)!=STATUS_NO_FIX) {
                if (m_gps_data.fix.mode == MODE_2D || m_gps_data.fix.mode == MODE_3D) {
                    if (!isnan(m_gps_data.fix.latitude) && !isnan(m_gps_data.fix.longitude)) {
                        ped_pos.lat = m_gps_data.fix.latitude;
                        ped_pos.lon = m_gps_data.fix.longitude;
                        if(m_gps_data.fix.mode == MODE_3D && !isnan(m_gps_data.fix.altitude)){
                            ped_pos.alt = m_gps_data.fix.altitude;
                        }
                    }
                }
            }
        }
        return ped_pos;
    } else {
        std::string fixModeUbx = m_serialParserPtr->getFixModeUbx();
        std::string fixModeNmea = m_serialParserPtr->getFixModeNmea();
        if (fixModeUbx != "Invalid" && fixModeUbx != "NoFix" && fixModeUbx != "Unknown/Invalid" &&
            fixModeNmea != "NoFix (V)" && fixModeNmea != "NoFix" &&
            fixModeNmea != "Unknown/Invalid (V)" && fixModeNmea != "Unknown/Invalid") {

            long pos_age = -1;
            double longitude = -1.0;
            double latitude = -1.0;
            double altitude = -8001.0;
            if (m_serialParserPtr->getPositionValidity(false) > 0) {
                std::pair<double, double> position = m_serialParserPtr->getPosition(&pos_age, false);
                latitude = position.first;
            } else latitude = (Latitude_t) Latitude_unavailable;

            if (m_serialParserPtr->getPositionValidity(false) > 0) {
                std::pair<double, double> position = m_serialParserPtr->getPosition(&pos_age, false);
                longitude = position.second;
            } else longitude = (Longitude_t) Longitude_unavailable;

            if (m_serialParserPtr->getAltitudeValidity(false) > 0) {
                altitude = m_serialParserPtr->getAltitude(nullptr, false);
            } else altitude = AltitudeValue_unavailable/100.0;

            std::pair<double, double> position = m_serialParserPtr->getPosition(nullptr, false);
            ped_pos.lat = latitude;
            ped_pos.lon = longitude;
            ped_pos.alt = altitude;
        }

        return ped_pos;
    }
}

// convertLatLontoXYZ_ECEF() still does not work as expected - kept for reference but it should NOT be used unless you know very well what you are doing!
VDPGPSClient::VRU_position_XYZ_t
VDPGPSClient::convertLatLontoXYZ_ECEF(VDPGPSClient::VRU_position_latlon_t pos_latlon){
    VDPGPSClient::VRU_position_XYZ_t pos_xyz = {-DBL_MAX,-DBL_MAX,-DBL_MAX};

    {
        std::cerr << "Error! convertLatLontoXYZ_ECEF() should not be used in the code! " <<
                  "It has been just kept for reference: using it doesn't guarantee correct results when computing the safe distances for VAMs." <<
                  std::endl;
        exit(EXIT_FAILURE);
    }

    double N = 6378137.0/sqrt(1-(1/298.257223563)*(sin(pos_latlon.lat)*sin(pos_latlon.lat)));

    if(pos_latlon.lat != -DBL_MAX && pos_latlon.lon != -DBL_MAX){
        if(pos_latlon.alt != -DBL_MAX && pos_latlon.alt != AltitudeValue_unavailable){
            pos_xyz.x = (N+pos_latlon.alt)*cos(pos_latlon.lat)*cos(pos_latlon.lon);
            pos_xyz.y = (N+pos_latlon.alt)*cos(pos_latlon.lat)*sin(pos_latlon.lon);
            pos_xyz.z = (N*(1-(1/298.257223563))+pos_latlon.alt)*sin(pos_latlon.lat);
        } else{
            pos_xyz.x = N*cos(pos_latlon.lat)*cos(pos_latlon.lon);
            pos_xyz.y = N*cos(pos_latlon.lat)*sin(pos_latlon.lon);
            pos_xyz.z = N*sin(pos_latlon.lat);
        }
    }

    return pos_xyz;
}

VDPGPSClient::VRU_position_XYZ_t
VDPGPSClient::convertLatLontoXYZ_TM(VDPGPSClient::VRU_position_latlon_t pos_latlon, double lon0){
    VDPGPSClient::VRU_position_XYZ_t pos_xyz = {-DBL_MAX,-DBL_MAX,-DBL_MAX};
    double x, y, gamma, k;
    transverse_mercator_t transmerc = UTMUPS_init_UTM_TransverseMercator();

    // Perform a Transverse Mercator projection considering "lon0" as center meridian
    TransverseMercator_Forward(&transmerc,lon0,pos_latlon.lat,pos_latlon.lon,&x,&y,&gamma,&k);

    if(m_vru_debug==true) {
        fprintf(stdout,"[DEBUG] [Transverse Mercator Forward] lat=%.7lf, lon=%.7lf [lon0=%.7lf] -> x=%.7lf, y=%.7lf\n",
                pos_latlon.lat,pos_latlon.lon,lon0,x,y);

        double dbg_rev_lat,dbg_rev_lon,dbg_gamma,dbg_k;

        TransverseMercator_Reverse(&transmerc,lon0,x,y,&dbg_rev_lat,&dbg_rev_lon,&dbg_gamma,&dbg_k);
        fprintf(stdout,"[DEBUG] [Transverse Mercator Reverse] x=%.7lf, y=%.7lf [lon0=%.7lf] -> lat=%.7lf, lon=%.7lf\n",
                x,y,lon0,dbg_rev_lat,dbg_rev_lon);
    }

    pos_xyz.x = x; // [m]
    pos_xyz.y = y; // [m]
    pos_xyz.z = pos_latlon.alt; // Altitude was already in [m] and it is not involved in the coordinate conversion

    return pos_xyz;
}

VDPGPSClient::VAM_mandatory_data_t
VDPGPSClient::getVAMMandatoryData() {
    VAM_mandatory_data_t VAMdata={.avail=false};

    if(m_use_gpsd==true) {
        int rval;

        rval=gps_read(&m_gps_data,nullptr,0);

        if(rval==-1) {
            throw std::runtime_error("Cannot read data from GNSS device: " + std::string(gps_errstr(rval)));
        } else {
            // Check if the mode is set and if a fix has been obtained
            if((m_gps_data.set & MODE_SET)==MODE_SET) { // && GPSSTATUS(m_gps_data)!=STATUS_NO_FIX) {
                if(m_gps_data.fix.mode == MODE_2D || m_gps_data.fix.mode == MODE_3D) {
                    /* Speed [0.01 m/s] */
                    VAMdata.speed = VDPValueConfidence<>(m_gps_data.fix.speed*CENTI,SpeedConfidence_unavailable);

                    /* Latitude WGS84 [0,1 microdegree] */
                    VAMdata.latitude = (Latitude_t)(m_gps_data.fix.latitude*DOT_ONE_MICRO);
                    /* Longitude WGS84 [0,1 microdegree] */
                    VAMdata.longitude = (Longitude_t)(m_gps_data.fix.longitude*DOT_ONE_MICRO);

                    int asnAltitudeValue=static_cast<int>(m_gps_data.fix.altitude*CENTI);
                    /* Altitude [0,01 m] */
                    if(m_gps_data.fix.mode == MODE_3D && asnAltitudeValue>=-100000 &&  asnAltitudeValue<=800000) {
                        VAMdata.altitude = VDPValueConfidence<>(static_cast<int>(m_gps_data.fix.altitude*CENTI),AltitudeConfidence_unavailable);
                    } else {
                        VAMdata.altitude = VDPValueConfidence<>(AltitudeValue_unavailable,AltitudeConfidence_unavailable);
                    }

                    /* Position Confidence Ellipse */
                    VAMdata.posConfidenceEllipse.semiMajorConfidence=SemiAxisLength_unavailable;
                    VAMdata.posConfidenceEllipse.semiMinorConfidence=SemiAxisLength_unavailable;
                    VAMdata.posConfidenceEllipse.semiMajorOrientation=HeadingValue_unavailable;

                    /* Heading WGS84 north [0.1 degree] - m_gps_data.fix.track should already provide a CW heading relative to North */
                    if(static_cast<int>(m_gps_data.fix.track*DECI)<0 || static_cast<int>(m_gps_data.fix.track*DECI)>3601) {
                        VAMdata.heading = VDPValueConfidence<>(HeadingValue_unavailable,HeadingConfidence_unavailable);
                    } else {
                        VAMdata.heading = VDPValueConfidence<>(static_cast<int>(m_gps_data.fix.track*DECI),HeadingConfidence_unavailable);
                    }

                    /* Longitudinal acceleration [0.1 m/s^2] */
                    VAMdata.longAcceleration = VDPValueConfidence<>(LongitudinalAccelerationValue_unavailable,AccelerationConfidence_unavailable);

                    /* This flag represents an easy way to understand if it was possible to read any valid data from the GNSS device */
                    VAMdata.avail = true;
                }
            } else{
                // Set everything to unavailable as no fix was possible (i.e., the resulting CAM will not be so useful...)

                /* Speed [0.01 m/s] */
                VAMdata.speed = VDPValueConfidence<>(SpeedValue_unavailable,SpeedConfidence_unavailable);

                /* Latitude WGS84 [0,1 microdegree] */
                VAMdata.latitude = (Latitude_t)Latitude_unavailable;
                /* Longitude WGS84 [0,1 microdegree] */
                VAMdata.longitude = (Longitude_t)Longitude_unavailable;

                /* Altitude [0,01 m] */
                VAMdata.altitude = VDPValueConfidence<>(AltitudeValue_unavailable,AltitudeConfidence_unavailable);

                /* Position Confidence Ellipse */
                VAMdata.posConfidenceEllipse.semiMajorConfidence=SemiAxisLength_unavailable;
                VAMdata.posConfidenceEllipse.semiMinorConfidence=SemiAxisLength_unavailable;
                VAMdata.posConfidenceEllipse.semiMajorOrientation=HeadingValue_unavailable;

                /* Longitudinal acceleration [0.1 m/s^2] */
                VAMdata.longAcceleration = VDPValueConfidence<>(LongitudinalAccelerationValue_unavailable,AccelerationConfidence_unavailable);

                /* Heading WGS84 north [0.1 degree] */
                VAMdata.heading = VDPValueConfidence<>(HeadingValue_unavailable,HeadingConfidence_unavailable);

                VAMdata.avail = false;
            }
        }
    } else {
        // Check if a valid fix is present
        std::string fixModeUbx = m_serialParserPtr->getFixModeUbx();
        std::string fixModeNmea = m_serialParserPtr->getFixModeNmea();
        if (fixModeUbx != "Invalid" && fixModeUbx != "NoFix" && fixModeUbx != "Unknown/Invalid" &&
            fixModeNmea != "NoFix (V)" && fixModeNmea != "NoFix" &&
            fixModeNmea != "Unknown/Invalid (V)" && fixModeNmea != "Unknown/Invalid") {

            /* Speed [0.01 m/s] */
            if (m_serialParserPtr->getSpeedValidity(false) > 0) {
                double speed = m_serialParserPtr->getSpeed(nullptr, false);
                if (speed >= 0 && speed <= 163.82) {
                    VAMdata.speed = VDPValueConfidence<>(speed * CENTI, SpeedConfidence_unavailable);
                }
                else {
                    VAMdata.speed = VDPValueConfidence<>(SpeedValue_unavailable, SpeedConfidence_unavailable);
                }
            } else {
                VAMdata.speed = VDPValueConfidence<>(SpeedValue_unavailable, SpeedConfidence_unavailable);
            }

            long pos_age = -1;
            double longitude = Latitude_unavailable;
            double latitude = Longitude_unavailable;

            /* Latitude [0.1 microdegrees] */
            if (m_serialParserPtr->getPositionValidity(false) > 0) {
                std::pair<double, double> position = m_serialParserPtr->getPosition(&pos_age, false);
                VAMdata.latitude = (Latitude_t) (position.first * DOT_ONE_MICRO);
                latitude = position.first;
            }
            else {
                VAMdata.latitude = (Latitude_t) Latitude_unavailable;
            }

            /* Longitude [0.1 microdegrees] */
            if (m_serialParserPtr->getPositionValidity(false) > 0) {
                std::pair<double, double> position = m_serialParserPtr->getPosition(&pos_age, false);
                VAMdata.longitude = (Longitude_t) (position.second * DOT_ONE_MICRO);
                longitude = position.second;
            }
            else {
                VAMdata.longitude = (Longitude_t) Longitude_unavailable;
            }

            /* Altitude [0.01 m] */
            if (m_serialParserPtr->getAltitudeValidity(false) == true) {
                double altitude = m_serialParserPtr->getAltitude(nullptr, false);
                if (altitude * CENTI >= -100000 && altitude * CENTI <= 800000) {
                    VAMdata.altitude = VDPValueConfidence<>(altitude * CENTI, AltitudeConfidence_unavailable);
                }
                else {
                    VAMdata.altitude = VDPValueConfidence<>(AltitudeValue_unavailable, AltitudeConfidence_unavailable);
                }
            }
            else {
                VAMdata.altitude = VDPValueConfidence<>(AltitudeValue_unavailable, AltitudeConfidence_unavailable);
            }

            VAMdata.posConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
            VAMdata.posConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
            VAMdata.posConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;

            /* Longitudinal acceleration [0.1 m/s^2] */
            if (m_serialParserPtr->getAccelerationsValidity(false) == true) {
                double long_acc = m_serialParserPtr->getLongitudinalAcceleration(nullptr, false);
                if (long_acc >= -16 && long_acc <= 16) {
                    //CAMdata.longAcceleration = VDPValueConfidence<>(AccelerationValue_unavailable,AccelerationConfidence_unavailable);
                    VAMdata.longAcceleration = VDPValueConfidence<>(long_acc * DECI, AccelerationConfidence_unavailable);
                }
                else {
                    VAMdata.longAcceleration = VDPValueConfidence<>(AccelerationValue_unavailable,AccelerationConfidence_unavailable);
                }
            }
            else {
                VAMdata.longAcceleration = VDPValueConfidence<>(AccelerationValue_unavailable,AccelerationConfidence_unavailable);
            }

            /* Heading WGS84 north [0.1 degree] */
            if (m_serialParserPtr->getCourseOverGroundValidity(false) > 0) {
                double heading = m_serialParserPtr->getCourseOverGround(nullptr, false);
                if (static_cast<int>(heading * DECI) < 0 || static_cast<int>(heading * DECI) > 3601) {
                    VAMdata.heading = VDPValueConfidence<>(HeadingValue_unavailable, HeadingConfidence_unavailable);
                }
                else {
                    VAMdata.heading = VDPValueConfidence<>(heading * DECI, HeadingConfidence_unavailable);
                }

            }
            else {
                VAMdata.heading = VDPValueConfidence<>(HeadingValue_unavailable, HeadingConfidence_unavailable);
            }

            VAMdata.avail = true;

            // compute the delta position considering the speed, heading and the pos age
            if (VAMdata.avail == true && (VAMdata.latitude!=Latitude_unavailable && VAMdata.longitude!=Longitude_unavailable)) {
                double delta_x = 0.0;
                double delta_y = 0.0;
                double gammar=0,kr=0;
//                std::cout << "Read latitude: " << CAMdata.latitude << std::endl;
//                std::cout << "Read longitude: " << CAMdata.longitude << std::endl;

                if (VAMdata.speed.getValue() != SpeedValue_unavailable && VAMdata.heading.getValue() != HeadingValue_unavailable) {
                    double speed = VAMdata.speed.getValue() / CENTI; // [m/s]
                    double heading = VAMdata.heading.getValue() / DECI; // [degrees]
                    heading = DEG_2_RAD_BSR_2((90-heading));

                    // Compute the delta position
                    double delta_t = pos_age / 1000000.0; // [s]
                    delta_x = speed * delta_t * sin(heading);
                    delta_y = speed * delta_t * cos(heading);
                }

                // Update the position
                transverse_mercator_t tmerc = UTMUPS_init_UTM_TransverseMercator();
                double lat1, lon1, ego_x, ego_y;
                TransverseMercator_Forward(&tmerc, longitude, latitude, longitude, &ego_x, &ego_y, &gammar, &kr);
                ego_x += delta_x;
                ego_y += delta_y;
                TransverseMercator_Reverse(&tmerc, longitude, ego_x, ego_y, &lat1, &lon1, &gammar, &kr);

                VAMdata.latitude = (Latitude_t) (lat1 * DOT_ONE_MICRO);
                VAMdata.longitude = (Longitude_t) (lon1 * DOT_ONE_MICRO);

                //print all the data
//                std::cout << "Speed: " << CAMdata.speed.getValue()/CENTI << "[m/s]"<< std::endl;
//                std::cout << "Heading: " << CAMdata.heading.getValue()/DECI  << "[degrees]"<< std::endl;
//                std::cout << "Delta x: " << delta_x << "[m]" << std::endl;
//                std::cout << "Delta y: " << delta_y << "[m]" << std::endl;
//                std::cout << "PositionDelta: " << sqrt(delta_x*delta_x + delta_y*delta_y) << "[m]" <<  std::endl;
//                std::cout << "New Latitude: " << CAMdata.latitude << std::endl;
//                std::cout << "New longitude: " << CAMdata.longitude << std::endl;
            }
        }
        else {
            // Set everything to unavailable as no fix was possible (i.e., the resulting CAM will not be so useful...)
            VAMdata.speed = VDPValueConfidence<>(SpeedValue_unavailable, SpeedConfidence_unavailable);
            VAMdata.latitude = (Latitude_t) Latitude_unavailable;
            VAMdata.longitude = (Longitude_t) Longitude_unavailable;
            VAMdata.altitude = VDPValueConfidence<>(AltitudeValue_unavailable, AltitudeConfidence_unavailable);
            VAMdata.posConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
            VAMdata.posConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
            VAMdata.posConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
            VAMdata.longAcceleration = VDPValueConfidence<>(AccelerationValue_unavailable,
                                                            AccelerationConfidence_unavailable);
            VAMdata.heading = VDPValueConfidence<>(HeadingValue_unavailable, HeadingConfidence_unavailable);
            VAMdata.avail = false;
        }
    }

    return VAMdata;
}

VDPValueConfidence<>
VDPGPSClient::getLongitudinalAccelerationValue() {
    if(m_use_gpsd == true) {
        /*
        int rval;
        rval = gps_read(&m_gps_data, nullptr, 0);

        if (rval == -1) {
            throw std::runtime_error("Cannot read the altitude from GNSS device: " + std::string(gps_errstr(rval)));
        } else {
            // Check if the mode is set and if a fix has been obtained
            if ((m_gps_data.set & MODE_SET) == MODE_SET) { // && GPSSTATUS(m_gps_data)!=STATUS_NO_FIX) {
                if (m_gps_data.fix.mode == MODE_2D || m_gps_data.fix.mode == MODE_3D) {
                    return VDPValueConfidence<>(m_gps_data.fix.altitude * CENTI, AltitudeConfidence_unavailable);
                }
            }
        }*/
        return VDPValueConfidence<>(AltitudeValue_unavailable, AltitudeConfidence_unavailable);

    } else {
        std::string fixModeUbx = m_serialParserPtr->getFixModeUbx();
        std::string fixModeNmea = m_serialParserPtr->getFixModeNmea();
        if (fixModeUbx != "Invalid" && fixModeUbx != "NoFix" && fixModeUbx != "Unknown/Invalid" &&
            fixModeNmea != "NoFix (V)" && fixModeNmea != "NoFix" &&
            fixModeNmea != "Unknown/Invalid (V)" && fixModeNmea != "Unknown/Invalid") {

            if (m_serialParserPtr->getAccelerationsValidity(false) == true ) {
                double long_acc = m_serialParserPtr->getLongitudinalAcceleration(nullptr, false);
                if (long_acc >= -16 && long_acc <= 16) {
                    return VDPValueConfidence<>(static_cast<int>(long_acc * DECI), AccelerationConfidence_unavailable);
                } else {
                    return VDPValueConfidence<>(LongitudinalAccelerationValue_unavailable,
                                                AccelerationConfidence_unavailable);
                }
            }
        }
        else {
            return VDPValueConfidence<>(LongitudinalAccelerationValue_unavailable, AccelerationConfidence_unavailable);
        }
    }
    return VDPValueConfidence<>(LongitudinalAccelerationValue_unavailable, AccelerationConfidence_unavailable);
}

VDPValueConfidence<>
VDPGPSClient::getYawRate() {
    if(m_use_gpsd == true) {
        /*
        int rval;
        rval = gps_read(&m_gps_data, nullptr, 0);

        if (rval == -1) {
            throw std::runtime_error("Cannot read the altitude from GNSS device: " + std::string(gps_errstr(rval)));
        } else {
            // Check if the mode is set and if a fix has been obtained
            if ((m_gps_data.set & MODE_SET) == MODE_SET) { // && GPSSTATUS(m_gps_data)!=STATUS_NO_FIX) {
                if (m_gps_data.fix.mode == MODE_2D || m_gps_data.fix.mode == MODE_3D) {
                    return VDPValueConfidence<>(m_gps_data.fix.altitude * CENTI, AltitudeConfidence_unavailable);
                }
            }
        }
        return VDPValueConfidence<>(AltitudeValue_unavailable, AltitudeConfidence_unavailable);
        */
    } else {
        std::string fixModeUbx = m_serialParserPtr->getFixModeUbx();
        std::string fixModeNmea = m_serialParserPtr->getFixModeNmea();
        if (fixModeUbx != "Invalid" && fixModeUbx != "NoFix" && fixModeUbx != "Unknown/Invalid" &&
            fixModeNmea != "NoFix (V)" && fixModeNmea != "NoFix" &&
            fixModeNmea != "Unknown/Invalid (V)" && fixModeNmea != "Unknown/Invalid") {

            if (m_serialParserPtr->getYawRateValidity(false) == true) {
                double yaw_rate = m_serialParserPtr->getYawRate(nullptr, false);
                return VDPValueConfidence<>(static_cast<int>(yaw_rate * CENTI), YawRateConfidence_unavailable);
            }
            else {
                return VDPValueConfidence<>(YawRateValue_unavailable, YawRateConfidence_unavailable);
            }
        }
        else {
            return VDPValueConfidence<>(YawRateValue_unavailable, YawRateConfidence_unavailable);
        }
    }
    return VDPValueConfidence<>(YawRateValue_unavailable, YawRateConfidence_unavailable);
}

double
VDPGPSClient::getHeadingValueDbl() {
    if(m_use_gpsd == true) {
        int rval;
        rval = gps_read(&m_gps_data, nullptr, 0);

        if (rval == -1) {
            throw std::runtime_error("Cannot read the heading from GNSS device: " + std::string(gps_errstr(rval)));
        } else {
            // Check if the mode is set and if a fix has been obtained
            if ((m_gps_data.set & MODE_SET) == MODE_SET) { //&& GPSSTATUS(m_gps_data)!=STATUS_NO_FIX) {
                if (m_gps_data.fix.mode == MODE_2D || m_gps_data.fix.mode == MODE_3D) {
                    if (static_cast<int>(m_gps_data.fix.track * DECI) < 0 ||
                        static_cast<int>(m_gps_data.fix.track * DECI) > 3601) {
                        return -DBL_MAX;
                    } else {
                        return m_gps_data.fix.track;
                    }
                }
            }
        }
    } else {
        std::string fixModeUbx = m_serialParserPtr->getFixModeUbx();
        std::string fixModeNmea = m_serialParserPtr->getFixModeNmea();
        if (fixModeUbx != "Invalid" && fixModeUbx != "NoFix" && fixModeUbx != "Unknown/Invalid" &&
            fixModeNmea != "NoFix (V)" && fixModeNmea != "NoFix" &&
            fixModeNmea != "Unknown/Invalid (V)" && fixModeNmea != "Unknown/Invalid") {
                if (m_serialParserPtr->getCourseOverGroundValidity(false) > 0) {
                    double heading_dbl = m_serialParserPtr->getCourseOverGround(nullptr,false);
                    if (static_cast<int>(heading_dbl * DECI) < 0 || static_cast<int>(heading_dbl * DECI) > 3601) {
                        return HeadingValue_unavailable;
                    } else {
                        return heading_dbl;
                    }
                } else {
                    return HeadingValue_unavailable;
                }
            } else {
            return HeadingValue_unavailable;
        }
    }
    return HeadingValue_unavailable;
}

double
VDPGPSClient::getSpeedValueDbl() {
    if(m_use_gpsd==true) {
        int rval;
        rval = gps_read(&m_gps_data, nullptr, 0);

        if (rval == -1) {
            throw std::runtime_error("Cannot read the speed from GNSS device: " + std::string(gps_errstr(rval)));
        } else {
            // Check if the mode is set and if a fix has been obtained
            if ((m_gps_data.set & MODE_SET) == MODE_SET) { // && GPSSTATUS(m_gps_data)!=STATUS_NO_FIX) {
                if (m_gps_data.fix.mode == MODE_2D || m_gps_data.fix.mode == MODE_3D) {
                    return m_gps_data.fix.speed;
                }
            }
        }
    } else {
        std::string fixModeUbx = m_serialParserPtr->getFixModeUbx();
        std::string fixModeNmea = m_serialParserPtr->getFixModeNmea();
        if (fixModeUbx != "Invalid" && fixModeUbx != "NoFix" && fixModeUbx != "Unknown/Invalid" &&
            fixModeNmea != "NoFix (V)" && fixModeNmea != "NoFix" &&
            fixModeNmea != "Unknown/Invalid (V)" && fixModeNmea != "Unknown/Invalid") {

            if (m_serialParserPtr->getSpeedValidity(false) > 0) {
                return m_serialParserPtr->getSpeed(nullptr, false);
            } else {
                return SpeedValue_unavailable;
            }
        } else {
            return SpeedValue_unavailable;
        }
    }
    return SpeedValue_unavailable;
}

double
VDPGPSClient::getAltitudeValueDbl() {
    if(m_use_gpsd == true) {
        int rval;
        rval = gps_read(&m_gps_data, nullptr, 0);

        if (rval == -1) {
            throw std::runtime_error("Cannot read the altitude from GNSS device: " + std::string(gps_errstr(rval)));
        } else {
            // Check if the mode is set and if a fix has been obtained
            if ((m_gps_data.set & MODE_SET) == MODE_SET) { // && GPSSTATUS(m_gps_data)!=STATUS_NO_FIX) {
                if (m_gps_data.fix.mode == MODE_2D || m_gps_data.fix.mode == MODE_3D) {
                    return m_gps_data.fix.altitude;
                }
            }
        }
    } else{
        std::string fixModeUbx = m_serialParserPtr->getFixModeUbx();
        std::string fixModeNmea = m_serialParserPtr->getFixModeNmea();
        if (fixModeUbx != "Invalid" && fixModeUbx != "NoFix" && fixModeUbx != "Unknown/Invalid" &&
            fixModeNmea != "NoFix (V)" && fixModeNmea != "NoFix" &&
            fixModeNmea != "Unknown/Invalid (V)" && fixModeNmea != "Unknown/Invalid") {

            if (m_serialParserPtr->getAttitudeValidity(false) == true) {
                return m_serialParserPtr->getAltitude(nullptr, false);
            } else {
                return AltitudeValue_unavailable;
            }
        } else {
            return AltitudeValue_unavailable;
        }
    }
    return AltitudeValue_unavailable;
}

std::pair<double,double>
VDPGPSClient::getCurrentPositionDbl() {
    if(m_use_gpsd==true) {
        int rval;
        rval = gps_read(&m_gps_data, nullptr, 0);

        if (rval == -1) {
            throw std::runtime_error("Cannot read the speed from GNSS device: " + std::string(gps_errstr(rval)));
        } else {
            // Check if the mode is set and if a fix has been obtained
            if ((m_gps_data.set & MODE_SET) == MODE_SET) { // && GPSSTATUS(m_gps_data)!=STATUS_NO_FIX) {
                if (m_gps_data.fix.mode == MODE_2D || m_gps_data.fix.mode == MODE_3D) {
                    if (!isnan(m_gps_data.fix.latitude) && !isnan(m_gps_data.fix.longitude)) {
                        return std::pair<double, double>(m_gps_data.fix.latitude, m_gps_data.fix.longitude);
                    }
                }
            }
        }
    } else {
        std::string fixModeUbx = m_serialParserPtr->getFixModeUbx();
        std::string fixModeNmea = m_serialParserPtr->getFixModeNmea();
        if (fixModeUbx != "Invalid" && fixModeUbx != "NoFix" && fixModeUbx != "Unknown/Invalid" &&
            fixModeNmea != "NoFix (V)" && fixModeNmea != "NoFix" &&
            fixModeNmea != "Unknown/Invalid (V)" && fixModeNmea != "Unknown/Invalid") {

            if (m_serialParserPtr->getPositionValidity(false) > 0) {
                return m_serialParserPtr->getPosition(nullptr, false);
            }
            else {
                return std::pair<double, double>(Latitude_unavailable, Longitude_unavailable);
            }
        }
        return std::pair<double, double>(Latitude_unavailable, Longitude_unavailable);
    }
    return std::pair<double, double>(Latitude_unavailable, Longitude_unavailable);
}

double
VDPGPSClient::getLongitudinalAccelerationValueDbl() {
    if(m_use_gpsd == true) {
        /*
        int rval;
        rval = gps_read(&m_gps_data, nullptr, 0);

        if (rval == -1) {
            throw std::runtime_error("Cannot read the speed from GNSS device: " + std::string(gps_errstr(rval)));
        } else {
            // Check if the mode is set and if a fix has been obtained
            if ((m_gps_data.set & MODE_SET) == MODE_SET) { // && GPSSTATUS(m_gps_data)!=STATUS_NO_FIX) {
                if (m_gps_data.fix.mode == MODE_2D || m_gps_data.fix.mode == MODE_3D) {
                    if (!isnan(m_gps_data.fix.latitude) && !isnan(m_gps_data.fix.longitude)) {
                        return std::pair<double, double>(m_gps_data.fix.latitude, m_gps_data.fix.longitude);
                    }
                }
            }
        }
         */
    } else {
        std::string fixModeUbx = m_serialParserPtr->getFixModeUbx();
        std::string fixModeNmea = m_serialParserPtr->getFixModeNmea();
        if (fixModeUbx != "Invalid" && fixModeUbx != "NoFix" && fixModeUbx != "Unknown/Invalid" &&
            fixModeNmea != "NoFix (V)" && fixModeNmea != "NoFix" &&
            fixModeNmea != "Unknown/Invalid (V)" && fixModeNmea != "Unknown/Invalid") {

            if (m_serialParserPtr->getAccelerationsValidity(false) == true) {
                return m_serialParserPtr->getLongitudinalAcceleration(nullptr, false);
            }
            else {
                return LongitudinalAccelerationValue_unavailable;
            }
        }
        else {
            return LongitudinalAccelerationValue_unavailable;
        }
    }
    return LongitudinalAccelerationValue_unavailable;
}

double
VDPGPSClient::getYawRateDbl() {
    if(m_use_gpsd == true) {
        /*
        int rval;
        rval = gps_read(&m_gps_data, nullptr, 0);

        if (rval == -1) {
            throw std::runtime_error("Cannot read the speed from GNSS device: " + std::string(gps_errstr(rval)));
        } else {
            // Check if the mode is set and if a fix has been obtained
            if ((m_gps_data.set & MODE_SET) == MODE_SET) { // && GPSSTATUS(m_gps_data)!=STATUS_NO_FIX) {
                if (m_gps_data.fix.mode == MODE_2D || m_gps_data.fix.mode == MODE_3D) {
                    if (!isnan(m_gps_data.fix.latitude) && !isnan(m_gps_data.fix.longitude)) {
                        return std::pair<double, double>(m_gps_data.fix.latitude, m_gps_data.fix.longitude);
                    }
                }
            }
        }
         */
    } else {
        std::string fixModeUbx = m_serialParserPtr->getFixModeUbx();
        std::string fixModeNmea = m_serialParserPtr->getFixModeNmea();
        if (fixModeUbx != "Invalid" && fixModeUbx != "NoFix" && fixModeUbx != "Unknown/Invalid" &&
            fixModeNmea != "NoFix (V)" && fixModeNmea != "NoFix" &&
            fixModeNmea != "Unknown/Invalid (V)" && fixModeNmea != "Unknown/Invalid") {

            if (m_serialParserPtr->getYawRateValidity(false) == true) {
                return m_serialParserPtr->getYawRate(nullptr, false);
            }
            else {
                return YawRateValue_unavailable;
            }
        }
        else {
            return YawRateValue_unavailable;
        }
    }
    return YawRateValue_unavailable;
}

VDPGPSClient::CAM_mandatory_data_t
VDPGPSClient::getCAMMandatoryData() {
	CAM_mandatory_data_t CAMdata={.avail=false};

    if(m_use_gpsd==true) {
        int rval;

        rval = gps_read(&m_gps_data, nullptr, 0);

        if (rval == -1) {
            throw std::runtime_error("Cannot read data from GNSS device: " + std::string(gps_errstr(rval)));
        } else {
            // Check if the mode is set and if a fix has been obtained
            if ((m_gps_data.set & MODE_SET) == MODE_SET) { // && GPSSTATUS(m_gps_data)!=STATUS_NO_FIX) {
                if (m_gps_data.fix.mode == MODE_2D || m_gps_data.fix.mode == MODE_3D) {
                    /* Speed [0.01 m/s] */
                    CAMdata.speed = VDPValueConfidence<>(m_gps_data.fix.speed * CENTI, SpeedConfidence_unavailable);

                    /* Latitude WGS84 [0,1 microdegree] */
                    CAMdata.latitude = (Latitude_t) (m_gps_data.fix.latitude * DOT_ONE_MICRO);
                    /* Longitude WGS84 [0,1 microdegree] */
                    CAMdata.longitude = (Longitude_t) (m_gps_data.fix.longitude * DOT_ONE_MICRO);

                    int asnAltitudeValue = static_cast<int>(m_gps_data.fix.altitude * CENTI);
                    /* Altitude [0,01 m] */
                    if (m_gps_data.fix.mode == MODE_3D && asnAltitudeValue >= -100000 && asnAltitudeValue <= 800000) {
                        CAMdata.altitude = VDPValueConfidence<>(static_cast<int>(m_gps_data.fix.altitude * CENTI),
                                                                AltitudeConfidence_unavailable);
                    } else {
                        CAMdata.altitude = VDPValueConfidence<>(AltitudeValue_unavailable,
                                                                AltitudeConfidence_unavailable);
                    }

                    /* Position Confidence Ellipse */
                    CAMdata.posConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
                    CAMdata.posConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
                    CAMdata.posConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;

                    /* Longitudinal acceleration [0.1 m/s^2] */
                    CAMdata.longAcceleration = VDPValueConfidence<>(AccelerationValue_unavailable,
                                                                    AccelerationConfidence_unavailable);

                    /* Heading WGS84 north [0.1 degree] - m_gps_data.fix.track should already provide a CW heading relative to North */
                    if (static_cast<int>(m_gps_data.fix.track * DECI) < 0 ||
                        static_cast<int>(m_gps_data.fix.track * DECI) > 3601) {
                        CAMdata.heading = VDPValueConfidence<>(HeadingValue_unavailable, HeadingConfidence_unavailable);
                    } else {
                        CAMdata.heading = VDPValueConfidence<>(static_cast<int>(m_gps_data.fix.track * DECI),
                                                               HeadingConfidence_unavailable);
                    }

                    /* Drive direction (backward driving is not fully supported by SUMO, at the moment */
                    CAMdata.driveDirection = DriveDirection_unavailable;

                    /* Curvature and CurvatureCalculationMode */
                    CAMdata.curvature = VDPValueConfidence<>(CurvatureValue_unavailable,
                                                             CurvatureConfidence_unavailable);
                    CAMdata.curvature_calculation_mode = CurvatureCalculationMode_unavailable;

                    /* Length and Width [0.1 m] */
                    CAMdata.VehicleLength = m_vehicle_length;
                    CAMdata.VehicleWidth = m_vehicle_width;

                    /* Yaw Rate */
                    CAMdata.yawRate = VDPValueConfidence<>(YawRateValue_unavailable, YawRateConfidence_unavailable);

                    /* This flag represents an easy way to understand if it was possible to read any valid data from the GNSS device */
                    CAMdata.avail = true;
                }
            } else {
                // Set everything to unavailable as no fix was possible (i.e., the resulting CAM will not be so useful...)

                /* Speed [0.01 m/s] */
                CAMdata.speed = VDPValueConfidence<>(SpeedValue_unavailable, SpeedConfidence_unavailable);

                /* Latitude WGS84 [0,1 microdegree] */
                CAMdata.latitude = (Latitude_t) Latitude_unavailable;
                /* Longitude WGS84 [0,1 microdegree] */
                CAMdata.longitude = (Longitude_t) Longitude_unavailable;

                /* Altitude [0,01 m] */
                CAMdata.altitude = VDPValueConfidence<>(AltitudeValue_unavailable, AltitudeConfidence_unavailable);

                /* Position Confidence Ellipse */
                CAMdata.posConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
                CAMdata.posConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
                CAMdata.posConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;

                /* Longitudinal acceleration [0.1 m/s^2] */
                CAMdata.longAcceleration = VDPValueConfidence<>(AccelerationValue_unavailable,
                                                                AccelerationConfidence_unavailable);

                /* Heading WGS84 north [0.1 degree] */
                CAMdata.heading = VDPValueConfidence<>(HeadingValue_unavailable, HeadingConfidence_unavailable);

                /* Drive direction (backward driving is not fully supported by SUMO, at the moment */
                CAMdata.driveDirection = DriveDirection_unavailable;

                /* Curvature and CurvatureCalculationMode */
                CAMdata.curvature = VDPValueConfidence<>(CurvatureValue_unavailable, CurvatureConfidence_unavailable);
                CAMdata.curvature_calculation_mode = CurvatureCalculationMode_unavailable;

                /* Length and Width [0.1 m] */
                // These can be set even in absence of a GPS fix
                CAMdata.VehicleLength = m_vehicle_length;
                CAMdata.VehicleWidth = m_vehicle_width;

                /* Yaw Rate */
                CAMdata.yawRate = VDPValueConfidence<>(YawRateValue_unavailable, YawRateConfidence_unavailable);

                CAMdata.avail = false;
            }
        }
    } else {
        std::string fixModeUbx = m_serialParserPtr->getFixModeUbx();
        std::string fixModeNmea = m_serialParserPtr->getFixModeNmea();
        if (fixModeUbx != "Invalid" && fixModeUbx != "NoFix" && fixModeUbx != "Unknown/Invalid" &&
            fixModeNmea != "NoFix (V)" && fixModeNmea != "NoFix" &&
            fixModeNmea != "Unknown/Invalid (V)" && fixModeNmea != "Unknown/Invalid") {

            if (m_serialParserPtr->getSpeedValidity(false) > 0) {

                double speed = m_serialParserPtr->getSpeed(nullptr, false);
                if (speed >= 0 && speed <= 163.82) {
                    CAMdata.speed = VDPValueConfidence<>(speed * CENTI, SpeedConfidence_unavailable);
                }
                else {
                    CAMdata.speed = VDPValueConfidence<>(SpeedValue_unavailable, SpeedConfidence_unavailable);
                }

            } else CAMdata.speed = VDPValueConfidence<>(SpeedValue_unavailable, SpeedConfidence_unavailable);

            long pos_age = -1;
            double longitude = Latitude_unavailable;
            double latitude = Longitude_unavailable;

            if (m_serialParserPtr->getPositionValidity(false) > 0) {
                std::pair<double, double> position = m_serialParserPtr->getPosition(&pos_age, false);
                CAMdata.latitude = (Latitude_t) (position.first * DOT_ONE_MICRO);
                latitude = position.first;
            }
            else {
                CAMdata.latitude = (Latitude_t) Latitude_unavailable;
            }

            if (m_serialParserPtr->getPositionValidity(false) > 0) {
                std::pair<double, double> position = m_serialParserPtr->getPosition(&pos_age, false);
                CAMdata.longitude = (Longitude_t) (position.second * DOT_ONE_MICRO);
                longitude = position.second;
            }
            else {
                CAMdata.longitude = (Longitude_t) Longitude_unavailable;
            }

//            long int time=duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
//            std::cout << "[getCAMMandatoryData] @ " << time << std::endl;
//            std::cout << "Position age: " << pos_age << "[us]" << std::endl;

            if (m_serialParserPtr->getAltitudeValidity(false) == true) {
                double altitude = m_serialParserPtr->getAltitude(nullptr, false);
                if (altitude * CENTI >= -100000 && altitude * CENTI <= 800000) {
                    CAMdata.altitude = VDPValueConfidence<>(altitude * CENTI, AltitudeConfidence_unavailable);
                }
                else {
                    CAMdata.altitude = VDPValueConfidence<>(AltitudeValue_unavailable, AltitudeConfidence_unavailable);
                }
            }
            else {
                CAMdata.altitude = VDPValueConfidence<>(AltitudeValue_unavailable, AltitudeConfidence_unavailable);
            }

            CAMdata.posConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
            CAMdata.posConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
            CAMdata.posConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;

            if (m_serialParserPtr->getAccelerationsValidity(false) == true) {
                double long_acc = m_serialParserPtr->getLongitudinalAcceleration(nullptr, false);
                if (long_acc >= -16 && long_acc <= 16) {
                    //CAMdata.longAcceleration = VDPValueConfidence<>(AccelerationValue_unavailable,AccelerationConfidence_unavailable);
                    CAMdata.longAcceleration = VDPValueConfidence<>(long_acc * DECI, AccelerationConfidence_unavailable);
                }
                else {
                    CAMdata.longAcceleration = VDPValueConfidence<>(AccelerationValue_unavailable,AccelerationConfidence_unavailable);
                }
            }
            else {
                CAMdata.longAcceleration = VDPValueConfidence<>(AccelerationValue_unavailable,AccelerationConfidence_unavailable);
            }

            if (m_serialParserPtr->getCourseOverGroundValidity(false) > 0) {
                double heading = m_serialParserPtr->getCourseOverGround(nullptr, false);
                if (static_cast<int>(heading * DECI) < 0 || static_cast<int>(heading * DECI) > 3601) {
                    CAMdata.heading = VDPValueConfidence<>(HeadingValue_unavailable, HeadingConfidence_unavailable);
                }
                else {
                    CAMdata.heading = VDPValueConfidence<>(heading * DECI, HeadingConfidence_unavailable);
                }

            }
            else {
                CAMdata.heading = VDPValueConfidence<>(HeadingValue_unavailable, HeadingConfidence_unavailable);
            }


            CAMdata.driveDirection = DriveDirection_unavailable;
            CAMdata.curvature = VDPValueConfidence<>(CurvatureValue_unavailable, CurvatureConfidence_unavailable);
            CAMdata.curvature_calculation_mode = CurvatureCalculationMode_unavailable;
            CAMdata.VehicleLength = m_vehicle_length;
            CAMdata.VehicleWidth = m_vehicle_width;

            if (m_serialParserPtr->getYawRateValidity(false) == true) {
                double yaw_rate = m_serialParserPtr->getYawRate(nullptr, false);

                if (yaw_rate >= -327.66 && yaw_rate <= 327.66) {
                    CAMdata.yawRate = VDPValueConfidence<>(yaw_rate * CENTI, YawRateConfidence_unavailable);
                }
                else {
                    CAMdata.yawRate = VDPValueConfidence<>(YawRateValue_unavailable, YawRateConfidence_unavailable);
                }
            }
            else {
                CAMdata.yawRate = VDPValueConfidence<>(YawRateValue_unavailable, YawRateConfidence_unavailable);
            }

            CAMdata.avail = true;

            // compute the delta position considering the speed, heading and the pos age
            if (CAMdata.avail == true && (CAMdata.latitude!=Latitude_unavailable && CAMdata.longitude!=Longitude_unavailable)) {
                double delta_x = 0.0;
                double delta_y = 0.0;
                double gammar=0,kr=0;
//                std::cout << "Read latitude: " << CAMdata.latitude << std::endl;
//                std::cout << "Read longitude: " << CAMdata.longitude << std::endl;

                if (CAMdata.speed.getValue() != SpeedValue_unavailable && CAMdata.heading.getValue() != HeadingValue_unavailable) {
                    double speed = CAMdata.speed.getValue() / CENTI; // [m/s]
                    double heading = CAMdata.heading.getValue() / DECI; // [degrees]
                    heading = DEG_2_RAD_BSR_2((90-heading));

                    // Compute the delta position
                    double delta_t = pos_age / 1000000.0; // [s]
                    delta_x = speed * delta_t * sin(heading);
                    delta_y = speed * delta_t * cos(heading);
                }

                // Update the position
                transverse_mercator_t tmerc = UTMUPS_init_UTM_TransverseMercator();
                double lat1, lon1, ego_x, ego_y;
                TransverseMercator_Forward(&tmerc, longitude, latitude, longitude, &ego_x, &ego_y, &gammar, &kr);
                ego_x += delta_x;
                ego_y += delta_y;
                TransverseMercator_Reverse(&tmerc, longitude, ego_x, ego_y, &lat1, &lon1, &gammar, &kr);

                CAMdata.latitude = (Latitude_t) (lat1 * DOT_ONE_MICRO);
                CAMdata.longitude = (Longitude_t) (lon1 * DOT_ONE_MICRO);

                //print all the data
//                std::cout << "Speed: " << CAMdata.speed.getValue()/CENTI << "[m/s]"<< std::endl;
//                std::cout << "Heading: " << CAMdata.heading.getValue()/DECI  << "[degrees]"<< std::endl;
//                std::cout << "Delta x: " << delta_x << "[m]" << std::endl;
//                std::cout << "Delta y: " << delta_y << "[m]" << std::endl;
//                std::cout << "PositionDelta: " << sqrt(delta_x*delta_x + delta_y*delta_y) << "[m]" <<  std::endl;
//                std::cout << "New Latitude: " << CAMdata.latitude << std::endl;
//                std::cout << "New longitude: " << CAMdata.longitude << std::endl;
            }
        }
        else {
            // Set everything to unavailable as no fix was possible (i.e., the resulting CAM will not be so useful...)
            CAMdata.speed = VDPValueConfidence<>(SpeedValue_unavailable, SpeedConfidence_unavailable);
            CAMdata.latitude = (Latitude_t) Latitude_unavailable;
            CAMdata.longitude = (Longitude_t) Longitude_unavailable;
            CAMdata.altitude = VDPValueConfidence<>(AltitudeValue_unavailable, AltitudeConfidence_unavailable);
            CAMdata.posConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
            CAMdata.posConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
            CAMdata.posConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
            CAMdata.longAcceleration = VDPValueConfidence<>(AccelerationValue_unavailable,
                                                            AccelerationConfidence_unavailable);
            CAMdata.heading = VDPValueConfidence<>(HeadingValue_unavailable, HeadingConfidence_unavailable);
            CAMdata.driveDirection = DriveDirection_unavailable;
            CAMdata.curvature = VDPValueConfidence<>(CurvatureValue_unavailable, CurvatureConfidence_unavailable);
            CAMdata.curvature_calculation_mode = CurvatureCalculationMode_unavailable;
            CAMdata.VehicleLength = m_vehicle_length;
            CAMdata.VehicleWidth = m_vehicle_width;
            CAMdata.yawRate = VDPValueConfidence<>(YawRateValue_unavailable, YawRateConfidence_unavailable);
            CAMdata.avail = false;
        }
    }

    return CAMdata;
}

VDPGPSClient::CAM_conditions_data
VDPGPSClient::getCAMConditionsData()
{
    CAM_conditions_data_t CAMdata={.avail=false};

    if(m_use_gpsd==true) {
        int rval;

        rval = gps_read(&m_gps_data, nullptr, 0);

        if (rval == -1) {
            throw std::runtime_error("Cannot read data from GNSS device: " + std::string(gps_errstr(rval)));
        } else {
            // Check if the mode is set and if a fix has been obtained
            if ((m_gps_data.set & MODE_SET) == MODE_SET) { // && GPSSTATUS(m_gps_data)!=STATUS_NO_FIX) {
                if (m_gps_data.fix.mode == MODE_2D || m_gps_data.fix.mode == MODE_3D) {
                    /* Speed [0.01 m/s] */
                    CAMdata.speedCheck = m_gps_data.fix.speed * CENTI;
                    /* Latitude WGS84 [0,1 microdegree] */
                    CAMdata.currPos.first = m_gps_data.fix.latitude;
                    /* Longitude WGS84 [0,1 microdegree] */
                    CAMdata.currPos.second = m_gps_data.fix.longitude;

                    /* Heading WGS84 north [0.1 degree] - m_gps_data.fix.track should already provide a CW heading relative to North */
                    if (static_cast<int>(m_gps_data.fix.track * DECI) < 0 ||
                        static_cast<int>(m_gps_data.fix.track * DECI) > 3601) {
                        CAMdata.headCheck = HeadingValue_unavailable;
                        CAMdata.headCheckDbl = -DBL_MAX;
                    } else {
                        CAMdata.headCheck = static_cast<int>(m_gps_data.fix.track * DECI);
                        CAMdata.headCheckDbl = m_gps_data.fix.track;
                    }

                    /* This flag represents an easy way to understand if it was possible to read any valid data from the GNSS device */
                    CAMdata.avail = true;
                }
            } else {
                // Set everything to unavailable as no fix was possible (i.e., the resulting CAM will not be so useful...)
                /* Speed [0.01 m/s] */
                CAMdata.speedCheck = SpeedValue_unavailable;
                /* Latitude WGS84 [0,1 microdegree] */
                CAMdata.currPos.first = -DBL_MAX;
                /* Longitude WGS84 [0,1 microdegree] */
                CAMdata.currPos.second = -DBL_MAX;
                /* Heading WGS84 north [0.1 degree] */
                CAMdata.headCheck = HeadingValue_unavailable;
                CAMdata.headCheckDbl = -DBL_MAX;
                CAMdata.avail = false;
            }
        }
    } else {
        std::string fixModeUbx = m_serialParserPtr->getFixModeUbx();
        std::string fixModeNmea = m_serialParserPtr->getFixModeNmea();
        if (fixModeUbx != "Invalid" && fixModeUbx != "NoFix" && fixModeUbx != "Unknown/Invalid" &&
            fixModeNmea != "NoFix (V)" && fixModeNmea != "NoFix" &&
            fixModeNmea != "Unknown/Invalid (V)" && fixModeNmea != "Unknown/Invalid") {

            if (m_serialParserPtr->getSpeedValidity(false) > 0) {
                double speed = m_serialParserPtr->getSpeed(nullptr, false);
                if (speed >= 0 && speed <= 163.82) {
                    CAMdata.speedCheck = speed * CENTI;
                }
                else {
                    CAMdata.speedCheck = SpeedValue_unavailable;
                }
            }
            else {
                CAMdata.speedCheck = SpeedValue_unavailable;
            }

            long pos_age = -1;
            double longitude = -1.0;
            double latitude = -1.0;

            if (m_serialParserPtr->getPositionValidity(false) > 0) {
                CAMdata.currPos = m_serialParserPtr->getPosition(&pos_age, false);
                latitude = CAMdata.currPos.first;
                longitude = CAMdata.currPos.second;
            } else {
                CAMdata.currPos.first = -DBL_MAX;
                CAMdata.currPos.second = -DBL_MAX;
            }

/*            long int time=duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
            std::cout << "[getCAMConditionsData] @ " << time << std::endl;
            std::cout << "Position age: " << pos_age << "[us]" << std::endl;*/

            // printf("BEFORE: %.2lf\n",hdd);
            // printf("VALID? %d\n",m_serialParserPtr->getSpeedAndCogValidity(true));
            if (m_serialParserPtr->getCourseOverGroundValidity(false) > 0) {
                double heading = m_serialParserPtr->getCourseOverGround(nullptr, false);
                if (static_cast<int>(heading * DECI) < 0 || static_cast<int>(heading * DECI) > 3601) {
                    CAMdata.headCheck = HeadingValue_unavailable;
                    CAMdata.headCheckDbl = -DBL_MAX;
                }
                else {
                    CAMdata.headCheck = heading * DECI;
                    CAMdata.headCheckDbl = heading;
                }
            }
            else {
                CAMdata.headCheck = HeadingValue_unavailable;
                CAMdata.headCheckDbl = -DBL_MAX;
            }

            CAMdata.avail = true;

            // compute the delta position considering the speed, heading and the pos age
            if (CAMdata.avail == true && (CAMdata.currPos.first != -DBL_MAX && CAMdata.currPos.second != -DBL_MAX)) {
                double delta_x = 0.0;
                double delta_y = 0.0;
                double gammar=0,kr=0;
                long speed_threshold = 1500; // [cm/s]
                long time_threshold = 20000; // [us]
/*                std::cout << "Read latitude: " << CAMdata.currPos.first << std::endl;
                std::cout << "Read longitude: " << CAMdata.currPos.second << std::endl;*/

                if (CAMdata.speedCheck != SpeedValue_unavailable && CAMdata.headCheck != HeadingValue_unavailable
                     && CAMdata.speedCheck >= speed_threshold && pos_age >= time_threshold) {
                    double speed = (double) CAMdata.speedCheck / CENTI; // [m/s]
                    double heading = CAMdata.headCheckDbl; // [degrees]
                    heading = DEG_2_RAD_BSR_2((90-heading));

                    // Compute the delta position
                    double delta_t = pos_age / 1000000.0; // [s]
                    delta_x = speed * delta_t * sin(heading);
                    delta_y = speed * delta_t * cos(heading);
                }

                // Update the position
                transverse_mercator_t tmerc = UTMUPS_init_UTM_TransverseMercator();
                double lat1, lon1, ego_x, ego_y;
                TransverseMercator_Forward(&tmerc, longitude, latitude, longitude, &ego_x, &ego_y, &gammar, &kr);
                ego_x += delta_x;
                ego_y += delta_y;
                TransverseMercator_Reverse(&tmerc, longitude, ego_x, ego_y, &lat1, &lon1, &gammar, &kr);

                CAMdata.currPos.first = lat1;
                CAMdata.currPos.second = lon1;

/*                //print all the data
                std::cout << "Speed: " << CAMdata.speedCheck/CENTI << "[m/s]"<< std::endl;
                std::cout << "Heading: " << CAMdata.headCheckDbl  << "[degrees]"<< std::endl;
                std::cout << "Delta x: " << delta_x << "[m]" << std::endl;
                std::cout << "Delta y: " << delta_y << "[m]" << std::endl;
                std::cout << "PositionDelta: " << sqrt(delta_x*delta_x + delta_y*delta_y) << "[m]" <<  std::endl;
                std::cout << "New Latitude: " << CAMdata.currPos.first << std::endl;
                std::cout << "New longitude: " << CAMdata.currPos.second  << std::endl;*/
            }
        }
        else {
            // Set everything to unavailable as no fix was possible (i.e., the resulting CAM will not be so useful...)
            CAMdata.speedCheck = SpeedValue_unavailable;
            CAMdata.currPos.first = -DBL_MAX;
            CAMdata.currPos.second = -DBL_MAX;
            CAMdata.headCheck = HeadingValue_unavailable;
            CAMdata.headCheckDbl = -DBL_MAX;
            CAMdata.avail = false;
        }
    }
    return CAMdata;
}

VDPGPSClient::CPM_mandatory_data_t VDPGPSClient::getCPMMandatoryData() {
    CPM_mandatory_data_t CPMdata={.avail=false};

    if(m_use_gpsd==true) {
        int rval;

        rval = gps_read(&m_gps_data, nullptr, 0);

        if (rval == -1) {
            throw std::runtime_error("Cannot read data from GNSS device: " + std::string(gps_errstr(rval)));
        } else {
            // Check if the mode is set and if a fix has been obtained
            if ((m_gps_data.set & MODE_SET) == MODE_SET) { // && GPSSTATUS(m_gps_data)!=STATUS_NO_FIX) {
                if (m_gps_data.fix.mode == MODE_2D || m_gps_data.fix.mode == MODE_3D) {
                    /* Speed [0.01 m/s] */
                    CPMdata.speed = VDPValueConfidence<>(m_gps_data.fix.speed * CENTI, SpeedConfidence_unavailable);

                    /* Latitude WGS84 [0,1 microdegree] */
                    CPMdata.latitude = (Latitude_t) (m_gps_data.fix.latitude * DOT_ONE_MICRO);
                    /* Longitude WGS84 [0,1 microdegree] */
                    CPMdata.longitude = (Longitude_t) (m_gps_data.fix.longitude * DOT_ONE_MICRO);

                    int asnAltitudeValue = static_cast<int>(m_gps_data.fix.altitude * CENTI);
                    /* Altitude [0,01 m] */
                    if (m_gps_data.fix.mode == MODE_3D && asnAltitudeValue >= -100000 && asnAltitudeValue <= 800000) {
                        CPMdata.altitude = VDPValueConfidence<>(static_cast<int>(m_gps_data.fix.altitude * CENTI),
                                                                AltitudeConfidence_unavailable);
                    } else {
                        CPMdata.altitude = VDPValueConfidence<>(AltitudeValue_unavailable,
                                                                AltitudeConfidence_unavailable);
                    }

                    /* Position Confidence Ellipse */
                    CPMdata.posConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
                    CPMdata.posConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
                    CPMdata.posConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;

                    /* Longitudinal acceleration [0.1 m/s^2] */
                    CPMdata.longAcceleration = VDPValueConfidence<>(AccelerationValue_unavailable,
                                                                    AccelerationConfidence_unavailable);

                    /* Heading WGS84 north [0.1 degree] - m_gps_data.fix.track should already provide a CW heading relative to North */
                    if (static_cast<int>(m_gps_data.fix.track * DECI) < 0 ||
                        static_cast<int>(m_gps_data.fix.track * DECI) > 3601) {
                        CPMdata.heading = VDPValueConfidence<>(HeadingValue_unavailable, HeadingConfidence_unavailable);
                    } else {
                        CPMdata.heading = VDPValueConfidence<>(static_cast<int>(m_gps_data.fix.track * DECI),
                                                               HeadingConfidence_unavailable);
                    }

                    /* Curvature and CurvatureCalculationMode */
                    CPMdata.curvature = VDPValueConfidence<>(CurvatureValue_unavailable,
                                                             CurvatureConfidence_unavailable);
                    CPMdata.curvature_calculation_mode = CurvatureCalculationMode_unavailable;

                    /* Length and Width [0.1 m] */
                    CPMdata.VehicleLength = m_vehicle_length;
                    CPMdata.VehicleWidth = m_vehicle_width;

                    /* Yaw Rate */
                    CPMdata.yawRate = VDPValueConfidence<>(YawRateValue_unavailable, YawRateConfidence_unavailable);

                    /* This flag represents an easy way to understand if it was possible to read any valid data from the GNSS device */
                    CPMdata.avail = true;
                }
            } else {
                // Set everything to unavailable as no fix was possible (i.e., the resulting CAM will not be so useful...)

                /* Speed [0.01 m/s] */
                CPMdata.speed = VDPValueConfidence<>(SpeedValue_unavailable, SpeedConfidence_unavailable);

                /* Latitude WGS84 [0,1 microdegree] */
                CPMdata.latitude = (Latitude_t) Latitude_unavailable;
                /* Longitude WGS84 [0,1 microdegree] */
                CPMdata.longitude = (Longitude_t) Longitude_unavailable;

                /* Altitude [0,01 m] */
                CPMdata.altitude = VDPValueConfidence<>(AltitudeValue_unavailable, AltitudeConfidence_unavailable);

                /* Position Confidence Ellipse */
                CPMdata.posConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
                CPMdata.posConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
                CPMdata.posConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;

                /* Longitudinal acceleration [0.1 m/s^2] */
                CPMdata.longAcceleration = VDPValueConfidence<>(AccelerationValue_unavailable,
                                                                AccelerationConfidence_unavailable);

                /* Heading WGS84 north [0.1 degree] */
                CPMdata.heading = VDPValueConfidence<>(HeadingValue_unavailable, HeadingConfidence_unavailable);

                /* Curvature and CurvatureCalculationMode */
                CPMdata.curvature = VDPValueConfidence<>(CurvatureValue_unavailable, CurvatureConfidence_unavailable);
                CPMdata.curvature_calculation_mode = CurvatureCalculationMode_unavailable;

                /* Length and Width [0.1 m] */
                // These can be set even in absence of a GPS fix
                CPMdata.VehicleLength = m_vehicle_length;
                CPMdata.VehicleWidth = m_vehicle_width;

                /* Yaw Rate */
                CPMdata.yawRate = VDPValueConfidence<>(YawRateValue_unavailable, YawRateConfidence_unavailable);

                CPMdata.avail = false;
            }
        }
    } else {
        std::string fixModeUbx = m_serialParserPtr->getFixModeUbx();
        std::string fixModeNmea = m_serialParserPtr->getFixModeNmea();
        if (fixModeUbx != "Invalid" && fixModeUbx != "NoFix" && fixModeUbx != "Unknown/Invalid" &&
            fixModeNmea != "NoFix (V)" && fixModeNmea != "NoFix" &&
            fixModeNmea != "Unknown/Invalid (V)" && fixModeNmea != "Unknown/Invalid") {
            std::pair<double, double> position = m_serialParserPtr->getPosition(nullptr, false);
            double speed = m_serialParserPtr->getSpeedNmea(nullptr, false);
            double heading = m_serialParserPtr->getCourseOverGroundNmea(nullptr, false);

            CPMdata.speed = VDPValueConfidence<>(speed * CENTI, SpeedConfidence_unavailable);
            CPMdata.latitude = (Latitude_t) (position.first * DOT_ONE_MICRO);
            CPMdata.longitude = (Longitude_t) (position.second * DOT_ONE_MICRO);
            CPMdata.altitude = VDPValueConfidence<>(AltitudeValue_unavailable, AltitudeConfidence_unavailable);
            CPMdata.posConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
            CPMdata.posConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
            CPMdata.posConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
            CPMdata.longAcceleration = VDPValueConfidence<>(AccelerationValue_unavailable,
                                                            AccelerationConfidence_unavailable);

            if (static_cast<int>(heading * DECI) < 0 ||
                static_cast<int>(heading * DECI) > 3601) {
                CPMdata.heading = VDPValueConfidence<>(HeadingValue_unavailable, HeadingConfidence_unavailable);
            } else {
                CPMdata.heading = VDPValueConfidence<>(static_cast<int>(heading * DECI),
                                                       HeadingConfidence_unavailable);
            }

            CPMdata.curvature = VDPValueConfidence<>(CurvatureValue_unavailable, CurvatureConfidence_unavailable);
            CPMdata.curvature_calculation_mode = CurvatureCalculationMode_unavailable;
            CPMdata.VehicleLength = m_vehicle_length;
            CPMdata.VehicleWidth = m_vehicle_width;
            CPMdata.yawRate = VDPValueConfidence<>(YawRateValue_unavailable, YawRateConfidence_unavailable);
            CPMdata.avail = true;
        } else {
            // Set everything to unavailable as no fix was possible (i.e., the resulting CPM will not be so useful...)
            CPMdata.speed = VDPValueConfidence<>(SpeedValue_unavailable, SpeedConfidence_unavailable);
            CPMdata.latitude = (Latitude_t) Latitude_unavailable;
            CPMdata.longitude = (Longitude_t) Longitude_unavailable;
            CPMdata.altitude = VDPValueConfidence<>(AltitudeValue_unavailable, AltitudeConfidence_unavailable);
            CPMdata.posConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
            CPMdata.posConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
            CPMdata.posConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
            CPMdata.longAcceleration = VDPValueConfidence<>(AccelerationValue_unavailable,
                                                            AccelerationConfidence_unavailable);
            CPMdata.heading = VDPValueConfidence<>(HeadingValue_unavailable, HeadingConfidence_unavailable);
            CPMdata.curvature = VDPValueConfidence<>(CurvatureValue_unavailable, CurvatureConfidence_unavailable);
            CPMdata.curvature_calculation_mode = CurvatureCalculationMode_unavailable;
            CPMdata.VehicleLength = m_vehicle_length;
            CPMdata.VehicleWidth = m_vehicle_width;
            CPMdata.yawRate = VDPValueConfidence<>(YawRateValue_unavailable, YawRateConfidence_unavailable);
            CPMdata.avail = false;
        }
    }

    return CPMdata;
}