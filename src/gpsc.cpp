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

extern "C" {
    #include "VAM.h"
    #include "utmuts.h"
}

#define MIN_DIST_EQUAL_x_EPSILON 0.0001 // [m]
static const double PI_CONST = 3.1415926535897932384626433832795028841971693993751058209;
#define VRUDP_HEADING_UNAVAILABLE -DBL_MAX

#define GPSSTATUS(gpsdata) gpsdata.fix.status

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

double VDPGPSClient::getParserSpeedUbx() {
    return m_serialParserPtr->getSpeedUbx(nullptr,false);
}

double VDPGPSClient::getParserSpeedNmea() {
    return m_serialParserPtr->getSpeedNmea(nullptr,false);
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

std::string VDPGPSClient::getParserUtcTimeUbx() {
    return m_serialParserPtr->getUtcTimeUbx();
}

std::string VDPGPSClient::getParserUtcTimeNmea() {
    return m_serialParserPtr->getUtcTimeNmea();
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
        // Check if at least a 2D fix is present and if the data is not outdated, else return 0 or unavailable
        if (m_serialParserPtr->getFixValidity2D(false) == true || m_serialParserPtr->getFixValidity3D(false) == true) {
            if (m_serialParserPtr->getSpeedAndCogValidity(false) == true) {
                double heading = m_serialParserPtr->getCourseOverGroundUbx(nullptr, false);
                if (heading == HeadingValue_unavailable) heading = m_serialParserPtr->getCourseOverGroundNmea(nullptr, false);

                if (static_cast<int>(heading * DECI) < 0 || static_cast<int>(heading * DECI) > 3601) {
                    return VDPValueConfidence<>(HeadingValue_unavailable, HeadingConfidence_unavailable);
                } else return VDPValueConfidence<>(static_cast<int>(heading * DECI), HeadingConfidence_unavailable);
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
        // Check if at least a 2D fix is present and if the data is not outdated, else return 0 or unavailable
        if (m_serialParserPtr->getFixValidity2D(false) == true || m_serialParserPtr->getFixValidity3D(false) == true) {
            if (m_serialParserPtr->getSpeedAndCogValidity(false) == true) {
                double heading = m_serialParserPtr->getCourseOverGroundUbx(nullptr, false);
                if (heading == 0) heading = m_serialParserPtr->getCourseOverGroundNmea(nullptr, false);

                if (static_cast<int>(heading * DECI) < 0 || static_cast<int>(heading * DECI) > 3601) {
                    return VRUDP_HEADING_UNAVAILABLE;
                } else return heading;
            }
        }
    }
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
        // Check if at least a 2D fix is present
        if (m_serialParserPtr->getFixValidity2D(false) == true || m_serialParserPtr->getFixValidity3D(false) == true) {
            if (m_serialParserPtr->getSpeedAndCogValidity(false) == true) {
                double speed = m_serialParserPtr->getSpeedUbx(nullptr, false);
                if (speed == SpeedValue_unavailable) speed = m_serialParserPtr->getSpeedNmea(nullptr, false);
                return VDPValueConfidence<>(static_cast<int>(speed * CENTI), SpeedConfidence_unavailable);
            } else return VDPValueConfidence<>(SpeedValue_unavailable, SpeedConfidence_unavailable);
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
        // Check if at least a 2D fix is present
        if (m_serialParserPtr->getFixValidity2D(false) == true || m_serialParserPtr->getFixValidity3D(false) == true) {
            if (m_serialParserPtr->getSpeedAndCogValidity(false) == true) {
                double speed = m_serialParserPtr->getSpeedUbx(nullptr, false);
                if (speed == 0) speed = m_serialParserPtr->getSpeedNmea(nullptr, false);
                return speed;
            } else return -DBL_MAX;
        }
    }
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
        // Check if at least a 2D fix is present and if the value il not outdated else return 0 or unavailable
        if (m_serialParserPtr->getFixValidity3D(false) == true && m_serialParserPtr->getAltitudeValidity(false) == true) {
            double altitude = m_serialParserPtr->getAltitude(nullptr, false);
            return VDPValueConfidence<>(static_cast<int>(altitude * CENTI), AltitudeConfidence_unavailable);
        }
    } return VDPValueConfidence<>(AltitudeValue_unavailable, AltitudeConfidence_unavailable);
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
        // Check if at least a 2D fix is present
        if (m_serialParserPtr->getFixValidity2D(false) == true || m_serialParserPtr->getFixValidity3D(false) == true) {
            if (m_serialParserPtr->getPositionValidity(false) == true) {
                std::pair<double, double> position = m_serialParserPtr->getPosition(nullptr, false);
                return std::pair<long, long>(position.first * DOT_ONE_MICRO, position.second * DOT_ONE_MICRO);
            }
        }
        return std::pair<double, double>(Latitude_unavailable, Longitude_unavailable);
    }
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
        // Check if at least a 2D fix is present
        if (m_serialParserPtr->getFixValidity2D(false) == true || m_serialParserPtr->getFixValidity3D(false) == true) {
            if (m_serialParserPtr->getPositionValidity(false) == true && m_serialParserPtr->getAltitudeValidity(false) == true) {
                std::pair<double, double> position = m_serialParserPtr->getPosition(nullptr, false);
                double altitude = m_serialParserPtr->getAltitude(nullptr, false);
                ped_pos.lat = position.first;
                ped_pos.lon = position.second;
                ped_pos.alt = altitude;
            }
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
        // Check if at least a 3D fix is present
        if (m_serialParserPtr->getFixValidity3D(false) == true && m_serialParserPtr->getAccelerationsValidity(false) == true) {
            double long_acc = m_serialParserPtr->getLongitudinalAcceleration(nullptr, false);
            if (long_acc >= -16 && long_acc <= 16) {
                return VDPValueConfidence<>(static_cast<int>(long_acc * DECI), AccelerationConfidence_unavailable);
            }
            else return VDPValueConfidence<>(LongitudinalAccelerationValue_unavailable, AccelerationConfidence_unavailable);
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
        // Check if at least a 2D fix is present, else return 0 or unavailable
        if (m_serialParserPtr->getFixValidity3D(false) == true && m_serialParserPtr->getYawRateValidity(false) == true) {
            double yaw_rate = m_serialParserPtr->getYawRate(nullptr, false);
            return VDPValueConfidence<>(static_cast<int>(yaw_rate * CENTI), YawRateConfidence_unavailable);
        }
        return VDPValueConfidence<>(YawRateValue_unavailable, YawRateConfidence_unavailable);
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
        if (m_serialParserPtr->getFixValidity2D(false) == true || m_serialParserPtr->getFixValidity3D(false) == true) {
            if (m_serialParserPtr->getSpeedAndCogValidity(false) == true) {
                double heading_dbl = m_serialParserPtr->getCourseOverGroundUbx(nullptr, false);
                if (heading_dbl == HeadingValue_unavailable) heading_dbl = m_serialParserPtr->getCourseOverGroundNmea(nullptr, false);

                if (static_cast<int>(heading_dbl * DECI) < 0 ||
                    static_cast<int>(heading_dbl * DECI) > 3601) {
                    return HeadingValue_unavailable;
                } else {
                    return heading_dbl;
                }
            }
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
        if (m_serialParserPtr->getFixValidity2D(false) == true || m_serialParserPtr->getFixValidity3D(false) == true) {
            if (m_serialParserPtr->getSpeedAndCogValidity(false) == true) {
                double speed_dbl = m_serialParserPtr->getSpeedUbx(nullptr, false);
                if (speed_dbl == SpeedValue_unavailable) speed_dbl = m_serialParserPtr->getSpeedNmea(nullptr, false);
                return speed_dbl;
            }
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
        if (m_serialParserPtr->getFixValidity3D(false) == true && m_serialParserPtr->getAttitudeValidity(false) == true) {
            return m_serialParserPtr->getAltitude(nullptr, false);
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
    } else if (m_serialParserPtr->getFixValidity2D(false) == true || m_serialParserPtr->getFixValidity3D(false) == true) {
        if (m_serialParserPtr->getPositionValidity(false) == true) {
            return m_serialParserPtr->getPosition(nullptr, false);
        }
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
        if (m_serialParserPtr->getFixValidity3D(false) == true && m_serialParserPtr->getAccelerationsValidity(false) == true) {
            return m_serialParserPtr->getLongitudinalAcceleration(nullptr, false);
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
        if (m_serialParserPtr->getFixValidity3D(false) == true && m_serialParserPtr->getYawRateValidity(false) == true) {
            return m_serialParserPtr->getYawRate(nullptr, false);
        }
    }
    return YawRateValue_unavailable;
}

VDPGPSClient::CAM_mandatory_data_t
VDPGPSClient::
getCAMMandatoryData() {
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

            if (m_serialParserPtr->getSpeedAndCogValidity(false) == true) {
                double speed = m_serialParserPtr->getSpeedUbx(nullptr, false);
                // If no speed is available from UBX, use NMEA
                if (speed == 0) {
                    //speed = m_serialParserPtr->getSpeedNmea(nullptr,false);
                }
                if (speed >= 0 && speed <= 163.82) {
                    CAMdata.speed = VDPValueConfidence<>(speed * CENTI, SpeedConfidence_unavailable);
                } else CAMdata.speed = VDPValueConfidence<>(SpeedValue_unavailable, SpeedConfidence_unavailable);

            } else CAMdata.speed = VDPValueConfidence<>(SpeedValue_unavailable, SpeedConfidence_unavailable);

            if (m_serialParserPtr->getPositionValidity(false) == true) {
                std::pair<double, double> position = m_serialParserPtr->getPosition(nullptr, false);
                CAMdata.latitude = (Latitude_t) (position.first * DOT_ONE_MICRO);
            } else CAMdata.latitude = (Latitude_t) Latitude_unavailable;

            if (m_serialParserPtr->getPositionValidity(false) == true) {
                std::pair<double, double> position = m_serialParserPtr->getPosition(nullptr, false);
                CAMdata.longitude = (Longitude_t) (position.second * DOT_ONE_MICRO);
            } else CAMdata.longitude = (Longitude_t) Longitude_unavailable;

            if (m_serialParserPtr->getAltitudeValidity(false) == true) {
                double altitude = m_serialParserPtr->getAltitude(nullptr, false);
                if (altitude * CENTI >= -100000 && altitude * CENTI <= 800000) {
                    CAMdata.altitude = VDPValueConfidence<>(altitude * CENTI, AltitudeConfidence_unavailable);
                } else CAMdata.altitude = VDPValueConfidence<>(AltitudeValue_unavailable, AltitudeConfidence_unavailable);
            } else CAMdata.altitude = VDPValueConfidence<>(AltitudeValue_unavailable, AltitudeConfidence_unavailable);

            CAMdata.posConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
            CAMdata.posConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
            CAMdata.posConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;

            if (m_serialParserPtr->getAccelerationsValidity(false) == true) {
                double long_acc = m_serialParserPtr->getLongitudinalAcceleration(nullptr, false);
                if (long_acc >= -16 && long_acc <= 16) {
                    //CAMdata.longAcceleration = VDPValueConfidence<>(AccelerationValue_unavailable,AccelerationConfidence_unavailable);
                    CAMdata.longAcceleration = VDPValueConfidence<>(long_acc * DECI, AccelerationConfidence_unavailable);
                } else CAMdata.longAcceleration = VDPValueConfidence<>(AccelerationValue_unavailable,AccelerationConfidence_unavailable);
            } else CAMdata.longAcceleration = VDPValueConfidence<>(AccelerationValue_unavailable,AccelerationConfidence_unavailable);

            double hdd = m_serialParserPtr->getCourseOverGroundUbx(nullptr, false);
            if (hdd == 0) hdd = m_serialParserPtr->getCourseOverGroundNmea(nullptr,false);

            // printf("BEFORE: %.2lf\n",hdd);
            // printf("VALID? %d\n",m_serialParserPtr->getSpeedAndCogValidity(true));
            if (m_serialParserPtr->getSpeedAndCogValidity(false) == true) {
                double heading = m_serialParserPtr->getCourseOverGroundUbx(nullptr, false);

                // If no heading is available from UBX, use NMEA
                if (heading == 0) heading = m_serialParserPtr->getCourseOverGroundNmea(nullptr,false);

                if (static_cast<int>(heading * DECI) < 0 || static_cast<int>(heading * DECI) > 3601) {
                    CAMdata.heading = VDPValueConfidence<>(HeadingValue_unavailable, HeadingConfidence_unavailable);
                } else CAMdata.heading = VDPValueConfidence<>(heading * DECI, HeadingConfidence_unavailable);

            } else CAMdata.heading = VDPValueConfidence<>(HeadingValue_unavailable, HeadingConfidence_unavailable);

            // printf("AFTER: %d\n",CAMdata.heading.getValue());

            CAMdata.driveDirection = DriveDirection_unavailable;
            CAMdata.curvature = VDPValueConfidence<>(CurvatureValue_unavailable, CurvatureConfidence_unavailable);
            CAMdata.curvature_calculation_mode = CurvatureCalculationMode_unavailable;
            CAMdata.VehicleLength = m_vehicle_length;
            CAMdata.VehicleWidth = m_vehicle_width;

            if (m_serialParserPtr->getYawRateValidity(false) == true) {
                double yaw_rate = m_serialParserPtr->getYawRate(nullptr, false);
                if (yaw_rate >= -327.66 && yaw_rate <= 327.66) {
                    //CAMdata.yawRate = VDPValueConfidence<>(YawRateValue_unavailable, YawRateConfidence_unavailable);
                    CAMdata.yawRate = VDPValueConfidence<>(yaw_rate * CENTI, YawRateConfidence_unavailable);
                } else CAMdata.yawRate = VDPValueConfidence<>(YawRateValue_unavailable, YawRateConfidence_unavailable);
            } else CAMdata.yawRate = VDPValueConfidence<>(YawRateValue_unavailable, YawRateConfidence_unavailable);

            CAMdata.avail = true;

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