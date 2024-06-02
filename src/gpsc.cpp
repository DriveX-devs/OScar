#include "gpsc.h"
#include <stdexcept>
#include <math.h> 
#include <cfloat>

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

        return VDPValueConfidence<>(HeadingValue_unavailable, HeadingConfidence_unavailable);
    } else {
        // Check if at least a 2D fix is present and if the data is not outdated, else return 0 or unavailable
        if ((m_serialParserPtr->getFixValidity2D() == true || m_serialParserPtr->getFixValidity3D() == true) && m_serialParserPtr->getSpeedAndCogValidity(false) == true) {
            double heading = m_serialParserPtr->getCourseOverGroundNmea(nullptr, false);
            return VDPValueConfidence<>(static_cast<int>(heading * DECI), HeadingConfidence_unavailable);
        } else return VDPValueConfidence<>(HeadingValue_unavailable, HeadingConfidence_unavailable);
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
                }
            }
        }

        return VDPValueConfidence<>(SpeedValue_unavailable, SpeedConfidence_unavailable);
    } else {
        // Check if at least a 2D fix is present, else return 0 or unavailable
        if ((m_serialParserPtr->getFixValidity2D() == true || m_serialParserPtr->getFixValidity3D() == true) && m_serialParserPtr->getSpeedAndCogValidity(false) == true) {
            double speed = m_serialParserPtr->getSpeed(nullptr, false);
            return VDPValueConfidence<>(static_cast<int>(speed * CENTI), SpeedConfidence_unavailable);
        } else return VDPValueConfidence<>(SpeedValue_unavailable, SpeedConfidence_unavailable);
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
                }
            }
        }
        return VDPValueConfidence<>(AltitudeValue_unavailable, AltitudeConfidence_unavailable);

    } else {
        // Check if at least a 2D fix is present and if the value il not outdated else return 0 or unavailable
        if (m_serialParserPtr->getFixValidity3D() == true && m_serialParserPtr->getAltitudeValidity(false) == true) {
            double altitude = m_serialParserPtr->getAltitude(nullptr, false);
            return VDPValueConfidence<>(static_cast<int>(altitude * CENTI), AltitudeConfidence_unavailable);
        } else return VDPValueConfidence<>(AltitudeValue_unavailable, AltitudeConfidence_unavailable);
    }
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
        // Check if at least a 2D fix is present, else return 0 or unavailable
        if ((m_serialParserPtr->getFixValidity2D() == true || m_serialParserPtr->getFixValidity3D() == true) && m_serialParserPtr->getPositionValidity(false) == true) {
            std::pair<double, double> position = m_serialParserPtr->getPosition(nullptr, false);
            return std::pair<long, long>(position.first * DOT_ONE_MICRO, position.second * DOT_ONE_MICRO);
        } else std::pair<double, double>(Latitude_unavailable, Longitude_unavailable);
    }
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
        }
        return VDPValueConfidence<>(AltitudeValue_unavailable, AltitudeConfidence_unavailable);
        */
    } else {
        // Check if at least a 2D fix is present, else return 0 or unavailable
        if (m_serialParserPtr->getFixValidity3D() == true && m_serialParserPtr->getAccelerationsValidity(false) == true) {
            double long_acc = m_serialParserPtr->getLongitudinalAcceleration(nullptr, false);
            return VDPValueConfidence<>(static_cast<int>(long_acc * DECI), AccelerationConfidence_unavailable);
        } else return VDPValueConfidence<>(AccelerationValue_unavailable, AccelerationConfidence_unavailable);
    }
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
        if (m_serialParserPtr->getFixValidity3D() == true && m_serialParserPtr->getYawRateValidity(false) == true) {
            double yaw_rate = m_serialParserPtr->getYawRate(nullptr, false);
            return VDPValueConfidence<>(static_cast<int>(yaw_rate * CENTI), YawRateConfidence_unavailable);
        } else return VDPValueConfidence<>(YawRateValue_unavailable, YawRateConfidence_unavailable);
    }
}

double
VDPGPSClient::getHeadingValueDbl() {
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
                        return -DBL_MAX;
                    } else {
                        return m_gps_data.fix.track;
                    }
                }
            }
        }
    } else {
        if ((m_serialParserPtr->getFixValidity2D() == true || m_serialParserPtr->getFixValidity3D() == true) && m_serialParserPtr->getSpeedAndCogValidity(false) == true) {
            return m_serialParserPtr->getCourseOverGroundNmea(nullptr, false);
        } else return 0;
    }

	return -DBL_MAX;
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
    } else{
        if ((m_serialParserPtr->getFixValidity2D() == true || m_serialParserPtr->getFixValidity3D() == true) && m_serialParserPtr->getSpeedAndCogValidity(false) == true) {
            return m_serialParserPtr->getSpeed(nullptr, false);
        } else return 0;
    }

	return -DBL_MAX;
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
        if (m_serialParserPtr->getFixValidity3D() == true && m_serialParserPtr->getAttitudeValidity(false) == true) {
            return m_serialParserPtr->getAltitude(nullptr, false);
        } else return 0;
    }

    return -DBL_MAX;
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
        // Check if at least a 2D fix is present and if data is not outdated, else return 0 or unavailable
        if ((m_serialParserPtr->getFixValidity2D() == true || m_serialParserPtr->getFixValidity3D() == true) && m_serialParserPtr->getPositionValidity(false) == true) {
            return m_serialParserPtr->getPosition(nullptr, false);
        } else return std::make_pair(0,0);
    }

	return std::pair<double,double>(-DBL_MAX,-DBL_MAX);
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
        if (m_serialParserPtr->getFixValidity3D() == true && m_serialParserPtr->getAccelerationsValidity(false) == true) {
            return m_serialParserPtr->getLongitudinalAcceleration(nullptr, false);
        } else return 0;
    }
    return -DBL_MAX;
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
        if (m_serialParserPtr->getFixValidity3D() == true && m_serialParserPtr->getYawRateValidity(false) == true) {
            return m_serialParserPtr->getYawRate(nullptr, false);
        } else return 0;
    }
    return -DBL_MAX;
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
        std::string fixMode = m_serialParserPtr->getFixMode();
        if (fixMode != "Fix Mode: No Fix" && fixMode != "Fix mode: Time-only Fix" &&
            fixMode != "Unknown/Invalid Fix Mode") {

            if (m_serialParserPtr->getSpeedAndCogValidity(false) == true) {
                double speed = m_serialParserPtr->getSpeed(nullptr, false);
                CAMdata.speed = VDPValueConfidence<>(speed * CENTI, SpeedConfidence_unavailable);
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
                CAMdata.altitude = VDPValueConfidence<>(altitude * CENTI, AltitudeConfidence_unavailable);
            } else CAMdata.altitude = VDPValueConfidence<>(AltitudeValue_unavailable, AltitudeConfidence_unavailable);

            CAMdata.posConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
            CAMdata.posConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
            CAMdata.posConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;

            if (m_serialParserPtr->getAccelerationsValidity(false) == true) {
                double long_acc = m_serialParserPtr->getLongitudinalAcceleration(nullptr, false);
                CAMdata.longAcceleration = VDPValueConfidence<>(long_acc * CENTI, AccelerationConfidence_unavailable);
            } else CAMdata.longAcceleration = VDPValueConfidence<>(AccelerationValue_unavailable,AccelerationConfidence_unavailable);

            if (m_serialParserPtr->getSpeedAndCogValidity(false) == true) {
                double heading = m_serialParserPtr->getCourseOverGroundNmea(nullptr, false);
                if (static_cast<int>(heading * DECI) < 0 || static_cast<int>(heading * DECI) > 3601) {
                    CAMdata.heading = VDPValueConfidence<>(HeadingValue_unavailable, HeadingConfidence_unavailable);
                } else CAMdata.heading = VDPValueConfidence<>(heading * DECI, HeadingConfidence_unavailable);
            } else CAMdata.heading = VDPValueConfidence<>(HeadingValue_unavailable, HeadingConfidence_unavailable);

            CAMdata.driveDirection = DriveDirection_unavailable;
            CAMdata.curvature = VDPValueConfidence<>(CurvatureValue_unavailable, CurvatureConfidence_unavailable);
            CAMdata.curvature_calculation_mode = CurvatureCalculationMode_unavailable;
            CAMdata.VehicleLength = m_vehicle_length;
            CAMdata.VehicleWidth = m_vehicle_width;

            if (m_serialParserPtr->getYawRateValidity(false) == true) {
                double yaw_rate = m_serialParserPtr->getYawRate(nullptr, false);
                CAMdata.yawRate = VDPValueConfidence<>(yaw_rate * CENTI, YawRateConfidence_unavailable);
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
        std::string fixMode = m_serialParserPtr->getFixMode();
        if (fixMode != "No Fix Detected!" && fixMode != "Fix mode: Time-only Fix" && fixMode != "Unknown fix mode") {
            std::pair<double, double> position = m_serialParserPtr->getPosition(nullptr, false);
            double speed = m_serialParserPtr->getSpeed(nullptr, false);
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