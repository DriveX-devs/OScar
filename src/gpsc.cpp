#include "gpsc.h"
#include <stdexcept>

VDPGPSClient::VDPGPSClient() {
	int rval;

	// Set some initial "unavailable" values for the Vehicle Width and Length
	m_vehicle_length=VDPValueConfidence<long,long>(VehicleLengthValue_unavailable,VehicleLengthConfidenceIndication_unavailable);
	m_vehicle_width=VehicleWidth_unavailable;

	if((rval=gps_open(m_server.c_str(),std::to_string(m_port).c_str(),&m_gps_data))!=0) {
		throw std::runtime_error("GNSS device open failed: " + std::string(gps_errstr(rval)));
	}

	(void)gps_stream(&m_gps_data, WATCH_ENABLE | WATCH_JSON, NULL);
};

VDPGPSClient::~VDPGPSClient() {
	int rval;

	(void)gps_stream(&m_gps_data, WATCH_DISABLE, NULL);
	(void)gps_close(&m_gps_data);
}

VDPGPSClient::CAM_mandatory_data_t
VDPGPSClient::getCAMMandatoryData() {
	CAM_mandatory_data_t CAMdata={.avail=false};
	int rval;

	rval=gps_read(&m_gps_data,nullptr,0);

	if(rval!=0) {
		throw std::runtime_error("Cannot read data from GNSS device: " + std::string(gps_errstr(rval)));
	} else {
		// Check if the mode is set and if a fix has been obtained
		if((m_gps_data.set & MODE_SET)==MODE_SET && m_gps_data.status!=STATUS_NO_FIX) {
			if(m_gps_data.fix.mode == MODE_2D || m_gps_data.fix.mode == MODE_3D) {
				/* Speed [0.01 m/s] */
				CAMdata.speed = VDPValueConfidence<>(m_gps_data.fix.speed*CENTI,SpeedConfidence_unavailable);

				/* Latitude WGS84 [0,1 microdegree] */
				CAMdata.latitude = (Latitude_t)(m_gps_data.fix.latitude*DOT_ONE_MICRO);
				/* Longitude WGS84 [0,1 microdegree] */
				CAMdata.longitude = (Longitude_t)(m_gps_data.fix.longitude*DOT_ONE_MICRO);

				/* Altitude [0,01 m] */
				if(m_gps_data.fix.mode == MODE_3D) {
					CAMdata.altitude = VDPValueConfidence<>(m_gps_data.fix.altitude*CENTI,AltitudeConfidence_unavailable);
				} else {
					CAMdata.altitude = VDPValueConfidence<>(AltitudeValue_unavailable,AltitudeConfidence_unavailable);
				}

				/* Position Confidence Ellipse */
				CAMdata.posConfidenceEllipse.semiMajorConfidence=SemiAxisLength_unavailable;
				CAMdata.posConfidenceEllipse.semiMinorConfidence=SemiAxisLength_unavailable;
				CAMdata.posConfidenceEllipse.semiMajorOrientation=HeadingValue_unavailable;

				/* Longitudinal acceleration [0.1 m/s^2] */
			    CAMdata.longAcceleration = VDPValueConfidence<>(LongitudinalAccelerationValue_unavailable,AccelerationConfidence_unavailable);

			    /* Heading WGS84 north [0.1 degree] */
			    CAMdata.heading = VDPValueConfidence<>(HeadingValue_unavailable,HeadingConfidence_unavailable);

				/* Drive direction (backward driving is not fully supported by SUMO, at the moment */
    			CAMdata.driveDirection = DriveDirection_unavailable;

			    /* Curvature and CurvatureCalculationMode */
			    CAMdata.curvature = VDPValueConfidence<>(CurvatureValue_unavailable,CurvatureConfidence_unavailable);
			    CAMdata.curvature_calculation_mode = CurvatureCalculationMode_unavailable;

			    /* Length and Width [0.1 m] */
				CAMdata.VehicleLength = m_vehicle_length;
				CAMdata.VehicleWidth = m_vehicle_width;

			    /* Yaw Rate */
			    CAMdata.yawRate = VDPValueConfidence<>(YawRateValue_unavailable,YawRateConfidence_unavailable);

			    /* This flag represents an easy way to understand if it was possible to read any valid data from the GNSS device */
				CAMdata.avail = true;
			}
		} else {
			// Set everything to unavailable as no fix was possible (i.e., the resulting CAM will not be so useful...)

			/* Speed [0.01 m/s] */
			CAMdata.speed = VDPValueConfidence<>(SpeedValue_unavailable,SpeedConfidence_unavailable);

			/* Latitude WGS84 [0,1 microdegree] */
			CAMdata.latitude = (Latitude_t)Latitude_unavailable;
			/* Longitude WGS84 [0,1 microdegree] */
			CAMdata.longitude = (Longitude_t)Longitude_unavailable;

			/* Altitude [0,01 m] */
			CAMdata.altitude = VDPValueConfidence<>(AltitudeValue_unavailable,AltitudeConfidence_unavailable);

			/* Position Confidence Ellipse */
			CAMdata.posConfidenceEllipse.semiMajorConfidence=SemiAxisLength_unavailable;
			CAMdata.posConfidenceEllipse.semiMinorConfidence=SemiAxisLength_unavailable;
			CAMdata.posConfidenceEllipse.semiMajorOrientation=HeadingValue_unavailable;

			/* Longitudinal acceleration [0.1 m/s^2] */
		    CAMdata.longAcceleration = VDPValueConfidence<>(LongitudinalAccelerationValue_unavailable,AccelerationConfidence_unavailable);

		    /* Heading WGS84 north [0.1 degree] */
		    CAMdata.heading = VDPValueConfidence<>(HeadingValue_unavailable,HeadingConfidence_unavailable);

			/* Drive direction (backward driving is not fully supported by SUMO, at the moment */
			CAMdata.driveDirection = DriveDirection_unavailable;

		    /* Curvature and CurvatureCalculationMode */
		    CAMdata.curvature = VDPValueConfidence<>(CurvatureValue_unavailable,CurvatureConfidence_unavailable);
		    CAMdata.curvature_calculation_mode = CurvatureCalculationMode_unavailable;

		    /* Length and Width [0.1 m] */
		    // These can be set even in absence of a GPS fix
			CAMdata.VehicleLength = m_vehicle_length;
			CAMdata.VehicleWidth = m_vehicle_width;

		    /* Yaw Rate */
		    CAMdata.yawRate = VDPValueConfidence<>(YawRateValue_unavailable,YawRateConfidence_unavailable);

			CAMdata.avail = false;
		}
	}

    return CAMdata;
}