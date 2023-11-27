#include "VRUdp.h"
#include "HeadingConfidence.h"
#include <stdexcept>
#include <iostream>
#include <math.h> 
#include <cfloat>

extern "C" {
  #include "VAM.h"
  #include "utmuts.h"
}

#define GPSSTATUS(gpsdata) gpsdata.fix.status

void VRUdp::openConnection() {
  int rval;

  if((rval=gps_open(m_server.c_str(),std::to_string(m_port).c_str(),&m_gps_data))!=0) {
  	throw std::runtime_error("GNSS device open failed: " + std::string(gps_errstr(rval)));
  }

  (void)gps_stream(&m_gps_data, WATCH_ENABLE | WATCH_JSON, NULL);
  (void)gps_waiting(&m_gps_data, 5000000);
}

void VRUdp::closeConnection() {
  (void)gps_stream(&m_gps_data, WATCH_DISABLE, NULL);
  (void)gps_close(&m_gps_data);
}

double VRUdp::getPedHeadingValue() {
  int rval;
  rval=gps_read(&m_gps_data,nullptr,0);

  if(rval==-1) {
  	throw std::runtime_error("Cannot read the heading from GNSS device: " + std::string(gps_errstr(rval)));
  } else {
    // Check if the mode is set and if a fix has been obtained
    if((m_gps_data.set & MODE_SET)==MODE_SET) { //&& GPSSTATUS(m_gps_data)!=STATUS_NO_FIX) {
			if(m_gps_data.fix.mode == MODE_2D || m_gps_data.fix.mode == MODE_3D) {
				if(static_cast<int>(m_gps_data.fix.track*DECI)<0 || static_cast<int>(m_gps_data.fix.track*DECI)>3601) {
					return -DBL_MAX;
				} else {
					return m_gps_data.fix.track;
				}
			}
    }
  }

  return -DBL_MAX;
}

double VRUdp::getPedSpeedValue() {
  int rval;
  rval=gps_read(&m_gps_data,nullptr,0);

  if(rval==-1) {
		throw std::runtime_error("Cannot read the speed from GNSS device: " + std::string(gps_errstr(rval)));
  } else {
	// Check if the mode is set and if a fix has been obtained
		if((m_gps_data.set & MODE_SET)==MODE_SET) { // && GPSSTATUS(m_gps_data)!=STATUS_NO_FIX) {
			if(m_gps_data.fix.mode == MODE_2D || m_gps_data.fix.mode == MODE_3D) {
				return m_gps_data.fix.speed;
			}
		}
  }

  return -DBL_MAX;
}

VRUdp_position_latlon_t VRUdp::getPedPosition() {
  int rval;
  VRUdp_position_latlon_t ped_pos{-DBL_MAX,-DBL_MAX,-DBL_MAX};
  
  rval=gps_read(&m_gps_data,nullptr,0);

  if(rval==-1) {
		throw std::runtime_error("Cannot read the position from GNSS device: " + std::string(gps_errstr(rval)));
  } else {
	// Check if the mode is set and if a fix has been obtained
		if((m_gps_data.set & MODE_SET)==MODE_SET) { // && GPSSTATUS(m_gps_data)!=STATUS_NO_FIX) {
			if(m_gps_data.fix.mode == MODE_2D || m_gps_data.fix.mode == MODE_3D) {
				if(!isnan(m_gps_data.fix.latitude) && !isnan(m_gps_data.fix.longitude)) {
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
}

VRUdp_position_XYZ_t
VRUdp::convertLatLontoXYZ(VRUdp_position_latlon_t pos_latlon){
  VRUdp_position_XYZ_t pos_xyz = {-DBL_MAX,-DBL_MAX,-DBL_MAX};
  
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

VAM_mandatory_data_t
VRUdp::getVAMMandatoryData(){
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
				VAMdata.speed = VRUdpValueConfidence<>(m_gps_data.fix.speed*CENTI,SpeedConfidence_unavailable);

				/* Latitude WGS84 [0,1 microdegree] */
				VAMdata.latitude = (Latitude_t)(m_gps_data.fix.latitude*DOT_ONE_MICRO);
				/* Longitude WGS84 [0,1 microdegree] */
				VAMdata.longitude = (Longitude_t)(m_gps_data.fix.longitude*DOT_ONE_MICRO);

				int asnAltitudeValue=static_cast<int>(m_gps_data.fix.altitude*CENTI);
				/* Altitude [0,01 m] */
				if(m_gps_data.fix.mode == MODE_3D && asnAltitudeValue>=-100000 &&  asnAltitudeValue<=800000) {
					VAMdata.altitude = VRUdpValueConfidence<>(static_cast<int>(m_gps_data.fix.altitude*CENTI),AltitudeConfidence_unavailable);
				} else {
					VAMdata.altitude = VRUdpValueConfidence<>(AltitudeValue_unavailable,AltitudeConfidence_unavailable);
				}
			
				/* Position Confidence Ellipse */
   			VAMdata.posConfidenceEllipse.semiMajorConfidence=SemiAxisLength_unavailable;
   			VAMdata.posConfidenceEllipse.semiMinorConfidence=SemiAxisLength_unavailable;
   			VAMdata.posConfidenceEllipse.semiMajorOrientation=HeadingValue_unavailable;
   			
   			/* Heading WGS84 north [0.1 degree] - m_gps_data.fix.track should already provide a CW heading relative to North */
				if(static_cast<int>(m_gps_data.fix.track*DECI)<0 || static_cast<int>(m_gps_data.fix.track*DECI)>3601) {
					VAMdata.heading = VRUdpValueConfidence<>(HeadingValue_unavailable,HeadingConfidence_unavailable);
				} else {
					VAMdata.heading = VRUdpValueConfidence<>(static_cast<int>(m_gps_data.fix.track*DECI),HeadingConfidence_unavailable);
				}
			
				/* Longitudinal acceleration [0.1 m/s^2] */
  		  VAMdata.longAcceleration = VRUdpValueConfidence<>(LongitudinalAccelerationValue_unavailable,AccelerationConfidence_unavailable);
    
  			/* This flag represents an easy way to understand if it was possible to read any valid data from the GNSS device */
  			VAMdata.avail = true;
			}
		} else{
		// Set everything to unavailable as no fix was possible (i.e., the resulting CAM will not be so useful...)

			/* Speed [0.01 m/s] */
			VAMdata.speed = VRUdpValueConfidence<>(SpeedValue_unavailable,SpeedConfidence_unavailable);

			/* Latitude WGS84 [0,1 microdegree] */
			VAMdata.latitude = (Latitude_t)Latitude_unavailable;
			/* Longitude WGS84 [0,1 microdegree] */
			VAMdata.longitude = (Longitude_t)Longitude_unavailable;

			/* Altitude [0,01 m] */
			VAMdata.altitude = VRUdpValueConfidence<>(AltitudeValue_unavailable,AltitudeConfidence_unavailable);

			/* Position Confidence Ellipse */
			VAMdata.posConfidenceEllipse.semiMajorConfidence=SemiAxisLength_unavailable;
			VAMdata.posConfidenceEllipse.semiMinorConfidence=SemiAxisLength_unavailable;
			VAMdata.posConfidenceEllipse.semiMajorOrientation=HeadingValue_unavailable;

			/* Longitudinal acceleration [0.1 m/s^2] */
			VAMdata.longAcceleration = VRUdpValueConfidence<>(LongitudinalAccelerationValue_unavailable,AccelerationConfidence_unavailable);

			/* Heading WGS84 north [0.1 degree] */
			VAMdata.heading = VRUdpValueConfidence<>(HeadingValue_unavailable,HeadingConfidence_unavailable);
		
			VAMdata.avail = false;
		}
  }

  return VAMdata;
}

std::vector<distance_t>
VRUdp::get_min_distance(ldmmap::LDMMap* LDM){
  std::vector<distance_t> min_distance(2,{MAXFLOAT,MAXFLOAT,MAXFLOAT,(StationID_t)0,(StationType_t)-1,false});
  min_distance[0].station_type = StationType_pedestrian;
  min_distance[1].station_type = StationType_passengerCar;

  std::vector<ldmmap::LDMMap::returnedVehicleData_t> selectedStations;
  
  // Extract all stations from the LDM
  VRUdp_position_latlon_t ped_pos = getPedPosition ();
  LDM->rangeSelect (MAXFLOAT,ped_pos.lat,ped_pos.lon,selectedStations);

  // Get position and heading of the current pedestrian
  VRUdp_position_XYZ_t pos_ped = convertLatLontoXYZ(ped_pos);
  double ped_heading = getPedHeadingValue();

  // Iterate over all stations present in the LDM
  for(std::vector<ldmmap::LDMMap::returnedVehicleData_t>::iterator it = selectedStations.begin (); it!=selectedStations.end (); ++it){
    distance_t curr_distance = {MAXFLOAT,MAXFLOAT,MAXFLOAT,(StationID_t)0,(StationType_t)-1,false};
    curr_distance.ID = it->vehData.stationID;
    curr_distance.station_type = it->vehData.stationType;
    VRUdp_position_latlon_t pos_node;
    VRUdp_position_XYZ_t pos_node_xyz;

    pos_node.lon = it->vehData.lon;
    pos_node.lat = it->vehData.lat;
    if(it->vehData.elevation != AltitudeValue_unavailable){
      pos_node.alt = it->vehData.elevation;
      pos_node_xyz = convertLatLontoXYZ(pos_node);
    } else{
      pos_node_xyz = convertLatLontoXYZ(pos_node);
    }

    // Computation of the distances
    curr_distance.lateral = abs((pos_node_xyz.x - pos_ped.x)*cos(ped_heading));
    curr_distance.longitudinal = abs((pos_node_xyz.y - pos_ped.y)*cos(ped_heading));
    curr_distance.vertical = abs(pos_node_xyz.z - pos_ped.z);

    if(curr_distance.station_type == StationType_pedestrian){
      if(curr_distance.lateral<min_distance[0].lateral && curr_distance.longitudinal<min_distance[0].longitudinal && curr_distance.vertical < min_distance[0].vertical){
        min_distance[0].lateral = curr_distance.lateral;
        min_distance[0].longitudinal = curr_distance.longitudinal;
        min_distance[0].vertical = curr_distance.vertical;

        min_distance[0].ID = curr_distance.ID;
      }
    } else{
      if(curr_distance.lateral<min_distance[1].lateral && curr_distance.longitudinal<min_distance[1].longitudinal && curr_distance.vertical < min_distance[1].vertical){
        min_distance[1].lateral = curr_distance.lateral;
        min_distance[1].longitudinal = curr_distance.longitudinal;
        min_distance[1].vertical = curr_distance.vertical;

        min_distance[1].ID = curr_distance.ID;
      }
    }
  }

  return min_distance;
}