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

#define MIN_DIST_EQUAL_x_EPSILON 0.0001 // [m]
static const double PI_CONST = 3.1415926535897932384626433832795028841971693993751058209;
#define VRUDP_HEADING_UNAVAILABLE -DBL_MAX

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
					return VRUDP_HEADING_UNAVAILABLE;
				} else {
					return m_gps_data.fix.track;
				}
			}
    }
  }

  return VRUDP_HEADING_UNAVAILABLE;
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

// convertLatLontoXYZ_ECEF() still does not work as expected - kept for reference but it should NOT be used unless you know very well what you are doing!
VRUdp_position_XYZ_t
VRUdp::convertLatLontoXYZ_ECEF(VRUdp_position_latlon_t pos_latlon){
  VRUdp_position_XYZ_t pos_xyz = {-DBL_MAX,-DBL_MAX,-DBL_MAX};

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

VRUdp_position_XYZ_t
VRUdp::convertLatLontoXYZ_TM(VRUdp_position_latlon_t pos_latlon, double lon0){
  VRUdp_position_XYZ_t pos_xyz = {-DBL_MAX,-DBL_MAX,-DBL_MAX};
  double x, y, gamma, k;
  transverse_mercator_t transmerc = UTMUPS_init_UTM_TransverseMercator();

  // Perform a Transverse Mercator projection considering "lon0" as center meridian
  TransverseMercator_Forward(&transmerc,lon0,pos_latlon.lat,pos_latlon.lon,&x,&y,&gamma,&k);

  if(m_debug==true) {
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


VAM_mandatory_data_t
VRUdp::getVAMMandatoryData() {
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
VRUdp::get_min_distance(ldmmap::LDMMap* LDM) {
  std::vector<distance_t> min_distance(2,{MAXFLOAT,MAXFLOAT,MAXFLOAT,(StationId_t)0,(StationType_t)-1,false});
  min_distance[0].station_type = StationType_pedestrian;
  min_distance[1].station_type = StationType_passengerCar;

  std::vector<ldmmap::LDMMap::returnedVehicleData_t> selectedStations;
  
  // Extract all stations from the LDM
  VRUdp_position_latlon_t ped_pos = getPedPosition ();
  LDM->rangeSelect(MAXFLOAT,ped_pos.lat,ped_pos.lon,selectedStations);

  // Reference longitude (center meridian) for the Transverse Mercator projection (used to convert (Lat,Lon) to (x,y))
  double lon0 = ped_pos.lon;

  // Get position and heading of the current pedestrian
  VRUdp_position_XYZ_t pos_ped = convertLatLontoXYZ_TM(ped_pos,lon0);
  double ped_heading = getPedHeadingValue();

  // Iterate over all stations present in the LDM
  for(std::vector<ldmmap::LDMMap::returnedVehicleData_t>::iterator it = selectedStations.begin (); it!=selectedStations.end (); ++it){
    distance_t curr_distance = {MAXFLOAT,MAXFLOAT,MAXFLOAT,(StationId_t)0,(StationType_t)-1,false};
    curr_distance.ID = it->vehData.stationID;
    curr_distance.station_type = it->vehData.stationType;
    VRUdp_position_latlon_t pos_node;
    VRUdp_position_XYZ_t pos_node_xyz;

    double node_heading = it->vehData.heading;

    // If no heading is available for neither the remote node, nor the ego node, skip the distance check for this node, and leave the distances
    // to the MAXFLOAT unavailable values
    // We follow this procedure as, to the best of our knowledge, the standard does not specify what to do for unavailable heading values
    if(node_heading==LDM_HEADING_UNAVAILABLE && ped_heading==VRUDP_HEADING_UNAVAILABLE) {
      continue; // Go to the next node, skipping the distance check for the current node that will not be considered for the min_distance computation
    }

    // If a node has no defined heading (but we can get a heading value for the current ego node), consider the heading of the ego node (VRU) 
    // instead of the one of the remote node (VRU or vehicle)
    // We follow this procedure as, to the best of our knowledge, the standard does not specify what to do for unavailable heading values
    if(node_heading==LDM_HEADING_UNAVAILABLE) {
    	node_heading = ped_heading;
    }

    pos_node.lon = it->vehData.lon;
    pos_node.lat = it->vehData.lat;
    if(it->vehData.elevation != AltitudeValue_unavailable) {
      pos_node.alt = it->vehData.elevation;
      pos_node_xyz = convertLatLontoXYZ_TM(pos_node,lon0);
    } else {
      pos_node_xyz = convertLatLontoXYZ_TM(pos_node,lon0);
    }

    // Computation of the distances
    // Old method: kept just for reference
    // curr_distance.lateral = abs((pos_node_xyz.x - pos_ped.x)*cos(ped_heading));
    // curr_distance.longitudinal = abs((pos_node_xyz.y - pos_ped.y)*cos(ped_heading));
    // curr_distance.vertical = abs(pos_node_xyz.z - pos_ped.z);

    // New method with angles computation
    // According to ETSI TS 103 300-2 V2.1.1 (2020-05):
    // Longitudinal Distance (LoD): estimated distance between the VRU and the vehicle along the direction of the vehicle heading
    // Lateral Distance (LaD): estimated distance between the VRU and the vehicle perpendicular to the direction of the vehicle heading
    // Vertical Distance (VD): estimated distance in vertical direction (height) between the VRU and the vehicle
    double node_distance = sqrt((pos_node_xyz.x-pos_ped.x)*(pos_node_xyz.x-pos_ped.x)+(pos_node_xyz.y-pos_ped.y)*(pos_node_xyz.y-pos_ped.y));
    if(pos_ped.y==pos_node_xyz.y && pos_ped.x==pos_node_xyz.x) {
    	curr_distance.longitudinal=0;
    	curr_distance.lateral=0;
    } else {
	    curr_distance.longitudinal = abs(node_distance*cos((PI_CONST/2.0)-DEG_2_RAD_ASN_UTILS(node_heading)-atan2(pos_ped.y-pos_node_xyz.y,pos_ped.x-pos_node_xyz.x)));
	    curr_distance.lateral = abs(node_distance*sin((PI_CONST/2.0)-DEG_2_RAD_ASN_UTILS(node_heading)-atan2(pos_ped.y-pos_node_xyz.y,pos_ped.x-pos_node_xyz.x)));
	  }
    curr_distance.vertical = abs(pos_node_xyz.z - pos_ped.z);

    if(m_debug==true) {
      fprintf(stdout,"[DEBUG - Safe Dist] long=%.7lf, lat=%.7lf, vert=%.7lf, h=%.2lf, n_d=%.2lf (%.2lf),"
        "x_V=%.4lf, y_V=%.7lf, x_P=%.4lf, y_P=%.7lf, lat_V=%.7lf, lon_V=%.7lf, angle=%.3lf deg, beta_ang=%.3lf deg, sum_ang=%.3lf\n",
        curr_distance.longitudinal,curr_distance.lateral,curr_distance.vertical,node_heading,node_distance,
        haversineDist(pos_node.lat,pos_node.lon,ped_pos.lat,ped_pos.lon),
        pos_node_xyz.x,pos_node_xyz.y,pos_ped.x,pos_ped.y,pos_node.lat,pos_node.lon,
        RAD_2_DEG_ASN_UTILS((PI_CONST/2.0)-DEG_2_RAD_ASN_UTILS(node_heading)-atan2(pos_ped.y-pos_node_xyz.y,pos_ped.x-pos_node_xyz.x)),
        RAD_2_DEG_ASN_UTILS(atan2(pos_ped.y-pos_node_xyz.y,pos_ped.x-pos_node_xyz.x)),
        node_heading+RAD_2_DEG_ASN_UTILS(atan2(pos_ped.y-pos_node_xyz.y,pos_ped.x-pos_node_xyz.x))+RAD_2_DEG_ASN_UTILS((PI_CONST/2.0)-DEG_2_RAD_ASN_UTILS(node_heading)-atan2(pos_ped.y-pos_node_xyz.y,pos_ped.x-pos_node_xyz.x)));
    }
    
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