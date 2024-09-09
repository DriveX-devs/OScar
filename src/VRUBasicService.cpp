//
//  VRUBasicService.cc
//

#include "VRUBasicService.h"
#include "gpsc.h"
#include "LDMmap.h"
#include "asn_utils.h"
#include "utils.h"
#include <future>
#include <chrono>
#include <ctime>
#include <iostream>
#include <cfloat>
#include <cmath>
#include <sys/sysinfo.h>
#include <netinet/ether.h>
#include <algorithm>
#include <arpa/inet.h>

using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

// This macro allows the user to schedule the asynchronous execution of a function (fcn) after "msec" milliseconds
#define SCHEDULE(msecs,fcn) \
  (void) std::async(std::launch::async, [&] { \
    std::this_thread::sleep_for(std::chrono::milliseconds(msecs)); \
    fcn(); \
  });

static const double PI_CONST = 3.1415926535897932384626433832795028841971693993751058209;
#define VRUDP_HEADING_UNAVAILABLE -DBL_MAX

// VRU Basic Service constructor
VRUBasicService::VRUBasicService(){
  m_station_id = ULONG_MAX;
  m_stationtype = LONG_MAX;
  m_LDM = nullptr;
  m_btp = nullptr;
  m_VRUdp = nullptr;

  m_T_GenVam_ms = T_GenVamMax_ms;
  m_T_CheckVamGen_ms = T_GenVamMin_ms;

  m_N_GenVam_red = 0;
  m_N_GenVam_max_red = 2;

  m_prev_heading = -1;
  m_prev_speed = -1;
  m_prev_pos.lat = -DBL_MAX;
  m_prev_pos.lon = -DBL_MAX;

  m_pos_th = 4.0;
  m_speed_th = 0.5;
  m_head_th = 4.0;

  m_long_safe_d = -1;
  m_lat_safe_d = 2;
  m_vert_safe_d = 5;

  lastVamGen = -1;

  m_vam_sent = 0;

  m_pos_sent = 0;
  m_speed_sent = 0;
  m_head_sent = 0;
  m_safedist_sent = 0;
  m_time_sent = 0;

  m_VRU_role = VRU_ROLE_ON;
  m_VRU_clust_state = VRU_IDLE;

  m_trigg_cond = NOT_VALID;
  
  m_terminateFlag = false;
  
  // The log file and .csv file are disabled by default
  m_log_filename = "dis";
}

std::vector<distance_t>
VRUBasicService::get_min_distance(ldmmap::LDMMap* LDM) {
    std::vector<distance_t> min_distance(2,{MAXFLOAT,MAXFLOAT,MAXFLOAT,(StationId_t)0,(StationType_t)-1,false});
    min_distance[0].station_type = StationType_pedestrian;
    min_distance[1].station_type = StationType_passengerCar;

    std::vector<ldmmap::LDMMap::returnedVehicleData_t> selectedStations;

    // Extract all stations from the LDM
    VDPGPSClient vrudp;
    VDPGPSClient::VRU_position_latlon_t ped_pos = vrudp.getPedPosition();
    LDM->rangeSelect(MAXFLOAT,ped_pos.lat,ped_pos.lon,selectedStations);

    // Reference longitude (center meridian) for the Transverse Mercator projection (used to convert (Lat,Lon) to (x,y))
    double lon0 = ped_pos.lon;

    // Get position and heading of the current pedestrian
    VDPGPSClient::VRU_position_XYZ_t pos_ped = vrudp.convertLatLontoXYZ_TM(ped_pos,lon0);
    double ped_heading = vrudp.getPedHeadingValue();

    // Iterate over all stations present in the LDM
    for(std::vector<ldmmap::LDMMap::returnedVehicleData_t>::iterator it = selectedStations.begin (); it!=selectedStations.end (); ++it){
        distance_t curr_distance = {MAXFLOAT,MAXFLOAT,MAXFLOAT,(StationId_t)0,(StationType_t)-1,false};
        curr_distance.ID = it->vehData.stationID;
        curr_distance.station_type = it->vehData.stationType;
        VDPGPSClient::VRU_position_latlon_t pos_node;
        VDPGPSClient::VRU_position_XYZ_t pos_node_xyz;

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
            pos_node_xyz = vrudp.convertLatLontoXYZ_TM(pos_node,lon0);
        } else {
            pos_node_xyz = vrudp.convertLatLontoXYZ_TM(pos_node,lon0);
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

        if(vrudp.getDebugPrintsState() == true) {
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

void VRUBasicService::setStationProperties(unsigned long fixed_stationid, long fixed_stationtype){
  m_station_id = fixed_stationid;
  m_stationtype = fixed_stationtype;
}

void VRUBasicService::setStationID(unsigned long fixed_stationid){
  m_station_id = fixed_stationid;
}

void VRUBasicService::setStationType(long fixed_stationtype){
  m_stationtype = fixed_stationtype;
}

void VRUBasicService::startVamDissemination(){
  // Set the termination condition to false
  m_terminateFlag=false;
  
  // Error check
  if(m_btp==nullptr) {
    fprintf(stderr,"Error: no BTP object has been set. The VAM dissemination will not start.\n");
    return;
  }
  
  //if(m_VRU_clust_state==VRU_IDLE && m_VRU_role==VRU_ROLE_ON){
    SCHEDULE(0,initDissemination);
    m_VRU_clust_state = VRU_ACTIVE_STANDALONE;
  //}

  while(m_terminateFlag==false); // Disseminate VAMs
}

void VRUBasicService::startVamDissemination(int desync_ms){
  // Set the termination condition to false
  m_terminateFlag=false;
  
  // Error check
  if(m_btp==nullptr) {
    fprintf(stderr,"Error: no BTP object has been set. The VAM dissemination will not start.\n");
    return;
  }
  
  if(m_VRU_clust_state==VRU_IDLE && m_VRU_role==VRU_ROLE_ON){
    SCHEDULE(desync_ms,initDissemination);
    m_VRU_clust_state = VRU_ACTIVE_STANDALONE;
  }

  while(m_terminateFlag==false); // Disseminate VAMs
}

std::string VRUBasicService::printMinDist(double minDist) {
    return ((minDist>-DBL_MAX && minDist<MAXFLOAT) ? std::to_string(minDist) : "unavailable");
}

void VRUBasicService::initDissemination(){
  m_trigg_cond = DISSEMINATION_START;
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wunused-variable"
  VRUBasicService_error_t vam_error = generateAndEncodeVam();
  #pragma GCC diagnostic pop

  //if((m_VRU_clust_state==VRU_ACTIVE_STANDALONE || m_VRU_clust_state==VRU_ACTIVE_CLUSTER_LEADER) && m_VRU_role==VRU_ROLE_ON)
    SCHEDULE(m_T_CheckVamGen_ms, checkVamConditions);
}

void VRUBasicService::checkVamConditions(){
  int64_t now;
  VRUBasicService_error_t vam_error;
  bool condition_verified;
  bool redundancy_mitigation;
  bool vamredmit_verified;
  FILE* f_out=nullptr;
  bool first=true;
  double currHead;
  double currSpeed;
  VDPGPSClient::VRU_position_latlon_t currPos;
  long int time_difference;
  double head_diff=-1;
  double pos_diff=-1;
  double speed_diff=-1;
  
  // Create a new timer to periodically check the CAM conditions, according to the standard
  struct pollfd pollfddata;
  int clockFd;
  
  // The last argument of timer_fd_create should be in microseconds
  if(timer_fd_create(pollfddata, clockFd, m_T_CheckVamGen_ms*1e3)<0) {
    std::cerr << "[ERROR] Fatal error! Cannot create timer for the VAM dissemination" << std::endl;
    terminateDissemination();
    return;
  }

  POLL_DEFINE_JUNK_VARIABLE();
  
  // If the print on log file is enabled, create the log file
  if(m_log_filename!="dis" && m_log_filename!="") {
    char filename[strlen(m_log_filename.c_str())+1];
    snprintf(filename,sizeof(filename),"%s",m_log_filename.c_str());

    f_out=fopen(filename,"w");
  }
  
  // The dissemination goes on until it is interrupted
  while(m_terminateFlag==false){
  	if(poll(&pollfddata,1,0)>0){
  		POLL_CLEAR_EVENT(clockFd);
  		
  		// Initializing
      condition_verified=false;
      vamredmit_verified=false;
      redundancy_mitigation=false;
      std::string data_head="";
      std::string data_pos="";
      std::string data_speed="";
      std::string data_safed="";
      std::string data_vamredmit="";
      std::string data_time="";
      		
      // If no initial VAM has been triggered before checkVamConditions() has been called, throw an error
      if(m_prev_heading==-1 || m_prev_speed==-1 || m_prev_pos.lat==-DBL_MAX || m_prev_pos.lon==-DBL_MAX)
      {
        std::cerr << "Error. checkVamConditions() was called before sending any VAM and this is not allowed." << std::endl;
        terminateDissemination();
        close(clockFd);
        break;
        //throw std::runtime_error("Error. checkCamConditions() was called before sending any CAM and this is not allowed.");
      }
      		
      // Check if redundancy mitigation has to be applied
  		if(m_N_GenVam_red == 0){
    			redundancy_mitigation = checkVamRedundancyMitigation ();
    	}
    		
    	/*
   		* ETSI TS 103 300-3 V2.2.1 chap. 8 table 17 (no DCC)
   		* One of the following ITS-S dynamics related conditions is given:
  		*/

  		/* 1a)
   		* The absolute difference between the current heading of the originating
   		* ITS-S and the heading included in the VAM previously transmitted by the
   		* originating ITS-S exceeds 4°;
  		*/
      currHead=(double)m_VRUdp->getPedHeadingValue();
      
    	if(currHead != (double)HeadingValue_unavailable && abs(currHead) < abs(-DBL_MAX)) { // Check if the heading has an out of range value
    		head_diff = currHead - m_prev_heading;
  			head_diff += (head_diff>180.0) ? -360.0 : (head_diff<-180.0) ? 360.0 : 0.0;

  			// Create the data for the log print
            data_head="[HEADING] HeadingUnavailable="+std::to_string((float)HeadingValue_unavailable/10)+" PrevHead="+std::to_string(m_prev_heading)+" CurrHead="+std::to_string(currHead)+" HeadDiff="+std::to_string(head_diff)+"\n";
        		
  			if (head_diff > m_head_th || head_diff < -m_head_th)
    		{
      		if(!redundancy_mitigation && (m_N_GenVam_red==0 || m_N_GenVam_red==m_N_GenVam_max_red)){
          	m_N_GenVam_red = 0;

          	m_trigg_cond = HEADING_CHANGE;
          	vam_error = generateAndEncodeVam ();
         		if(vam_error==VAM_NO_ERROR)
            {
              condition_verified = true;
              m_head_sent++;
            } else {
              std::cerr << "Cannot generate VAM. Error code: " << std::to_string(vam_error) << std::endl;
            }
        	} else{
          	m_N_GenVam_red++;
          	vamredmit_verified = true;
        	}
    		}	
    	} else {
    		m_prev_heading = (double)HeadingValue_unavailable;
        currHead=(double)HeadingValue_unavailable;
        head_diff = currHead - m_prev_heading;

        // Create the data for the log print
        data_head="[HEADING] HeadingUnavailable="+std::to_string((float)HeadingValue_unavailable)+" PrevHead="+std::to_string(m_prev_heading)+" CurrHead="+std::to_string(currHead)+" HeadDiff="+std::to_string(head_diff)+"\n";
      }
      		
      /* 1b)
   		* the distance between the current position of the originating ITS-S and
   		* the position included in the VAM previously transmitted by the originating
   		* ITS-S exceeds 4 m;
  		*/
      currPos=m_VRUdp->getPedPosition();

  		pos_diff = haversineDist(currPos.lat, currPos.lon, m_prev_pos.lat, m_prev_pos.lon); // Compute the position difference with the previous VAM sent
  		// Create the data for the log print
      data_pos="[DISTANCE] PrevLat="+std::to_string(m_prev_pos.lat)+" PrevLon="+std::to_string(m_prev_pos.lon)+" CurrLat="+std::to_string(currPos.lat)+" CurrLon="+std::to_string(currPos.lon)+" PosDiff="+std::to_string(pos_diff)+"\n";
      		
      if (!condition_verified && !vamredmit_verified && (pos_diff > m_pos_th || pos_diff < -m_pos_th)){
        if(!redundancy_mitigation && (m_N_GenVam_red==0 || m_N_GenVam_red==m_N_GenVam_max_red)){
          m_N_GenVam_red = 0;

          m_trigg_cond = POSITION_CHANGE;
          vam_error = generateAndEncodeVam ();
          if(vam_error==VAM_NO_ERROR)
          {
            condition_verified = true;
            m_pos_sent++;
          } else {
            std::cerr << "Cannot generate VAM. Error code: " << std::to_string(vam_error) << std::endl;
          }
        } else{
          m_N_GenVam_red++;
          vamredmit_verified = true;
        }
      }
    		
    	/* 1c)
   		* The absolute difference between the current speed of the originating ITS-S
   		* and the speed included in the VAM previously transmitted by the originating
   		* ITS-S exceeds 0,5 m/s.
  		*/
      currSpeed=m_VRUdp->getPedSpeedValue();

  		if(currSpeed != (double)SpeedValue_unavailable && abs(currSpeed) < 1000000) { // Check if the speed has an out of range value
  			speed_diff = currSpeed - m_prev_speed;

            data_speed="[SPEED] SpeedUnavailable="+std::to_string((float)SpeedValue_unavailable)+" PrevSpeed="+std::to_string(m_prev_speed)+" CurrSpeed="+std::to_string(currSpeed)+" SpeedDiff="+std::to_string(speed_diff)+"\n";
        
  			if (!condition_verified && !vamredmit_verified && (speed_diff > m_speed_th || speed_diff < -m_speed_th))
    		{
      		if(!redundancy_mitigation && (m_N_GenVam_red==0 || m_N_GenVam_red==m_N_GenVam_max_red)){
          	m_N_GenVam_red = 0;

          	m_trigg_cond = SPEED_CHANGE;
         		vam_error = generateAndEncodeVam ();
          	if(vam_error==VAM_NO_ERROR)
            {
              condition_verified = true;
              m_speed_sent++;
            } else {
             	std::cerr << "Cannot generate VAM. Error code: " << std::to_string(vam_error) << std::endl;
            }
        	} else{
          	m_N_GenVam_red++;
          	vamredmit_verified = true;
        	}
    		}
  		} else{
  			m_prev_speed=SpeedValue_unavailable;
            currSpeed=SpeedValue_unavailable;
            speed_diff = currSpeed - m_prev_speed;

            // Create the data for the log print
            data_speed="[SPEED] SpeedUnavailable="+std::to_string((float)SpeedValue_unavailable)+" PrevSpeed="+std::to_string(m_prev_speed)+" CurrSpeed="+std::to_string(currSpeed)+" SpeedDiff="+std::to_string(speed_diff)+"\n";
  		}
  		
  		// Computation of the longitudinal safe distance
  		m_long_safe_d = abs(m_VRUdp->getPedSpeedValue ()*(T_GenVamMax_ms/1000));

  		// Get the minimum distance of other vehicles/pedestrians from the current pedestrian
  		m_min_dist = get_min_distance (m_LDM);
  		
  		/* 1d)
   		* If the longitudinal distance between the originating ITS-S and the nearest pedestrian/vehicle
   		* is smaller than v/t, where v is the current longitudinal speed of the originating ITS-S
   		* and t is the maximum time interval between the generation of two consecutive VAMs, the lateral
   		* distance smaller than 2 m and the vertical distance smaller than 5 m, a VAM must be transmitted
  		*/
  		
  		data_safed = "[SAFE DISTANCES] LongSafeDist="+std::to_string(m_long_safe_d)+" LatSafeDist="+std::to_string(m_lat_safe_d)+" VertSafeDist="+std::to_string(m_vert_safe_d)+" MinLongDistVeh="+printMinDist(m_min_dist[1].longitudinal)+" MinLatDistVeh="+printMinDist(m_min_dist[1].lateral)+" MinVertDistVeh="+printMinDist(m_min_dist[1].vertical)+" MinLongDistPed="+printMinDist(m_min_dist[0].longitudinal)+" MinLatDistPed="+printMinDist(m_min_dist[0].lateral)+" MinVertDistPed="+printMinDist(m_min_dist[0].vertical)+"\n";
  		
  		if (!condition_verified && m_min_dist[1].longitudinal < m_long_safe_d && m_min_dist[1].lateral < m_lat_safe_d && m_min_dist[1].vertical < m_vert_safe_d)
    	{
      	m_N_GenVam_red = 0;

      	m_trigg_cond = SAFE_DISTANCES;
      	m_min_dist[1].safe_dist = true;
      	vam_error = generateAndEncodeVam ();
      	if(vam_error==VAM_NO_ERROR)
        {
          condition_verified = true;
          vamredmit_verified = false;
          m_safedist_sent++;
        } else {
          std::cerr << "Cannot generate VAM. Error code: " << std::to_string(vam_error) << std::endl;
        }
    	} else{
      	if(!condition_verified && !vamredmit_verified && m_min_dist[0].longitudinal < m_long_safe_d && m_min_dist[0].lateral < m_lat_safe_d && m_min_dist[0].vertical < m_vert_safe_d)
        {
          if(!redundancy_mitigation && (m_N_GenVam_red==0 || m_N_GenVam_red==m_N_GenVam_max_red)){
            m_N_GenVam_red = 0;

            m_trigg_cond = SAFE_DISTANCES;
            m_min_dist[0].safe_dist = true;
            vam_error = generateAndEncodeVam ();
            if(vam_error==VAM_NO_ERROR)
            {
              condition_verified = true;
              m_safedist_sent++;
            } else {
              std::cerr << "Cannot generate VAM. Error code: " << std::to_string(vam_error) << std::endl;
            }
          } else{
            m_N_GenVam_red++;
            vamredmit_verified = true;
          }
        }
    	}
    		
    	/* 2)
   		* The time elapsed since the last VAM generation is equal to or greater than T_GenVam
  		*/
  		now=computeTimestampUInt64()/NANO_TO_MILLI;
      time_difference=now-lastVamGen;
      		
      // Create the data for the log print
      data_time="[TIME] Timestamp="+std::to_string(now)+" LastVAMSent="+std::to_string(lastVamGen)+" TimeThreshold="+std::to_string(m_T_GenVam_ms)+" TimeDiff="+std::to_string(time_difference)+" TimeNextVAM="+std::to_string(m_T_GenVam_ms - time_difference)+"\n";
      		
      if(!condition_verified && !vamredmit_verified && (now-lastVamGen>=m_T_GenVam_ms))
      {
      	if(!redundancy_mitigation && (m_N_GenVam_red==0 || m_N_GenVam_red==m_N_GenVam_max_red)){
          m_N_GenVam_red = 0;

          m_trigg_cond = MAX_TIME_ELAPSED;
          vam_error = generateAndEncodeVam ();
          if(vam_error==VAM_NO_ERROR)
          {
            condition_verified = true;
            m_time_sent++;
          } else {
            std::cerr << "Cannot generate VAM. Error code: " << std::to_string(vam_error) << std::endl;
          }
        } else{
          m_N_GenVam_red++;
          vamredmit_verified = true;
        }
    	}
    		
    	// Create data for the log print in case of VAM redundancy mitigation
    	data_vamredmit = "[REDUNDANCY MITIGATION] numSkipVAMsForRedMitMax="+std::to_string(m_N_GenVam_max_red)+" numSkipVAMsForRedMit="+std::to_string(m_N_GenVam_red)+" TimestampLastVAMGen="+std::to_string(lastVamGen)+" TimeIntervalSinceLastVAMGen="+std::to_string(time_difference)+"\n";
    		
    	if(m_log_filename!="dis" && m_log_filename!=""){
    		// If the log file has not been set to append mode, then set it to append mode
        if(first==true) {
          char filename[strlen(m_log_filename.c_str())+1];
          snprintf(filename,sizeof(filename),"%s",m_log_filename.c_str());

          f_out=fopen(filename,"a");
          first=false;
        }
        		
        // Initializing
        long int time=duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

        std::string data="";
        std::string sent="false";

        std::string motivation;
        std::string joint;
        int numConditions=0;
        std::string num_VAMs_sent="";

        // Check the motivation of the VAM sent
        if (!condition_verified && !vamredmit_verified) {
          motivation="none";
          num_VAMs_sent="unavailable";
        } else if(vamredmit_verified){
        	motivation="VAM Redundancy Mitigation";
          num_VAMs_sent="unavailable";
        } else {
          data="[VAM] VAM sent\n";
          sent="true";

          if(head_diff > 4.0 || head_diff < -4.0) {
            motivation="heading";
            joint=joint+"H";
            num_VAMs_sent=std::to_string(m_head_sent);
            numConditions++;
          }

          if((pos_diff > 4.0 || pos_diff < -4.0)) {
            motivation="position";
            joint=joint+"P";
            num_VAMs_sent=std::to_string(m_pos_sent);
            numConditions++;
          }

          if(speed_diff > 0.5 || speed_diff < -0.5) {
           	motivation="speed";
            joint=joint+"S";
            num_VAMs_sent=std::to_string(m_speed_sent);
            numConditions++;
          }
          
          // fprintf(stdout,"[TBR] (%.7lf<%.7lf && %.7lf<%.7lf && %.7lf<%.7lf) || (%.7lf<%.7lf && %.7lf<%.7lf && %.7lf<%.7lf)\n",
          //   m_min_dist[1].longitudinal,
          //   m_long_safe_d,
          //   m_min_dist[1].lateral,
          //   m_lat_safe_d,
          //   m_min_dist[1].vertical,
          //   m_vert_safe_d,
          //   m_min_dist[0].longitudinal,
          //   m_long_safe_d,
          //   m_min_dist[0].lateral,
          //   m_lat_safe_d,
          //   m_min_dist[0].vertical,
          //   m_vert_safe_d
          //   );

          if((m_min_dist[1].longitudinal < m_long_safe_d && m_min_dist[1].lateral < m_lat_safe_d && m_min_dist[1].vertical < m_vert_safe_d) || (m_min_dist[0].longitudinal < m_long_safe_d && m_min_dist[0].lateral < m_lat_safe_d && m_min_dist[0].vertical < m_vert_safe_d)){
          	motivation="safe_distances";
            joint=joint+"D";
            num_VAMs_sent=std::to_string(m_safedist_sent);
            numConditions++;
          }

          if(abs(time_difference - m_T_GenVam_ms) <= 10 || (m_T_GenVam_ms - time_difference) <= 0 ) {
            motivation="time";
            joint=joint+"T";
            num_VAMs_sent=std::to_string(m_time_sent);
            numConditions++;
          }

          // When joint with a single other motivation, the joint motivation should not be considered
          if(numConditions>1) {
            motivation="joint("+joint+")";
            if(joint=="HT") {
              motivation="heading";
            }
            if(joint=="PT") {
              motivation="position";
            }
            if(joint=="ST") {
              motivation="speed";
            }
            if(joint=="DT") {
              motivation="safe_distances";
            }
          }

          if(condition_verified && strlen(motivation.c_str())==0) {
            motivation="numPkt";
          }
        }

        // Create the data for the log print
        data+="[LOG] Timestamp="+std::to_string(time)+" VAMSent="+sent+" Motivation="+motivation+" NUMVAMsSent="+num_VAMs_sent+" HeadDiff="+std::to_string(head_diff)+" PosDiff="+std::to_string(pos_diff)+" SpeedDiff="+std::to_string(speed_diff)+" TimeDiff="+std::to_string(time_difference)+"\n";
        data=data+data_head+data_pos+data_speed+data_safed+data_time+data_vamredmit+"\n";

        // Print the data for the log
        fprintf(f_out,"%s", data.c_str());
    	}
  	}
  }
}

bool VRUBasicService::checkVamRedundancyMitigation(){
  bool redundancy_mitigation = false;
  int64_t now = computeTimestampUInt64 ()/NANO_TO_MILLI;
  std::vector<ldmmap::LDMMap::returnedVehicleData_t> selectedStations;

  VDPGPSClient::VRU_position_latlon_t ped_pos = m_VRUdp->getPedPosition ();
  double ped_speed = m_VRUdp->getPedSpeedValue ();
  double ped_heading = m_VRUdp->getPedHeadingValue ();
  ped_heading += (ped_heading>180.0) ? -360.0 : (ped_heading<-180.0) ? 360.0 : 0.0;

  if(now-lastVamGen < m_N_GenVam_max_red*5000){
    m_LDM->rangeSelect (4,ped_pos.lat,ped_pos.lon,selectedStations);

    for(std::vector<ldmmap::LDMMap::returnedVehicleData_t>::iterator it = selectedStations.begin (); it!=selectedStations.end () && !redundancy_mitigation; ++it){
      if((StationType_t)it->vehData.stationType == StationType_pedestrian){
        double speed_diff = it->vehData.speed_ms - ped_speed;
        double near_VRU_heading = it->vehData.heading;
        near_VRU_heading += (near_VRU_heading>180.0) ? -360.0 : (near_VRU_heading<-180.0) ? 360.0 : 0.0;
        double heading_diff = near_VRU_heading - ped_heading;

        if((speed_diff < 0.5 && speed_diff > -0.5) && (heading_diff < 4 && heading_diff > -4)){
          redundancy_mitigation = true;
          m_N_GenVam_max_red = (int16_t)std::round(((double)std::rand()/RAND_MAX)*8) + 2;
        }
      }
    }
  }

  return redundancy_mitigation;
}

VRUBasicService_error_t
VRUBasicService::generateAndEncodeVam(){
  VRUBasicService_error_t errval = VAM_NO_ERROR;
  VDPGPSClient::VAM_mandatory_data_t vam_mandatory_data;

  int64_t now;

  /* Collect data for mandatory containers */
  auto vam = asn1cpp::makeSeq(VAM);

  if(bool(vam)==false)
    {
      return VAM_ALLOC_ERROR;
    }

  /* Fill the header */
  asn1cpp::setField(vam->header.messageId, FIX_VAMID);
  asn1cpp::setField(vam->header.protocolVersion , 3);
  asn1cpp::setField(vam->header.stationId, m_station_id);

  /*
   * Compute the generationDeltaTime, computed as the time corresponding to the
   * time of the reference position in the VAM, considered as time of the VAM generation.
   * The value of the generationDeltaTime shall be wrapped to 65 536. This value shall be set as the
   * remainder of the corresponding value of TimestampIts divided by 65 536 as below:
   * generationDeltaTime = TimestampIts mod 65 536"
  */
  asn1cpp::setField(vam->vam.generationDeltaTime, compute_timestampIts () % 65536);

  /* Fill the basicContainer's station type */
  asn1cpp::setField(vam->vam.vamParameters.basicContainer.stationType, m_stationtype);

  vam_mandatory_data = m_VRUdp->getVAMMandatoryData();

  /* Fill the basicContainer */
  asn1cpp::setField(vam->vam.vamParameters.basicContainer.referencePosition.altitude.altitudeValue, vam_mandatory_data.altitude.getValue ());
  asn1cpp::setField(vam->vam.vamParameters.basicContainer.referencePosition.altitude.altitudeConfidence, vam_mandatory_data.altitude.getConfidence ());
  asn1cpp::setField(vam->vam.vamParameters.basicContainer.referencePosition.latitude, vam_mandatory_data.latitude);
  asn1cpp::setField(vam->vam.vamParameters.basicContainer.referencePosition.longitude, vam_mandatory_data.longitude);
  asn1cpp::setField(vam->vam.vamParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorAxisLength, vam_mandatory_data.posConfidenceEllipse.semiMajorConfidence);
  asn1cpp::setField(vam->vam.vamParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMinorAxisLength, vam_mandatory_data.posConfidenceEllipse.semiMinorConfidence);
  asn1cpp::setField(vam->vam.vamParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorAxisOrientation, vam_mandatory_data.posConfidenceEllipse.semiMajorOrientation);

  /* Fill the highFrequencyContainer */
  asn1cpp::setField(vam->vam.vamParameters.vruHighFrequencyContainer.heading.value, vam_mandatory_data.heading.getValue ());
  asn1cpp::setField(vam->vam.vamParameters.vruHighFrequencyContainer.heading.confidence, vam_mandatory_data.heading.getConfidence ());
  asn1cpp::setField(vam->vam.vamParameters.vruHighFrequencyContainer.speed.speedValue, vam_mandatory_data.speed.getValue ());
  asn1cpp::setField(vam->vam.vamParameters.vruHighFrequencyContainer.speed.speedConfidence, vam_mandatory_data.speed.getConfidence ());
  asn1cpp::setField(vam->vam.vamParameters.vruHighFrequencyContainer.longitudinalAcceleration.longitudinalAccelerationValue,
                    vam_mandatory_data.longAcceleration.getValue ());
  asn1cpp::setField(vam->vam.vamParameters.vruHighFrequencyContainer.longitudinalAcceleration.longitudinalAccelerationConfidence,
                    vam_mandatory_data.longAcceleration.getConfidence ());

  // Store all the "previous" values used in checkVamConditions()
  m_prev_pos = m_VRUdp->getPedPosition();
  m_prev_speed = m_VRUdp->getPedSpeedValue ();
  m_prev_heading = m_VRUdp->getPedHeadingValue ();

  /* VAM encoding */
  std::string encode_result = asn1cpp::uper::encode(vam);

  if(encode_result.size()<1)
  {
    std::cerr << "VAM encoding error." << std::endl;
    std::cerr << "Info: Lat: " << vam->vam.vamParameters.basicContainer.referencePosition.latitude
    << " Lon: " << vam->vam.vamParameters.basicContainer.referencePosition.longitude
    << " Heading: " << vam->vam.vamParameters.vruHighFrequencyContainer.heading.value 
    << " Speed: " << vam->vam.vamParameters.vruHighFrequencyContainer.speed.speedValue 
    << " Altitude: " << vam->vam.vamParameters.basicContainer.referencePosition.altitude.altitudeValue 
    << std::endl;
    return VAM_ASN1_UPER_ENC_ERROR;
  }

  BTPDataRequest_t dataRequest = {};
  dataRequest.BTPType = BTP_B; //!< BTP-B
  dataRequest.destPort = VA_PORT;
  dataRequest.destPInfo = 0;
  dataRequest.GNType = TSB;
  dataRequest.GNCommProfile = UNSPECIFIED;
  dataRequest.GNRepInt =0;
  dataRequest.GNMaxRepInt=0;
  dataRequest.GNMaxLife = 1;
  dataRequest.GNMaxHL = 1;
  dataRequest.GNTraClass = 0x02; // Store carry foward: no - Channel offload: no - Traffic Class ID: 2
  dataRequest.lenght = encode_result.size();
  // Create the packet and the BTP header
  packetBuffer pktbuf(encode_result.c_str(),static_cast<unsigned int>(encode_result.size()));
  dataRequest.data = pktbuf;
  m_btp->sendBTP(dataRequest);

  // Update the VAM statistics
  m_vam_sent++;

  // Compute the time in which the VAM has been sent
  now = computeTimestampUInt64 ()/NANO_TO_MILLI;

  // Store the time elapsed since last VAM generation
  // long time_elapsed = -1;
  // if(lastVamGen == -1)
  //   time_elapsed = now;
  // else
  //   time_elapsed = now - lastVamGen;

  // Store the time in which the last VAM (i.e. this one) has been generated and successfully sent
  lastVamGen = now;

  return errval;
}

uint64_t VRUBasicService::terminateDissemination(){
  if(m_terminateFlag==false) {
    m_terminateFlag=true;
  }

  return m_vam_sent;
}

int64_t VRUBasicService::computeTimestampUInt64(){
  int64_t int_tstamp=0;

  struct timespec tv;

  clock_gettime (CLOCK_MONOTONIC, &tv);

  int_tstamp=tv.tv_sec*1e9+tv.tv_nsec;

  return int_tstamp;
}
