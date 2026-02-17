#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <pthread.h>
#include <unistd.h>
#include <cstring>
#include <set>

#include "JSONserver.h"
#include "utils.h"
#include "asn_utils.h"

#define MAKE_NUM(num) json11::Json(num)
#define MAKE_STR(str) json11::Json(str)
#define MAKE_INT(num) json11::Json(static_cast<int>(num))
#define GET_NUM(json,key) (json[key].number_value())
#define GET_INT(json,key) (json[key].int_value())
#define GET_STR(json,key) (json[key].string_value())

//static inline double haversineDist(double lat_a, double lon_a, double lat_b, double lon_b) {
	// 12742000 is the mean Earth radius (6371 km) * 2 * 1000 (to convert from km to m)
//	return 12742000.0*asin(sqrt(sin(DEG_2_RAD_AIM(lat_b-lat_a)/2)*sin(DEG_2_RAD_AIM(lat_b-lat_a)/2)+cos(DEG_2_RAD_AIM(lat_a))*cos(DEG_2_RAD_AIM(lat_b))*sin(DEG_2_RAD_AIM(lon_b-lon_a)/2)*sin(DEG_2_RAD_AIM(lon_b-lon_a)/2)));
//}

void *JSONthread_callback(void *arg) {
	pthread_setname_np(pthread_self(), "JSON_API_thr");

	fd_set sockdescrs,rd_sockdescrs;
	std::set<int> sockdescrs_set = {};
	char rx_buff[8192];
	std::string rx_str;

	JSONserver *srvObj = static_cast<JSONserver *>(arg);

	int sockd=srvObj->getSock();

	// Zero out the set of descriptors to be monitored with select()
	FD_ZERO(&sockdescrs);
	// Add the main TCP socket to the FD set
	FD_SET(sockd,&sockdescrs);

	fprintf(stdout,"[INFO] The JSON server is ready to provide data to the clients. Main TCP socket descriptor: %d\n",sockd);

	while(true) {
		int connectiond;

		rd_sockdescrs=sockdescrs;

		// "+1" to consider the main TCP socket
		if(select(FD_SETSIZE,&rd_sockdescrs,nullptr,nullptr,nullptr)<0 && errno!=EINTR) {
			perror("[ERROR] JSON server died on select(). Reason");
			srvObj->setJSONThreatRunningStatus(false);
			pthread_exit(0);
		}

		// Main socket generated an event
		if(FD_ISSET(sockd,&rd_sockdescrs)) {
			fprintf(stdout,"[INFO] Received connection request.\n");

			// Accept the new connection
			if((connectiond=accept(sockd,nullptr,nullptr))<0) {  
                perror("[ERROR] JSON server cannot accept a client connection. Reason");  
            } else {
            	FD_SET(connectiond,&sockdescrs);
            	sockdescrs_set.insert(connectiond);

            	// Send a confirmation to the client
            	char confirm_msg[]="Connection: confirmed";
           		if(send(connectiond,confirm_msg,strlen(confirm_msg),0)!=static_cast<ssize_t>(strlen(confirm_msg))) {
           			perror("[ERROR] Could not send confirmation to connected client. The connection will still remain active. Reason");
           		}

           		fprintf(stdout,"[INFO] Connection from client accepted. Connected socket descriptor: %d\n",connectiond);
            }
        // Other socket generated an event
		} else {
			fprintf(stdout,"[INFO] Received message from client.\n");
			ssize_t readbytes=-1;

			for(auto currd_it=sockdescrs_set.begin(); currd_it!=sockdescrs_set.end();) {
				int currd = *currd_it;

				if(FD_ISSET(currd,&rd_sockdescrs)) {
					// Read from the connection
					readbytes = recv(currd,rx_buff,sizeof(rx_buff),0);

					// Connection is closing
					if(readbytes==0) {
						fprintf(stdout,"[INFO] Client with descriptor %d disconnected.\n",currd);
						close(currd);
						sockdescrs_set.erase(currd_it++);
						FD_CLR(currd,&sockdescrs);
					} else if(readbytes>0) {
						// Send the information the client requested
						json11::Json db_data = {};
						std::string err="";

						rx_str=std::string(rx_buff,readbytes);
						json11::Json request = json11::Json::parse(rx_str,err);

						if(err=="") {
							json11::Json::object response = srvObj->handleRequest(request);
							std::string strjson = json11::Json(response).dump();

							if(send(currd,strjson.c_str(),strjson.length(),0)!=static_cast<ssize_t>(strjson.length())) {
								perror("[ERROR] Could not send data to connected client. The connection will still remain active. Reason");
							}
						} else {
							fprintf(stdout,"[ERROR] Cannot parse a client request. Descriptor: %d. Error details: %s\n",currd,err.c_str());

							// An invalid request was provided: send error response
							json11::Json::object error_response;
							error_response["status"] = json11::Json("error");
							error_response["message"] = json11::Json("Invalid JSON request: " + err);
							std::string error_json = json11::Json(error_response).dump();

							if(send(currd,error_json.c_str(),error_json.length(),0)!=static_cast<ssize_t>(error_json.length())) {
								perror("[ERROR] Could not send error response to client. Reason");
							}
						}

						++currd_it;
					} else {
						perror("[ERROR] Cannot read a data request from a client. Reason");

						++currd_it;
					}
				}
			}
		}
	}

	perror("[INFO] JSON server terminated. Last errno value");
	srvObj->setJSONThreatRunningStatus(false);
	pthread_exit(NULL);
}

json11::Json::object JSONserver::make_AIM_json(double lat, double lon) {
	// Variable to store the relative distance of vehicles w.r.t. the reference vehicle
	double refRelDist = -1.0;

	// Create a new JSON structure
	json11::Json::object AIM_json = {};

	// Returned vehicle data vector
	std::vector<ldmmap::LDMMap::returnedVehicleData_t> returnedvehs;

	// "now" timestamp (i.e., the timestamp at which this JSON request is being generated: "generation_tstamp")
	uint64_t now_us = get_timestamp_us();

	AIM_json["generation_tstamp"] = json11::Json(now_us);
	AIM_json["reference_lat"] = json11::Json(lat);
	AIM_json["reference_lon"] = json11::Json(lon);

	if(m_db_ptr->rangeSelect(m_range_m,lat,lon,returnedvehs)!=ldmmap::LDMMap::LDMMAP_OK) {
		AIM_json["return_code"] = json11::Json("error");
		return AIM_json;
	} else {
		AIM_json["error"] = json11::Json("ok");
	}
	
	json11::Json::array vehicles = {};

	for(ldmmap::LDMMap::returnedVehicleData_t vehdata : returnedvehs) {
		// Compute the relative distance w.r.t. the reference lat and lon
		refRelDist=haversineDist(lat,lon,vehdata.vehData.lat,vehdata.vehData.lon);

		vehicles.push_back(make_vehicle_standard(vehdata.vehData.stationID,
			vehdata.vehData.lat,
			vehdata.vehData.lon,
			vehdata.vehData.camTimestamp,
			vehdata.vehData.gnTimestamp,
			vehdata.vehData.vehicleLength,
			vehdata.vehData.vehicleWidth,
			vehdata.vehData.speed_ms,
			vehdata.phData,
			refRelDist,
			vehdata.vehData.stationType,
			now_us-vehdata.vehData.timestamp_us,
			vehdata.vehData.heading));
	}

	AIM_json["vehicles"] = vehicles;

	return AIM_json;
}

json11::Json::object JSONserver::make_vehicle_standard(uint64_t stationID, 
	double lat, 
	double lon, 
	uint16_t CAM_tstamp,
	uint64_t GN_tstamp,
	ldmmap::OptionalDataItem<long> car_length_mm,
	ldmmap::OptionalDataItem<long> car_width_mm,
	double speed_ms,
	ldmmap::PHpoints *path_history,
	double relative_dist_m,
	ldmmap::e_StationTypeLDM stationType,
	uint64_t diff_ref_tstamp,
	double heading
	) {

	json11::Json::object vehicle = {};

	vehicle["stationID"] = MAKE_NUM(stationID);
	vehicle["lat"] = MAKE_NUM(lat);
	vehicle["lon"] = MAKE_NUM(lon);
	vehicle["speed_ms"] = MAKE_NUM(speed_ms);
	vehicle["CAM_tstamp"] = MAKE_NUM(CAM_tstamp);
	vehicle["GN_tstamp"] = MAKE_NUM(GN_tstamp);
	vehicle["relative_dist_to_reference_m"] = MAKE_NUM(relative_dist_m);
	vehicle["stationType"] = MAKE_NUM(stationType);
	vehicle["heading"] = MAKE_NUM(heading);

	// This value represents the difference between when the database is being read for this vehicle (i.e., now) and when the data for that vehicle was last stored
	vehicle["time_since_generation_tstamp"] = MAKE_NUM(diff_ref_tstamp);

	if(car_length_mm.isAvailable()) {
		vehicle["car_len_mm"] = MAKE_INT(car_length_mm.getData());
	} else {
		vehicle["car_len_mm"] = MAKE_STR("unavailable");
	}

	if(car_width_mm.isAvailable()) {
		vehicle["car_wid_mm"] = MAKE_INT(car_width_mm.getData());
	} else {
		vehicle["car_wid_mm"] = MAKE_STR("unavailable");
	}

	json11::Json::array PH_points_lat_json = {};
	json11::Json::array PH_points_lon_json = {};

	if(path_history!=nullptr) {
		PHDATAITER_INITIALIZER(phdataiter);

		// Iterate over all the path history points, using the "iterate" method of the PHPoints object, 
		// storing the Path History points for each vehicle in the LDMMap database
		while(path_history->iterate(phdataiter,nullptr)!=ldmmap::PHpoints::PHP_TERMINATE_ITERATION) {
			PH_points_lat_json.push_back(MAKE_STR(std::to_string(phdataiter.data.lat)));
			PH_points_lon_json.push_back(MAKE_STR(std::to_string(phdataiter.data.lon)));
		}

		vehicle["PH_points_lat"] = PH_points_lat_json;
		vehicle["PH_points_lon"] = PH_points_lon_json;
	}

	return vehicle;
}

bool JSONserver::startServer(void) {
	// Return immediately if the JSON server is already running
	if(m_thread_running == true) {
		fprintf(stderr,"[ERROR] The program has attempted to start an already running JSON server. Please report this bug to the developers.\n");
		return false;
	}

	// Create the TCP socket for data exchange
	m_sockd=socket(AF_INET,SOCK_STREAM,0);

	if(m_sockd<0) {
		perror("[ERROR] JSONserver: cannot create TCP socket. Details");
		return false;
	}

	struct sockaddr_in bindaddr;
	memset(&bindaddr,0,sizeof(bindaddr));

	bindaddr.sin_family=AF_INET;
	bindaddr.sin_port=htons(m_port);
	bindaddr.sin_addr.s_addr=INADDR_ANY;

	// Bind the socket to the port stored inside m_port
	if(bind(m_sockd,(struct sockaddr *) &bindaddr,sizeof(bindaddr))<0) {
		perror("[ERROR] JSONserver: cannot bind TCP socket. Details");
		close(m_sockd);
		m_sockd=-1;
		return false;
	}

	// Listed for incoming connections
	if(listen(m_sockd,m_backlog_size)<0) {
		perror("[ERROR] JSONserver: cannot listen on TCP socket. Details");
		close(m_sockd);
		m_sockd=-1;
		return false;
	}

	// The "accept" part is then performed in the separate thread
	if(pthread_create(&m_tid,NULL,JSONthread_callback,(void *) this)==0) {
		m_thread_running = true;
	} else {
		perror("[ERROR] JSONserver: cannot create receiving thread. Details");
		close(m_sockd);
		m_sockd=-1;
		m_tid=-1;
		return false;
	}

	return true;
}

json11::Json::object JSONserver::handleRequest(const json11::Json &request) {
    std::string request_type = GET_STR(request, "request_type");

	std::cout << "[INFO] Received JSON request: " << request_type << std::endl;

	// Empty request types are also accepted for back-compatibility
    if (request_type == "LDM" || request_type.empty()) {
        double lat = GET_NUM(request, "lat");
        double lon = GET_NUM(request, "lon");
        return make_AIM_json(lat, lon);
    } else if (request_type == "DENM_trigger" || request_type == "DENM_update" || request_type == "DENM_termination") {
        return handleDENMRequest(request);
    } else {
    	std::cout << "[ERROR] Received unknown JSON request: " << request_type << std::endl;

        json11::Json::object response;
    	response["status"] = MAKE_STR("error");
    	response["message"] = MAKE_STR("Unknown request_type: " + request_type);
        return response;
    }
}

// TODO: this function should be significantly extended to consider many different DENMs
denData JSONserver::fillDenDataFromJson(const json11::Json &request) {
    denData data;
	bool situation_field_set = false;

    data.setDenmMandatoryFields(
        compute_timestampIts(),
        request["lat"].is_null() ? Latitude_unavailable : GET_NUM(request, "lat")*DOT_ONE_MICRO,
        request["lon"].is_null() ? Longitude_unavailable : GET_NUM(request, "lon")*DOT_ONE_MICRO
    );

	// If not specified, the defailt validityDuration (600 s) will be used by the DEN service
	if (!request["validityDuration"].is_null()) {
		if (GET_NUM(request,"validityDuration")<=0) {
			std::cerr << "[ERROR] Received a negative or null validityDuration. No validity duration will be set." << std::endl;
		} else {
			data.setValidityDuration(GET_NUM(request,"validityDuration"));
		}
	}

	denData::denDataSituation situationData = {.informationQuality = 0, .causeCode = 0, .subCauseCode = 0};

    if (!request["causeCode"].is_null()) {
    	situationData.causeCode=GET_NUM(request,"causeCode");
    	situation_field_set = true;
    }
    if (!request["subCauseCode"].is_null()) {
    	situationData.subCauseCode=GET_NUM(request,"subCauseCode");
    	situation_field_set = true;
    }
    if (!request["informationQuality"].is_null()) {
    	situationData.informationQuality=GET_NUM(request,"informationQuality");
    	situation_field_set = true;
    }

	if (!request["repetitionDuration_ms"].is_null()) {
		std::cout << "[INFO] Sending DENM with repetition duration " << GET_NUM(request,"repetitionDuration_ms")/1000.0 << " seconds." << std::endl;
		if (GET_NUM(request,"repetitionDuration_ms")<=0) {
			std::cerr << "[ERROR] Received a negative or null repetitionDuration_ms. No repetition duration will be set." << std::endl;
		} else {
			data.setDenmRepetitionDuration(GET_NUM(request,"repetitionDuration_ms"));
		}
	}

	if (!request["repetitionInterval_ms"].is_null()) {
		std::cout << "[INFO] Sending DENM with repetition interval " << GET_NUM(request,"repetitionInterval_ms")/1000.0 << " seconds." << std::endl;
		if (GET_NUM(request,"repetitionInterval_ms")<=0) {
			std::cerr << "[ERROR] Received a negative or null repetitionInterval_ms. No repetition interval will be set." << std::endl;
		} else {
			data.setDenmRepetitionInterval(GET_NUM(request,"repetitionInterval_ms"));
		}
	}

	if(situation_field_set) {
		data.setDenmSituationData_asn_types(situationData);
	}

    return data;
}

json11::Json::object JSONserver::handleDENMRequest(const json11::Json &request) {
    json11::Json::object response;
	GeoArea_t geoArea;

    if (m_den_service == nullptr) {
        response["status"] = MAKE_STR("error");
        response["message"] = MAKE_STR("DEN service not available");
        return response;
    }

    std::string request_type = GET_STR(request, "request_type");
    denData data = fillDenDataFromJson(request);

    DENBasicService_error_t result = DENM_UNKNOWN_STATUS;

    if (request_type == "DENM_trigger" || request_type == "DENM_update" || request_type == "DENM_termination") {
    	// No GeoArea required for appDENM_termination()
    	if (request_type == "DENM_trigger" || request_type == "DENM_update") {
    		if (request["GeoArea_lat"].is_null() || request["GeoArea_lon"].is_null() || request["GeoArea_radius"].is_null()) {
    			response["status"] = MAKE_STR("error");
    			response["message"] = MAKE_STR("Missing GeoArea");
    			return response;
    		}

    		if ((request_type == "DENM_update" || request_type == "DENM_termination") && request["ActionID_seq"].is_null()) {
    			response["status"] = MAKE_STR("error");
    			response["message"] = MAKE_STR("DENM_update and DENM_termination require an ActionID sequence number; please add a field ActionID_seq");
    			return response;
    		}

    		// TODO: support also non circular areas (.shape = 0 -> GEOBROACAST_CIRCLE - see 9.7.4 in ETSI EN 302 636-4-1 V1.4.1)
    		geoArea.shape = 0;
    		geoArea.posLat = GET_NUM(request, "GeoArea_lat")*DOT_ONE_MICRO;
    		geoArea.posLong = GET_NUM(request, "GeoArea_lon")*DOT_ONE_MICRO;;
    		geoArea.angle = 0;
    		geoArea.distA = GET_NUM(request, "GeoArea_radius");
    		geoArea.distB = 0;
    	}

    	denData::DEN_ActionID_t actionID;
    	actionID.originatingStationID = m_den_service->getStationID();

    	if (request_type == "DENM_trigger") {
    		actionID.sequenceNumber = 0;
    		result = m_den_service->appDENM_trigger(data,actionID,geoArea);
    	} else if (request_type == "DENM_update") {
    		actionID.sequenceNumber = GET_NUM(request,"ActionID_seq");
    		result = m_den_service->appDENM_update(data,actionID,geoArea);
    	} else if (request_type == "DENM_termination") {
    		actionID.sequenceNumber = GET_NUM(request,"ActionID_seq");

    		if (!request["ActionID_stationID"].is_null()) {
    			actionID.originatingStationID = GET_NUM(request, "ActionID_stationID");
    		}

    		// No GeoArea required
    		result = m_den_service->appDENM_termination(data,actionID);
    	}

        if (result == DENM_NO_ERROR) {
            response["status"] = MAKE_STR("ok");
            response["message"] = MAKE_STR("DENM action performed.");
        } else {
            response["status"] = MAKE_STR("error");
            response["message"] = MAKE_STR("DENM action failed. Error code: " + std::to_string(result));
        }
    } else {
    	response["status"] = MAKE_STR("error");
    	response["message"] = MAKE_STR("Unknown request_type: " + request_type);
    }

    return response;
}

bool JSONserver::stopServer(void) {
	// Return immediately if the server is not running, or if m_tid is not valid
	if(m_thread_running==false || m_tid<0) {
		return false;
	}

	if(pthread_cancel(m_tid)!=0) {
		return false;
	}

	if(pthread_join(m_tid,NULL)!=0) {
		return false;
	}

	m_tid=-1;

	return true;
}
