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
#define GET_ARR(json, key) (json[key].array_items())

std::vector<JSONserver::ContainerMapping> m_containers_mapping = {{
	{"VehicleManeuverContainer", JSONserver::Container::VehicleManeuverContainer},
	{"ManeuverAdviseContainer",    JSONserver::Container::ManeuverAdviseContainer},
	{"AcknowledgmentContainer",    JSONserver::Container::AcknowledgmentContainer},
	{"ResponseContainer",          JSONserver::Container::ResponseContainer},
	{"TerminationContainer",       JSONserver::Container::TerminationContainer}
}};

std::vector<std::string> m_basic_fields = {"MCMConcept", "MCMCost", "MCMGoal", "MCMType", "MCMManeuverID", "MCMITSRole"};

static const std::unordered_map<int, const char*> strategy_json_fields = {
    { SubmanoeuvreStrategy_PR_undefined,                      "Undefined"                      },
    { SubmanoeuvreStrategy_PR_transitToHumanDrivenMode,       "TransitToHumanDrivenMode"       },
    { SubmanoeuvreStrategy_PR_transitToAutomatedDrivingMode,  "TransitToAutomatedDrivingMode"  },
    { SubmanoeuvreStrategy_PR_driveStraight,                  "DriveStraight"                  },
    { SubmanoeuvreStrategy_PR_turnLeft,                       "TurnLeft"                       },
    { SubmanoeuvreStrategy_PR_turnRight,                      "TurnRight"                      },
    { SubmanoeuvreStrategy_PR_uTurn,                          "UTurn"                          },
    { SubmanoeuvreStrategy_PR_moveBackward,                   "MoveBackward"                   },
    { SubmanoeuvreStrategy_PR_overtake,                       "Overtake"                       },
    { SubmanoeuvreStrategy_PR_accelerate,                     "Accelerate"                     },
    { SubmanoeuvreStrategy_PR_slowDown,                       "SlowDown"                       },
    { SubmanoeuvreStrategy_PR_stop,                           "Stop"                           },
    { SubmanoeuvreStrategy_PR_goToLeftLane,                   "GoToLeftLane"                   },
    { SubmanoeuvreStrategy_PR_goToRightLane,                  "GoToRightLane"                  },
    { SubmanoeuvreStrategy_PR_getOnHighway,                   "GetOnHighway"                   },
    { SubmanoeuvreStrategy_PR_exitHighway,                    "ExitHighway"                    },
    { SubmanoeuvreStrategy_PR_takeTollingLane,                "TakeTollingLane"                },
    { SubmanoeuvreStrategy_PR_stopAndWait,                    "StopAndWait"                    },
    { SubmanoeuvreStrategy_PR_emergencyBrakeAndStop,          "EmergencyBrakeAndStop"          },
    { SubmanoeuvreStrategy_PR_resetStopAndRestartMoving,      "ResetStopAndRestartMoving"      },
    { SubmanoeuvreStrategy_PR_stayInLane,                     "StayInLane"                     },
    { SubmanoeuvreStrategy_PR_resetStayInLane,                "ResetStayInLane"                },
    { SubmanoeuvreStrategy_PR_stayAway,                       "StayAway"                       },
    { SubmanoeuvreStrategy_PR_resetStayAway,                  "ResetStayAway"                  },
    { SubmanoeuvreStrategy_PR_followMe,                       "FollowMe"                       },
    { SubmanoeuvreStrategy_PR_existingGroup,                  "ExistingGroup"                  },
    { SubmanoeuvreStrategy_PR_temporarilyDisbandAnExistingGroup, "TemporarilyDisbandAnExistingGroup" },
    { SubmanoeuvreStrategy_PR_constituteATemporarilyGroup,    "ConstituteATemporarilyGroup"    },
    { SubmanoeuvreStrategy_PR_disbandATemporarilyGroup,       "DisbandATemporarilyGroup"       },
};

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
	} else if (request_type == "MCM_trigger") {
		return handleMCMRequest(request);
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

void extractSubmaneuverDescriptions(const json11::Json::array &submaneuvers_json, MCSpecification* specification) {
	for (auto& subm_json : submaneuvers_json) {
		// --- SubmanoeuvreDescription ---
		SubmanoeuvreDescription* subm = specification->create<SubmanoeuvreDescription_t>(asn_DEF_SubmanoeuvreDescription);
		if (!subm) {
			specification->setCreationError("Failed to allocate SubmanoeuvreDescription");
			return;
		}
	
		// --- submanoeuvreID ---
		if (!subm_json["SubmanoeuvreID"].is_null()) {
			specification->set(&subm->submanoeuvreID, GET_NUM(subm_json, "SubmanoeuvreID"));
		} else {
			specification->setCreationError("SubmaneuverDescription needs SubmaneuverID");
			return;
		}
		
		// --- temporalCharacteristics ---
		if (!subm_json["TemporalCharacteristics"].is_null()) {
			for (auto field : {"StartTime", "EndTime"}) {
				if (subm_json["TemporalCharacteristics"][field].is_null()) {
					specification->setCreationError(std::string(field) + " in TemporalCharacteristics not found");
					return;
				}
			}
			specification->set(&subm->temporalCharateristics.tRROccupancyStartTime, GET_NUM(subm_json["TemporalCharacteristics"], "StartTime"));
			specification->set(&subm->temporalCharateristics.tRROccupancyEndTime, GET_NUM(subm_json["TemporalCharacteristics"], "EndTime"));
		} else {
			specification->setCreationError("SubmaneuverDescription needs TemporalCharacteristics");
			return;
		}

		// --- submanoeuvreStrategy (optional) ---
		if (!subm_json["SubmaneuverStrategy"].is_null()) {
			auto* strategy = specification->create<SubmanoeuvreStrategy>(asn_DEF_SubmanoeuvreStrategy);
			if (!strategy) {
				specification->setCreationError("Failed to allocate SubmaneuverStrategy");
				return;
			}
			int present_val = GET_NUM(subm_json["SubmaneuverStrategy"], "Strategy");
    		specification->set(&strategy->present, present_val);
			auto it = strategy_json_fields.find(present_val);
			if (it == strategy_json_fields.end()) {
				specification->setCreationError("Unknown strategy present value: " + std::to_string(present_val));
				return;
			}
			const char* json_field = it->second;
			specification->set(reinterpret_cast<long*>(&strategy->choice), GET_NUM(subm_json["SubmaneuverStrategy"], json_field));
			specification->setOptional(&subm->submanoeuvreStrategy, strategy);
		}

		// --- referenceTrajectory (optional) ---
		if (!subm_json["ReferenceTrajectory"].is_null()) {
			auto* traj = specification->create<Trajectory_t>(asn_DEF_Trajectory);
			if (!traj) {
				specification->setCreationError("Failed to allocate Trajectory");
				return;
			}

			if (subm_json["ReferenceTrajectory"]["WayPointType"].is_null()) {
				specification->setCreationError("WayPointType in ReferenceTrajectory not found");
				return;
			}
			specification->set(&traj->wayPointType, GET_NUM(subm_json["ReferenceTrajectory"], "WayPointType"));
			
			// --- wayPoints ---
			auto& wps_json = GET_ARR(subm_json["ReferenceTrajectory"], "WayPoints");
			for (auto& wp_json : wps_json) {
				auto* wp_created = specification->create<WayPoint_t>(asn_DEF_WayPoint);
				if (!wp_created) {
					specification->setCreationError("Failed to allocate WayPoint");
					return;
				}
			
				for (auto field : {"DeltaLatitude", "DeltaLongitude", "DeltaAltitude"}) {
					if (wp_json[field].is_null()) {
						specification->setCreationError(std::string(field) + " in WayPoint not found");
						return;
					}
				}
				specification->set(&wp_created->pathPosition.deltaLatitude, GET_NUM(wp_json, "DeltaLatitude"));
				specification->set(&wp_created->pathPosition.deltaLongitude, GET_NUM(wp_json, "DeltaLongitude"));
				specification->set(&wp_created->pathPosition.deltaAltitude, GET_NUM(wp_json, "DeltaAltitude"));
			
				// --- pathDeltaTime (optional) ---
				if (!wp_json["PathDeltaTime"].is_null()) {
					auto* pdt = specification->create<PathDeltaTime_t>(asn_DEF_PathDeltaTime);
					if (!pdt) {
						specification->setCreationError("Failed to allocate PathDeltaTime");
						return;
					}
					*pdt = GET_NUM(wp_json, "PathDeltaTime");
					specification->setOptional(&wp_created->pathDeltaTime, pdt);
				}
				if (!specification->add(asn_DEF_WayPoint, &traj->wayPoints, wp_created)) {
					specification->setCreationError("Failed to add WayPoint to trajectory");
					return;
				}
			}

			// --- speed ---
			json11::Json::array speeds_json;
			if (!subm_json["ReferenceTrajectory"]["Speeds"].is_null()) {
				speeds_json = GET_ARR(subm_json["ReferenceTrajectory"], "Speeds");
			} else {
				specification->setCreationError("ReferenceTrajectory in SubmaneuverDescription needs Speeds");
				return;
			}
			for (auto& sp_json : speeds_json) {
				auto* sp_created = specification->create<Speed_t>(asn_DEF_Speed);
				if (!sp_created) {
					specification->setCreationError("Failed to allocate Speed");
					return;
				}
				for (auto field : {"SpeedValue", "SpeedConfidence"}) {
					if (sp_json[field].is_null()) {
						specification->setCreationError(std::string(field) + " in Speed not found");
						return;
					}
				}
				specification->set(&sp_created->speedValue, GET_NUM(sp_json, "SpeedValue"));
				specification->set(&sp_created->speedConfidence, GET_NUM(sp_json, "SpeedConfidence"));
				if (!specification->add(asn_DEF_Speed, &traj->speed, sp_created)) {
					specification->setCreationError("Failed to add Speed to trajectory");
					return;
				}
			}
			// TODO Diego, add list of latitude, longitude, etc... (optionals)
			specification->setOptional(&subm->referenceTrajectory, traj);
		}

		// --- targetRoadResourceIContainer (optional) ---
		if (!subm_json["TargetRoadResource"].is_null()) {
			auto* trr = specification->create<TrrDescription_t>(asn_DEF_TrrDescription);
			if (!trr) {
				specification->setCreationError("Failed to allocate TrrDescription");
				return;
			}

			for (auto field : {"TrrType", "LaneCount", "TrrWidth", "TrrLength"}) {
				if (subm_json["TargetRoadResource"][field].is_null()) {
					specification->setCreationError(std::string(field) + " in TargetRoadResource not found");
					return;
				}
			}
			specification->set(&trr->trrType, GET_NUM(subm_json["TargetRoadResource"], "TrrType"));
			specification->set(&trr->laneCount, GET_NUM(subm_json["TargetRoadResource"], "LaneCount"));
			specification->set(&trr->trrWidth, GET_NUM(subm_json["TargetRoadResource"], "TrrWidth"));
			specification->set(&trr->trrLength, GET_NUM(subm_json["TargetRoadResource"], "TrrLength"));
			
			if (!subm_json["TargetRoadResource"]["StartingLaneNumber"].is_null()) {
				auto* starting_lane_number = specification->create<LaneCount_t>(asn_DEF_LaneCount);
				if (!starting_lane_number) {
					specification->setCreationError("Failed to allocate StartingLaneNumber");
					return;
				}
				specification->set(&starting_lane_number, GET_NUM(subm_json["TargetRoadResource"], "StartingLaneNumber"));
				specification->setOptional(&trr->startingLaneNumber, starting_lane_number);
			}
			
			if (!subm_json["TargetRoadResource"]["EndingLaneNumber"].is_null()) {
				auto* ending_lane_number = specification->create<LaneCount_t>(asn_DEF_LaneCount);
				if (!ending_lane_number) {
					specification->setCreationError("Failed to allocate EndingLaneNumber");
					return;
				}
				specification->set(&ending_lane_number, GET_NUM(subm_json["TargetRoadResource"], "EndingLaneNumber"));
				specification->setOptional(&trr->endingLaneNumber, ending_lane_number);
			}

			auto& wps_json = GET_ARR(subm_json["TargetRoadResource"], "WayPoints");
			for (auto& wp_json : wps_json) {
				auto* wp_created = specification->create<WayPoint_t>(asn_DEF_WayPoint);
				if (!wp_created) {
					specification->setCreationError("Failed to allocate WayPoint");
					return;
				}
			
				for (auto field : {"DeltaLatitude", "DeltaLongitude", "DeltaAltitude"}) {
					if (wp_json[field].is_null()) {
						specification->setCreationError(std::string(field) + " in WayPoint not found");
						return;
					}
				}
				specification->set(&wp_created->pathPosition.deltaLatitude, GET_NUM(wp_json, "DeltaLatitude"));
				specification->set(&wp_created->pathPosition.deltaLongitude, GET_NUM(wp_json, "DeltaLongitude"));
				specification->set(&wp_created->pathPosition.deltaAltitude, GET_NUM(wp_json, "DeltaAltitude"));
			
				// --- pathDeltaTime (optional) ---
				if (!wp_json["PathDeltaTime"].is_null()) {
					auto* pdt = specification->create<PathDeltaTime_t>(asn_DEF_PathDeltaTime);
					if (!pdt) {
						specification->setCreationError("Failed to allocate PathDeltaTime");
						return;
					}
					*pdt = GET_NUM(wp_json, "PathDeltaTime");
					specification->setOptional(&wp_created->pathDeltaTime, pdt);
				}
				if (!specification->add(asn_DEF_WayPoint, &trr->waypoints, wp_created)) {
					specification->setCreationError("Failed to add WayPoint to trajectory");
					return;
				}
			}

			auto& headings_json = GET_ARR(subm_json["TargetRoadResource"], "Heading");
			for (auto& h_json : headings_json) {
				auto* h = specification->create<Wgs84Angle>(asn_DEF_Wgs84Angle);
				if (!h) {
					specification->setCreationError("Failed to allocate Wgs84Angle");
					return;
				}
				for (auto field : {"HeadingValue", "HeadingConfidence"}) {
					if (h_json[field].is_null()) {
						specification->setCreationError(std::string(field) + " in Heading not found");
						return;
					}
				}
				specification->set(&h->value, GET_NUM(h_json, "HeadingValue"));
				specification->set(&h->value, GET_NUM(h_json, "HeadingConfidence"));
				if (!specification->add(asn_DEF_Wgs84Angle, &trr->heading, h)) {
					specification->setCreationError("Failed to add Wgs84Angle to heading");
					return;
				}
			}

			specification->setOptional(&subm->targetRoadResourceIContainer, trr);
		}

		// --- kinematicsCharacteristics (optional) ---
		// For the moment the ASN indicates this field as nullptr
	}
}

std::tuple<std::string, JSONserver::Container> JSONserver::json_for_MCM_is_valid(const json11::Json &request) {
	// Basic fields
    for (const auto& field : m_basic_fields) {
        if (request[field].is_null()) {
            return {"Missing basic field in JSON: " + std::string(field), Container::NotPresent};
        }
    }

	// Containers, at least and at most one present
	JSONserver::Container found_container = JSONserver::Container::NotPresent;
    bool res = false;
    for (const auto& item : m_containers_mapping) {
        if (!request[item.name].is_null()) {
            if (!res) {
                res = true;
                found_container = item.type;
            } else {
                return {"Too many MCM containers in JSON", Container::NotPresent};
            }
        }
    }

    if (res) {
        return {"", found_container};
    }
    return {"Missing field in JSON: a MCM container is required", Container::NotPresent};
}

void JSONserver::fillMCSpecificationFromJson(const json11::Json &request, MCSpecification* specification) {
	// Check whether there are some missing fields in the JSON request
	auto [error_msg, container_type] = json_for_MCM_is_valid(request);
    if (!error_msg.empty()) {
        specification->setCreationError(error_msg); 
        return;
    }

	// Set general informaiton for MCM fields
	specification->setMCMConcept(GET_NUM(request, "MCMConcept"));
	specification->setMCMCost(GET_NUM(request, "MCMCost"));
	specification->setMCMGoal(GET_NUM(request, "MCMGoal"));
	specification->setMCMType(GET_NUM(request, "MCMType"));
	specification->setManeuverID(static_cast<ManeuverID>(GET_NUM(request, "MCMManeuverID")));
	specification->setMCMItsRole(GET_NUM(request, "MCMITSRole"));

	// Fill the required container
	switch (container_type) {
		case JSONserver::Container::VehicleManeuverContainer:
			specification->setManeuverContainer();
			// Get the Submanouvers
			if (!request["MCMSubmaneuvers"].is_null()) {
				auto& submaneuvers_json = GET_ARR(request, "MCMSubmaneuvers");
				extractSubmaneuverDescriptions(submaneuvers_json, specification);
				if (std::get<0>(specification->getCreationError())) {
					return;
				}
			} else {
				// Throw an error
				specification->setCreationError("ManeuverContainer needs a MCMSubmaneuvers");
			}
			// Get the ManeuverAdvice (optional)
			if (!request["MCMManeuverAdvice"].is_null()) {
				auto& advices_json = GET_ARR(request, "MCMManeuverAdvice");
				// TODO Diego (not urgent)
			}
			break;
		case JSONserver::Container::ManeuverAdviseContainer:
			specification->setAdviseContainer();
			// Get the ManeuverAdvice
			if (!request["MCMManeuverAdvice"].is_null()) {
				auto& advices_json = GET_ARR(request, "MCMManeuverAdvice");
				// TODO Diego (not urgent)
			} else {
				// Throw an error
				specification->setCreationError("AdviseContainer needs a MCMManeuverAdvice");
			}
			break;
		case JSONserver::Container::ResponseContainer:
			specification->setResponseContainer();
			if (!request["ResponseContainer"]["MCMResponse"].is_null()) {
				specification->setMCMResponse(GET_NUM(request["ResponseContainer"], "MCMResponse"));
				if (specification->getMCMResponse() == 1) {
					// If the station is sending a refusal, we need to know the reason
					if (!request["ResponseContainer"]["MCMResponseDeclineReason"].is_null()) {
						specification->setMCMResponseDeclineReason(GET_NUM(request["ResponseContainer"], "MCMResponseDeclineReason"));
					} else {
						// Throw an error
						specification->setCreationError("ResponseContainer needs a MCMResponseDeclineReason when the maneuver is declined");
					}
				}
				// Get the Submanouvers if indicated (optional)
				if (!request["MCMSubmaneuvers"].is_null()) {
					auto& submaneuvers_json = GET_ARR(request, "MCMSubmaneuvers");
					extractSubmaneuverDescriptions(submaneuvers_json, specification);
					if (std::get<0>(specification->getCreationError())) {
						return;
					}
				}
			} else {
				// Throw an error
				specification->setCreationError("ResponseContainer always needs a MCMResponse");
			}
			break;
		case JSONserver::Container::AcknowledgmentContainer:
			specification->setAcknowledgmentContainer(); // Just set the container, no other actions needed
			break;
		case JSONserver::Container::TerminationContainer:
			specification->setTerminatorContainer(); // Just set the container, no other actions needed
			break;
		default:
			break;
	}
}

json11::Json::object JSONserver::handleMCMRequest(const json11::Json &request) {
    json11::Json::object response;

    if (m_mc_service == nullptr) {
		// Ensure that the MC Basic Service is available
        response["status"] = MAKE_STR("error");
        response["message"] = MAKE_STR("MC service is not available");
        return response;
    }

    std::string request_type = GET_STR(request, "request_type");
	MCSpecification specification;
    fillMCSpecificationFromJson(request, &specification);

	auto [error, reason] = specification.getCreationError();
	if (error) {
		// In case of creation error during the JSON parsing
		response["status"] = MAKE_STR("error");
		response["message"] = MAKE_STR(reason);
		return response;
	}

	// The parsing didn't show issues, send the MCM
	MCBasicService_error_t result = m_mc_service->generateAndEncodeMCM(&specification);

	if(result != MCM_NO_ERROR) {
		// In case of error during the encoding or sending phases
		response["status"] = MAKE_STR("error");
		response["message"] = MAKE_STR("MC encoding or sending failed");
        
	}
	else {
		// MCM correctly sent
		response["status"] = MAKE_STR("ok");
		response["message"] = MAKE_STR("MCM sent correctly");
	}

	return response;
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
