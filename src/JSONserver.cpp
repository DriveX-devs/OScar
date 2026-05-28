#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <pthread.h>
#include <unistd.h>
#include <cstring>

#include "JSONserver.h"
#include "utils.h"
#include "asn_utils.h"

#include "mcData.h"
#include <iostream>
#include <string>
#include <tuple>

#define MAKE_NUM(num) json11::Json(num)
#define MAKE_STR(str) json11::Json(str)
#define MAKE_INT(num) json11::Json(static_cast<int>(num))
#define GET_NUM(json,key) (json[key].number_value())
#define GET_INT(json,key) (json[key].int_value())
#define GET_STR(json,key) (json[key].string_value())
#define GET_ARR(json, key) (json[key].array_items())

std::vector<JSONserver::ContainerMapping> JSONserver::m_containers_mapping = {{
	{"VehicleManeuverContainer",   JSONserver::Container::VehicleManeuverContainer},
	{"ManeuverAdviseContainer",    JSONserver::Container::ManeuverAdviseContainer},
	{"AcknowledgmentContainer",    JSONserver::Container::AcknowledgmentContainer},
	{"ResponseContainer",          JSONserver::Container::ResponseContainer},
	{"TerminationContainer",       JSONserver::Container::TerminationContainer}
}};

std::vector<std::string> JSONserver::m_basic_fields = {"MCMConcept", "MCMCost", "MCMGoal", "MCMType", "MCMManeuverID", "MCMITSRole"};

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
                        srvObj->handleMCMDisconnection(currd);
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
							json11::Json::object response = srvObj->handleRequest(request, currd);
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
				} else {
					++currd_it;
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

json11::Json::object JSONserver::handleRequest(const json11::Json &request, int client_fd) {
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
    } else if (request_type == "MCM_subscribe") {
        return handleMCMConnection(client_fd);
    } else if (request_type == "MCM_unsubscribe") {
        return handleMCMDisconnection(client_fd);
    }
    else {
    	std::cout << "[ERROR] Received unknown JSON request: " << request_type << std::endl;

        json11::Json::object response;
    	response["status"] = MAKE_STR("error");
    	response["message"] = MAKE_STR("Unknown request_type: " + request_type);
        return response;
    }
}

json11::Json::object JSONserver::handleMCMConnection(int client_fd) {
    std::lock_guard<std::mutex> lk(m_mcm_mtx);
    m_mcm_subscribers.insert(client_fd);

    json11::Json::object response;
    response["status"]  = MAKE_STR("ok");
    response["message"] = MAKE_STR("Subscribed to MCM stream");
    return response;
}

json11::Json::object JSONserver::handleMCMDisconnection(int client_fd) {
    std::lock_guard<std::mutex> lk(m_mcm_mtx);
    m_mcm_subscribers.erase(client_fd);

    json11::Json::object response;
    response["status"]  = MAKE_STR("ok");
    response["message"] = MAKE_STR("Unsubscribed from MCM stream");
    return response;
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

// --- native → JSON serializers (inverse dei parser sottostanti) ---

// Helper: long → JSON number (cast esplicito a double per evitare
// l'ambiguità Json(int)/Json(double) e la troncatura di MAKE_INT su
// StationID a 32 bit unsigned).

static inline json11::Json numberToJson(long value) {
    return json11::Json(static_cast<double>(value));
}

static json11::Json::object pathPointToJson(const mcData::mcDataPathPoint& pathPoint) {
    json11::Json::object pathPointJson;
    pathPointJson["DeltaLatitude"]  = numberToJson(pathPoint.deltaLatitude);
    pathPointJson["DeltaLongitude"] = numberToJson(pathPoint.deltaLongitude);
    pathPointJson["DeltaAltitude"]  = numberToJson(pathPoint.deltaAltitude);
    if (pathPoint.pathDeltaTime.isAvailable()) {
        pathPointJson["PathDeltaTime"] = numberToJson(pathPoint.pathDeltaTime.getData());
    }
    return pathPointJson;
}

static json11::Json::object trajectoryToJson(const mcData::mcDataTrajectory& trajectory) {
    json11::Json::object trajectoryJson;
    trajectoryJson["WayPointType"] = numberToJson(trajectory.wayPointType);

    json11::Json::array waypointsJson;
    for (const auto& waypoint : trajectory.wayPoints) {
        waypointsJson.push_back(pathPointToJson(waypoint));
    }
    trajectoryJson["WayPoints"] = waypointsJson;

    json11::Json::array speedsJson;
    for (const auto& speedSample : trajectory.speed) {
        speedsJson.push_back(json11::Json::object{
            {"SpeedValue",      numberToJson(speedSample.getValue())},
            {"SpeedConfidence", numberToJson(speedSample.getConfidence())}
        });
    }
    trajectoryJson["Speed"] = speedsJson;

    json11::Json::array headingsJson;
    for (const auto& headingSample : trajectory.headings) {
        headingsJson.push_back(json11::Json::object{
            {"HeadingValue",      numberToJson(headingSample.getValue())},
            {"HeadingConfidence", numberToJson(headingSample.getConfidence())}
        });
    }
    trajectoryJson["Heading"] = headingsJson;

    json11::Json::array longitudesJson;
    for (long longitudeValue : trajectory.longitudePositions) {
        longitudesJson.push_back(json11::Json::object{
            {"LongitudeValue", numberToJson(longitudeValue)}
        });
    }
    trajectoryJson["Longitude"] = longitudesJson;

    json11::Json::array latitudesJson;
    for (long latitudeValue : trajectory.latitudePositions) {
        latitudesJson.push_back(json11::Json::object{
            {"LatitudeValue", numberToJson(latitudeValue)}
        });
    }
    trajectoryJson["Latitude"] = latitudesJson;

    json11::Json::array altitudesJson;
    for (const auto& altitudeSample : trajectory.altitudePositions) {
        altitudesJson.push_back(json11::Json::object{
            {"AltitudeValue",      numberToJson(altitudeSample.getValue())},
            {"AltitudeConfidence", numberToJson(altitudeSample.getConfidence())}
        });
    }
    trajectoryJson["Altitude"] = altitudesJson;

    return trajectoryJson;
}

static json11::Json::object trrDescriptionToJson(const mcData::mcDataTrrDescription& trrDescription) {
    json11::Json::object trrJson;
    trrJson["TrrType"]   = numberToJson(trrDescription.trrType);
    trrJson["LaneCount"] = numberToJson(trrDescription.laneCount);
    trrJson["TrrWidth"]  = numberToJson(trrDescription.trrWidth);
    trrJson["TrrLength"] = numberToJson(trrDescription.trrLength);

    if (trrDescription.startingLaneNumber.isAvailable()) {
        trrJson["StartingLaneNumber"] = numberToJson(trrDescription.startingLaneNumber.getData());
    }
    if (trrDescription.endingLaneNumber.isAvailable()) {
        trrJson["EndingLaneNumber"] = numberToJson(trrDescription.endingLaneNumber.getData());
    }

    json11::Json::array waypointsJson;
    for (const auto& waypoint : trrDescription.waypoints) {
        waypointsJson.push_back(pathPointToJson(waypoint));
    }
    trrJson["WayPoints"] = waypointsJson;

    json11::Json::array headingsJson;
    for (const auto& headingSample : trrDescription.heading) {
        headingsJson.push_back(json11::Json::object{
            {"HeadingValue",      numberToJson(headingSample.getValue())},
            {"HeadingConfidence", numberToJson(headingSample.getConfidence())}
        });
    }
    trrJson["Heading"] = headingsJson;

    return trrJson;
}

static json11::Json::object submaneuverToJson(const mcData::mcDataSubmaneuverDescription& submaneuver) {
    json11::Json::object submaneuverJson;
    submaneuverJson["SubmaneuverID"] = numberToJson(submaneuver.submaneuverID);

    submaneuverJson["TemporalCharacteristics"] = json11::Json::object{
        {"StartTime", numberToJson(submaneuver.temporalCharacteristics.tRROccupancyStartTime)},
        {"EndTime",   numberToJson(submaneuver.temporalCharacteristics.tRROccupancyEndTime)}
    };

    if (submaneuver.submaneuverStrategy.isAvailable()) {
        const auto& strategy = submaneuver.submaneuverStrategy.getData();
        auto strategyNameIt = strategy_json_fields.find(strategy.present);
        if (strategyNameIt != strategy_json_fields.end()) {
            submaneuverJson["SubmaneuverStrategy"] = json11::Json::object{
                {"Strategy",             numberToJson(strategy.present)},
                {strategyNameIt->second, numberToJson(strategy.value)}
            };
        } else {
            std::cerr << "[WARN] convertToJson: unknown strategy.present="
                      << strategy.present << ", skipping" << std::endl;
        }
    }
    if (submaneuver.referenceTrajectory.isAvailable()) {
        submaneuverJson["ReferenceTrajectory"] =
            trajectoryToJson(submaneuver.referenceTrajectory.getData());
    }
    if (submaneuver.targetRoadResource.isAvailable()) {
        submaneuverJson["TargetRoadResource"] =
            trrDescriptionToJson(submaneuver.targetRoadResource.getData());
    }

    return submaneuverJson;
}

static json11::Json::object maneuverAdviceToJson(const mcData::mcDataManeuverAdvice& maneuverAdvice) {
    json11::Json::object adviceJson;
    adviceJson["ExecutantID"] = numberToJson(maneuverAdvice.executantID);

    if (maneuverAdvice.currentStateAdvisedChange.isAvailable()) {
        adviceJson["CurrentStateAdvisedChange"] = json11::Json::object{
            {"Present", numberToJson(maneuverAdvice.currentStateAdvisedChange.getData())}
        };
    }

    json11::Json::array advisedSubmaneuversJson;
    for (const auto& advisedSubmaneuver : maneuverAdvice.submaneuvers) {
        json11::Json::object advisedSubmaneuverJson;
        advisedSubmaneuverJson["SubmaneuverID"] = numberToJson(advisedSubmaneuver.submaneuverID);

        if (advisedSubmaneuver.advisedTrajectory.isAvailable()) {
            advisedSubmaneuverJson["AdvisedTrajectory"] =
                trajectoryToJson(advisedSubmaneuver.advisedTrajectory.getData());
        }

        if (advisedSubmaneuver.advisedTrrContainer.isAvailable()) {
            const auto& advisedTrr = advisedSubmaneuver.advisedTrrContainer.getData();
            // parseTrrDescription + StartTime/EndTime "mischiati" allo stesso livello
            json11::Json::object advisedTrrJson = trrDescriptionToJson(advisedTrr.trrDescription);
            advisedTrrJson["StartTime"] =
                numberToJson(advisedTrr.temporalCharacteristics.tRROccupancyStartTime);
            advisedTrrJson["EndTime"]   =
                numberToJson(advisedTrr.temporalCharacteristics.tRROccupancyEndTime);
            advisedSubmaneuverJson["AdvisedTargetRoadResource"] = advisedTrrJson;
        }
        advisedSubmaneuversJson.push_back(advisedSubmaneuverJson);
    }
    adviceJson["Submaneuvres"] = advisedSubmaneuversJson;

    return adviceJson;
}

static json11::Json::object convertToJson(const mcData& mcmData) {
    json11::Json::object mcmJson;

    // --- Basic container: campi al top-level, come fa handleMCMRequest ---
    if (mcmData.getBasicContainer().isAvailable()) {
        const auto& basicContainer = mcmData.getBasicContainer().getData();
        mcmJson["MCMStationID"]   = numberToJson(basicContainer.stationID);
        mcmJson["MCMITSRole"]     = numberToJson(basicContainer.itsRole);
        mcmJson["MCMStationType"] = numberToJson(basicContainer.stationType);
        mcmJson["MCMType"]        = numberToJson(basicContainer.mcmType);
        mcmJson["MCMManeuverID"]  = numberToJson(basicContainer.maneuverID);
        mcmJson["MCMConcept"]     = numberToJson(basicContainer.concept);

        // concept: 1 = cost, 0 = goal (cfr. handleMCMRequest:750-764)
        if (basicContainer.concept == 1) {
            mcmJson["MCMCost"] = numberToJson(basicContainer.cost);
        } else {
            mcmJson["MCMGoal"] = numberToJson(basicContainer.goal);
        }

        // ExecutionStatus richiesto solo per MCMType ∈ {4,7}, ma scriviamolo
        // ogni volta che è disponibile (è il dato più informativo per il client)
        if (basicContainer.executionStatus.isAvailable()) {
            mcmJson["MCMExecutionStatus"] =
                numberToJson(basicContainer.executionStatus.getData());
        }
    }

    // --- Esattamente uno dei container (cfr. cascata in handleMCMRequest) ---
    if (mcmData.getAdviceContainer().isAvailable()) {
        const auto& adviceContainer = mcmData.getAdviceContainer().getData();
        json11::Json::array maneuverAdvicesJson;
        for (const auto& maneuverAdvice : adviceContainer.advices) {
            maneuverAdvicesJson.push_back(maneuverAdviceToJson(maneuverAdvice));
        }
        mcmJson["MCManeuverAdviceContainer"] = json11::Json::object{
            {"MCManeuverAdvices", maneuverAdvicesJson}
        };

    } else if (mcmData.getManeuverContainer().isAvailable()) {
        const auto& maneuverContainer = mcmData.getManeuverContainer().getData();
        json11::Json::object vehicleContainerJson;

        if (maneuverContainer.advices.isAvailable()) {
            json11::Json::array maneuverAdvicesJson;
            for (const auto& maneuverAdvice : maneuverContainer.advices.getData()) {
                maneuverAdvicesJson.push_back(maneuverAdviceToJson(maneuverAdvice));
            }
            vehicleContainerJson["MCManeuverAdvices"] = maneuverAdvicesJson;
        }

        json11::Json::array submaneuversJson;
        for (const auto& submaneuver : maneuverContainer.submaneuvers) {
            submaneuversJson.push_back(submaneuverToJson(submaneuver));
        }
        vehicleContainerJson["MCSubmaneuvers"] = submaneuversJson;

        mcmJson["MCVehicleManeuverContainer"] = vehicleContainerJson;

    } else if (mcmData.getResponseContainer().isAvailable()) {
        const auto& responseContainer = mcmData.getResponseContainer().getData();
        json11::Json::object responseJson;
        responseJson["MCResponse"] = numberToJson(responseContainer.response);

        if (responseContainer.response == 1 && responseContainer.declineReason.isAvailable()) {
            responseJson["MCDeclineReason"] =
                numberToJson(responseContainer.declineReason.getData());
        }
        if (responseContainer.submaneuvers.isAvailable()) {
            json11::Json::array submaneuversJson;
            for (const auto& submaneuver : responseContainer.submaneuvers.getData()) {
                submaneuversJson.push_back(submaneuverToJson(submaneuver));
            }
            responseJson["MCSubmaneuvers"] = submaneuversJson;
        }
        mcmJson["MCResponseContainer"] = responseJson;

    } else if (mcmData.getAcknowledgmentContainer().isAvailable()) {
        const auto& acknowledgmentContainer = mcmData.getAcknowledgmentContainer().getData();
        mcmJson["MCAcknowledgmentContainer"] = json11::Json::object{
            {"MCAcknowledgmentType", numberToJson(acknowledgmentContainer.type)}
        };

    } else if (mcmData.getTerminationContainer().isAvailable()) {
        mcmJson["MCTerminationContainer"] = json11::Json::object{};

    } else {
        std::cerr << "[WARN] convertToJson: no container set on mcData" << std::endl;
    }

    return mcmJson;
}



void JSONserver::createJSONFromMCM(MCM_t* decoded_mcm) {
	std::cout << " [INFO] MCM received, forwarding information via JSON-over-TCP to " << m_mcm_subscribers.size() << " subscribers." << std::endl;
    mcData mcmData = m_mc_service->convertASN1IntoMcData(decoded_mcm);
    json11::Json::object mcm_json = convertToJson(mcmData);
    std::string strjson = json11::Json(mcm_json).dump();

    std::lock_guard<std::mutex> lk(m_mcm_mtx);
    for (int fd : m_mcm_subscribers) {
        if (send(fd, strjson.c_str(), strjson.length(), MSG_NOSIGNAL)
                != static_cast<ssize_t>(strjson.length())) {
            perror("[ERROR] Could not send MCM JSON to subscriber");
        }
    }
}

// Utility function to encapsulate the JSON parsing logic into native mcData structures
// --- JSON → native parsers ---

static bool parseTrajectory(const json11::Json& traj_json, mcData::mcDataTrajectory& out) {
    if (traj_json["WayPointType"].is_null()) {
        std::cerr << "[ERROR] WayPointType in Trajectory not found" << std::endl;
        return false;
    }
    out.wayPointType = GET_NUM(traj_json, "WayPointType");

    for (const auto& wp_json : GET_ARR(traj_json, "WayPoints")) {
        for (const auto field : {"DeltaLatitude", "DeltaLongitude", "DeltaAltitude"}) {
            if (wp_json[field].is_null()) {
                std::cerr << "[ERROR] " << field << " in WayPoint not found" << std::endl;
                return false;
            }
        }
        mcData::mcDataPathPoint wp;
        wp.deltaLatitude  = GET_NUM(wp_json, "DeltaLatitude");
        wp.deltaLongitude = GET_NUM(wp_json, "DeltaLongitude");
        wp.deltaAltitude  = GET_NUM(wp_json, "DeltaAltitude");
        if (!wp_json["PathDeltaTime"].is_null()) {
            wp.pathDeltaTime.setData(GET_NUM(wp_json, "PathDeltaTime"));
        }
        out.wayPoints.push_back(wp);
    }

    for (const auto& sp_json : GET_ARR(traj_json, "Speed")) {
        for (const auto field : {"SpeedValue", "SpeedConfidence"}) {
            if (sp_json[field].is_null()) {
                std::cerr << "[ERROR] " << field << " in Speed not found" << std::endl;
                return false;
            }
        }
        out.speed.emplace_back(GET_NUM(sp_json, "SpeedValue"),
                               GET_NUM(sp_json, "SpeedConfidence"));
    }

    for (const auto& hd_json : GET_ARR(traj_json, "Heading")) {
        for (const auto field : {"HeadingValue", "HeadingConfidence"}) {
            if (hd_json[field].is_null()) {
                std::cerr << "[ERROR] " << field << " in Heading not found" << std::endl;
                return false;
            }
        }
        out.headings.emplace_back(GET_NUM(hd_json, "HeadingValue"),
                                  GET_NUM(hd_json, "HeadingConfidence"));
    }

    for (const auto& lon_json : GET_ARR(traj_json, "Longitude")) {
        if (lon_json["LongitudeValue"].is_null()) {
            std::cerr << "[ERROR] LongitudeValue in Longitude not found" << std::endl;
            return false;
        }
        out.longitudePositions.push_back(GET_NUM(lon_json, "LongitudeValue"));
    }

    for (const auto& lat_json : GET_ARR(traj_json, "Latitude")) {
        if (lat_json["LatitudeValue"].is_null()) {
            std::cerr << "[ERROR] LatitudeValue in Latitude not found" << std::endl;
            return false;
        }
        out.latitudePositions.push_back(GET_NUM(lat_json, "LatitudeValue"));
    }

    for (const auto& alt_json : GET_ARR(traj_json, "Altitude")) {
        for (const auto field : {"AltitudeValue", "AltitudeConfidence"}) {
            if (alt_json[field].is_null()) {
                std::cerr << "[ERROR] " << field << " in Altitude not found" << std::endl;
                return false;
            }
        }
        out.altitudePositions.emplace_back(GET_NUM(alt_json, "AltitudeValue"),
                                           GET_NUM(alt_json, "AltitudeConfidence"));
    }

    return true;
}

static bool parseTrrDescription(const json11::Json& trr_json, mcData::mcDataTrrDescription& out) {
    for (const auto field : {"TrrType", "LaneCount", "TrrWidth", "TrrLength"}) {
        if (trr_json[field].is_null()) {
            std::cerr << "[ERROR] " << field << " in TrrDescription not found" << std::endl;
            return false;
        }
    }
    out.trrType   = GET_NUM(trr_json, "TrrType");
    out.laneCount = GET_NUM(trr_json, "LaneCount");
    out.trrWidth  = GET_NUM(trr_json, "TrrWidth");
    out.trrLength = GET_NUM(trr_json, "TrrLength");

    if (!trr_json["StartingLaneNumber"].is_null()) {
        out.startingLaneNumber.setData(GET_NUM(trr_json, "StartingLaneNumber"));
    }
    if (!trr_json["EndingLaneNumber"].is_null()) {
        out.endingLaneNumber.setData(GET_NUM(trr_json, "EndingLaneNumber"));
    }

    for (const auto& wp_json : GET_ARR(trr_json, "WayPoints")) {
        for (const auto field : {"DeltaLatitude", "DeltaLongitude", "DeltaAltitude"}) {
            if (wp_json[field].is_null()) {
                std::cerr << "[ERROR] " << field << " in WayPoint not found" << std::endl;
                return false;
            }
        }
        mcData::mcDataPathPoint wp;
        wp.deltaLatitude  = GET_NUM(wp_json, "DeltaLatitude");
        wp.deltaLongitude = GET_NUM(wp_json, "DeltaLongitude");
        wp.deltaAltitude  = GET_NUM(wp_json, "DeltaAltitude");
        if (!wp_json["PathDeltaTime"].is_null()) {
            wp.pathDeltaTime.setData(GET_NUM(wp_json, "PathDeltaTime"));
        }
        out.waypoints.push_back(wp);
    }

    for(const auto& hd_json : GET_ARR(trr_json, "Heading")) {
        for(const auto field : {"HeadingValue", "HeadingConfidence"}) {
            if(hd_json[field].is_null()) {
                std::cerr << "[ERROR] " << field << " in Heading not found" << std::endl;
                return false;
            }
        }
        out.heading.emplace_back(GET_NUM(hd_json, "HeadingValue"),
                                 GET_NUM(hd_json, "HeadingConfidence"));
    }

    return true;
}

static bool extractSubmaneuvers(const json11::Json::array& subms_json,
                                 std::vector<mcData::mcDataSubmaneuverDescription>& out) {
    for(const auto& subm_json : subms_json) {
        mcData::mcDataSubmaneuverDescription subm;

        if(subm_json["SubmaneuverID"].is_null()) {
            std::cerr << "[ERROR] SubmaneuverID is required in SubmaneuverDescription" << std::endl;
            return false;
        }
        subm.submaneuverID = GET_NUM(subm_json, "SubmaneuverID");

        // --- temporalCharacteristics (mandatory) ---
        if(subm_json["TemporalCharacteristics"].is_null()) {
            std::cerr << "[ERROR] TemporalCharacteristics is required in SubmaneuverDescription" << std::endl;
            return false;
        }
        for(const auto field : {"StartTime", "EndTime"}) {
            if(subm_json["TemporalCharacteristics"][field].is_null()) {
                std::cerr << "[ERROR] " << field << " in TemporalCharacteristics not found" << std::endl;
                return false;
            }
        }
        subm.temporalCharacteristics.tRROccupancyStartTime =
            GET_NUM(subm_json["TemporalCharacteristics"], "StartTime");
        subm.temporalCharacteristics.tRROccupancyEndTime =
            GET_NUM(subm_json["TemporalCharacteristics"], "EndTime");

        // --- submaneuverStrategy (optional) ---
        if(!subm_json["SubmaneuverStrategy"].is_null()) {
            mcData::mcDataSubmaneuverStrategy strategy;
            strategy.present = GET_NUM(subm_json["SubmaneuverStrategy"], "Strategy");

            auto it = strategy_json_fields.find(strategy.present);
            if(it == strategy_json_fields.end()) {
                std::cerr << "[ERROR] Unknown strategy present value: "
                          << strategy.present << std::endl;
                return false;
            }
            strategy.value = GET_NUM(subm_json["SubmaneuverStrategy"], it->second);
            subm.submaneuverStrategy.setData(strategy);
        }

        // --- referenceTrajectory (optional) ---
        if(!subm_json["ReferenceTrajectory"].is_null()) {
            mcData::mcDataTrajectory traj;
            if (!parseTrajectory(subm_json["ReferenceTrajectory"], traj)) return false;
            subm.referenceTrajectory.setData(traj);
        }

        // --- targetRoadResource (optional) ---
        if(!subm_json["TargetRoadResource"].is_null()) {
            mcData::mcDataTrrDescription trr;
            if (!parseTrrDescription(subm_json["TargetRoadResource"], trr)) return false;
            subm.targetRoadResource.setData(trr);
        }

        out.push_back(subm);
    }
    return true;
}

static bool extractManeuverAdvice(const json11::Json::array& advs_json,
                                   std::vector<mcData::mcDataManeuverAdvice>& out) {
    for(const auto& adv_json : advs_json) {
        mcData::mcDataManeuverAdvice adv;

        if(adv_json["ExecutantID"].is_null()) {
            std::cerr << "[ERROR] ExecutantID is required in ManeuverAdvice" << std::endl;
            return false;
        }
        adv.executantID = GET_NUM(adv_json, "ExecutantID");

        // --- currentStateAdvisedChange (optional) ---
        if(!adv_json["CurrentStateAdvisedChange"].is_null()) {
            adv.currentStateAdvisedChange.setData(
                GET_NUM(adv_json["CurrentStateAdvisedChange"], "Present"));
        }

        // --- submaneuvres (mandatory) ---
        if(adv_json["Submaneuvres"].is_null()) {
            std::cerr << "[ERROR] Submaneuvres is required in ManeuverAdvice" << std::endl;
            return false;
        }
        for(const auto& subm_json : GET_ARR(adv_json, "Submaneuvres")) {
            mcData::mcDataAdvisedSubmaneuver subm;

            if(subm_json["SubmaneuverID"].is_null()) {
                std::cerr << "[ERROR] SubmaneuverID is required in Submanoeuvre" << std::endl;
                return false;
            }
            subm.submaneuverID = GET_NUM(subm_json, "SubmaneuverID");

            // --- advisedTrajectory (optional) ---
            if(!subm_json["AdvisedTrajectory"].is_null()) {
                mcData::mcDataTrajectory traj;
                if(!parseTrajectory(subm_json["AdvisedTrajectory"], traj)) return false;
                subm.advisedTrajectory.setData(traj);
            }

            // --- advisedTrrContainer (optional) ---
            if(!subm_json["AdvisedTargetRoadResource"].is_null()) {
                mcData::mcDataAdvisedTrrContainer atrr;

                if(!parseTrrDescription(subm_json["AdvisedTargetRoadResource"],
                                         atrr.trrDescription)) return false;

                for(const auto field : {"StartTime", "EndTime"}) {
                    if(subm_json["AdvisedTargetRoadResource"][field].is_null()) {
                        std::cerr << "[ERROR] " << field
                                  << " in AdvisedTargetRoadResource not found" << std::endl;
                        return false;
                    }
                }
                atrr.temporalCharacteristics.tRROccupancyStartTime =
                    GET_NUM(subm_json["AdvisedTargetRoadResource"], "StartTime");
                atrr.temporalCharacteristics.tRROccupancyEndTime =
                    GET_NUM(subm_json["AdvisedTargetRoadResource"], "EndTime");

                subm.advisedTrrContainer.setData(atrr);
            }

            adv.submaneuvers.push_back(subm);
        }

        out.push_back(adv);
    }
    return true;
}

json11::Json::object JSONserver::handleMCMRequest(const json11::Json& request) {
    json11::Json::object response;

    if(m_mc_service == nullptr) {
        response["status"] = MAKE_STR("error");
        response["message"] = MAKE_STR("MC service is not available");
        return response;
    }

    mcData mcmData;

	mcData::mcBasicContainer mcBasicContainer{};

	// MCMCost and MCMGoal map to an ASN.1 CHOICE — exactly one must be present.
	if(!request["MCMCost"].is_null() && !request["MCMGoal"].is_null()) {
		response["status"] = MAKE_STR("error");
		response["message"] = MAKE_STR("MCMCost and MCMGoal are mutually exclusive; only one can be provided.");
		return response;
	}
	if(request["MCMCost"].is_null() && request["MCMGoal"].is_null()) {
		response["status"] = MAKE_STR("error");
		response["message"] = MAKE_STR("MCMGoal and MCMCost cannot be both empty.");
		return response;
	}
	if(!request["MCMCost"].is_null()) {
		mcBasicContainer.cost = GET_NUM(request, "MCMCost");
		mcBasicContainer.concept = 1;
	} else {
		mcBasicContainer.goal = GET_NUM(request, "MCMGoal");
		mcBasicContainer.concept = 0;
	}
	if(!request["MCMManeuverID"].is_null()) {
		mcBasicContainer.maneuverID = GET_NUM(request, "MCMManeuverID");
	} else {
		response["status"] = MAKE_STR("error");
		response["message"] = MAKE_STR("MCMManeuverID cannot be empty.");
		return response;
	}
	if(!request["MCMITSRole"].is_null()) {
		mcBasicContainer.itsRole = GET_NUM(request, "MCMITSRole");
	} else {
		response["status"] = MAKE_STR("error");
		response["message"] = MAKE_STR("MCMITSRole cannot be empty.");
		return response;
	}
	if(!request["MCMType"].is_null()) {
		mcBasicContainer.mcmType = GET_NUM(request, "MCMType");
		if(mcBasicContainer.mcmType == 4 || mcBasicContainer.mcmType == 7) {
			if(!request["MCMExecutionStatus"].is_null()) {
				mcBasicContainer.executionStatus.setData(GET_NUM(request, "MCMExecutionStatus"));
			} else {
				response["status"] = MAKE_STR("error");
				response["message"] = MAKE_STR("MCMExecutionStatus cannot be empty in case MCMType is 4 or 7.");
				return response;
			}
		}
	} else {
		response["status"] = MAKE_STR("error");
		response["message"] = MAKE_STR("MCMType cannot be empty.");
		return response;
	}
	if(!request["MCMStationType"].is_null()) {
		mcBasicContainer.stationType = GET_NUM(request, "MCMStationType");
	} else {
		response["status"] = MAKE_STR("error");
		response["message"] = MAKE_STR("MCMStationType cannot be empty.");
		return response;
	}
	if(!request["MCMStationID"].is_null()) {
		mcBasicContainer.stationID = GET_NUM(request, "MCMStationID");
	} else {
		response["status"] = MAKE_STR("error");
		response["message"] = MAKE_STR("MCMStationID cannot be empty.");
		return response;
	}
	mcmData.setBasicContainer(mcBasicContainer);


    if(!request["MCManeuverAdviceContainer"].is_null() &&
        !request["MCManeuverAdviceContainer"]["MCManeuverAdvices"].is_null()) {

        auto adv_array = GET_ARR(request["MCManeuverAdviceContainer"], "MCManeuverAdvices");
        if(adv_array.empty()) {
            std::cerr << "[ERROR] MCManeuverAdvices array is empty." << std::endl;
            response["status"] = MAKE_STR("error");
            response["message"] = MAKE_STR("MCManeuverAdvices array cannot be empty.");
            return response;
        }

        std::vector<mcData::mcDataManeuverAdvice> parsed_advices;
        if(!extractManeuverAdvice(adv_array, parsed_advices)) {
            std::cerr << "[ERROR] Failed to parse MCManeuverAdvices." << std::endl;
            response["status"] = MAKE_STR("error");
            response["message"] = MAKE_STR("Failed to parse MCManeuverAdvices content.");
            return response;
        }

        mcData::mcAdviceContainer advice_container;
        advice_container.advices = parsed_advices;
        mcmData.setAdviceContainer(advice_container);

    } else if(!request["MCVehicleManeuverContainer"].is_null()) {
        mcData::mcManeuverContainer maneuver_container;

        if(!request["MCVehicleManeuverContainer"]["MCManeuverAdvices"].is_null()) {
            auto adv_array = GET_ARR(request["MCVehicleManeuverContainer"], "MCManeuverAdvices");
            if(!adv_array.empty()) {
                std::vector<mcData::mcDataManeuverAdvice> parsed_advices;
                if(!extractManeuverAdvice(adv_array, parsed_advices)) {
                    std::cerr << "[ERROR] Failed to parse MCManeuverAdvices in vehicle container." << std::endl;
                    response["status"] = MAKE_STR("error");
                    response["message"] = MAKE_STR("Failed to parse MCManeuverAdvices inside vehicle container.");
                    return response;
                }
                maneuver_container.advices.setData(parsed_advices);  // MCDataItem<vector>
            }
        }

        if(request["MCVehicleManeuverContainer"]["MCSubmaneuvers"].is_null()) {
            std::cerr << "[ERROR] MCSubmaneuvers is required in MCVehicleManeuverContainer." << std::endl;
            response["status"] = MAKE_STR("error");
            response["message"] = MAKE_STR("MCSubmaneuvers is required inside MCVehicleManeuverContainer.");
            return response;
        }

        auto sub_array = GET_ARR(request["MCVehicleManeuverContainer"], "MCSubmaneuvers");
        if(sub_array.empty()) {
            std::cerr << "[ERROR] MCSubmaneuvers array is empty." << std::endl;
            response["status"] = MAKE_STR("error");
            response["message"] = MAKE_STR("MCSubmaneuvers array cannot be empty.");
            return response;
        }

        if(!extractSubmaneuvers(sub_array, maneuver_container.submaneuvers)) {
            std::cerr << "[ERROR] Failed to parse MCSubmaneuvers in vehicle container." << std::endl;
            response["status"] = MAKE_STR("error");
            response["message"] = MAKE_STR("Failed to parse submaneuvers inside vehicle container.");
            return response;
        }

        mcmData.setManeuverContainer(maneuver_container);

    } else if(!request["MCResponseContainer"].is_null()) {
        mcData::mcResponseContainer resp_container;

        if(request["MCResponseContainer"]["MCResponse"].is_null()) {
            std::cerr << "[ERROR] MCResponse is required in MCResponseContainer." << std::endl;
            response["status"] = MAKE_STR("error");
            response["message"] = MAKE_STR("MCResponse value is missing.");
            return response;
        }
        resp_container.response = GET_NUM(request["MCResponseContainer"], "MCResponse");
		if(resp_container.response == 1) {
			if(!request["MCResponseContainer"]["MCDeclineReason"].is_null()) {
				resp_container.declineReason.setData(GET_NUM(request["MCResponseContainer"], "MCDeclineReason"));
			} else {
				response["status"] = MAKE_STR("error");
				response["message"] = MAKE_STR("A declined maneuver needs a MCDeclineReason.");
				return response;
			}
		}
        if (!request["MCResponseContainer"]["MCSubmaneuvers"].is_null()) {
            auto sub_array = GET_ARR(request["MCResponseContainer"], "MCSubmaneuvers");
            if(sub_array.empty()) {
                std::cerr << "[ERROR] MCSubmaneuvers array in MCResponseContainer is empty." << std::endl;
                response["status"] = MAKE_STR("error");
                response["message"] = MAKE_STR("MCSubmaneuvers array in MCResponseContainer cannot be empty.");
                return response;
            }
            std::vector<mcData::mcDataSubmaneuverDescription> parsed_subms;
            if(!extractSubmaneuvers(sub_array, parsed_subms)) {
                std::cerr << "[ERROR] Failed to parse MCSubmaneuvers in response container." << std::endl;
                response["status"] = MAKE_STR("error");
                response["message"] = MAKE_STR("Failed to parse submaneuvers in response container.");
                return response;
            }
            resp_container.submaneuvers.setData(parsed_subms);
        }

        mcmData.setResponseContainer(resp_container);

    } else if(!request["MCAcknowledgmentContainer"].is_null()) {
        if(request["MCAcknowledgmentContainer"]["MCAcknowledgmentType"].is_null()) {
            std::cerr << "[ERROR] MCAcknowledgmentType is required." << std::endl;
            response["status"] = MAKE_STR("error");
            response["message"] = MAKE_STR("MCAcknowledgmentType value is missing.");
            return response;
        }
        mcData::mcAcknowledgeContainer ack_container;
        ack_container.type = GET_NUM(request["MCAcknowledgmentContainer"], "MCAcknowledgmentType");
        mcmData.setAcknowledgmentContainer(ack_container);

    } else if(!request["MCTerminationContainer"].is_null()) {
        mcmData.setTerminationContainer(mcData::mcTerminationContainer{});

    } else {
        std::cerr << "[ERROR] No valid container found in request." << std::endl;
        response["status"] = MAKE_STR("error");
        response["message"] = MAKE_STR("No valid container found in request.");
        return response;
    }

    auto err = m_mc_service->generateAndEncodeMCM(mcmData);

	if(err != MCM_NO_ERROR) {
		// In case of creation error during the JSON parsing
		response["status"] = MAKE_STR("error");
		if (err == MCM_JSON_ERROR) response["message"] = MAKE_STR("Error during the JSON parsing");
		else if (err == MCM_CANNOT_SEND) response["message"] = MAKE_STR("Error during MCM sendind");
		else if (err == MCM_ASN1_UPER_ENC_ERROR) response["message"] = MAKE_STR("Error during ASN1 encoding");
		else response["message"] = MAKE_STR("Error during MCM allocation");
		return response;
	} else {
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
