/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 * Created by:
 *  Diego Gasco, Politecnico di Torino (diego.gasco@polito.it, diego.gasco99@gmail.com)
*/

#include "mcBasicService.h"
#include "asn_utils.h"
#include <cmath>

#include "asn_SEQUENCE_OF.h"
#include "err.h"

#define GET_NUM(json,key) (json[key].number_value())
#define GET_INT(json,key) (json[key].int_value())
#define GET_STR(json,key) (json[key].string_value())
#define GET_ARR(json, key) (json[key].array_items())

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

MCBasicService::~MCBasicService()=default;

MCBasicService::MCBasicService() {
  m_station_id = ULONG_MAX;
  m_stationtype = LONG_MAX;
  m_btp = NULL;

  // Setting a default value of m_T_CheckMCMGen_ms equal to 100 ms (i.e. T_GenMCMMin_ms)
  m_T_CheckMCMGen_ms=T_GenMCMMin_ms;

  m_T_GenMCM_ms=T_GenMCMMax_ms;

  lastMCMGen=-1;

  m_vehicle=true;

  // MCM generation interval for RSU ITS-Ss (default: 1 s)
  m_RSU_GenMCM_ms=T_GenMCMMax_ms;

  m_MCM_sent=0;
}

MCBasicService::MCBasicService(unsigned long fixed_stationid,long fixed_stationtype,VDPGPSClient* vdp, bool real_time, bool is_vehicle) {
  m_btp = NULL;

  // Setting a default value of m_T_CheckMCMGen_ms equal to 100 ms (i.e. T_GenMCMMin_ms)
  m_T_CheckMCMGen_ms=T_GenMCMMin_ms;

  m_T_GenMCM_ms=T_GenMCMMax_ms;

  lastMCMGen=-1;

  // MCM generation interval for RSU ITS-Ss (default: 1 s)
  m_RSU_GenMCM_ms=1000;

  m_MCM_sent=0;

  m_station_id = (StationId_t) fixed_stationid;
  m_stationtype = (StationType_t) fixed_stationtype;

  m_vdp=vdp;

  m_vehicle=is_vehicle;
}

void
MCBasicService::setStationProperties(unsigned long fixed_stationid,long fixed_stationtype,double latitude_deg,double longitude_deg) {
  m_station_id=fixed_stationid;
  m_stationtype=fixed_stationtype;
}

void
MCBasicService::setFixedPositionRSU(double latitude_deg, double longitude_deg) {
  m_vehicle = false;
  m_RSUlon = longitude_deg;
  m_RSUlat = latitude_deg;
  //High frequency RSU container
  m_protectedCommunicationsZonesRSU = asn1cpp::makeSeq(RSUContainerHighFrequency);
  auto protectedComm = asn1cpp::makeSeq(ProtectedCommunicationZone);
  asn1cpp::setField(protectedComm->protectedZoneType,ProtectedZoneType_permanentCenDsrcTolling);
  asn1cpp::setField(protectedComm->protectedZoneLatitude,Latitude_unavailable);
  asn1cpp::setField(protectedComm->protectedZoneLongitude,Longitude_unavailable);
  asn1cpp::sequenceof::pushList(m_protectedCommunicationsZonesRSU->protectedCommunicationZonesRSU,protectedComm);
}

void
MCBasicService::setStationID(unsigned long fixed_stationid) {
  m_station_id=fixed_stationid;
  m_btp->setStationID(fixed_stationid);
}

void
MCBasicService::setStationType(long fixed_stationtype) {
  m_stationtype=fixed_stationtype;
  m_btp->setStationType(fixed_stationtype);
}

std::tuple<asn1cpp::Seq<ListOfSubmanoeuvreDescriptionsContainer>, bool> extractSubmaneuverDescriptions(const json11::Json::array &submaneuvers_json) {
	auto list_subm = asn1cpp::makeSeq(ListOfSubmanoeuvreDescriptionsContainer);

	for (auto& subm_json : submaneuvers_json) {
		auto subm = asn1cpp::makeSeq(SubmanoeuvreDescription);

		// --- submanoeuvreID ---
		if (!subm_json["SubmanoeuvreID"].is_null()) {
			asn1cpp::setField(subm->submanoeuvreID, GET_NUM(subm_json, "SubmanoeuvreID"));
		} else {
			std::cerr << "SubmaneuverDescription needs SubmaneuverID" << std::endl;
			return {nullptr, false};
		}

		// --- temporalCharacteristics ---
		if (!subm_json["TemporalCharacteristics"].is_null()) {
			for (auto field : {"StartTime", "EndTime"}) {
				if (subm_json["TemporalCharacteristics"][field].is_null()) {
					std::cerr << std::string(field) + " in TemporalCharacteristics not found" << std::endl;
					return {nullptr, false};
				}
			}
			asn1cpp::setField(subm->temporalCharateristics.tRROccupancyStartTime, GET_NUM(subm_json["TemporalCharacteristics"], "StartTime"));
			asn1cpp::setField(subm->temporalCharateristics.tRROccupancyEndTime, GET_NUM(subm_json["TemporalCharacteristics"], "EndTime"));
		} else {
			std::cerr << "SubmaneuverDescription needs TemporalCharacteristics" << std::endl;
			return {nullptr, false};
		}

		// --- submanoeuvreStrategy (optional) ---
		if (!subm_json["SubmaneuverStrategy"].is_null()) {
			auto strategy = asn1cpp::makeSeq(SubmanoeuvreStrategy);

			int present_val = GET_NUM(subm_json["SubmaneuverStrategy"], "Strategy");
			asn1cpp::setField(strategy->present, static_cast<SubmanoeuvreStrategy_PR>(present_val));

			auto it = strategy_json_fields.find(present_val);
			if (it == strategy_json_fields.end()) {
				std::cerr << "Unknown strategy present value: " + std::to_string(present_val) << std::endl;;
				return {nullptr, false};
			}
			const char* json_field = it->second;
			asn1cpp::setField(strategy->choice, GET_NUM(subm_json["SubmaneuverStrategy"], json_field));
			asn1cpp::setField(subm->submanoeuvreStrategy, strategy);
		}

		// --- referenceTrajectory (optional) ---
		if (!subm_json["ReferenceTrajectory"].is_null()) {
			auto traj = asn1cpp::makeSeq(Trajectory);

			if (subm_json["ReferenceTrajectory"]["WayPointType"].is_null()) {
				std::cerr << "WayPointType in ReferenceTrajectory not found" << std::endl;
				return {nullptr, false};
			}
			asn1cpp::setField(traj->wayPointType, GET_NUM(subm_json["ReferenceTrajectory"], "WayPointType"));

			// --- wayPoints ---
			auto& wps_json = GET_ARR(subm_json["ReferenceTrajectory"], "WayPoints");
			for (auto& wp_json : wps_json) {
				auto wp = asn1cpp::makeSeq(PathPoint);

				for (auto field : {"DeltaLatitude", "DeltaLongitude", "DeltaAltitude"}) {
					if (wp_json[field].is_null()) {
						std::cerr << std::string(field) + " in WayPoint not found" << std::endl;
						return {nullptr, false};
					}
				}
				asn1cpp::setField(wp->pathPosition.deltaLatitude,  GET_NUM(wp_json, "DeltaLatitude"));
				asn1cpp::setField(wp->pathPosition.deltaLongitude, GET_NUM(wp_json, "DeltaLongitude"));
				asn1cpp::setField(wp->pathPosition.deltaAltitude,  GET_NUM(wp_json, "DeltaAltitude"));

				// --- pathDeltaTime (optional) ---
				if (!wp_json["PathDeltaTime"].is_null()) {
					asn1cpp::setField(wp->pathDeltaTime, GET_NUM(wp_json, "PathDeltaTime"));
				}

				asn1cpp::sequenceof::pushList(traj->wayPoints, wp);
			}

			// --- speed ---
			if (subm_json["ReferenceTrajectory"]["Speed"].is_null()) {
				std::cerr << "ReferenceTrajectory in SubmaneuverDescription needs Speeds" << std::endl;
				return {nullptr, false};
			}
			auto& speeds_json = GET_ARR(subm_json["ReferenceTrajectory"], "Speed");
			for (auto& sp_json : speeds_json) {
				auto sp = asn1cpp::makeSeq(Speed);

				for (auto field : {"SpeedValue", "SpeedConfidence"}) {
					if (sp_json[field].is_null()) {
						std::cerr << std::string(field) + " in Speed not found" << std::endl;
						return {nullptr, false};
					}
				}
				asn1cpp::setField(sp->speedValue,      GET_NUM(sp_json, "SpeedValue"));
				asn1cpp::setField(sp->speedConfidence, GET_NUM(sp_json, "SpeedConfidence"));
				asn1cpp::sequenceof::pushList(traj->speed, sp);
			}

			// --- headings for referenceTrajectory (optional) ---
			auto& headings_json = GET_ARR(subm_json["ReferenceTrajectory"], "Heading");
			for (auto& head_json : headings_json) {
				auto head = asn1cpp::makeSeq(Wgs84Angle);

				for (auto field : {"HeadingValue", "HeadingConfidence"}) {
					if (head_json[field].is_null()) {
						std::cerr << std::string(field) + " in Heading not found" << std::endl;
						return {nullptr, false};
					}
				}
				asn1cpp::setField(head->value,      GET_NUM(head_json, "HeadingValue"));
				asn1cpp::setField(head->confidence, GET_NUM(head_json, "HeadingConfidence"));
				asn1cpp::sequenceof::pushList(traj->headings, head);
			}

			// --- longitudePositions for referenceTrajectory (optional) ---
			auto& longitudes_json = GET_ARR(subm_json["ReferenceTrajectory"], "Longitude");
			for (auto& longi_json : longitudes_json) {
				auto longi = asn1cpp::makeSeq(Longitude);
				if (longi_json["LongitudeValue"].is_null()) {
					std::cerr << "LongitudeValue in Longitude not found" << std::endl;
					return {nullptr, false};
				}
				asn1cpp::setField(*longi, GET_NUM(longi_json, "LongitudeValue"));
				asn1cpp::sequenceof::pushList(traj->longitudePositions, longi);
			}

			// --- latitudePositions for referenceTrajectory (optional) ---
			auto& latitudes_json = GET_ARR(subm_json["ReferenceTrajectory"], "Latitude");
			for (auto& lati_json : latitudes_json) {
				auto lati = asn1cpp::makeSeq(Latitude);
				if (lati_json["LatitudeValue"].is_null()) {
					std::cerr << "LatitudeValue in Latitude not found" << std::endl;
					return {nullptr, false};
				}
				asn1cpp::setField(*lati, GET_NUM(lati_json, "LatitudeValue"));
				asn1cpp::sequenceof::pushList(traj->latitudePositions, lati);
			}

			// --- altitudePositions for referenceTrajectory (optional) ---
			auto& altitudes_json = GET_ARR(subm_json["ReferenceTrajectory"], "Altitude");
			for (auto& alti_json : altitudes_json) {
				auto alti = asn1cpp::makeSeq(Altitude);

				for (auto field : {"AltitudeValue", "AltitudeConfidence"}) {
					if (alti_json[field].is_null()) {
						std::cerr << std::string(field) + " in Altitude not found" << std::endl;
						return {nullptr, false};
					}
				}
				asn1cpp::setField(alti->altitudeValue,      GET_NUM(alti_json, "AltitudeValue"));
				asn1cpp::setField(alti->altitudeConfidence, GET_NUM(alti_json, "AltitudeConfidence"));
				asn1cpp::sequenceof::pushList(traj->altitudePositions, alti);
			}

			asn1cpp::setField(subm->referenceTrajectory, traj);
		}

		// --- targetRoadResourceIContainer (optional) ---
		if (!subm_json["TargetRoadResource"].is_null()) {
			auto trr = asn1cpp::makeSeq(TrrDescription);

			for (auto field : {"TrrType", "LaneCount", "TrrWidth", "TrrLength"}) {
				if (subm_json["TargetRoadResource"][field].is_null()) {
					std::cerr << std::string(field) + " in TargetRoadResource not found" << std::endl;
					return {nullptr, false};
				}
			}
			asn1cpp::setField(trr->trrType,    GET_NUM(subm_json["TargetRoadResource"], "TrrType"));
			asn1cpp::setField(trr->laneCount,  GET_NUM(subm_json["TargetRoadResource"], "LaneCount"));
			asn1cpp::setField(trr->trrWidth,   GET_NUM(subm_json["TargetRoadResource"], "TrrWidth"));
			asn1cpp::setField(trr->trrLength,  GET_NUM(subm_json["TargetRoadResource"], "TrrLength"));

			// --- startingLaneNumber (optional) ---
			if (!subm_json["TargetRoadResource"]["StartingLaneNumber"].is_null()) {
				asn1cpp::setField(trr->startingLaneNumber, GET_NUM(subm_json["TargetRoadResource"], "StartingLaneNumber"));
			}

			// --- endingLaneNumber (optional) ---
			if (!subm_json["TargetRoadResource"]["EndingLaneNumber"].is_null()) {
				asn1cpp::setField(trr->endingLaneNumber, GET_NUM(subm_json["TargetRoadResource"], "EndingLaneNumber"));
			}

			// --- waypoints for targetRoadResource (optional) ---
			auto& wps_json = GET_ARR(subm_json["TargetRoadResource"], "WayPoints");
			for (auto& wp_json : wps_json) {
				auto wp = asn1cpp::makeSeq(PathPoint);

				for (auto field : {"DeltaLatitude", "DeltaLongitude", "DeltaAltitude"}) {
					if (wp_json[field].is_null()) {
						std::cerr << std::string(field) + " in WayPoint not found"<< std::endl;
						return {nullptr, false};
					}
				}
				asn1cpp::setField(wp->pathPosition.deltaLatitude,  GET_NUM(wp_json, "DeltaLatitude"));
				asn1cpp::setField(wp->pathPosition.deltaLongitude, GET_NUM(wp_json, "DeltaLongitude"));
				asn1cpp::setField(wp->pathPosition.deltaAltitude,  GET_NUM(wp_json, "DeltaAltitude"));

				// --- pathDeltaTime (optional) ---
				if (!wp_json["PathDeltaTime"].is_null()) {
					asn1cpp::setField(wp->pathDeltaTime, GET_NUM(wp_json, "PathDeltaTime"));
				}

				asn1cpp::sequenceof::pushList(trr->waypoints, wp);
			}

			// --- heading for targetRoadResource (optional) ---
			auto& headings_json = GET_ARR(subm_json["TargetRoadResource"], "Heading");
			for (auto& head_json : headings_json) {
				auto head = asn1cpp::makeSeq(Wgs84Angle);

				for (auto field : {"HeadingValue", "HeadingConfidence"}) {
					if (head_json[field].is_null()) {
						std::cerr << std::string(field) + " in Heading not found" << std::endl;
						return {nullptr, false};
					}
				}
				asn1cpp::setField(head->value,      GET_NUM(head_json, "HeadingValue"));
				asn1cpp::setField(head->confidence, GET_NUM(head_json, "HeadingConfidence"));
				asn1cpp::sequenceof::pushList(trr->heading, head);
			}

			asn1cpp::setField(subm->targetRoadResourceIContainer, trr);
		}

		// --- kinematicsCharacteristics (optional) ---
		// For the moment the ASN indicates this field as nullptr

		asn1cpp::sequenceof::pushList(list_subm, subm);
	}
  return {list_subm, true};
}

MCBasicService_error_t
MCBasicService::generateAndEncodeMCM(MCSpecification* specification) {
  // Only one container must be activated for one message
  specification->checkContainers();
  auto request = specification->getRequest();
  VDPGPSClient::MCM_mandatory_data_t MCM_mandatory_data;

  BTPDataRequest_t dataRequest = {};

  /* Collect data for mandatory containers */
  auto MCM_message = asn1cpp::makeSeq(MCM);

  if(bool(MCM_message)==false) {
      return MCM_ALLOC_ERROR;
    }

  /* Fill the header */
  asn1cpp::setField(MCM_message->header.messageId, FIX_MCMID);
  asn1cpp::setField(MCM_message->header.protocolVersion, 1);
  asn1cpp::setField(MCM_message->header.stationId, m_station_id);

  /* Fill the basicContainer */

  /*
    * Compute the generationDeltaTime, "computed as the time corresponding to the
    * time of the reference position in the MCM, considered as time of the MCM generation.
    * The value of the generationDeltaTime shall be wrapped to 65 536. This value shall be set as the
    * remainder of the corresponding value of TimestampIts divided by 65 536 as below:
    * generationDeltaTime = TimestampIts mod 65 536"
  */
  asn1cpp::setField(MCM_message->payload.basicContainer.generationDeltaTime, compute_timestampIts () % 65536);

  asn1cpp::setField(MCM_message->payload.basicContainer.stationID, m_station_id);
  asn1cpp::setField(MCM_message->payload.basicContainer.itssRole, GET_NUM(request, "MCMITSRole"));
  if(m_vehicle==true) {
      asn1cpp::setField(MCM_message->payload.basicContainer.stationType, McmStationType_vehicle);
  } else {
      asn1cpp::setField(MCM_message->payload.basicContainer.stationType, McmStationType_roadsideUnit);
  }
  
  long type = GET_NUM(request, "MCMType");
  MCM_mandatory_data = m_vdp->getMCMMandatoryData();
  asn1cpp::setField(MCM_message->payload.basicContainer.position.latitude, MCM_mandatory_data.latitude);
  asn1cpp::setField(MCM_message->payload.basicContainer.position.longitude, MCM_mandatory_data.longitude);
  asn1cpp::setField(MCM_message->payload.basicContainer.position.altitude.altitudeValue, MCM_mandatory_data.altitude.getValue());
  asn1cpp::setField(MCM_message->payload.basicContainer.position.altitude.altitudeConfidence, MCM_mandatory_data.altitude.getConfidence());
  asn1cpp::setField(MCM_message->payload.basicContainer.position.positionConfidenceEllipse.semiMajorAxisLength, MCM_mandatory_data.posConfidenceEllipse.semiMajorConfidence);
  asn1cpp::setField(MCM_message->payload.basicContainer.position.positionConfidenceEllipse.semiMinorAxisLength, MCM_mandatory_data.posConfidenceEllipse.semiMinorConfidence);
  asn1cpp::setField(MCM_message->payload.basicContainer.position.positionConfidenceEllipse.semiMajorAxisOrientation, MCM_mandatory_data.posConfidenceEllipse.semiMajorOrientation);
  asn1cpp::setField(MCM_message->payload.basicContainer.mcmType, type);
  asn1cpp::setField(MCM_message->payload.basicContainer.manoeuvreId, GET_NUM(request, "MCMManeuverID"));
  asn1cpp::setField(MCM_message->payload.basicContainer.concept, GET_NUM(request, "MCMConcept"));
    
  // ManoeuvreCoordinationRational_t* rational = specification->create<ManoeuvreCoordinationRational_t>(asn_DEF_ManoeuvreCoordinationRational);
  auto rational = asn1cpp::makeSeq(ManoeuvreCoordinationRational);
  if (specification->getMCMConcept() == 0) {
      asn1cpp::setField(rational->present, ManoeuvreCoordinationRational_PR_manoeuvreCooperationGoal);
      asn1cpp::setField(rational->choice.manoeuvreCooperationGoal, GET_NUM(request, "MCMGoal"));
  } else {
      asn1cpp::setField(rational->present, ManoeuvreCoordinationRational_PR_manoeuvreCooperationCost);
      asn1cpp::setField(rational->choice.manoeuvreCooperationCost,  GET_NUM(request, "MCMCost"));
  }

  if (type == 4 || type == 7) {
      asn1cpp::setField (MCM_message->payload.basicContainer.executionStatus, specification->getMCMStatus());
  }

  if (specification->getManeuverContainer()) {
    // Select this choice
    asn1cpp::setField(MCM_message->payload.mcmContainer.present, McmContainer_PR_vehicleManoeuvreContainer);

    // Allocate + fill VehicleManoeuvreContainer
    auto &man = MCM_message->payload.mcmContainer.choice.vehicleManoeuvreContainer;
    asn1cpp::setField(man.vehicleCurrentStateContainer.vehicleHeading.value, MCM_mandatory_data.heading.getValue());
    asn1cpp::setField(man.vehicleCurrentStateContainer.vehicleHeading.confidence, MCM_mandatory_data.heading.getConfidence());
    asn1cpp::setField(man.vehicleCurrentStateContainer.vehicleSpeed.speedValue, MCM_mandatory_data.speed.getValue());
    asn1cpp::setField(man.vehicleCurrentStateContainer.vehicleSpeed.speedConfidence, MCM_mandatory_data.speed.getConfidence());
    asn1cpp::setField(man.vehicleCurrentStateContainer.vehicleSize.vehicleWidth, MCM_mandatory_data.VehicleWidth);
    asn1cpp::setField(man.vehicleCurrentStateContainer.vehicleSize.vehicleLenth.vehicleLengthValue, MCM_mandatory_data.VehicleLength.getValue());
    asn1cpp::setField(man.vehicleCurrentStateContainer.vehicleSize.vehicleLenth.vehicleLengthConfidenceIndication, MCM_mandatory_data.VehicleLength.getConfidence());
    asn1cpp::setField(man.vehicleCurrentStateContainer.vehicleSize.vehicleHeight, VehicleHeight_unavailable);
    asn1cpp::setField(man.vehicleCurrentStateContainer.vehicleSize.vehicleType, GET_NUM(request, "MCMVehicleType"));

    // Loop Submanoeuvres
    auto& submaneuvers_json = GET_ARR(specification->getRequest(), "MCMSubmaneuvers");
    std::tuple<asn1cpp::Seq<ListOfSubmanoeuvreDescriptionsContainer>, bool> ret = extractSubmaneuverDescriptions(submaneuvers_json);
    bool ret_bool = std::get<1>(ret);
    if (ret_bool) {
      asn1cpp::Seq<ListOfSubmanoeuvreDescriptionsContainer> subms = std::get<0>(ret);
      asn1cpp::setField(man.submaneuvres, subms);
    }

    // Loop ManoeuvreAdvice
    auto* mac = specification->create<ManoeuvreAdviceContainer_t>(asn_DEF_ManoeuvreAdviceContainer);
    for (auto* advice : specification->getManeuverAdviceList()) {
      specification->add(asn_DEF_ManoeuvreAdvice, mac, advice);
    }
    specification->setOptional(&man.manoeuvreAdvice, mac);
  } else if (specification->getAdviseContainer()) {
    asn1cpp::setField(MCM_message->payload.mcmContainer.present, McmContainer_PR_advisedManoeuvreContainer);
    auto &adv = MCM_message->payload.mcmContainer.choice.advisedManoeuvreContainer;
    for (auto* advice_ptr : specification->getManeuverAdviceList()) {
      specification->add(asn_DEF_ManoeuvreAdvice, &adv, advice_ptr);
    }
  } else if (specification->getAcknowledgmentContainer()) {
      asn1cpp::setField(MCM_message->payload.mcmContainer.present, McmContainer_PR_acknowledgmentContainer);
      asn1cpp::setField(MCM_message->payload.mcmContainer.choice.acknowledgmentContainer.acknowledgedType, McmType_acknowledgment);
      asn1cpp::setField(MCM_message->payload.mcmContainer.choice.acknowledgmentContainer.generationDeltaTime, compute_timestampIts () % 65536);
  } else if (specification->getResponseContainer()) {
      asn1cpp::setField(MCM_message->payload.mcmContainer.present, McmContainer_PR_responseContainer);
      if (specification->getMCMResponse() == 0) {
          asn1cpp::setField(MCM_message->payload.mcmContainer.choice.responseContainer.manouevreResponse, ManouevreResponse_accept);
      } else {
          asn1cpp::setField(MCM_message->payload.mcmContainer.choice.responseContainer.manouevreResponse, ManouevreResponse_decline);
          asn1cpp::setField(MCM_message->payload.mcmContainer.choice.responseContainer.declineReason, specification->getMCMResponseDeclineReason());
      }
      
      // Loop Submanoeuvres
      for (auto* subm : specification->getSubmanoeuvreDescriptionList()) {
        specification->add(asn_DEF_Submanoeuvre, &MCM_message->payload.mcmContainer.choice.responseContainer.submaneuvres, subm);
      }
  }
  else if (specification->getTerminatorContainer()) {
      asn1cpp::setField(MCM_message->payload.mcmContainer.present, McmContainer_PR_terminationContainer);
  } else {
      std::cerr << "[ERROR] Fatal error! Cannot create containers for the MCM dissemination" << std::endl;
      terminateDissemination();
      return MCM_CANNOT_SEND;
  }

  std::string encode_result = asn1cpp::uper::encode(MCM_message);

  if(encode_result.size()<1) {
    return MCM_ASN1_UPER_ENC_ERROR;
  }

  dataRequest.BTPType = BTP_B;
  dataRequest.destPort = MC_PORT;
  dataRequest.destPInfo = 0;
  dataRequest.GNType = TSB;
  dataRequest.GNCommProfile = UNSPECIFIED;
  dataRequest.GNRepInt =0;
  dataRequest.GNMaxRepInt=0;
  dataRequest.GNMaxLife = 1;
  dataRequest.GNMaxHL = 1;
  dataRequest.GNTraClass = 0x02;
  dataRequest.lenght = encode_result.size();
  /* Create the packet and the BTP header */
  packetBuffer pktbuf(encode_result.c_str(),static_cast<unsigned int>(encode_result.size()));
  dataRequest.data = pktbuf;
  std::tuple<GNDataConfirm_t, MessageId_t> status = m_btp->sendBTP(dataRequest, m_priority, MessageId_cam);
  GNDataConfirm_t dataConfirm = std::get<0>(status);
  MessageId_t message_id = std::get<1>(status);
  /* Update the CAM statistics */
  if(m_met_sup_ptr!=nullptr && dataConfirm == ACCEPTED) {
    if (message_id == MessageId_cam) m_MCM_sent++;
    m_met_sup_ptr->signalSentPacket(message_id);
  }

  if (dataConfirm == ACCEPTED) return MCM_NO_ERROR;
  else return MCM_CANNOT_SEND;
}

uint64_t
MCBasicService::terminateDissemination() {
  if(m_terminateFlag==false) {
    m_terminateFlag=true;
  }

  return m_MCM_sent;
}
