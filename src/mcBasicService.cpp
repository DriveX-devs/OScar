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

enum Container {
    NotPresent,
    VehicleManeuverContainer,
    ManeuverAdviseContainer,
    AcknowledgmentContainer,
    ResponseContainer,
    TerminationContainer,
  };

std::unordered_map<std::string, Container> containers_mapping = {
	{"VehicleManeuverContainer", Container::VehicleManeuverContainer},
	{"ManeuverAdviseContainer", Container::ManeuverAdviseContainer},
	{"AcknowledgmentContainer", Container::AcknowledgmentContainer},
	{"ResponseContainer", Container::ResponseContainer},
	{"TerminationContainer", Container::TerminationContainer}
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
          std::cerr << "Unknown strategy present value: " + std::to_string(present_val) << std::endl;
          return {nullptr, false};
      }

      const char* json_field = it->second;
      double val = GET_NUM(subm_json["SubmaneuverStrategy"], json_field);

      switch (static_cast<SubmanoeuvreStrategy_PR>(present_val)) {
          case SubmanoeuvreStrategy_PR_undefined:
              asn1cpp::setField(strategy->choice.undefined, (long)val);
              break;
          case SubmanoeuvreStrategy_PR_transitToHumanDrivenMode:
              asn1cpp::setField(strategy->choice.transitToHumanDrivenMode, (long)val);
              break;
          case SubmanoeuvreStrategy_PR_transitToAutomatedDrivingMode:
              asn1cpp::setField(strategy->choice.transitToAutomatedDrivingMode, (long)val);
              break;
          case SubmanoeuvreStrategy_PR_driveStraight:
              asn1cpp::setField(strategy->choice.driveStraight, (long)val);
              break;
          case SubmanoeuvreStrategy_PR_turnLeft:
              asn1cpp::setField(strategy->choice.turnLeft, (long)val);
              break;
          case SubmanoeuvreStrategy_PR_turnRight:
              asn1cpp::setField(strategy->choice.turnRight, (long)val);
              break;
          case SubmanoeuvreStrategy_PR_uTurn:
              asn1cpp::setField(strategy->choice.uTurn, (long)val);
              break;
          case SubmanoeuvreStrategy_PR_moveBackward:
              asn1cpp::setField(strategy->choice.moveBackward, (long)val);
              break;
          case SubmanoeuvreStrategy_PR_overtake:
              asn1cpp::setField(strategy->choice.overtake, (long)val);
              break;
          case SubmanoeuvreStrategy_PR_accelerate:
              asn1cpp::setField(strategy->choice.accelerate, (long)val);
              break;
          case SubmanoeuvreStrategy_PR_slowDown:
              asn1cpp::setField(strategy->choice.slowDown, (long)val);
              break;
          case SubmanoeuvreStrategy_PR_stop:
              asn1cpp::setField(strategy->choice.stop, (long)val);
              break;
          case SubmanoeuvreStrategy_PR_goToLeftLane:
              asn1cpp::setField(strategy->choice.goToLeftLane, (long)val);
              break;
          case SubmanoeuvreStrategy_PR_goToRightLane:
              asn1cpp::setField(strategy->choice.goToRightLane, (long)val);
              break;
          case SubmanoeuvreStrategy_PR_getOnHighway:
              asn1cpp::setField(strategy->choice.getOnHighway, (long)val);
              break;
          case SubmanoeuvreStrategy_PR_exitHighway:
              asn1cpp::setField(strategy->choice.exitHighway, (long)val);
              break;
          case SubmanoeuvreStrategy_PR_takeTollingLane:
              asn1cpp::setField(strategy->choice.takeTollingLane, (long)val);
              break;
          case SubmanoeuvreStrategy_PR_stopAndWait:
              asn1cpp::setField(strategy->choice.stopAndWait, (long)val);
              break;
          case SubmanoeuvreStrategy_PR_emergencyBrakeAndStop:
              asn1cpp::setField(strategy->choice.emergencyBrakeAndStop, (long)val);
              break;
          case SubmanoeuvreStrategy_PR_resetStopAndRestartMoving:
              asn1cpp::setField(strategy->choice.resetStopAndRestartMoving, (long)val);
              break;
          case SubmanoeuvreStrategy_PR_stayInLane:
              asn1cpp::setField(strategy->choice.stayInLane, (long)val);
              break;
          case SubmanoeuvreStrategy_PR_resetStayInLane:
              asn1cpp::setField(strategy->choice.resetStayInLane, (long)val);
              break;
          case SubmanoeuvreStrategy_PR_stayAway:
              asn1cpp::setField(strategy->choice.stayAway, (long)val);
              break;
          case SubmanoeuvreStrategy_PR_resetStayAway:
              asn1cpp::setField(strategy->choice.resetStayAway, (long)val);
              break;
          case SubmanoeuvreStrategy_PR_followMe:
              asn1cpp::setField(strategy->choice.followMe, (long)val);
              break;
          case SubmanoeuvreStrategy_PR_existingGroup:
              asn1cpp::setField(strategy->choice.existingGroup, (long)val);
              break;
          case SubmanoeuvreStrategy_PR_temporarilyDisbandAnExistingGroup:
              asn1cpp::setField(strategy->choice.temporarilyDisbandAnExistingGroup, (long)val);
              break;
          case SubmanoeuvreStrategy_PR_constituteATemporarilyGroup:
              asn1cpp::setField(strategy->choice.constituteATemporarilyGroup, (long)val);
              break;
          case SubmanoeuvreStrategy_PR_disbandATemporarilyGroup:
              asn1cpp::setField(strategy->choice.disbandATemporarilyGroup, (long)val);
              break;
          default:
              std::cerr << "Unhandled strategy present value: " + std::to_string(present_val) << std::endl;
              return {nullptr, false};
      }
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
				asn1cpp::setField(sp->speedValue, GET_NUM(sp_json, "SpeedValue"));
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
				asn1cpp::setField(head->value, GET_NUM(head_json, "HeadingValue"));
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
				asn1cpp::sequenceof::pushList(traj->longitudePositions, (long) GET_NUM(longi_json, "LongitudeValue"));
			}

			// --- latitudePositions for referenceTrajectory (optional) ---
			auto& latitudes_json = GET_ARR(subm_json["ReferenceTrajectory"], "Latitude");
			for (auto& lati_json : latitudes_json) {
				auto lati = asn1cpp::makeSeq(Latitude);
				if (lati_json["LatitudeValue"].is_null()) {
					std::cerr << "LatitudeValue in Latitude not found" << std::endl;
					return {nullptr, false};
				}
				asn1cpp::sequenceof::pushList(traj->latitudePositions, (long) GET_NUM(lati_json, "LatitudeValue"));
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
				asn1cpp::setField(alti->altitudeValue, GET_NUM(alti_json, "AltitudeValue"));
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
			asn1cpp::setField(trr->trrType, GET_NUM(subm_json["TargetRoadResource"], "TrrType"));
			asn1cpp::setField(trr->laneCount, GET_NUM(subm_json["TargetRoadResource"], "LaneCount"));
			asn1cpp::setField(trr->trrWidth, GET_NUM(subm_json["TargetRoadResource"], "TrrWidth"));
			asn1cpp::setField(trr->trrLength, GET_NUM(subm_json["TargetRoadResource"], "TrrLength"));

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
				asn1cpp::setField(wp->pathPosition.deltaLatitude, GET_NUM(wp_json, "DeltaLatitude"));
				asn1cpp::setField(wp->pathPosition.deltaLongitude, GET_NUM(wp_json, "DeltaLongitude"));
				asn1cpp::setField(wp->pathPosition.deltaAltitude, GET_NUM(wp_json, "DeltaAltitude"));

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
				asn1cpp::setField(head->value, GET_NUM(head_json, "HeadingValue"));
				asn1cpp::setField(head->confidence, GET_NUM(head_json, "HeadingConfidence"));
				asn1cpp::sequenceof::pushList(trr->heading, head);
			}

			asn1cpp::setField(subm->targetRoadResourceIContainer, trr);
		}

		// --- kinematicsCharacteristics (optional) ---
		// For the moment the ASN indicates this field as nullptr

		asn1cpp::sequenceof::pushList(*list_subm, subm);
	}
  return {list_subm, true};
}

std::tuple<asn1cpp::Seq<ManoeuvreAdviceContainer>, bool> extractManeuverAdvice(const json11::Json::array &advices_json) {
	auto list_adv = asn1cpp::makeSeq(ManoeuvreAdviceContainer);

	for (auto& adv_json : advices_json) {
		auto adv = asn1cpp::makeSeq(ManoeuvreAdvice);

		// --- executantID (mandatory) ---
		if (!adv_json["ExecutantID"].is_null()) {
			asn1cpp::setField(adv->executantID, GET_NUM(adv_json, "ExecutantID"));
		} else {
			std::cerr << "ManoeuvreAdvice needs ExecutantID" << std::endl;
			return {nullptr, false};
		}

		// --- currentStateAdvisedChange (optional) ---
		if (!adv_json["CurrentStateAdvisedChange"].is_null()) {
			auto csac = asn1cpp::makeSeq(CurrentStateAdvisedChange);

			int present_val = GET_NUM(adv_json["CurrentStateAdvisedChange"], "Present");
			asn1cpp::setField(csac->present, static_cast<CurrentStateAdvisedChange_PR>(present_val));

			asn1cpp::setField(adv->currentStateAdvisedChange, csac);
		}

		// --- submaneuvres (mandatory) ---
		if (adv_json["Submaneuvres"].is_null()) {
			std::cerr << "ManoeuvreAdvice needs Submaneuvres" << std::endl;
			return {nullptr, false};
		}

		auto& subms_json = GET_ARR(adv_json, "Submaneuvres");
		for (auto& subm_json : subms_json) {
			auto subm = asn1cpp::makeSeq(Submanoeuvre);

			// --- submanoeuvreId (mandatory) ---
			if (!subm_json["SubmanoeuvreId"].is_null()) {
				asn1cpp::setField(subm->submanoeuvreId, GET_NUM(subm_json, "SubmanoeuvreId"));
			} else {
				std::cerr << "Submanoeuvre needs SubmanoeuvreId" << std::endl;
				return {nullptr, false};
			}

			// --- advisedTrajectory (optional) ---
			if (!subm_json["AdvisedTrajectory"].is_null()) {
				auto traj = asn1cpp::makeSeq(Trajectory);

				if (subm_json["AdvisedTrajectory"]["WayPointType"].is_null()) {
					std::cerr << "WayPointType in AdvisedTrajectory not found" << std::endl;
					return {nullptr, false};
				}
				asn1cpp::setField(traj->wayPointType, GET_NUM(subm_json["AdvisedTrajectory"], "WayPointType"));

				// --- wayPoints ---
				auto& wps_json = GET_ARR(subm_json["AdvisedTrajectory"], "WayPoints");
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
				if (subm_json["AdvisedTrajectory"]["Speed"].is_null()) {
					std::cerr << "AdvisedTrajectory in Submanoeuvre needs Speeds" << std::endl;
					return {nullptr, false};
				}
				auto& speeds_json = GET_ARR(subm_json["AdvisedTrajectory"], "Speed");
				for (auto& sp_json : speeds_json) {
					auto sp = asn1cpp::makeSeq(Speed);

					for (auto field : {"SpeedValue", "SpeedConfidence"}) {
						if (sp_json[field].is_null()) {
							std::cerr << std::string(field) + " in Speed not found" << std::endl;
							return {nullptr, false};
						}
					}
					asn1cpp::setField(sp->speedValue, GET_NUM(sp_json, "SpeedValue"));
					asn1cpp::setField(sp->speedConfidence, GET_NUM(sp_json, "SpeedConfidence"));
					asn1cpp::sequenceof::pushList(traj->speed, sp);
				}

				// --- headings (optional) ---
				auto& headings_json = GET_ARR(subm_json["AdvisedTrajectory"], "Heading");
				for (auto& head_json : headings_json) {
					auto head = asn1cpp::makeSeq(Wgs84Angle);

					for (auto field : {"HeadingValue", "HeadingConfidence"}) {
						if (head_json[field].is_null()) {
							std::cerr << std::string(field) + " in Heading not found" << std::endl;
							return {nullptr, false};
						}
					}
					asn1cpp::setField(head->value, GET_NUM(head_json, "HeadingValue"));
					asn1cpp::setField(head->confidence, GET_NUM(head_json, "HeadingConfidence"));
					asn1cpp::sequenceof::pushList(traj->headings, head);
				}

				// --- longitudePositions (optional) ---
				auto& longitudes_json = GET_ARR(subm_json["AdvisedTrajectory"], "Longitude");
				for (auto& longi_json : longitudes_json) {
					auto longi = asn1cpp::makeSeq(Longitude);
					if (longi_json["LongitudeValue"].is_null()) {
						std::cerr << "LongitudeValue in Longitude not found" << std::endl;
						return {nullptr, false};
					}
					// asn1cpp::setField(*longi, GET_NUM(longi_json, "LongitudeValue"));
					asn1cpp::sequenceof::pushList(traj->longitudePositions, GET_NUM(longi_json, "LongitudeValue"));
				}

				// --- latitudePositions (optional) ---
				auto& latitudes_json = GET_ARR(subm_json["AdvisedTrajectory"], "Latitude");
				for (auto& lati_json : latitudes_json) {
					auto lati = asn1cpp::makeSeq(Latitude);
					if (lati_json["LatitudeValue"].is_null()) {
						std::cerr << "LatitudeValue in Latitude not found" << std::endl;
						return {nullptr, false};
					}
					//asn1cpp::setField(*lati, GET_NUM(lati_json, "LatitudeValue"));
					asn1cpp::sequenceof::pushList(traj->latitudePositions, GET_NUM(lati_json, "LatitudeValue"));
				}

				// --- altitudePositions (optional) ---
				auto& altitudes_json = GET_ARR(subm_json["AdvisedTrajectory"], "Altitude");
				for (auto& alti_json : altitudes_json) {
					auto alti = asn1cpp::makeSeq(Altitude);

					for (auto field : {"AltitudeValue", "AltitudeConfidence"}) {
						if (alti_json[field].is_null()) {
							std::cerr << std::string(field) + " in Altitude not found" << std::endl;
							return {nullptr, false};
						}
					}
					asn1cpp::setField(alti->altitudeValue, GET_NUM(alti_json, "AltitudeValue"));
					asn1cpp::setField(alti->altitudeConfidence, GET_NUM(alti_json, "AltitudeConfidence"));
					asn1cpp::sequenceof::pushList(traj->altitudePositions, alti);
				}

				asn1cpp::setField(subm->advisedTrajectory, traj);
			}

			// --- advisedTargetRoadResource (optional) ---
      if (!subm_json["AdvisedTargetRoadResource"].is_null()) {
          auto atrr = asn1cpp::makeSeq(AdvisedTrrContainer);

          // --- trrDescription (mandatory) ---
          for (auto field : {"TrrType", "LaneCount", "TrrWidth", "TrrLength"}) {
              if (subm_json["AdvisedTargetRoadResource"][field].is_null()) {
                  std::cerr << std::string(field) + " in AdvisedTargetRoadResource not found" << std::endl;
                  return {nullptr, false};
              }
          }
          asn1cpp::setField(atrr->trrDescription.trrType, GET_NUM(subm_json["AdvisedTargetRoadResource"], "TrrType"));
          asn1cpp::setField(atrr->trrDescription.laneCount, GET_NUM(subm_json["AdvisedTargetRoadResource"], "LaneCount"));
          asn1cpp::setField(atrr->trrDescription.trrWidth, GET_NUM(subm_json["AdvisedTargetRoadResource"], "TrrWidth"));
          asn1cpp::setField(atrr->trrDescription.trrLength, GET_NUM(subm_json["AdvisedTargetRoadResource"], "TrrLength"));

          if (!subm_json["AdvisedTargetRoadResource"]["StartingLaneNumber"].is_null()) {
              asn1cpp::setField(atrr->trrDescription.startingLaneNumber, GET_NUM(subm_json["AdvisedTargetRoadResource"], "StartingLaneNumber"));
          }
          if (!subm_json["AdvisedTargetRoadResource"]["EndingLaneNumber"].is_null()) {
              asn1cpp::setField(atrr->trrDescription.endingLaneNumber, GET_NUM(subm_json["AdvisedTargetRoadResource"], "EndingLaneNumber"));
          }

          // --- waypoints (optional) ---
          auto& wps_json = GET_ARR(subm_json["AdvisedTargetRoadResource"], "WayPoints");
          for (auto& wp_json : wps_json) {
              auto wp = asn1cpp::makeSeq(PathPoint);

              for (auto field : {"DeltaLatitude", "DeltaLongitude", "DeltaAltitude"}) {
                  if (wp_json[field].is_null()) {
                      std::cerr << std::string(field) + " in WayPoint not found" << std::endl;
                      return {nullptr, false};
                  }
              }
              asn1cpp::setField(wp->pathPosition.deltaLatitude, GET_NUM(wp_json, "DeltaLatitude"));
              asn1cpp::setField(wp->pathPosition.deltaLongitude, GET_NUM(wp_json, "DeltaLongitude"));
              asn1cpp::setField(wp->pathPosition.deltaAltitude, GET_NUM(wp_json, "DeltaAltitude"));

              if (!wp_json["PathDeltaTime"].is_null()) {
                  asn1cpp::setField(wp->pathDeltaTime, GET_NUM(wp_json, "PathDeltaTime"));
              }
              asn1cpp::sequenceof::pushList(atrr->trrDescription.waypoints, wp);
          }

          // --- heading (optional) ---
          auto& headings_json = GET_ARR(subm_json["AdvisedTargetRoadResource"], "Heading");
          for (auto& head_json : headings_json) {
              auto head = asn1cpp::makeSeq(Wgs84Angle);

              for (auto field : {"HeadingValue", "HeadingConfidence"}) {
                  if (head_json[field].is_null()) {
                      std::cerr << std::string(field) + " in Heading not found" << std::endl;
                      return {nullptr, false};
                  }
              }
              asn1cpp::setField(head->value, GET_NUM(head_json, "HeadingValue"));
              asn1cpp::setField(head->confidence, GET_NUM(head_json, "HeadingConfidence"));
              asn1cpp::sequenceof::pushList(atrr->trrDescription.heading, head);
          }

          // --- temporalCharacteristics (mandatory) ---
          for (auto field : {"StartTime", "EndTime"}) {
              if (subm_json["AdvisedTargetRoadResource"][field].is_null()) {
                  std::cerr << std::string(field) + " in AdvisedTargetRoadResource TemporalCharacteristics not found" << std::endl;
                  return {nullptr, false};
              }
          }
          asn1cpp::setField(atrr->temporalCharacteristics.tRROccupancyStartTime, GET_NUM(subm_json["AdvisedTargetRoadResource"], "StartTime"));
          asn1cpp::setField(atrr->temporalCharacteristics.tRROccupancyEndTime, GET_NUM(subm_json["AdvisedTargetRoadResource"], "EndTime"));

          // --- kinematicsCharacteristics (optional, currently NULL type) ---
          // Skipped as KinematicsCharacteristics_t is defined as NULL_t

          asn1cpp::setField(subm->advisedTargetRoadResource, atrr);
      }

			asn1cpp::sequenceof::pushList(adv->submaneuvres, subm);
		}

		asn1cpp::sequenceof::pushList(*list_adv, adv);
	}

	return {list_adv, true};
}

std::tuple<asn1cpp::Seq<ListOfSubmanoeuvreDescriptionsContainer>, bool>
convertSubmaneuversToAsn1(const std::vector<mcData::MCSubmaneuvers>& native_subms) {
    auto asn_list = asn1cpp::makeSeq(ListOfSubmanoeuvreDescriptionsContainer);

    for (const auto& native_subm : native_subms) {
        auto asn_subm = asn1cpp::makeSeq(SubmanoeuvreDescription);

        // --- submanoeuvreID ---
        asn1cpp::setField(asn_subm->submanoeuvreID, native_subm.submanoeuvreId);

        // --- temporalCharacteristics ---
        asn1cpp::setField(asn_subm->temporalCharateristics.tRROccupancyStartTime,
                          native_subm.targetRoadResource.getData().temporalCharacteristics.tRROccupancyStartTime);
        asn1cpp::setField(asn_subm->temporalCharateristics.tRROccupancyEndTime,
                          native_subm.targetRoadResource.getData().temporalCharacteristics.tRROccupancyEndTime);

        // --- submanoeuvreStrategy (optional) ---
        if (native_subm.submanoeuvreStrategy.isAvailable()) {
            const auto& native_strategy = native_subm.submanoeuvreStrategy.getData();
            auto strategy = asn1cpp::makeSeq(SubmanoeuvreStrategy);

            asn1cpp::setField(strategy->present,
                              static_cast<SubmanoeuvreStrategy_PR>(native_strategy.present));

            switch (static_cast<SubmanoeuvreStrategy_PR>(native_strategy.present)) {
                case SubmanoeuvreStrategy_PR_undefined:
                    asn1cpp::setField(strategy->choice.undefined, native_strategy.value);
                    break;
                case SubmanoeuvreStrategy_PR_transitToHumanDrivenMode:
                    asn1cpp::setField(strategy->choice.transitToHumanDrivenMode, native_strategy.value);
                    break;
                case SubmanoeuvreStrategy_PR_transitToAutomatedDrivingMode:
                    asn1cpp::setField(strategy->choice.transitToAutomatedDrivingMode, native_strategy.value);
                    break;
                case SubmanoeuvreStrategy_PR_driveStraight:
                    asn1cpp::setField(strategy->choice.driveStraight, native_strategy.value);
                    break;
                case SubmanoeuvreStrategy_PR_turnLeft:
                    asn1cpp::setField(strategy->choice.turnLeft, native_strategy.value);
                    break;
                case SubmanoeuvreStrategy_PR_turnRight:
                    asn1cpp::setField(strategy->choice.turnRight, native_strategy.value);
                    break;
                case SubmanoeuvreStrategy_PR_uTurn:
                    asn1cpp::setField(strategy->choice.uTurn, native_strategy.value);
                    break;
                case SubmanoeuvreStrategy_PR_moveBackward:
                    asn1cpp::setField(strategy->choice.moveBackward, native_strategy.value);
                    break;
                case SubmanoeuvreStrategy_PR_overtake:
                    asn1cpp::setField(strategy->choice.overtake, native_strategy.value);
                    break;
                case SubmanoeuvreStrategy_PR_accelerate:
                    asn1cpp::setField(strategy->choice.accelerate, native_strategy.value);
                    break;
                case SubmanoeuvreStrategy_PR_slowDown:
                    asn1cpp::setField(strategy->choice.slowDown, native_strategy.value);
                    break;
                case SubmanoeuvreStrategy_PR_stop:
                    asn1cpp::setField(strategy->choice.stop, native_strategy.value);
                    break;
                case SubmanoeuvreStrategy_PR_goToLeftLane:
                    asn1cpp::setField(strategy->choice.goToLeftLane, native_strategy.value);
                    break;
                case SubmanoeuvreStrategy_PR_goToRightLane:
                    asn1cpp::setField(strategy->choice.goToRightLane, native_strategy.value);
                    break;
                case SubmanoeuvreStrategy_PR_getOnHighway:
                    asn1cpp::setField(strategy->choice.getOnHighway, native_strategy.value);
                    break;
                case SubmanoeuvreStrategy_PR_exitHighway:
                    asn1cpp::setField(strategy->choice.exitHighway, native_strategy.value);
                    break;
                case SubmanoeuvreStrategy_PR_takeTollingLane:
                    asn1cpp::setField(strategy->choice.takeTollingLane, native_strategy.value);
                    break;
                case SubmanoeuvreStrategy_PR_stopAndWait:
                    asn1cpp::setField(strategy->choice.stopAndWait, native_strategy.value);
                    break;
                case SubmanoeuvreStrategy_PR_emergencyBrakeAndStop:
                    asn1cpp::setField(strategy->choice.emergencyBrakeAndStop, native_strategy.value);
                    break;
                case SubmanoeuvreStrategy_PR_resetStopAndRestartMoving:
                    asn1cpp::setField(strategy->choice.resetStopAndRestartMoving, native_strategy.value);
                    break;
                case SubmanoeuvreStrategy_PR_stayInLane:
                    asn1cpp::setField(strategy->choice.stayInLane, native_strategy.value);
                    break;
                case SubmanoeuvreStrategy_PR_resetStayInLane:
                    asn1cpp::setField(strategy->choice.resetStayInLane, native_strategy.value);
                    break;
                case SubmanoeuvreStrategy_PR_stayAway:
                    asn1cpp::setField(strategy->choice.stayAway, native_strategy.value);
                    break;
                case SubmanoeuvreStrategy_PR_resetStayAway:
                    asn1cpp::setField(strategy->choice.resetStayAway, native_strategy.value);
                    break;
                case SubmanoeuvreStrategy_PR_followMe:
                    asn1cpp::setField(strategy->choice.followMe, native_strategy.value);
                    break;
                case SubmanoeuvreStrategy_PR_existingGroup:
                    asn1cpp::setField(strategy->choice.existingGroup, native_strategy.value);
                    break;
                case SubmanoeuvreStrategy_PR_temporarilyDisbandAnExistingGroup:
                    asn1cpp::setField(strategy->choice.temporarilyDisbandAnExistingGroup, native_strategy.value);
                    break;
                case SubmanoeuvreStrategy_PR_constituteATemporarilyGroup:
                    asn1cpp::setField(strategy->choice.constituteATemporarilyGroup, native_strategy.value);
                    break;
                case SubmanoeuvreStrategy_PR_disbandATemporarilyGroup:
                    asn1cpp::setField(strategy->choice.disbandATemporarilyGroup, native_strategy.value);
                    break;
                default:
                    std::cerr << "Unhandled strategy present value: "
                              << std::to_string(native_strategy.present) << std::endl;
                    return {nullptr, false};
            }
            asn1cpp::setField(asn_subm->submanoeuvreStrategy, strategy);
        }

        // --- referenceTrajectory (optional) ---
        if (native_subm.referenceTrajectory.isAvailable()) {
            const auto& native_traj = native_subm.referenceTrajectory.getData();
            auto traj = asn1cpp::makeSeq(Trajectory);

            asn1cpp::setField(traj->wayPointType, native_traj.wayPointType);

            for (const auto& wp : native_traj.wayPoints) {
                auto asn_wp = asn1cpp::makeSeq(PathPoint);
                asn1cpp::setField(asn_wp->pathPosition.deltaLatitude,  wp.deltaLatitude);
                asn1cpp::setField(asn_wp->pathPosition.deltaLongitude, wp.deltaLongitude);
                asn1cpp::setField(asn_wp->pathPosition.deltaAltitude,  wp.deltaAltitude);
                if (wp.pathDeltaTime.isAvailable()) {
                    asn1cpp::setField(asn_wp->pathDeltaTime, wp.pathDeltaTime.getData());
                }
                asn1cpp::sequenceof::pushList(traj->wayPoints, asn_wp);
            }

            for (const auto& sp : native_traj.speed) {
                auto asn_sp = asn1cpp::makeSeq(Speed);
                asn1cpp::setField(asn_sp->speedValue,      sp.getValue());
                asn1cpp::setField(asn_sp->speedConfidence, sp.getConfidence());
                asn1cpp::sequenceof::pushList(traj->speed, asn_sp);
            }

            for (const auto& hd : native_traj.headings) {
                auto asn_hd = asn1cpp::makeSeq(Wgs84Angle);
                asn1cpp::setField(asn_hd->value,      hd.getValue());
                asn1cpp::setField(asn_hd->confidence, hd.getConfidence());
                asn1cpp::sequenceof::pushList(traj->headings, asn_hd);
            }

            for (const auto& lon : native_traj.longitudePositions) {
                asn1cpp::sequenceof::pushList(traj->longitudePositions, (long)lon);
            }

            for (const auto& lat : native_traj.latitudePositions) {
                asn1cpp::sequenceof::pushList(traj->latitudePositions, (long)lat);
            }

            for (const auto& alt : native_traj.altitudePositions) {
                auto asn_alt = asn1cpp::makeSeq(Altitude);
                asn1cpp::setField(asn_alt->altitudeValue,      alt.getValue());
                asn1cpp::setField(asn_alt->altitudeConfidence, alt.getConfidence());
                asn1cpp::sequenceof::pushList(traj->altitudePositions, asn_alt);
            }

            asn1cpp::setField(asn_subm->referenceTrajectory, traj);
        }

        // --- targetRoadResourceIContainer (optional) ---
        if (native_subm.targetRoadResource.isAvailable()) {
            const auto& native_trr = native_subm.targetRoadResource.getData();
            auto trr = asn1cpp::makeSeq(TrrDescription);

            asn1cpp::setField(trr->trrType,   native_trr.trrDescription.trrType);
            asn1cpp::setField(trr->laneCount, native_trr.trrDescription);
            asn1cpp::setField(trr->trrWidth,  native_trr.trrDescription);
            asn1cpp::setField(trr->trrLength, native_trr.trrDescription);

            if (native_trr.trrDescription.startingLaneNumber.isAvailable()) {
                asn1cpp::setField(trr->startingLaneNumber, native_trr.trrDescription.startingLaneNumber.getData());
            }
            if (native_trr.trrDescription.endingLaneNumber.isAvailable()) {
                asn1cpp::setField(trr->endingLaneNumber, native_trr.trrDescription.endingLaneNumber.getData());
            }

            for (const auto& wp : native_trr.trrDescription.waypoints) {
                auto asn_wp = asn1cpp::makeSeq(PathPoint);
                asn1cpp::setField(asn_wp->pathPosition.deltaLatitude,  wp.deltaLatitude);
                asn1cpp::setField(asn_wp->pathPosition.deltaLongitude, wp.deltaLongitude);
                asn1cpp::setField(asn_wp->pathPosition.deltaAltitude,  wp.deltaAltitude);
                if (wp.pathDeltaTime.isAvailable()) {
                    asn1cpp::setField(asn_wp->pathDeltaTime, wp.pathDeltaTime.getData());
                }
                asn1cpp::sequenceof::pushList(trr->waypoints, asn_wp);
            }

            for (const auto& hd : native_trr.trrDescription.heading) {
                auto asn_hd = asn1cpp::makeSeq(Wgs84Angle);
                asn1cpp::setField(asn_hd->value,      hd.getValue());
                asn1cpp::setField(asn_hd->confidence, hd.getConfidence());
                asn1cpp::sequenceof::pushList(trr->heading, asn_hd);
            }

            asn1cpp::setField(asn_subm->targetRoadResourceIContainer, trr);
        }

        asn1cpp::sequenceof::pushList(*asn_list, asn_subm);
    }

    return {asn_list, true};
}


std::tuple<asn1cpp::Seq<ManoeuvreAdviceContainer>, bool>
convertAdvicesToAsn1(const std::vector<mcData::MCManeuverAdvice>& native_advices) {
    auto asn_list = asn1cpp::makeSeq(ManoeuvreAdviceContainer);

    for (const auto& native_adv : native_advices) {
        auto asn_adv = asn1cpp::makeSeq(ManoeuvreAdvice);

        // --- executantID ---
        asn1cpp::setField(asn_adv->executantID, native_adv.executantID);

        // --- currentStateAdvisedChange (optional) ---
        if (native_adv.currentStateAdvisedChange.isAvailable()) {
            auto csac = asn1cpp::makeSeq(CurrentStateAdvisedChange);
            asn1cpp::setField(csac->present,
                              static_cast<CurrentStateAdvisedChange_PR>(
                                  native_adv.currentStateAdvisedChange.getData()));
            asn1cpp::setField(asn_adv->currentStateAdvisedChange, csac);
        }

        // --- submaneuvres ---
        for (const auto& native_subm : native_adv.submaneuvres) {
            auto asn_subm = asn1cpp::makeSeq(Submanoeuvre);

            asn1cpp::setField(asn_subm->submanoeuvreId, native_subm.submanoeuvreId);

            // --- advisedTrajectory (optional) ---
            if (native_subm.referenceTrajectory.isAvailable()) {
                const auto& native_traj = native_subm.referenceTrajectory.getData();
                auto traj = asn1cpp::makeSeq(Trajectory);

                asn1cpp::setField(traj->wayPointType, native_traj.wayPointType);

                for (const auto& wp : native_traj.wayPoints) {
                    auto asn_wp = asn1cpp::makeSeq(PathPoint);
                    asn1cpp::setField(asn_wp->pathPosition.deltaLatitude,  wp.deltaLatitude);
                    asn1cpp::setField(asn_wp->pathPosition.deltaLongitude, wp.deltaLongitude);
                    asn1cpp::setField(asn_wp->pathPosition.deltaAltitude,  wp.deltaAltitude);
                    if (wp.pathDeltaTime.isAvailable()) {
                        asn1cpp::setField(asn_wp->pathDeltaTime, wp.pathDeltaTime.getData());
                    }
                    asn1cpp::sequenceof::pushList(traj->wayPoints, asn_wp);
                }

                for (const auto& sp : native_traj.speed) {
                    auto asn_sp = asn1cpp::makeSeq(Speed);
                    asn1cpp::setField(asn_sp->speedValue,      sp.getValue());
                    asn1cpp::setField(asn_sp->speedConfidence, sp.getConfidence());
                    asn1cpp::sequenceof::pushList(traj->speed, asn_sp);
                }

                for (const auto& hd : native_traj.headings) {
                    auto asn_hd = asn1cpp::makeSeq(Wgs84Angle);
                    asn1cpp::setField(asn_hd->value,      hd.getValue());
                    asn1cpp::setField(asn_hd->confidence, hd.getConfidence());
                    asn1cpp::sequenceof::pushList(traj->headings, asn_hd);
                }

                for (const auto& lon : native_traj.longitudePositions) {
                    asn1cpp::sequenceof::pushList(traj->longitudePositions, (long)lon);
                }

                for (const auto& lat : native_traj.latitudePositions) {
                    asn1cpp::sequenceof::pushList(traj->latitudePositions, (long)lat);
                }

                for (const auto& alt : native_traj.altitudePositions) {
                    auto asn_alt = asn1cpp::makeSeq(Altitude);
                    asn1cpp::setField(asn_alt->altitudeValue,      alt.getValue());
                    asn1cpp::setField(asn_alt->altitudeConfidence, alt.getConfidence());
                    asn1cpp::sequenceof::pushList(traj->altitudePositions, asn_alt);
                }

                asn1cpp::setField(asn_subm->advisedTrajectory, traj);
            }

            // --- advisedTargetRoadResource (optional) ---
            if (native_subm.targetRoadResource.isAvailable()) {
                const auto& native_atrr = native_subm.targetRoadResource.getData();
                auto atrr = asn1cpp::makeSeq(AdvisedTrrContainer);

                asn1cpp::setField(atrr->trrDescription.trrType,   native_atrr.trrDescription.trrType);
                asn1cpp::setField(atrr->trrDescription.laneCount, native_atrr.trrDescription.laneCount);
                asn1cpp::setField(atrr->trrDescription.trrWidth,  native_atrr.trrDescription.trrWidth);
                asn1cpp::setField(atrr->trrDescription.trrLength, native_atrr.trrDescription.trrLength);

                if (native_atrr.trrDescription.startingLaneNumber.isAvailable()) {
                    asn1cpp::setField(atrr->trrDescription.startingLaneNumber,
                                      native_atrr.trrDescription.startingLaneNumber.getData());
                }
                if (native_atrr.trrDescription.endingLaneNumber.isAvailable()) {
                    asn1cpp::setField(atrr->trrDescription.endingLaneNumber,
                                      native_atrr.trrDescription.endingLaneNumber.getData());
                }

                for (const auto& wp : native_atrr.trrDescription.waypoints) {
                    auto asn_wp = asn1cpp::makeSeq(PathPoint);
                    asn1cpp::setField(asn_wp->pathPosition.deltaLatitude,  wp.deltaLatitude);
                    asn1cpp::setField(asn_wp->pathPosition.deltaLongitude, wp.deltaLongitude);
                    asn1cpp::setField(asn_wp->pathPosition.deltaAltitude,  wp.deltaAltitude);
                    if (wp.pathDeltaTime.isAvailable()) {
                        asn1cpp::setField(asn_wp->pathDeltaTime, wp.pathDeltaTime.getData());
                    }
                    asn1cpp::sequenceof::pushList(atrr->trrDescription.waypoints, asn_wp);
                }

                for (const auto& hd : native_atrr.trrDescription.heading) {
                    auto asn_hd = asn1cpp::makeSeq(Wgs84Angle);
                    asn1cpp::setField(asn_hd->value,      hd.getValue());
                    asn1cpp::setField(asn_hd->confidence, hd.getConfidence());
                    asn1cpp::sequenceof::pushList(atrr->trrDescription.heading, asn_hd);
                }

                asn1cpp::setField(atrr->temporalCharacteristics.tRROccupancyStartTime,
                                  native_atrr.temporalCharacteristics.tRROccupancyStartTime);
                asn1cpp::setField(atrr->temporalCharacteristics.tRROccupancyEndTime,
                                  native_atrr.temporalCharacteristics.tRROccupancyEndTime);

                asn1cpp::setField(asn_subm->advisedTargetRoadResource, atrr);
            }

            asn1cpp::sequenceof::pushList(asn_adv->submaneuvres, asn_subm);
        }

        asn1cpp::sequenceof::pushList(*asn_list, asn_adv);
    }

    return {asn_list, true};
}

MCBasicService_error_t
MCBasicService::generateAndEncodeMCM(const mcData& mcmData) {
  VDPGPSClient::MCM_mandatory_data_t MCM_mandatory_data;
  BTPDataRequest_t dataRequest = {};

  auto MCM_message = asn1cpp::makeSeq(MCM);
  if (bool(MCM_message) == false) {
    return MCM_ALLOC_ERROR;
  }

  /* Fill the header */
  asn1cpp::setField(MCM_message->header.messageId, FIX_MCMID);
  asn1cpp::setField(MCM_message->header.protocolVersion, 1);
  asn1cpp::setField(MCM_message->header.stationId, m_station_id);

  /* Fill the basicContainer */
  asn1cpp::setField(MCM_message->payload.basicContainer.generationDeltaTime, compute_timestampIts() % 65536);
  asn1cpp::setField(MCM_message->payload.basicContainer.stationID, m_station_id);
  asn1cpp::setField(MCM_message->payload.basicContainer.itssRole, mcmData.getITSRole());

  if (m_vehicle == true) {
    asn1cpp::setField(MCM_message->payload.basicContainer.stationType, McmStationType_vehicle);
  } else {
    asn1cpp::setField(MCM_message->payload.basicContainer.stationType, McmStationType_roadsideUnit);
  }

  MCM_mandatory_data = m_vdp->getMCMMandatoryData();
  long type    = mcmData.getMCMType();
  long concept = mcmData.getConcept();

  asn1cpp::setField(MCM_message->payload.basicContainer.position.latitude, MCM_mandatory_data.latitude);
  asn1cpp::setField(MCM_message->payload.basicContainer.position.longitude, MCM_mandatory_data.longitude);
  asn1cpp::setField(MCM_message->payload.basicContainer.position.altitude.altitudeValue, MCM_mandatory_data.altitude.getValue());
  asn1cpp::setField(MCM_message->payload.basicContainer.position.altitude.altitudeConfidence, MCM_mandatory_data.altitude.getConfidence());
  asn1cpp::setField(MCM_message->payload.basicContainer.position.positionConfidenceEllipse.semiMajorAxisLength, MCM_mandatory_data.posConfidenceEllipse.semiMajorConfidence);
  asn1cpp::setField(MCM_message->payload.basicContainer.position.positionConfidenceEllipse.semiMinorAxisLength, MCM_mandatory_data.posConfidenceEllipse.semiMinorConfidence);
  asn1cpp::setField(MCM_message->payload.basicContainer.position.positionConfidenceEllipse.semiMajorAxisOrientation, MCM_mandatory_data.posConfidenceEllipse.semiMajorOrientation);
  asn1cpp::setField(MCM_message->payload.basicContainer.mcmType, type);
  asn1cpp::setField(MCM_message->payload.basicContainer.manoeuvreId, mcmData.getManeuverID());
  asn1cpp::setField(MCM_message->payload.basicContainer.concept, concept);

  auto rational = asn1cpp::makeSeq(ManoeuvreCoordinationRational);
  if (concept == 0) {
    asn1cpp::setField(rational->present, ManoeuvreCoordinationRational_PR_manoeuvreCooperationGoal);
    asn1cpp::setField(rational->choice.manoeuvreCooperationGoal, mcmData.getGoal());
  } else {
    asn1cpp::setField(rational->present, ManoeuvreCoordinationRational_PR_manoeuvreCooperationCost);
    asn1cpp::setField(rational->choice.manoeuvreCooperationCost, mcmData.getCost());
  }
  asn1cpp::setField(MCM_message->payload.basicContainer.rational, rational);

  if (type == 4 || type == 7) {
    asn1cpp::setField(MCM_message->payload.basicContainer.executionStatus, mcmData.getExecutionStatus().getData());
  }

  /* Identify and fill the active container variant */
  if (mcmData.getManeuverContainer().isAvailable()) {
    asn1cpp::setField(MCM_message->payload.mcmContainer.present, McmContainer_PR_vehicleManoeuvreContainer);

    auto& man_ctx       = MCM_message->payload.mcmContainer.choice.vehicleManoeuvreContainer;
    const auto& man_data = mcmData.getManeuverContainer().getData();

    asn1cpp::setField(man_ctx.vehicleCurrentStateContainer.vehicleHeading.value, MCM_mandatory_data.heading.getValue());
    asn1cpp::setField(man_ctx.vehicleCurrentStateContainer.vehicleHeading.confidence, MCM_mandatory_data.heading.getConfidence());
    asn1cpp::setField(man_ctx.vehicleCurrentStateContainer.vehicleSpeed.speedValue, MCM_mandatory_data.speed.getValue());
    asn1cpp::setField(man_ctx.vehicleCurrentStateContainer.vehicleSpeed.speedConfidence, MCM_mandatory_data.speed.getConfidence());
    asn1cpp::setField(man_ctx.vehicleCurrentStateContainer.vehicleSize.vehicleWidth, MCM_mandatory_data.VehicleWidth);
    asn1cpp::setField(man_ctx.vehicleCurrentStateContainer.vehicleSize.vehicleLenth.vehicleLengthValue, MCM_mandatory_data.VehicleLength.getValue());
    asn1cpp::setField(man_ctx.vehicleCurrentStateContainer.vehicleSize.vehicleLenth.vehicleLengthConfidenceIndication, MCM_mandatory_data.VehicleLength.getConfidence());
    asn1cpp::setField(man_ctx.vehicleCurrentStateContainer.vehicleSize.vehicleHeight, VehicleHeight_unavailable);
    asn1cpp::setField(man_ctx.vehicleCurrentStateContainer.vehicleSize.vehicleType, man_data.vehicleType);

    // submaneuvers è mandatory nel vehicleManoeuvreContainer
    if (!man_data.submaneuvers.isAvailable()) {
      std::cerr << "[ERROR] vehicleManoeuvreContainer requires submaneuvers." << std::endl;
      return MCM_JSON_ERROR;
    }
    auto [asn_subms, subms_ok] = convertSubmaneuversToAsn1(man_data.submaneuvers.getData());
    if (!subms_ok) {
      std::cerr << "[ERROR] Failed to convert submaneuvers." << std::endl;
      return MCM_JSON_ERROR;
    }
    asn1cpp::setField(man_ctx.submaneuvres, asn_subms);

    if (man_data.advices.isAvailable()) {
      auto [asn_advs, advs_ok] = convertAdvicesToAsn1(man_data.advices.getData());
      if (!advs_ok) {
        std::cerr << "[ERROR] Failed to convert manoeuvre advices." << std::endl;
        return MCM_JSON_ERROR;
      }
      asn1cpp::setField(man_ctx.manoeuvreAdvice, asn_advs);
    }

  } else if (mcmData.getAdviceContainer().isAvailable()) {
    asn1cpp::setField(MCM_message->payload.mcmContainer.present, McmContainer_PR_advisedManoeuvreContainer);
    const auto& adv_data = mcmData.getAdviceContainer().getData();

    if (!adv_data.advices.isAvailable()) {
      std::cerr << "[ERROR] advisedManoeuvreContainer requires advices." << std::endl;
      return MCM_JSON_ERROR;
    }
    auto [asn_advs, advs_ok] = convertAdvicesToAsn1(adv_data.advices.getData());
    if (!advs_ok) {
      std::cerr << "[ERROR] Failed to convert advised manoeuvre advices." << std::endl;
      return MCM_JSON_ERROR;
    }
    asn1cpp::setField(MCM_message->payload.mcmContainer.choice.advisedManoeuvreContainer, asn_advs);

  } else if (mcmData.getResponseContainer().isAvailable()) {
    asn1cpp::setField(MCM_message->payload.mcmContainer.present, McmContainer_PR_responseContainer);
    const auto& resp_data = mcmData.getResponseContainer().getData();

    if (resp_data.response == 0) {
      asn1cpp::setField(MCM_message->payload.mcmContainer.choice.responseContainer.manouevreResponse, ManouevreResponse_accept);
    } else {
      asn1cpp::setField(MCM_message->payload.mcmContainer.choice.responseContainer.manouevreResponse, ManouevreResponse_decline);
      asn1cpp::setField(MCM_message->payload.mcmContainer.choice.responseContainer.declineReason, resp_data.declineReason);
    }

    if (resp_data.submaneuvers.isAvailable()) {
      auto [asn_subms, subms_ok] = convertSubmaneuversToAsn1(resp_data.submaneuvers.getData());
      if (!subms_ok) {
        std::cerr << "[ERROR] Failed to convert response submaneuvers." << std::endl;
        return MCM_JSON_ERROR;
      }
      asn1cpp::setField(MCM_message->payload.mcmContainer.choice.responseContainer.submaneuvres, asn_subms);
    }

  } else if (mcmData.getAcknowledgmentContainer().isAvailable()) {
    asn1cpp::setField(MCM_message->payload.mcmContainer.present, McmContainer_PR_acknowledgmentContainer);
    const auto& ack_data = mcmData.getAcknowledgmentContainer().getData();

    asn1cpp::setField(MCM_message->payload.mcmContainer.choice.acknowledgmentContainer.acknowledgedType, ack_data.type);
    asn1cpp::setField(MCM_message->payload.mcmContainer.choice.acknowledgmentContainer.generationDeltaTime, compute_timestampIts() % 65536);

  } else if (mcmData.getTerminationContainer().isAvailable()) {
    asn1cpp::setField(MCM_message->payload.mcmContainer.present, McmContainer_PR_terminationContainer);

  } else {
    std::cerr << "[ERROR] No valid active container variation found in mcData block." << std::endl;
    return MCM_JSON_ERROR;
  }

  /* Serialization & Network Transport */
  std::string encode_result = asn1cpp::uper::encode(MCM_message);
  if (encode_result.size() < 1) {
    return MCM_ASN1_UPER_ENC_ERROR;
  }

  dataRequest.BTPType       = BTP_B;
  dataRequest.destPort      = MC_PORT;
  dataRequest.destPInfo     = 0;
  dataRequest.GNType        = TSB;
  dataRequest.GNCommProfile = UNSPECIFIED;
  dataRequest.GNRepInt      = 0;
  dataRequest.GNMaxRepInt   = 0;
  dataRequest.GNMaxLife     = 1;
  dataRequest.GNMaxHL       = 1;
  dataRequest.GNTraClass    = 0x02;
  dataRequest.lenght        = encode_result.size();

  packetBuffer pktbuf(encode_result.c_str(), static_cast<unsigned int>(encode_result.size()));
  dataRequest.data = pktbuf;

  std::tuple<GNDataConfirm_t, MessageId_t> status = m_btp->sendBTP(dataRequest, m_priority, MessageId_cam);
  GNDataConfirm_t dataConfirm = std::get<0>(status);
  MessageId_t     message_id  = std::get<1>(status);

  if (m_met_sup_ptr != nullptr && dataConfirm == ACCEPTED) {
    if (message_id == MessageId_cam) m_MCM_sent++;
    m_met_sup_ptr->signalSentPacket(message_id);
  }

  return (dataConfirm == ACCEPTED) ? MCM_NO_ERROR : MCM_CANNOT_SEND;
}

uint64_t
MCBasicService::terminateDissemination() {
  if(m_terminateFlag==false) {
    m_terminateFlag=true;
  }

  return m_MCM_sent;
}
