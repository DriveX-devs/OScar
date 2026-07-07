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

static std::tuple<asn1cpp::Seq<ListOfSubmanoeuvreDescriptionsContainer>, bool>
convertSubmaneuversToAsn1(const std::vector<mcData::mcDataSubmaneuverDescription>& native_subms) {
    auto asn_list = asn1cpp::makeSeq(ListOfSubmanoeuvreDescriptionsContainer);

    for (const auto& native_subm : native_subms) {
        auto asn_subm = asn1cpp::makeSeq(SubmanoeuvreDescription);

        // --- submanoeuvreID ---
        asn1cpp::setField(asn_subm->submanoeuvreID, native_subm.submaneuverID);

        // --- temporalCharacteristics ---
        asn1cpp::setField(asn_subm->temporalCharateristics.tRROccupancyStartTime,
                          native_subm.temporalCharacteristics.tRROccupancyStartTime);
        asn1cpp::setField(asn_subm->temporalCharateristics.tRROccupancyEndTime,
                          native_subm.temporalCharacteristics.tRROccupancyEndTime);

        // --- submanoeuvreStrategy (optional) ---
        if (native_subm.submaneuverStrategy.isAvailable()) {
            const auto& native_strategy = native_subm.submaneuverStrategy.getData();
            auto strategy = asn1cpp::makeSeq(SubmanoeuvreStrategy);

            asn1cpp::setField(strategy->present,
                              static_cast<SubmanoeuvreStrategy_PR>(native_strategy.present));

            switch (static_cast<SubmanoeuvreStrategy_PR>(native_strategy.present)) {
                case SubmanoeuvreStrategy_PR_undefined:
                    asn1cpp::setField(strategy->choice.undefined, native_strategy.value); break;
                case SubmanoeuvreStrategy_PR_transitToHumanDrivenMode:
                    asn1cpp::setField(strategy->choice.transitToHumanDrivenMode, native_strategy.value); break;
                case SubmanoeuvreStrategy_PR_transitToAutomatedDrivingMode:
                    asn1cpp::setField(strategy->choice.transitToAutomatedDrivingMode, native_strategy.value); break;
                case SubmanoeuvreStrategy_PR_driveStraight:
                    asn1cpp::setField(strategy->choice.driveStraight, native_strategy.value); break;
                case SubmanoeuvreStrategy_PR_turnLeft:
                    asn1cpp::setField(strategy->choice.turnLeft, native_strategy.value); break;
                case SubmanoeuvreStrategy_PR_turnRight:
                    asn1cpp::setField(strategy->choice.turnRight, native_strategy.value); break;
                case SubmanoeuvreStrategy_PR_uTurn:
                    asn1cpp::setField(strategy->choice.uTurn, native_strategy.value); break;
                case SubmanoeuvreStrategy_PR_moveBackward:
                    asn1cpp::setField(strategy->choice.moveBackward, native_strategy.value); break;
                case SubmanoeuvreStrategy_PR_overtake:
                    asn1cpp::setField(strategy->choice.overtake, native_strategy.value); break;
                case SubmanoeuvreStrategy_PR_accelerate:
                    asn1cpp::setField(strategy->choice.accelerate, native_strategy.value); break;
                case SubmanoeuvreStrategy_PR_slowDown:
                    asn1cpp::setField(strategy->choice.slowDown, native_strategy.value); break;
                case SubmanoeuvreStrategy_PR_stop:
                    asn1cpp::setField(strategy->choice.stop, native_strategy.value); break;
                case SubmanoeuvreStrategy_PR_goToLeftLane:
                    asn1cpp::setField(strategy->choice.goToLeftLane, native_strategy.value); break;
                case SubmanoeuvreStrategy_PR_goToRightLane:
                    asn1cpp::setField(strategy->choice.goToRightLane, native_strategy.value); break;
                case SubmanoeuvreStrategy_PR_getOnHighway:
                    asn1cpp::setField(strategy->choice.getOnHighway, native_strategy.value); break;
                case SubmanoeuvreStrategy_PR_exitHighway:
                    asn1cpp::setField(strategy->choice.exitHighway, native_strategy.value); break;
                case SubmanoeuvreStrategy_PR_takeTollingLane:
                    asn1cpp::setField(strategy->choice.takeTollingLane, native_strategy.value); break;
                case SubmanoeuvreStrategy_PR_stopAndWait:
                    asn1cpp::setField(strategy->choice.stopAndWait, native_strategy.value); break;
                case SubmanoeuvreStrategy_PR_emergencyBrakeAndStop:
                    asn1cpp::setField(strategy->choice.emergencyBrakeAndStop, native_strategy.value); break;
                case SubmanoeuvreStrategy_PR_resetStopAndRestartMoving:
                    asn1cpp::setField(strategy->choice.resetStopAndRestartMoving, native_strategy.value); break;
                case SubmanoeuvreStrategy_PR_stayInLane:
                    asn1cpp::setField(strategy->choice.stayInLane, native_strategy.value); break;
                case SubmanoeuvreStrategy_PR_resetStayInLane:
                    asn1cpp::setField(strategy->choice.resetStayInLane, native_strategy.value); break;
                case SubmanoeuvreStrategy_PR_stayAway:
                    asn1cpp::setField(strategy->choice.stayAway, native_strategy.value); break;
                case SubmanoeuvreStrategy_PR_resetStayAway:
                    asn1cpp::setField(strategy->choice.resetStayAway, native_strategy.value); break;
                case SubmanoeuvreStrategy_PR_followMe:
                    asn1cpp::setField(strategy->choice.followMe, native_strategy.value); break;
                case SubmanoeuvreStrategy_PR_existingGroup:
                    asn1cpp::setField(strategy->choice.existingGroup, native_strategy.value); break;
                case SubmanoeuvreStrategy_PR_temporarilyDisbandAnExistingGroup:
                    asn1cpp::setField(strategy->choice.temporarilyDisbandAnExistingGroup, native_strategy.value); break;
                case SubmanoeuvreStrategy_PR_constituteATemporarilyGroup:
                    asn1cpp::setField(strategy->choice.constituteATemporarilyGroup, native_strategy.value); break;
                case SubmanoeuvreStrategy_PR_disbandATemporarilyGroup:
                    asn1cpp::setField(strategy->choice.disbandATemporarilyGroup, native_strategy.value); break;
                default:
                    std::cerr << "[ERROR] Unhandled strategy present value: "
                              << native_strategy.present << std::endl;
                    return {nullptr, false};
            }
            asn1cpp::setField(asn_subm->submanoeuvreStrategy, strategy);
        }

        // --- referenceTrajectory (optional) ---
        if (native_subm.referenceTrajectory.isAvailable()) {
            const auto& native_traj = native_subm.referenceTrajectory.getData();
            auto traj = asn1cpp::makeSeq(Trajectory);

            asn1cpp::setField(traj->wayPointType, native_traj.wayPointType);

            if (native_traj.wayPoints.size() < 2) {
                std::cerr << "[ERROR] WayPoints should be at least 2."<< std::endl;
                return {nullptr, false};
            }

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

            if (native_traj.speed.size() < 2) {
                std::cerr << "[ERROR] Speeds should be at least 2."<< std::endl;
                return {nullptr, false};
            }
            for (const auto& sp : native_traj.speed) {
                auto asn_sp = asn1cpp::makeSeq(Speed);
                asn1cpp::setField(asn_sp->speedValue,      sp.getValue());
                asn1cpp::setField(asn_sp->speedConfidence, sp.getConfidence());
                asn1cpp::sequenceof::pushList(traj->speed, asn_sp);
            }

            if (native_traj.headings.size() < 2) {
                std::cerr << "[ERROR] Headings should be at least 2."<< std::endl;
                return {nullptr, false};
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

            asn1cpp::setField(trr->trrType,   native_trr.trrType);
            asn1cpp::setField(trr->laneCount, native_trr.laneCount);
            if (native_trr.trrWidth > 15 || native_trr.trrWidth < 0) {
                std::cerr << "[ERROR] TrrWidth must not be negative or greater than 15."<< std::endl;
                return {nullptr, false};
            }
            asn1cpp::setField(trr->trrWidth,  native_trr.trrWidth);
            if (native_trr.trrLength > 4900 || native_trr.trrLength < 0) {
                std::cerr << "[ERROR] TrrLength must not be negative or greater than 4900."<< std::endl;
                return {nullptr, false};
            }
            asn1cpp::setField(trr->trrLength, native_trr.trrLength);
            if (native_trr.startingLaneNumber.isAvailable()) {
                asn1cpp::setField(trr->startingLaneNumber, native_trr.startingLaneNumber.getData());
            }
            if (native_trr.endingLaneNumber.isAvailable()) {
                asn1cpp::setField(trr->endingLaneNumber, native_trr.endingLaneNumber.getData());
            }

            for (const auto& wp : native_trr.waypoints) {
                auto asn_wp = asn1cpp::makeSeq(PathPoint);
                asn1cpp::setField(asn_wp->pathPosition.deltaLatitude,  wp.deltaLatitude);
                asn1cpp::setField(asn_wp->pathPosition.deltaLongitude, wp.deltaLongitude);
                asn1cpp::setField(asn_wp->pathPosition.deltaAltitude,  wp.deltaAltitude);
                if (wp.pathDeltaTime.isAvailable()) {
                    asn1cpp::setField(asn_wp->pathDeltaTime, wp.pathDeltaTime.getData());
                }
                asn1cpp::sequenceof::pushList(trr->waypoints, asn_wp);
            }

            for (const auto& hd : native_trr.heading) {
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


static std::tuple<asn1cpp::Seq<ManoeuvreAdviceContainer>, bool>
convertAdvicesToAsn1(const std::vector<mcData::mcDataManeuverAdvice>& native_advices) {
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

        // --- submaneuvers (da mcDataAdvisedSubmaneuver) ---
        for (const auto& native_subm : native_adv.submaneuvers) {
            auto asn_subm = asn1cpp::makeSeq(Submanoeuvre);

            asn1cpp::setField(asn_subm->submanoeuvreId, native_subm.submaneuverID);

            // --- advisedTrajectory (optional) ---
            if (native_subm.advisedTrajectory.isAvailable()) {
                const auto& native_traj = native_subm.advisedTrajectory.getData();
                auto traj = asn1cpp::makeSeq(Trajectory);

                asn1cpp::setField(traj->wayPointType, native_traj.wayPointType);

                if (native_traj.wayPoints.size() < 2) {
                    std::cerr << "[ERROR] WayPoints should be at least 2."<< std::endl;
                    return {nullptr, false};
                }
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

                if (native_traj.headings.size() < 2) {
                    std::cerr << "[ERROR] Headings should be at least 2."<< std::endl;
                    return {nullptr, false};
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
            if (native_subm.advisedTrrContainer.isAvailable()) {
                const auto& native_atrr = native_subm.advisedTrrContainer.getData();
                auto atrr = asn1cpp::makeSeq(AdvisedTrrContainer);

                asn1cpp::setField(atrr->trrDescription.trrType,   native_atrr.trrDescription.trrType);
                asn1cpp::setField(atrr->trrDescription.laneCount, native_atrr.trrDescription.laneCount);
                if (native_atrr.trrDescription.trrWidth > 15 || native_atrr.trrDescription.trrWidth < 0) {
                    std::cerr << "[ERROR] TrrWidth must not be negative or greater than 15"<< std::endl;
                    return {nullptr, false};
                }
                asn1cpp::setField(atrr->trrDescription.trrWidth,  native_atrr.trrDescription.trrWidth);
                if (native_atrr.trrDescription.trrLength > 4900 || native_atrr.trrDescription.trrLength < 0) {
                    std::cerr << "[ERROR] TrrLength must not be negative or greater than 4900"<< std::endl;
                    return {nullptr, false};
                }
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

// ============================================================================
//  ASN.1 → mcData (decoder helpers, inverse of generateAndEncodeMCM)
// ============================================================================

static mcData::mcDataPathPoint pathPointFromAsn1(const PathPoint_t* asn_wp) {
    mcData::mcDataPathPoint wp;
    wp.deltaLatitude  = asn1cpp::getField(asn_wp->pathPosition.deltaLatitude,  long);
    wp.deltaLongitude = asn1cpp::getField(asn_wp->pathPosition.deltaLongitude, long);
    wp.deltaAltitude  = asn1cpp::getField(asn_wp->pathPosition.deltaAltitude,  long);
    if (asn_wp->pathDeltaTime != nullptr) {
        wp.pathDeltaTime.setData(asn1cpp::getField(*asn_wp->pathDeltaTime, long));
    }
    return wp;
}

static mcData::mcDataTrajectory trajectoryFromAsn1(const Trajectory_t* asn_traj) {
    mcData::mcDataTrajectory traj;
    traj.wayPointType = asn1cpp::getField(asn_traj->wayPointType, long);

    for (int i = 0; i < asn_traj->wayPoints.list.count; ++i) {
        traj.wayPoints.push_back(pathPointFromAsn1(asn_traj->wayPoints.list.array[i]));
    }
    for (int i = 0; i < asn_traj->speed.list.count; ++i) {
        const auto* sp = asn_traj->speed.list.array[i];
        traj.speed.emplace_back(asn1cpp::getField(sp->speedValue,      long),
                                asn1cpp::getField(sp->speedConfidence, long));
    }
    if (asn_traj->headings != nullptr) {
        for (int i = 0; i < asn_traj->headings->list.count; ++i) {
            const auto* hd = asn_traj->headings->list.array[i];
            traj.headings.emplace_back(asn1cpp::getField(hd->value,      long),
                                       asn1cpp::getField(hd->confidence, long));
        }
    }
    if (asn_traj->longitudePositions != nullptr) {
        for (int i = 0; i < asn_traj->longitudePositions->list.count; ++i) {
            traj.longitudePositions.push_back(*asn_traj->longitudePositions->list.array[i]);
        }
    }
    if (asn_traj->latitudePositions != nullptr) {
        for (int i = 0; i < asn_traj->latitudePositions->list.count; ++i) {
            traj.latitudePositions.push_back(*asn_traj->latitudePositions->list.array[i]);
        }
    }
    if (asn_traj->altitudePositions != nullptr) {
        for (int i = 0; i < asn_traj->altitudePositions->list.count; ++i) {
            const auto* alt = asn_traj->altitudePositions->list.array[i];
            traj.altitudePositions.emplace_back(asn1cpp::getField(alt->altitudeValue,      long),
                                                asn1cpp::getField(alt->altitudeConfidence, long));
        }
    }
    return traj;
}

static mcData::mcDataTrrDescription trrFromAsn1(const TrrDescription_t* asn_trr) {
    mcData::mcDataTrrDescription trr;
    trr.trrType   = asn1cpp::getField(asn_trr->trrType,   long);
    trr.laneCount = asn1cpp::getField(asn_trr->laneCount, long);
    trr.trrWidth  = asn1cpp::getField(asn_trr->trrWidth,  long);
    trr.trrLength = asn1cpp::getField(asn_trr->trrLength, long);

    if (asn_trr->startingLaneNumber != nullptr) {
        trr.startingLaneNumber.setData(asn1cpp::getField(*asn_trr->startingLaneNumber, long));
    }
    if (asn_trr->endingLaneNumber != nullptr) {
        trr.endingLaneNumber.setData(asn1cpp::getField(*asn_trr->endingLaneNumber, long));
    }
    if (asn_trr->waypoints != nullptr) {
        for (int i = 0; i < asn_trr->waypoints->list.count; ++i) {
            trr.waypoints.push_back(pathPointFromAsn1(asn_trr->waypoints->list.array[i]));
        }
    }
    if (asn_trr->heading != nullptr) {
        for (int i = 0; i < asn_trr->heading->list.count; ++i) {
            const auto* hd = asn_trr->heading->list.array[i];
            trr.heading.emplace_back(asn1cpp::getField(hd->value,      long),
                                     asn1cpp::getField(hd->confidence, long));
        }
    }
    return trr;
}

static std::vector<mcData::mcDataSubmaneuverDescription>
submaneuversFromAsn1(const ListOfSubmanoeuvreDescriptionsContainer_t& asn_list) {
    std::vector<mcData::mcDataSubmaneuverDescription> result;

    for (int i = 0; i < asn_list.list.count; ++i) {
        const auto* asn_subm = asn_list.list.array[i];
        mcData::mcDataSubmaneuverDescription subm;

        subm.submaneuverID = asn1cpp::getField(asn_subm->submanoeuvreID, long);
        subm.temporalCharacteristics.tRROccupancyStartTime =
            asn1cpp::getField(asn_subm->temporalCharateristics.tRROccupancyStartTime, long);
        subm.temporalCharacteristics.tRROccupancyEndTime =
            asn1cpp::getField(asn_subm->temporalCharateristics.tRROccupancyEndTime,   long);

        if (asn_subm->submanoeuvreStrategy != nullptr) {
            mcData::mcDataSubmaneuverStrategy strategy;
            strategy.present = static_cast<int>(asn_subm->submanoeuvreStrategy->present);
            // Only takeTollingLane carries a meaningful long; the other choices are NULL.
            if (asn_subm->submanoeuvreStrategy->present == SubmanoeuvreStrategy_PR_takeTollingLane) {
                strategy.value = asn_subm->submanoeuvreStrategy->choice.takeTollingLane;
            } else {
                strategy.value = 0;
            }
            subm.submaneuverStrategy.setData(strategy);
        }
        if (asn_subm->referenceTrajectory != nullptr) {
            subm.referenceTrajectory.setData(trajectoryFromAsn1(asn_subm->referenceTrajectory));
        }
        if (asn_subm->targetRoadResourceIContainer != nullptr) {
            subm.targetRoadResource.setData(trrFromAsn1(asn_subm->targetRoadResourceIContainer));
        }
        if (asn_subm->kinematicsCharacteristics != nullptr) {
            // KinematicsCharacteristics_t is ASN.1 NULL → presence-only flag.
            subm.kinematicsCharacteristics.setData(1);
        }

        result.push_back(subm);
    }
    return result;
}

static std::vector<mcData::mcDataManeuverAdvice>
advicesFromAsn1(const ManoeuvreAdviceContainer_t& asn_list) {
    std::vector<mcData::mcDataManeuverAdvice> result;

    for (int i = 0; i < asn_list.list.count; ++i) {
        const auto* asn_adv = asn_list.list.array[i];
        mcData::mcDataManeuverAdvice adv;

        adv.executantID = asn1cpp::getField(asn_adv->executantID, long);

        if (asn_adv->currentStateAdvisedChange != nullptr) {
            adv.currentStateAdvisedChange.setData(
                static_cast<long>(asn_adv->currentStateAdvisedChange->present));
        }

        for (int j = 0; j < asn_adv->submaneuvres.list.count; ++j) {
            const auto* asn_sm = asn_adv->submaneuvres.list.array[j];
            mcData::mcDataAdvisedSubmaneuver sm;
            sm.submaneuverID = asn1cpp::getField(asn_sm->submanoeuvreId, long);

            if (asn_sm->advisedTrajectory != nullptr) {
                sm.advisedTrajectory.setData(trajectoryFromAsn1(asn_sm->advisedTrajectory));
            }
            if (asn_sm->advisedTargetRoadResource != nullptr) {
                mcData::mcDataAdvisedTrrContainer atrr;
                atrr.trrDescription = trrFromAsn1(&asn_sm->advisedTargetRoadResource->trrDescription);
                atrr.temporalCharacteristics.tRROccupancyStartTime = asn1cpp::getField(
                    asn_sm->advisedTargetRoadResource->temporalCharacteristics.tRROccupancyStartTime, long);
                atrr.temporalCharacteristics.tRROccupancyEndTime = asn1cpp::getField(
                    asn_sm->advisedTargetRoadResource->temporalCharacteristics.tRROccupancyEndTime,   long);
                if (asn_sm->advisedTargetRoadResource->kinematicsCharacteristics != nullptr) {
                    // KinematicsCharacteristics_t is ASN.1 NULL → presence-only flag.
                    atrr.kinematicsCharacteristics.setData(1);
                }
                sm.advisedTrrContainer.setData(atrr);
            }

            adv.submaneuvers.push_back(sm);
        }

        result.push_back(adv);
    }
    return result;
}

mcData
MCBasicService::convertASN1IntoMcData(MCM_t* decoded_mcm) {
    mcData mcmData;

    if (decoded_mcm == nullptr) {
        std::cerr << "[ERROR] convertASN1IntoMcData: null MCM pointer." << std::endl;
        return mcmData;
    }

    // --- Header ---
    mcData::mcDataHeader hdr;
    hdr.messageID       = asn1cpp::getField(decoded_mcm->header.messageId,       long);
    hdr.protocolVersion = asn1cpp::getField(decoded_mcm->header.protocolVersion, long);
    hdr.stationID       = asn1cpp::getField(decoded_mcm->header.stationId,       unsigned long);
    mcmData.setHeader(hdr);

    // --- Basic container ---
    const auto& asn_bc = decoded_mcm->payload.basicContainer;
    mcData::mcBasicContainer bc;
    bc.stationID   = asn1cpp::getField(asn_bc.stationID,   long);
    bc.itsRole     = asn1cpp::getField(asn_bc.itssRole,    long);
    bc.stationType = asn1cpp::getField(asn_bc.stationType, long);
    bc.mcmType     = asn1cpp::getField(asn_bc.mcmType,     long);
    bc.maneuverID  = asn1cpp::getField(asn_bc.manoeuvreId, long);
    bc.concept     = asn1cpp::getField(asn_bc.concept,     long);
    bc.cost        = 0;
    bc.goal        = 0;
    if (asn_bc.rational != nullptr) {
        if (asn_bc.rational->present == ManoeuvreCoordinationRational_PR_manoeuvreCooperationCost) {
            bc.cost    = asn1cpp::getField(asn_bc.rational->choice.manoeuvreCooperationCost, long);
            bc.concept = 1;
        } else if (asn_bc.rational->present == ManoeuvreCoordinationRational_PR_manoeuvreCooperationGoal) {
            bc.goal    = asn1cpp::getField(asn_bc.rational->choice.manoeuvreCooperationGoal, long);
            bc.concept = 0;
        }
    }
    if (asn_bc.executionStatus != nullptr) {
        bc.executionStatus.setData(asn1cpp::getField(*asn_bc.executionStatus, long));
    }
    mcmData.setBasicContainer(bc);

    // --- Active container variant (CHOICE) ---
    const auto& asn_mcmc = decoded_mcm->payload.mcmContainer;
    switch (asn_mcmc.present) {
        case McmContainer_PR_vehicleManoeuvreContainer: {
            const auto& asn_man = asn_mcmc.choice.vehicleManoeuvreContainer;
            mcData::mcManeuverContainer man;
            man.vehicleType = asn1cpp::getField(
                asn_man.vehicleCurrentStateContainer.vehicleSize.vehicleType, long);
            man.submaneuvers = submaneuversFromAsn1(asn_man.submaneuvres);
            if (asn_man.manoeuvreAdvice != nullptr) {
                man.advices.setData(advicesFromAsn1(*asn_man.manoeuvreAdvice));
            }
            mcmData.setManeuverContainer(man);
            break;
        }
        case McmContainer_PR_advisedManoeuvreContainer: {
            mcData::mcAdviceContainer ac;
            ac.advices = advicesFromAsn1(asn_mcmc.choice.advisedManoeuvreContainer);
            mcmData.setAdviceContainer(ac);
            break;
        }
        case McmContainer_PR_responseContainer: {
            const auto& asn_resp = asn_mcmc.choice.responseContainer;
            mcData::mcResponseContainer rc;
            rc.response = asn1cpp::getField(asn_resp.manouevreResponse, long);
            if (asn_resp.declineReason != nullptr) {
                rc.declineReason.setData(asn1cpp::getField(*asn_resp.declineReason, long));
            }
            if (asn_resp.submaneuvres != nullptr) {
                rc.submaneuvers.setData(submaneuversFromAsn1(*asn_resp.submaneuvres));
            }
            mcmData.setResponseContainer(rc);
            break;
        }
        case McmContainer_PR_acknowledgmentContainer: {
            mcData::mcAcknowledgeContainer ack;
            ack.type = asn1cpp::getField(
                asn_mcmc.choice.acknowledgmentContainer.acknowledgedType, long);
            mcmData.setAcknowledgmentContainer(ack);
            break;
        }
        case McmContainer_PR_terminationContainer:
            mcmData.setTerminationContainer(mcData::mcTerminationContainer{});
            break;
        case McmContainer_PR_NOTHING:
        default:
            std::cerr << "[WARN] convertASN1IntoMcData: no container present in MCM." << std::endl;
            break;
    }

    return mcmData;
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
    if (mcmData.getExecutionStatus().isAvailable()) {
        asn1cpp::setField(MCM_message->payload.basicContainer.executionStatus, mcmData.getExecutionStatus().getData());
    } else {
        return MCM_JSON_ERROR;
    }
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

    if (man_data.submaneuvers.empty()) {
        std::cerr << "[ERROR] vehicleManoeuvreContainer requires submaneuvers." << std::endl;
        return MCM_JSON_ERROR;
    }
    auto [asn_subms, subms_ok] = convertSubmaneuversToAsn1(man_data.submaneuvers);
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

    if (adv_data.advices.empty()) {
        std::cerr << "[ERROR] advisedManoeuvreContainer requires advices." << std::endl;
        return MCM_JSON_ERROR;
    }
    auto [asn_advs, advs_ok] = convertAdvicesToAsn1(adv_data.advices);
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
      if (resp_data.declineReason.isAvailable()) {
        asn1cpp::setField(MCM_message->payload.mcmContainer.choice.responseContainer.declineReason, resp_data.declineReason.getData());
      } else {
        std::cerr << "[ERROR] A Refusal need a Decline Reason." << std::endl;
        return MCM_JSON_ERROR;
      }
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

  std::tuple<GNDataConfirm_t, MessageId_t> status = m_btp->sendBTP(dataRequest, m_priority, MessageId_mcm);
  GNDataConfirm_t dataConfirm = std::get<0>(status);
  MessageId_t     message_id  = std::get<1>(status);

  if (m_met_sup_ptr != nullptr && dataConfirm == ACCEPTED) {
    if (message_id == MessageId_mcm) m_MCM_sent++;
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
