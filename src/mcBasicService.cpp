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

static std::tuple<asn1cpp::Seq<ListOfSubmanoeuvreDescriptionsContainer>, bool>
convertSubmaneuversToAsn1(const std::vector<mcData::mcDataSubmaneuverDescription>& native_subms) {
    auto asn_list = asn1cpp::makeSeq(ListOfSubmanoeuvreDescriptionsContainer);

    for (const auto& native_subm : native_subms) {
        auto asn_subm = asn1cpp::makeSeq(SubmanoeuvreDescription);

        // --- submanoeuvreID ---
        asn1cpp::setField(asn_subm->submanoeuvreID, native_subm.submanoeuvreId);

        // --- temporalCharacteristics ---
        asn1cpp::setField(asn_subm->temporalCharateristics.tRROccupancyStartTime,
                          native_subm.temporalCharacteristics.tRROccupancyStartTime);
        asn1cpp::setField(asn_subm->temporalCharateristics.tRROccupancyEndTime,
                          native_subm.temporalCharacteristics.tRROccupancyEndTime);

        // --- submanoeuvreStrategy (optional) ---
        if (native_subm.submanoeuvreStrategy.isAvailable()) {
            const auto& native_strategy = native_subm.submanoeuvreStrategy.getData();
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

            asn1cpp::setField(trr->trrType,   native_trr.trrType);
            asn1cpp::setField(trr->laneCount, native_trr.laneCount);
            asn1cpp::setField(trr->trrWidth,  native_trr.trrWidth);
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

        // --- submaneuvres (da mcDataAdvisedSubmaneuver) ---
        for (const auto& native_subm : native_adv.submaneuvres) {
            auto asn_subm = asn1cpp::makeSeq(Submanoeuvre);

            asn1cpp::setField(asn_subm->submanoeuvreId, native_subm.submaneuverID);

            // --- advisedTrajectory (optional) ---
            if (native_subm.advisedTrajectory.isAvailable()) {
                const auto& native_traj = native_subm.advisedTrajectory.getData();
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
            if (native_subm.advisedTrrContainer.isAvailable()) {
                const auto& native_atrr = native_subm.advisedTrrContainer.getData();
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
