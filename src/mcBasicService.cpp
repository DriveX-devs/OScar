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

MCSpecification::~MCSpecification()
{
  for (auto &adv : m_maneuver_advice)
    {
        if (adv != nullptr)
        {
            ASN_STRUCT_FREE(asn_DEF_ManoeuvreAdvice, adv);
        }
    }

    for (auto &subm : m_submaneuver_description)
    {
        if (subm != nullptr)
        {
            ASN_STRUCT_FREE(asn_DEF_SubmanoeuvreDescription, subm);
        }
    }
}

bool MCSpecification::checkContainers()
{
  int count = m_vehicle_advise_container
              + m_vehicle_maneuver_container
              + m_vehicle_acknowledgement_container
              + m_vehicle_response_container
              + m_vehicle_terminator_container;
  return count == 1;
}


MCBasicService::~MCBasicService()=default;

MCBasicService::MCBasicService()
{
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

MCBasicService::MCBasicService(unsigned long fixed_stationid,long fixed_stationtype,VDPGPSClient* vdp, bool real_time, bool is_vehicle)
{
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
MCBasicService::setStationProperties(unsigned long fixed_stationid,long fixed_stationtype,double latitude_deg,double longitude_deg)
{
  m_station_id=fixed_stationid;
  m_stationtype=fixed_stationtype;
}

void
MCBasicService::setFixedPositionRSU(double latitude_deg, double longitude_deg)
{
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
MCBasicService::setStationID(unsigned long fixed_stationid)
{
  m_station_id=fixed_stationid;
  m_btp->setStationID(fixed_stationid);
}

void
MCBasicService::setStationType(long fixed_stationtype)
{
  m_stationtype=fixed_stationtype;
  m_btp->setStationType(fixed_stationtype);
}

MCBasicService_error_t
MCBasicService::generateAndEncodeMCM(MCSpecification* specification)
{
  // Only one container must be activated for one message
  specification->checkContainers();
  VDPGPSClient::MCM_mandatory_data_t MCM_mandatory_data;

  BTPDataRequest_t dataRequest = {};

  /* Collect data for mandatory containers */
  auto MCM_message = asn1cpp::makeSeq(MCM);

  if(bool(MCM_message)==false)
    {
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
  asn1cpp::setField(MCM_message->payload.basicContainer.itssRole, specification->getMCMItsRole());
  if(m_vehicle==true)
    {
      asn1cpp::setField(MCM_message->payload.basicContainer.stationType, McmStationType_vehicle);
      MCM_mandatory_data = m_vdp->getMCMMandatoryData();
      asn1cpp::setField(MCM_message->payload.basicContainer.position.latitude, MCM_mandatory_data.latitude);
      asn1cpp::setField(MCM_message->payload.basicContainer.position.longitude, MCM_mandatory_data.longitude);
      asn1cpp::setField(MCM_message->payload.basicContainer.position.altitude.altitudeValue, MCM_mandatory_data.altitude.getValue());
      asn1cpp::setField(MCM_message->payload.basicContainer.position.altitude.altitudeConfidence, MCM_mandatory_data.altitude.getConfidence());
      asn1cpp::setField(MCM_message->payload.basicContainer.position.positionConfidenceEllipse.semiMajorAxisLength, MCM_mandatory_data.posConfidenceEllipse.semiMajorConfidence);
      asn1cpp::setField(MCM_message->payload.basicContainer.position.positionConfidenceEllipse.semiMinorAxisLength, MCM_mandatory_data.posConfidenceEllipse.semiMinorConfidence);
      asn1cpp::setField(MCM_message->payload.basicContainer.position.positionConfidenceEllipse.semiMajorAxisOrientation, MCM_mandatory_data.posConfidenceEllipse.semiMajorOrientation);
      asn1cpp::setField(MCM_message->payload.basicContainer.mcmType, specification->getMCMType());
      asn1cpp::setField(MCM_message->payload.basicContainer.manoeuvreId, specification->getManeuverID());
      asn1cpp::setField(MCM_message->payload.basicContainer.concept, specification->getMCMConcept());
      
      auto *rational = (ManoeuvreCoordinationRational_t*)CALLOC(1, sizeof(ManoeuvreCoordinationRational_t));
      if (specification->getMCMConcept() == 0) {
          asn1cpp::setField(rational->present, ManoeuvreCoordinationRational_PR_manoeuvreCooperationGoal);
          asn1cpp::setField(rational->choice.manoeuvreCooperationGoal, specification->getMCMGoal());
        } else {
          asn1cpp::setField(rational->present, ManoeuvreCoordinationRational_PR_manoeuvreCooperationCost);
          asn1cpp::setField(rational->choice.manoeuvreCooperationCost, specification->getMCMCost());
        }
      MCM_message->payload.basicContainer.rational = rational;
      if (specification->getMCMType() == 4 || specification->getMCMType() == 7)
        {
          asn1cpp::setField (MCM_message->payload.basicContainer.executionStatus, specification->getMCMStatus());
        }
    }
  else
    {
      /* Fill the basicContainer for RSU*/
      asn1cpp::setField(MCM_message->payload.basicContainer.stationType, McmStationType_roadsideUnit);
      // TODO
    }

  if (specification->getManeuverContainer())
    {
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
      asn1cpp::setField(man.vehicleCurrentStateContainer.vehicleSize.vehicleType, specification->getVehicleType());

      for (auto it = specification->getSubmaneuverDescription().begin(); it != specification->getSubmaneuverDescription().end(); ++it)
        {
          auto *subd = (SubmanoeuvreDescription *)CALLOC(1, sizeof(SubmanoeuvreDescription));
          if (asn_copy(&asn_DEF_SubmanoeuvreDescription, (void**)&subd, &(*it)) == 0)
            {
              if (ASN_SEQUENCE_ADD (&man.submaneuvres, subd) != 0)
                {
                  ASN_STRUCT_FREE (asn_DEF_SubmanoeuvreDescription, subd);
                }
            }
          else
            {
              ASN_STRUCT_FREE(asn_DEF_SubmanoeuvreDescription, subd);
            }
        }

      for (auto it = specification->getManeuverAdvice().begin(); it != specification->getManeuverAdvice().end(); ++it)
        {
          auto *advice = (ManoeuvreAdvice *)CALLOC(1, sizeof(ManoeuvreAdvice));
          if (asn_copy(&asn_DEF_ManoeuvreAdvice, (void**)&advice, &(*it)) == 0)
            {
              if (ASN_SEQUENCE_ADD (&man.manoeuvreAdvice, advice) != 0)
                {
                  ASN_STRUCT_FREE (asn_DEF_ManoeuvreAdvice, advice);
                }
            }
          else
            {
              ASN_STRUCT_FREE(asn_DEF_ManoeuvreAdvice, advice);
            }
        }
    }
  
  else if (specification->getAdviseContainer())
    {
      asn1cpp::setField(MCM_message->payload.mcmContainer.present, McmContainer_PR_advisedManoeuvreContainer);
      auto &adv = MCM_message->payload.mcmContainer.choice.advisedManoeuvreContainer;
      for (auto it = specification->getManeuverAdvice().begin(); it != specification->getManeuverAdvice().end(); ++it)
        {
          auto *advice = (ManoeuvreAdvice *)CALLOC(1, sizeof(ManoeuvreAdvice));
          if (asn_copy(&asn_DEF_ManoeuvreAdvice, (void**)&advice, *it) == 0)
            {
              if (ASN_SEQUENCE_ADD (&adv, advice) != 0)
                {
                  ASN_STRUCT_FREE (asn_DEF_ManoeuvreAdvice, advice);
                }
            }
          else
            {
              ASN_STRUCT_FREE(asn_DEF_ManoeuvreAdvice, advice);
            }
        }
    }
  else if (specification->getAcknowledgmentContainer())
    {
      asn1cpp::setField(MCM_message->payload.mcmContainer.present, McmContainer_PR_acknowledgmentContainer);
      asn1cpp::setField(MCM_message->payload.mcmContainer.choice.acknowledgmentContainer.acknowledgedType, McmType_acknowledgment);
      asn1cpp::setField(MCM_message->payload.mcmContainer.choice.acknowledgmentContainer.generationDeltaTime, compute_timestampIts () % 65536);
    }
  else if (specification->getResponseContainer())
    {
      asn1cpp::setField(MCM_message->payload.mcmContainer.present, McmContainer_PR_responseContainer);
      if (specification->getMCMResponse() == 0)
        {
        asn1cpp::setField(MCM_message->payload.mcmContainer.choice.responseContainer.manouevreResponse, ManouevreResponse_accept);
        }
      else
        {
          asn1cpp::setField(MCM_message->payload.mcmContainer.choice.responseContainer.manouevreResponse, ManouevreResponse_decline);
          asn1cpp::setField(MCM_message->payload.mcmContainer.choice.responseContainer.declineReason, specification->getMCMResponseDeclineReason());
        }
      }
  else if (specification->getTerminatorContainer())
    {
      asn1cpp::setField(MCM_message->payload.mcmContainer.present, McmContainer_PR_terminationContainer);
    }
  else
    {
      std::cerr << "[ERROR] Fatal error! Cannot create containers for the MCM dissemination" << std::endl;
      terminateDissemination();
      return MCM_CANNOT_SEND;
    }

  std::string encode_result = asn1cpp::uper::encode(MCM_message);

  if(encode_result.size()<1)
  {
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
MCBasicService::terminateDissemination()
{
  if(m_terminateFlag==false) {
    m_terminateFlag=true;
  }

  return m_MCM_sent;
}
