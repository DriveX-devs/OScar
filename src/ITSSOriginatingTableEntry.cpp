#include "ITSSOriginatingTableEntry.h"

  ITSSOriginatingTableEntry::ITSSOriginatingTableEntry() {
    m_status = STATE_UNSET;
    m_actionid.originatingStationID = 0;
    m_actionid.sequenceNumber = -1;
    m_referenceTime = -1;
  }

  ITSSOriginatingTableEntry::ITSSOriginatingTableEntry(packetBuffer asnDenmPacket, denm_table_state_t status, denData::DEN_ActionID_t actionID)
  {
    m_denm_encoded = asnDenmPacket;
    m_status = status;
    m_actionid = actionID;
  }

  ITSSOriginatingTableEntry::ITSSOriginatingTableEntry(packetBuffer asnDenmPacket, denm_table_state_t status, denData::DEN_ActionID_t actionID, long referenceTime)
  {
    m_denm_encoded = asnDenmPacket;
    m_status = status;
    m_actionid = actionID;
    m_referenceTime = referenceTime;
  }