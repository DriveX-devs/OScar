#include "ITSSReceivingTableEntry.h"

ITSSReceivingTableEntry::ITSSReceivingTableEntry() {
    m_status = STATE_UNSET;
    m_actionid.originatingStationID = 0;
    m_actionid.sequenceNumber = -1;
    m_referenceTime = -1;
    m_detectionTime = -1;
    m_termination = ITSS_RX_ENTRY_TERMINATION_UNSET;
}

ITSSReceivingTableEntry::ITSSReceivingTableEntry(packetBuffer asnDenmPacket, denm_table_state_t status, denData::DEN_ActionID_t actionID, long referenceTime, long detectionTime) {
    m_denm_encoded = asnDenmPacket;
    m_status = status;
    m_actionid = actionID;
    m_referenceTime = referenceTime;
    m_detectionTime = detectionTime;
    m_termination = ITSS_RX_ENTRY_TERMINATION_UNSET;
}

ITSSReceivingTableEntry::ITSSReceivingTableEntry(packetBuffer asnDenmPacket, denm_table_state_t status, denData::DEN_ActionID_t actionID, long referenceTime, long detectionTime, long *termination_ptr) {
    m_denm_encoded = asnDenmPacket;
    m_status = status;
    m_actionid = actionID;
    m_referenceTime = referenceTime;
    m_detectionTime = detectionTime;
    m_termination = termination_ptr == nullptr ? ITSS_RX_ENTRY_TERMINATION_UNSET : *(termination_ptr);
}
