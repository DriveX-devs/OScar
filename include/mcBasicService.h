/** @file */
#ifndef MCBASICSERVICE_H
#define MCBASICSERVICE_H

#include "btp.h"
#include "LDMmap.h"
#include "asn_utils.h"
#include "asn1cpp/Seq.hpp"
#include "asn1cpp/Setter.hpp"
#include "asn1cpp/SequenceOf.hpp"

extern "C" {
  #include "MCM.h"
}

//#define CURRENT_VDP_TYPE VDPTraCI

enum ManeuverID
{
  // --- Traffic ---
  Undefined                         = 0x00,
  TransitToHumanDrivenMode          = 0x01,
  TransitToAutomatedDrivingMode     = 0x02,
  DriveStraight                     = 0x03,
  TurnLeft                          = 0x04,
  TurnRight                         = 0x05,
  UTurn                             = 0x06,
  MoveBackward                      = 0x07,
  Overtake                          = 0x08,
  Accelerate                        = 0x09,
  Slowdown                          = 0x0A,
  Stop                              = 0x0B,
  GoToLeftLane                      = 0x0C,
  GoToRightLane                     = 0x0D,
  GetOnHighway                      = 0x0E,
  ExitHighway                       = 0x0F,
  TakeTollingLane                   = 0x10,
  StopAndWait                       = 0x11,

  // --- Safety ---
  EmergencyBrakeAndStop             = 0x40,
  ResetStopAndRestartMoving         = 0x41,
  StayInLane                        = 0x42,
  ResetStayInLane                   = 0x43,
  StayAway                          = 0x44,
  ResetStayAway                     = 0x45,

  // --- Group Manoeuvre ---
  FollowMe                          = 0x80,
  ExistingGroup                     = 0x81,
  TemporarilyDisbandGroup           = 0x82,
  ConstituteTemporaryGroup          = 0x83,
  DisbandTemporaryGroup             = 0x84
};

  typedef enum {
    MCM_NO_ERROR=0,
    MCM_WRONG_INTERVAL=1,
    MCM_ALLOC_ERROR=2,
    MCM_NO_RSU_CONTAINER=3,
    MCM_ASN1_UPER_ENC_ERROR=4,
    MCM_CANNOT_SEND=5
  } MCBasicService_error_t;

  class MCSpecification
  {
  public:

    struct AllocationInfo {
        void* ptr;
        asn_TYPE_descriptor_t* type;
        bool ownedByParent;
    };

    MCSpecification()
        : m_mcm_type(0), m_mcm_its_role(0), m_mcm_status(0),
          m_mcm_concept(0), m_mcm_goal(0), m_maneuver_id(ManeuverID::Undefined), m_mcm_cost(0),
          m_vehicle_maneuver_container(false),
          m_vehicle_advise_container(false),
          m_vehicle_acknowledgement_container(false),
          m_vehicle_response_container(false),
          m_vehicle_terminator_container(false),
          m_vehicle_type{}, m_mcm_response(0), m_creation_error(false), m_creation_error_str("") {}

    template <typename T>
    T* create(asn_TYPE_descriptor_t& typeDescriptor)
    {
      T* new_item = (T *)CALLOC(1, sizeof(T));
      if (new_item)
      {
          AllocationInfo info;
          info.ptr = new_item;
          info.type = &typeDescriptor;
          info.ownedByParent = false;

          m_allocations[new_item] = info;

          // Top level types
          if (&typeDescriptor == &asn_DEF_ManoeuvreAdvice) {
            m_maneuver_advice_list.push_back((ManoeuvreAdvice*)new_item);
          } else if (&typeDescriptor == &asn_DEF_Submanoeuvre) {
            m_submaneuver_description_list.push_back((Submanoeuvre_t*)new_item);
          }
      }
      return new_item;
    }

    template <typename T, typename Container, typename Item>
    int add(T type, Container* container, Item* item) {
      if (ASN_SEQUENCE_ADD(container, item) != 0) {
          unregisterAllocation(item);
          ASN_STRUCT_FREE(type, item);
          return 0;
        }
      else {
        markOwned(item);
        return 1;
      }
    }

    template<typename T>
    void markOwned(T* obj) {
      auto it = m_allocations.find(obj);

      if (it != m_allocations.end()) {
          it->second.ownedByParent = true;
      }
    }

    template<typename T>
    void unregisterAllocation(T* obj) {
      m_allocations.erase(obj);
    }

    template <typename Q>
    void free(Q* item) {
      if (!item) return;

      auto it = m_allocations.find(item);
      if (it != m_allocations.end()) {
          // 1. Free the memory using the descriptor we saved earlier
          ASN_STRUCT_FREE(*it->second.type, it->first);
          // 2. Remove it from the map so cleanup() doesn't touch it again
          m_allocations.erase(it);
      } else {
      }
    }

    template<typename T, typename Q>
    void set(T* container, Q item) {
      asn1cpp::setField(container, item);
    }

    void cleanup() {
      for (auto& [ptr, info] : m_allocations) {
        if (!info.ownedByParent) {
            ASN_STRUCT_FREE(*info.type, ptr);
        }
      }
      m_allocations.clear();
    }

    ~MCSpecification() {
      cleanup();
    }

    bool checkContainers() {
      int count = m_vehicle_advise_container
                  + m_vehicle_maneuver_container
                  + m_vehicle_acknowledgement_container
                  + m_vehicle_response_container
                  + m_vehicle_terminator_container;
      return count == 1;
    }

    void setAdviseContainer() {m_vehicle_advise_container = true;};
    bool getAdviseContainer() {return m_vehicle_advise_container;};
    void setManeuverContainer() {m_vehicle_maneuver_container = true;};
    bool getManeuverContainer() {return m_vehicle_maneuver_container;};
    void setAcknowledgmentContainer() {m_vehicle_acknowledgement_container = true;};
    bool getAcknowledgmentContainer() {return m_vehicle_acknowledgement_container;};
    void setResponseContainer() { m_vehicle_response_container = true; };
    bool getResponseContainer() { return m_vehicle_response_container; };
    void setTerminatorContainer() {m_vehicle_terminator_container = true;};
    bool getTerminatorContainer() {return m_vehicle_terminator_container;};
    void setMCMType(long type) {m_mcm_type = type;};
    long getMCMType() {return m_mcm_type;};
    void setMCMItsRole(long role) { m_mcm_its_role = role; };
    long getMCMItsRole() { return m_mcm_its_role; };
    void setMCMStatus(long status) { m_mcm_status = status; };
    long getMCMStatus() { return m_mcm_status; };
    void setMCMConcept(long concept) { m_mcm_concept = concept; };
    long getMCMConcept() { return m_mcm_concept; };
    void setMCMGoal(long goal) { m_mcm_goal = goal; };
    long getMCMGoal() { return m_mcm_goal; };
    void setMCMCost(long cost) { m_mcm_cost = cost; };
    long getMCMCost() { return m_mcm_cost; };
    void setMCMResponse(long response) { m_mcm_response = response; };
    long getMCMResponse() { return m_mcm_response; };
    void setManeuverID(ManeuverID id) { m_maneuver_id = id; };
    void setCoordinatorID(StationId_t coordinator) { m_coordinator_id = coordinator; };
    StationId_t getCoordinatorID() { return m_coordinator_id; };
    ManeuverID getManeuverID() { return m_maneuver_id; };
    void setMCMResponseDeclineReason(DeclineReason_t reason) {m_decline_reason = reason;};
    DeclineReason_t getMCMResponseDeclineReason() {return m_decline_reason;};
    void setVehicleType(Iso3833VehicleType type) { m_vehicle_type = type; };
    Iso3833VehicleType getVehicleType() { return m_vehicle_type; };
    const std::vector<ManoeuvreAdvice*>& getManeuverAdviceList() const {
      return m_maneuver_advice_list;
    }
    const std::vector<Submanoeuvre_t*>& getSubmanoeuvreDescriptionList() const {
      return m_submaneuver_description_list;
    }
    void setCreationError(std::string reason) {m_creation_error = true; m_creation_error_str = reason;}
    std::tuple<bool, std::string> getCreationError() {return {m_creation_error, m_creation_error_str};}

  private:
    long m_mcm_type;
    ManeuverID m_maneuver_id;
    long m_mcm_its_role;
    long m_mcm_status;
    long m_mcm_concept;
    long m_mcm_goal;
    long m_mcm_cost;
    bool m_vehicle_maneuver_container;
    bool m_vehicle_advise_container;
    bool m_vehicle_acknowledgement_container;
    bool m_vehicle_response_container;
    bool m_vehicle_terminator_container;
    bool m_creation_error;
    std::string m_creation_error_str;
    std::vector<SubmanoeuvreDescription*> m_submaneuver_description; // For Vehicle Maneuver Container
    std::vector<ManoeuvreAdvice*> m_maneuver_advice; // For Vehicle Maneuver Container and Vehicle Advice Container
    DeclineReason_t m_decline_reason;
    StationId_t m_coordinator_id;

    Iso3833VehicleType m_vehicle_type;
    long m_mcm_response;

    std::unordered_map<void*, AllocationInfo> m_allocations;
    std::vector<ManoeuvreAdvice*> m_maneuver_advice_list;
    std::vector<Submanoeuvre_t*> m_submaneuver_description_list;
  };


  /**
   * \ingroup automotive
   * \brief This class implements the Maneuver Coordination Basic Service
   *
   * This class implements the Maneuver Coordination Basic Service (MC Basic Service) as defined in ETSI TS 103 561 V0.0.10 (2025-01).
   * The MC Basic Service is a service that allows vehicles and RSUs to exchange information about their current maneuver intentions.
   *
   */
  class MCBasicService
  {
  public:
    /**
     * \brief Default constructor
     *
     * This constructor creates a MC Basic Service object with default values.
     */
    MCBasicService();
    ~MCBasicService();
    /**
     * @brief Constructor
     *
     * This constructor creates a MC Basic Service object with the given station ID and station type.
     *
     * @param fixed_stationid The station ID of the vehicle or RSU
     * @param fixed_stationtype The station type of the vehicle or RSU
     * @param vdp  The VDP object to be used by the MC Basic Service
     * @param real_time  If true, the MC Basic Service will generate MCM messages using real time timestamps
     * @param is_vehicle  If true, the MC Basic Service will generate MCM messages as a vehicle, otherwise it will generate MCM messages as an RSUs
     */
    MCBasicService(unsigned long fixed_stationid,long fixed_stationtype,VDPGPSClient* vdp,bool real_time,bool is_vehicle);

    /**
     * @brief Set the station properties
     *
     * This function sets the station ID and station type of the vehicle or RSU.
     *
     * @param fixed_stationid   The station ID of the vehicle or RSU
     * @param fixed_stationtype   The station type of the vehicle or RSU
     */
    void setStationProperties(unsigned long fixed_stationid,long fixed_stationtype,double latitude_deg=0.0,double longitude_deg=0.0);

    /**
     * @brief  Set the fixed position of the RSU
     *
     * This function sets the fixed position of the RSU.
     *
     * @param latitude_deg
     * @param longitude_deg
     */
    void setFixedPositionRSU(double latitude_deg, double longitude_deg);

    /**
     * @brief Set the vehicle station ID
     * @param fixed_stationid
     */
    void setStationID(unsigned long fixed_stationid);
    /**
     * @brief Set the station type
     * @param fixed_stationtype
     */
    void setStationType(long fixed_stationtype);
    void setRSU() {m_vehicle=false;}
    /**
     * @brief Set the VDP object
     *
     * This function sets the VDP object to be used by the MC Basic Service.
     *
     * @param vdp   The VDP object to be used by the MC Basic Service
     */
    void setVDP(VDPGPSClient* vdp) {m_vdp=vdp;}
    /**
     * @brief Set the LDM object
     *
     * This function sets the LDM object to be used by the MC Basic Service.
     *
     * @param LDM   The LDM object to be used by the MC Basic Service
     */
    void setLDM(ldmmap::LDMMap* LDM){m_LDM = LDM;}
    /**
     * @brief Set the BTP object
     *
     * This function sets the BTP object to be used by the MC Basic Service.
     *
     * @param btp   The BTP object to be used by the MC Basic Service
     */
    void setBTP(btp* btp){m_btp = btp;}

    void changeRSUGenInterval(long RSU_GenMCM_ms) {m_RSU_GenMCM_ms=RSU_GenMCM_ms;}

    /**
     * @brief Stop the MCM dissemination
     *
     * This function stops the MCM dissemination process.
     */
    uint64_t terminateDissemination();

    long T_GenMCMMin_ms = 100;
    long T_GenMCMMax_ms = 10000;

    void SetLogTriggering(bool log, std::string log_filename) {m_log_triggering = log; m_log_filename = log_filename;};

    // void write_log_triggering(bool condition_verified, float head_diff, float pos_diff, float speed_diff, long time_difference, std::string data_head, std::string data_pos, std::string data_speed, std::string data_time, std::string data_dcc);

    /**
     * @brief Generate and encode a MCM message
     *
     * This function generates and encodes a MCM message.
     *
     * @return MCBasicService_error_t   The error code
     */
    MCBasicService_error_t generateAndEncodeMCM(MCSpecification* specification);

    void setMetricSupervisor(MetricSupervisor *met_sup_ptr) {m_met_sup_ptr = met_sup_ptr;}


  private:
    const size_t m_MaxPHLength = 23;

    /**
     * @brief Check the conditions to generate a MCM message
     *
     * This is function periodically checks the conditions to generate a MCM message according to the ETSI TS 103 561 V0.0.10 (2025-01) standard.
     */
    void checkMCMConditions();
    int64_t computeTimestampUInt64();

    btp* m_btp; //! BTP object

    double m_T_CheckMCMGen_ms; //! MCM generation check interval

    long m_T_GenMCM_ms; //! MCM generation interval

    long m_RSU_GenMCM_ms; //! MCM generation interval for RSU ITS-Ss

    int64_t lastMCMGen; //! Last MCM generation timestamp


    std::atomic<bool> m_terminateFlag;

    bool m_vehicle; //! If true, the MC Basic Service will generate MCM messages as a vehicle, otherwise it will generate MCM messages as an RSU

    VDPGPSClient* m_vdp; //! VDP object


    ldmmap::LDMMap* m_LDM; //!< LDM object

    StationId_t m_station_id; //! Station ID

    StationType_t m_stationtype; //! Station type

    uint64_t m_MCM_sent;//! Number of MCMs successfully sent since the MC Basic Service has been started. The MC Basic Service can count up to 18446744073709551615 (UINT64_MAX) MCMs

    //High frequency RSU container
    asn1cpp::Seq<RSUContainerHighFrequency> m_protectedCommunicationsZonesRSU;
    double m_RSUlon;
    double m_RSUlat;

    double m_last_transmission = 0;

    bool m_log_triggering = false;
    std::string m_log_filename;

    // Statistics: number of MCMs sent per triggering conditions
    uint64_t m_pos_sent = 0;
    uint64_t m_speed_sent = 0;
    uint64_t m_head_sent = 0;
    uint64_t m_time_sent = 0;

    long m_T_next_dcc = -1;
    MetricSupervisor *m_met_sup_ptr = nullptr;
    uint8_t m_priority = 3;

  };

#endif // MCBASICSERVICE_H
