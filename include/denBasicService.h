#ifndef DENBASICSERVICE_H
#define DENBASICSERVICE_H

#include "gpsc.h"
#include "asn1cpp/Seq.hpp"
#include "btp.h"
#include "MetricSupervisor.h"
#include "denData.h"
#include "ITSSOriginatingTableEntry.h"
#include "ITSSReceivingTableEntry.h"
#include <queue>
#include <future>
#include <chrono>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <unordered_set>

extern "C" {
    #include "DENM.h"
}

#define V_O_VALIDITY_INDEX 0
#define T_REPETITION_INDEX 1
#define T_REPETITION_DURATION_INDEX 2

// Timer structures for DENM timer managament
// TODO: it would be better to move those two (TimerEntry and TimerManager) to a dedicated .cpp/.h module
struct TimerEntry {
    std::chrono::steady_clock::time_point expiry;
    std::function<void()> callback;
    uint64_t id;  // Used for timer cancellation

    // TimerEntry operations and comparison
    bool operator>(const TimerEntry& other) const {
        return expiry > other.expiry;
    }

    bool operator<(const TimerEntry& other) const {
        return expiry < other.expiry;
    }

    bool operator==(const TimerEntry& other) const {
        return expiry == other.expiry;
    }
};

class TimerManager {
public:
    // The event loop thread is created when the object is created: one TimerManager for each service should be enough not to spawn too many threads!
    TimerManager() {m_worker = std::thread(&TimerManager::worker_thread, this);}
    ~TimerManager() {stop();};

    // Schedule a new timer event: assign an ID, and call m_cv.notify_one() to unlock the event loop in case this event should go to the top
    uint64_t schedule(uint64_t delay_ms, std::function<void()> callback) {
        std::lock_guard lock(m_mutex);
        const uint64_t id = m_next_id++;
        m_queue.push({std::chrono::steady_clock::now() + std::chrono::milliseconds(delay_ms),std::move(callback),id});
        m_cv.notify_one();
        return id;
    }

    // Delete a timer event
    void cancel(uint64_t timer_id) {
        std::lock_guard lock(m_mutex);
        m_cancelled.insert(timer_id);
    }

    // Force stop the "event loop"; this is executed when the TimerManager object is destroyed
    void stop() {
        m_running.store(false);
        m_cv.notify_all();
        if (m_worker.joinable()) {
            m_worker.join();
        }
    };

private:
    // Main event loop: this is run until stop() is called, that will set the atomic m_running to false
    void worker_thread() {
        while (m_running.load()) {
            std::unique_lock lock(m_mutex);

            if (m_queue.empty()) {
                m_cv.wait(lock, [this]() { return !m_queue.empty() || !m_running.load(); });
                continue;
            }

            auto now = std::chrono::steady_clock::now();
            // Extract the queue top event
            auto& top = m_queue.top();

            if (top.expiry<=now) {
                TimerEntry entry = m_queue.top();
                m_queue.pop();

                // Check if the entry is canceled and th callback should not be called
                if (m_cancelled.count(entry.id)) {
                    m_cancelled.erase(entry.id);
                    continue;
                }

                lock.unlock();
                entry.callback(); // The callback should be executed outside the lock
            } else {
                // This is unblocked either when m_cv.notify_one() is called, or when it is time for the queue top event to be executed
                // This avoids a CPU consuming polling event loop
                m_cv.wait_until(lock, top.expiry);
            }
        }
    }

    std::priority_queue<TimerEntry, std::vector<TimerEntry>, std::greater<TimerEntry>> m_queue;
    std::unordered_set<uint64_t> m_cancelled;
    std::mutex m_mutex;
    std::condition_variable m_cv;
    std::thread m_worker;
    std::atomic<bool> m_running{true};
    std::atomic<uint64_t> m_next_id{1};
};

// DENTimer is just a simple wrapper around TimerManager
struct DENTimer {
    uint64_t timer_id = 0;

    void Cancel(TimerManager& mgr) {
        if (timer_id != 0) {
            mgr.cancel(timer_id);
            timer_id = 0;
        }
    }
};

typedef enum {
    DENM_NO_ERROR = 0,
    DENM_ATTRIBUTES_UNSET = 1,
    DENM_ALLOC_ERROR = 2,
    DENM_WRONG_DE_DATA = 3,
    DENM_WRONG_TABLE_DATA = 4,
    DENM_T_O_VALIDITY_EXPIRED = 5,
    DENM_ASN1_UPER_ENC_ERROR = 6,
    DENM_UNKNOWN_ACTIONID = 7,
    DENM_UNKNOWN_ACTIONID_RECEIVING = 8,
    DENM_UNKNOWN_ACTIONID_ORIGINATING = 9,
    DENM_NON_ACTIVE_ACTIONID_RECEIVING = 10,
    DENM_NON_ACTIVE_ACTIONID_ORIGINATING = 11,
    DENM_TX_SOCKET_NOT_SET = 12,
    DENM_UNKNOWN_STATUS = 13
} DENBasicService_error_t;

class DENBasicService
{
public:
    // This constructor creates a DENBasicService object with the default values.
    DENBasicService();

    /*
     * This constructor creates a DENBasicService object with the specified values.
     * Arguments:
     * - fixed_stationid The station ID of the DENM sender.
     * - fixed_stationtype The station type of the DENM sender.
     */
    DENBasicService(unsigned long fixed_stationid, long fixed_stationtype);

    /*
     * This function triggers the transmission of a DENM message.
     * Arguments:
     * - data The data to be included in the DENM message
     * - actionid The action ID of the DENM message
     * - geoArea Destination GeoArea for the GBC packet
     */
    DENBasicService_error_t appDENM_trigger(denData data, denData::DEN_ActionID_t &actionid, GeoArea_t geoArea);

    /*
     * This function updates a DENM message.
     * Arguments:
     * - data The data to be included in the DENM message
     * - actionid  The action ID of the DENM message
     * - geoArea Destination GeoArea for the GBC packet
     */
    DENBasicService_error_t appDENM_update(denData data, const denData::DEN_ActionID_t actionid,GeoArea_t geoArea);
    /*
     * This function terminates a DENM message.
     *
     * - data The data to be included in the DENM message.
     * - actionid The action ID of the DENM message.
     */
    DENBasicService_error_t appDENM_termination(denData data, const denData::DEN_ActionID_t actionid);

    /*
     * This function must be called when a DENM message is received by the BTP layer.
     * Arguments:
     * - dataIndication The data indication of the received DENM message.
     */
    bool receiveDENM(DENM_t *decoded_denm,denData &den_data);

    /*
     * This function sets the station properties
     * Arguments:
     * - fixed_stationid Station ID of the ITS-S
     * - fixed_stationtype Station type of the ITS-S
     */
    void setStationProperties(unsigned long fixed_stationid, long fixed_stationtype);

    /*
     * This function sets the station ID of the ITS-S
     * Arguments:
     * - fixed_stationid Station ID of the ITS-S
     */
    void setStationID(unsigned long fixed_stationid);

    /*
     * This function sets the station type of the ITS-S
     * Arguments:
     * - fixed_stationtype Station type of the ITS-S
     */
    void setStationType(long fixed_stationtype);

    // Getter for the station ID
    [[nodiscard]] unsigned long getStationID() const {return m_station_id;}

    void setVDP(VDPGPSClient* vdp) {m_vdp=vdp;}
    // A BTP object must always be associated with the DEN Service
    void setBTP(btp *btp) {m_btp = btp;}

    // Cleanup function - should be called when terminating any DEN Service instance
    void cleanup(void);

private:
    btp *m_btp; // The BTP object has a reference to a GeoNetworking object, which in turn has the right socket descriptor to enable the transmission of DENMs
    bool CheckMainAttributes(void);

    DENBasicService_error_t fillDENM(asn1cpp::Seq<DENM>&denm, denData&data, const denData::DEN_ActionID_t actionID, long referenceTimeLong);

    template < typename MEM_PTR > void
    setDENTimer(DENTimer & timer, uint64_t delay_ms, MEM_PTR callback_fcn, denData::DEN_ActionID_t actionID);

    void T_O_ValidityStop(denData::DEN_ActionID_t entry_actionid);
    void T_RepetitionDurationStop(denData::DEN_ActionID_t entry_actionid);
    void T_RepetitionStop(denData::DEN_ActionID_t entry_actionid);

    void T_R_ValidityStop(denData::DEN_ActionID_t entry_actionid);

    template <typename T> static int asn_maybe_assign_optional_data(T *data, T **asn_structure, std::queue<void *>&ptr_queue);

    unsigned long m_station_id; //! Station ID of the ITS-S

    long m_stationtype; //! Station type of the ITS-S

    uint16_t m_seq_number; //! Sequence number of the DENM messages
    std::map<std::pair<unsigned long, long>, ITSSOriginatingTableEntry> m_originatingITSSTable;
    std::map<std::pair<unsigned long, long>, ITSSReceivingTableEntry> m_receivingITSSTable;
    std::map<std::pair<unsigned long, long>, std::tuple<DENTimer, DENTimer, DENTimer> > m_originatingTimerTable;
    std::map<std::pair<unsigned long, long>, DENTimer> m_T_R_Validity_Table; //! Validity timer table
    std::map<std::pair<unsigned long, long>, uint64_t> m_repetitionIntervals; //! Store repetition intervals in ms for rescheduling

    void fillDenDataHeader(asn1cpp::Seq<ItsPduHeader> denm_header, denData&denm_data);
    void fillDenDataManagement(asn1cpp::Seq<ManagementContainer> denm_mgmt_container, denData&denm_data);
    void fillDenDataSituation(asn1cpp::Seq<SituationContainer> denm_situation_container, denData&denm_data);
    void fillDenDataLocation(asn1cpp::Seq<LocationContainer> denm_location_container, denData&denm_data);
    void fillDenDataAlacarte(asn1cpp::Seq<AlacarteContainer> denm_alacarte_container, denData&denm_data);

    /*
    * Mutex to protect m_originatingITSSTable when appDENM_update() and the callback for the expiration of the T_Repetion timer may try to
    * access the map concurrently, resulting in a thread-unsafe code.
    */
    std::mutex T_Repetition_Mutex;

    VDPGPSClient* m_vdp;

    TimerManager m_timerManager;

};

#endif // DENBASICSERVICE_H
