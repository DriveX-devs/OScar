#include <string>
#include <vector>
#include <fstream>
#include <mutex>
#include <unordered_map>
#include <atomic>
#include "utils.h"
#include "CBRReader.h"

extern "C" {
    #include "MessageId.h"
}

#ifndef OSCAR_METRICSUPERVISOR_H
#define OSCAR_METRICSUPERVISOR_H

class MetricSupervisor {
    private:
        std::thread m_thread;
        uint64_t m_time_window;
        std::string m_log_filename;
        CBRReader m_cbr_reader;

        // nl80211 socket info structure to retrieve RSSI values
        nl_sock_info_t m_nl_sock_info;

        // Dissemination interface name
        std::string m_dissem_vif;

        // Statistics
        std::atomic<uint64_t> m_tx_CAMs=0;
        std::atomic<uint64_t> m_tx_VAMs=0;
        std::atomic<uint64_t> m_tx_CPMs=0;
        std::atomic<uint64_t> m_tx_others=0;
        std::atomic<uint64_t> m_tx_total=0;

        std::atomic<uint64_t> m_rx_CAMs=0;
        std::atomic<uint64_t> m_rx_VAMs=0;
        std::atomic<uint64_t> m_rx_CPMs=0;
        std::atomic<uint64_t> m_rx_others=0;
        std::atomic<uint64_t> m_rx_total=0;

        void writeLogFile();

    public:
        MetricSupervisor();
        ~MetricSupervisor();
        void signalSentPacket(MessageId_t messageId);
        void signalReceivedPacket(MessageId_t messageId);
        void clearTxRxMetrics();
        void setupMetricSupervisor(std::string log_filename, uint64_t time_window, std::string dissemination_interface);
        void start();
};

#endif