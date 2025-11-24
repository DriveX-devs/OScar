#include "MetricSupervisor.h"
#include "utils.h"
#include <unistd.h>

MetricSupervisor::MetricSupervisor() = default;

MetricSupervisor::~MetricSupervisor()
{
    std::cout << "Deleting Metric Supervisor object..." << std::endl;
    if (m_thread.joinable())
    {
        m_thread.join();
    }
    free_nl_socket(m_nl_sock_info);
}

void MetricSupervisor::signalSentPacket(MessageId_t messageId) {
    switch(messageId) {
        case MessageId_cam:
            m_tx_CAMs++;
            break;
        case MessageId_cpm:
            m_tx_CPMs++;
            break;
        case MessageId_vam:
            m_tx_VAMs++;
            break;
        default:
            m_tx_others++;
            break;
    }

    m_tx_total++;
}

void MetricSupervisor::signalReceivedPacket(MessageId_t messageId) {
    switch(messageId) {
        case MessageId_cam:
            m_rx_CAMs++;
            break;
        case MessageId_cpm:
            m_rx_CPMs++;
            break;
        case MessageId_vam:
            m_rx_VAMs++;
            break;
        default:
            m_rx_others++;
            break;
    }

    m_rx_total++;
}

void MetricSupervisor::clearTxRxMetrics() {
    // Reset tx counters
    m_tx_CAMs.store(0);
    m_tx_CPMs.store(0);
    m_tx_VAMs.store(0);
    m_tx_others.store(0);
    m_tx_total.store(0);

    // Reset rx counters
    m_rx_CAMs.store(0);
    m_rx_CPMs.store(0);
    m_rx_VAMs.store(0);
    m_rx_others.store(0);
    m_rx_total.store(0);
}

void MetricSupervisor::setupMetricSupervisor(std::string log_filename, uint64_t time_window, std::string dissemination_interface)
{
    m_log_filename = log_filename;
    m_time_window = time_window;
    m_dissem_vif = dissemination_interface;
    m_cbr_reader.setupCBRReader(false, m_dissem_vif);
}

void MetricSupervisor::writeLogFile()
{
    bool retry_flag = true;
    std::ofstream file_sta_info;
    std::ofstream file_rssi_info;
    file_sta_info.open(m_log_filename + "_station_info.csv", std::ios::out);
    file_rssi_info.open(m_log_filename + "_rssi_info.csv", std::ios::out);
    uint64_t start = get_timestamp_us();
    file_sta_info << "timestamp_unix_s,timestamp_relative_us,CBR,busy_time,tx_time,rx_time,num_tx,num_rx\n";
    file_rssi_info << "timestamp_unix_s,timestamp_relative_us,mac_address,last_rssi\n";
    do
    {
        try
        {
            std::unordered_map<std::string, double> rssi_map = get_all_rssi_from_netlink(m_nl_sock_info);
            m_cbr_reader.wrapper_start_reading_cbr();
            float cbr = m_cbr_reader.get_current_cbr();

            auto now = static_cast<long int>(get_timestamp_us()-start);
            auto now_unix = static_cast<double>(get_timestamp_us_realtime())/1000000.0;
            file_sta_info << std::fixed << now_unix << "," << now << "," << cbr << "," << m_cbr_reader.get_current_busy_time() << "," << m_cbr_reader.get_current_tx_time() << "," << m_cbr_reader.get_current_rx_time() << "," << m_tx_total << "," << m_rx_total << "\n";
            for (auto it = rssi_map.begin(); it != rssi_map.end(); ++it)
            {
                file_rssi_info << std::fixed << now_unix << "," << now << "," << it->first << "," << it->second << "\n";
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(m_time_window));
        } catch(const std::exception& e) {
            std::cerr << "[ERROR] Error in Metric Supervisor: " << e.what() << std::endl;
            sleep(2);
            file_sta_info.close();
            file_rssi_info.close();
            retry_flag = false;
        }
        
    } while (retry_flag);
}

void MetricSupervisor::start()
{
    m_nl_sock_info = open_nl_socket(m_dissem_vif);
    m_thread = std::thread(&MetricSupervisor::writeLogFile, this);

}
