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
}

void MetricSupervisor::setupMetricSupervisor(std::string log_filename, uint64_t time_window, bool enable_CAM_dissemination, bool enable_CPM_dissemination, bool enable_VAM_dissemination, CABasicService *cabs, CPBasicService *cpbs, VRUBasicService *vrub, SocketClient **sockClient, std::string dissemination_interface)
{
    m_log_filename = log_filename;
    m_time_window = time_window;
    m_enable_CAM_dissemination = enable_CAM_dissemination;
    m_enable_CPM_dissemination = enable_CPM_dissemination;
    m_enable_VAM_dissemination = enable_VAM_dissemination;
    m_cabs = cabs;
    m_cpbs = cpbs;
    m_vrubs = vrub;
    m_sock_client = sockClient;
    m_cbr_reader.setupCBRReader(false, dissemination_interface);
}

void MetricSupervisor::writeLogFile()
{
    bool retry_flag = true;
    std::ofstream file;
    file.open(m_log_filename, std::ios::out);
    uint64_t start = get_timestamp_us();
    file << "timestamp_relative_us,rssi_dBm,CBR,busy_time,tx_time,rx_time,num_tx,num_rx\n";
    do 
    {
        try
        {
            float rssi = get_current_rssi();
            m_cbr_reader.wrapper_start_reading_cbr();
            float cbr = m_cbr_reader.get_current_cbr();
            uint64_t num_tx = 0;
            if (m_enable_CAM_dissemination)
            {
                num_tx += m_cabs->get_CAM_sent();
            }

            if (m_enable_CPM_dissemination)
            {
                num_tx += m_cpbs->get_CPM_sent();
            }

            if (m_enable_VAM_dissemination)
            {
                num_tx += m_vrubs->get_VAM_sent();
            }

            int num_rx = 0;

            if (m_sock_client != nullptr && *m_sock_client != nullptr)
            {
                num_rx += (*m_sock_client)->get_received_messages();
            }

            file << static_cast<long int>(get_timestamp_us()-start) << "," << rssi << "," << cbr << "," << m_cbr_reader.get_current_busy_time() << "," << m_cbr_reader.get_current_tx_time() << "," << m_cbr_reader.get_current_rx_time() << "," << num_tx << "," << num_rx << "\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(m_time_window));
        }
        catch(const std::exception& e)
        {
            std::cerr << "Error in Metric Supervisor: " << e.what() << std::endl;
            sleep(5);
            file.close();
            retry_flag = false;
        }
        
    } while (retry_flag);
    
}

void MetricSupervisor::start()
{
    m_thread = std::thread(&MetricSupervisor::writeLogFile, this);
}
