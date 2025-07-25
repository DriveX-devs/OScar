#include "DCC.h"
#include <stdexcept>


DCC::DCC ()
{
    m_parameters_map[ReactiveState::Relaxed] = {0.20, 24.0, 100, -95.0};
    m_parameters_map[ReactiveState::Active1] = {0.30, 18.0, 200, -95.0};
    m_parameters_map[ReactiveState::Active2] = {0.40, 12.0, 400, -95.0};
    m_parameters_map[ReactiveState::Active3] = {0.50, 6.0, 500, -95.0};
    m_parameters_map[ReactiveState::Restrictive] = {1.0, 2.0, 1000, -65.0};
}

DCC::~DCC()
{
    std::cout << "Deleting DCC object..." << std::endl;
    if (m_reactive_thread.joinable())
    {
        m_reactive_thread.join();
    }
    if (m_adaptive_thread.joinable())
    {
        m_adaptive_thread.join();
    }
}

void DCC::setupDCC(unsigned long long dcc_interval, std::string dissemination_interface, CABasicService* cabs, CPBasicService* cpbs, VRUBasicService* vrubs, bool enable_CAM_dissemination, bool enable_CPM_dissemination, bool enable_VAM_dissemination, float tolerance, bool verbose, std::string log_file)
{
    m_dcc_interval = dcc_interval;
    m_time_to_compute = false;
    m_dissemination_interface = dissemination_interface;
    m_verbose = verbose;
    m_tolerance = tolerance;
    m_caBasicService = cabs;
    m_cpBasicService = cpbs;
    m_vruBasicService = vrubs;
    m_cam_enabled = enable_CAM_dissemination;
    m_cpm_enabled = enable_CPM_dissemination;
    m_vam_enabled = enable_VAM_dissemination;
    m_log_file = log_file;
    m_cbr_reader.setupCBRReader(verbose, dissemination_interface);
}

void DCC::functionReactive()
{
    uint64_t start = get_timestamp_us();
    if (m_dcc_interval == 0 || m_dissemination_interface == "")
    {
        throw std::runtime_error("DCC not set properly.");
    }

    bool retry_flag = true;
    setNewTxPower(m_parameters_map[m_current_state].tx_power, m_dissemination_interface);
    if (m_cam_enabled) m_caBasicService->setCheckCamGenMs(m_parameters_map[m_current_state].tx_inter_packet_time);
    if (m_cpm_enabled) m_cpBasicService->setCheckCpmGenMs(m_parameters_map[m_current_state].tx_inter_packet_time);
    if (m_vam_enabled) m_vruBasicService->setCheckVamGenMs(m_parameters_map[m_current_state].tx_inter_packet_time);

    do
    {
        try
        {
            m_cbr_reader.wrapper_start_reading_cbr();
            double currentCbr = m_cbr_reader.get_current_cbr();
            if (currentCbr != -1.0f)
            {
                ReactiveState new_state;
                bool relaxed_flag = true ? m_current_state == ReactiveState::Relaxed : false;
                if (currentCbr > m_parameters_map[m_current_state].cbr_threshold + m_tolerance)
                {
                    new_state = static_cast<ReactiveState> (m_current_state + 1);
                }
                else if (relaxed_flag)
                {
                    new_state = ReactiveState::Relaxed;
                }
                else if (currentCbr < m_parameters_map[static_cast<ReactiveState> (m_current_state - 1)].cbr_threshold - m_tolerance)
                {
                    new_state = static_cast<ReactiveState> (m_current_state - 1);
                }
                else
                {
                    new_state = m_current_state;
                }
                
                if (m_current_state != new_state)
                {
                    if (m_verbose)
                    {
                        std::cout << "Old State: " << m_current_state << ", New State: " << new_state << std::endl;
                    }
                    
                    setNewTxPower(m_parameters_map[new_state].tx_power, m_dissemination_interface);
                    if (m_cam_enabled) m_caBasicService->setCheckCamGenMs(m_parameters_map[new_state].tx_inter_packet_time);
                    if (m_cpm_enabled) m_cpBasicService->setCheckCpmGenMs(m_parameters_map[new_state].tx_inter_packet_time);
                    if (m_vam_enabled) m_vruBasicService->setCheckVamGenMs(m_parameters_map[new_state].tx_inter_packet_time);
                    m_current_state = new_state;
                    // TODO sensitivity through netlink if it is possible
                }

                if(m_log_file != "")
                {
                    std::ofstream file;
                    file.open(m_log_file, std::ios::app);
                    file << static_cast<long int>(get_timestamp_us()-start) << "," << currentCbr << "," << m_current_state << "," << m_parameters_map[m_current_state].tx_power << "," << m_parameters_map[m_current_state].tx_inter_packet_time << "\n";
                    file.close();
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(m_dcc_interval));
        }
        catch(const std::exception& e)
        {
            std::cerr << "Error in managing DCC: " << e.what() << std::endl;
            sleep(5);
            retry_flag = false;
        }
    } while (retry_flag);
}

void DCC::reactiveDCC()
{
    m_reactive_thread = std::thread(&DCC::functionReactive, this);
    if (m_verbose)
    {
        std::cout << "Reactive DCC thread started" << std::endl;
    }
    if (m_log_file != "")
    {
        std::ofstream file;
        file.open(m_log_file, std::ios::out);
        file << "timestamp_relative_us,CBR,state,tx_pwr,int_pkt_time\n";
        file.close();
    }
}

void DCC::functionAdaptive()
{
    if (m_dcc_interval == 0 || m_dissemination_interface == "")
    {
        throw std::runtime_error("DCC not set properly.");
    }
    
    bool retry_flag = true;
    uint64_t start = get_timestamp_us();

    do
    {
        try
        {
            m_cbr_reader.wrapper_start_reading_cbr();
            double currentCbr = m_cbr_reader.get_current_cbr();
            if (m_time_to_compute == false && currentCbr != -1.0f)
            {
                m_previous_cbr = currentCbr;
                m_time_to_compute = true;
            }
            else if (m_time_to_compute == true && currentCbr != -1.0f)
            {
                // Step 1
                if (m_CBR_its != -1.0f)
                {
                    m_CBR_its = 0.5 * m_CBR_its + 0.25 * ((currentCbr + m_previous_cbr) / 2);
                }
                else
                {
                    m_CBR_its = (0.5 * currentCbr + 0.25 * m_previous_cbr) / 2;
                }

                // Step 2
                float delta_offset;
                if ((m_CBR_target - m_CBR_its) > 0)
                {
                    delta_offset = std::min (m_beta * (m_CBR_target - m_CBR_its), m_Gmax);
                }
                else
                {
                    delta_offset = std::max (m_beta * (m_CBR_target - m_CBR_its), m_Gmin);
                }

                 // Step 3
                m_delta = (1 - m_alpha) * m_delta + delta_offset;

                // Step 4
                if (m_delta > m_delta_max)
                {
                    m_delta = m_delta_max;
                }

                // Step 5
                if (m_delta < m_delta_min)
                {
                    m_delta = m_delta_min;
                }

                if (m_cam_enabled) m_caBasicService->toffUpdateAfterDeltaUpdate (m_delta);
                if (m_cpm_enabled) m_cpBasicService->toffUpdateAfterDeltaUpdate (m_delta);
                if (m_vam_enabled) m_vruBasicService->toffUpdateAfterDeltaUpdate (m_delta);

                if(m_log_file != "")
                {
                    std::ofstream file;
                    file.open(m_log_file, std::ios::app);
                    file << static_cast<long int>(get_timestamp_us()-start) << "," << currentCbr << "," << m_CBR_its << "," << m_delta << "\n";
                    file.close();
                }
                m_time_to_compute = false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(m_dcc_interval));
        }
        catch(const std::exception& e)
        {
            std::cerr << "Error in managing DCC: " << e.what() << std::endl;
            sleep(5);
            retry_flag = false;
        }
    } while (retry_flag);
}

void DCC::adaptiveDCC()
{
    m_dcc_interval = (long) m_dcc_interval / 2;
    m_adaptive_thread = std::thread(&DCC::functionAdaptive, this);
    if (m_verbose)
    {
        std::cout << "Adaptive DCC thread started" << std::endl;
    }
    if (m_log_file != "")
    {
        std::ofstream file;
        file.open(m_log_file, std::ios::out);
        file << "timestamp_relative_us,currentCBR,CBRITS,new_delta\n";
        file.close();
    }
}
