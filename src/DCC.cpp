#include "DCC.h"
#include <stdexcept>


DCC::DCC () = default;

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
    if (m_check_cbr_thread.joinable())
    {
        m_check_cbr_thread.join();
    }
}

void DCC::addCaBasicService(CABasicService* caBasicService)
{
    m_caBasicService = caBasicService;
    m_cam_enabled = true;
}

void DCC::addCpBasicService(CPBasicService* cpBasicService)
{
    m_cpBasicService = cpBasicService;
    m_cpm_enabled = true;
}

void DCC::addVruBasicService(VRUBasicService* vruBasicService)
{
    m_vruBasicService = vruBasicService;
    m_vam_enabled = true;
}

std::unordered_map<DCC::ReactiveState, DCC::ReactiveParameters> DCC::getConfiguration(double Ton, double currentCBR)
{
    std::unordered_map<DCC::ReactiveState, DCC::ReactiveParameters> map;
    if (Ton < 0.5)
      {
        map = m_reactive_parameters_Ton_500_us;
      }
    else if (Ton < 1)
      {
        map = m_reactive_parameters_Ton_1ms;
      }
    else
      {
        // Default
        map = m_reactive_parameters_Ton_1ms;
      }
    ReactiveState old_state = m_current_state;
    if (currentCBR >= map[m_current_state].cbr_threshold && m_current_state != ReactiveState::Restrictive)
    {
      m_current_state = static_cast<ReactiveState>(m_current_state + 1);
    }
    else
    {
      if (m_current_state != ReactiveState::Relaxed)
      {
        ReactiveState prev_state = static_cast<ReactiveState> (m_current_state - 1);
        if (currentCBR <= map[prev_state].cbr_threshold)
        {
          m_current_state = static_cast<ReactiveState>(m_current_state - 1);
        }
      }
    }
    if (old_state == m_current_state)
    {
      map.clear();
    }
    else
      {
        // if (m_current_state > 1) std::cout << "State changed from: " << old_state << "; to: " << m_current_state << std::endl;
      }
    return map;
}

void DCC::setupDCC(unsigned long long dcc_interval, std::string modality, std::string dissemination_interface, float tolerance, bool verbose, std::string log_file)
{
    assert(dcc_interval > 0 && dcc_interval <= MAXIMUM_TIME_WINDOW_DCC);
    assert(modality == "reactive" || modality == "adaptive");
    m_dcc_interval = dcc_interval;
    m_dissemination_interface = dissemination_interface;
    m_verbose = verbose;
    m_tolerance = tolerance;
    m_log_file = log_file;
    m_modality = modality;
    m_main_cbr_reader.setupCBRReader(verbose, dissemination_interface);
    if (m_modality == "adaptive")
    {
        m_second_cbr_reader.setupCBRReader(verbose, dissemination_interface);
    }
}

void DCC::startDCC()
{
  if (m_modality == "adaptive")
  {
    if (m_cam_enabled) m_caBasicService->setAdaptiveDCC();
    if (m_cpm_enabled) m_cpBasicService->setAdaptiveDCC();
    if (m_vam_enabled) m_vruBasicService->setAdaptiveDCC();
    adaptiveDCC();
  }
  else
  {
    reactiveDCC();
  }
}

void DCC::functionReactive()
{
    if (m_dcc_interval == 0 || m_dissemination_interface == "")
    {
        throw std::runtime_error("DCC not set properly.");
    }

    bool retry_flag = true;
    uint64_t start = get_timestamp_us();
    std::this_thread::sleep_for(std::chrono::milliseconds(m_dcc_interval));
    /*
    setNewTxPower(m_parameters_map[m_current_state].tx_power, m_dissemination_interface);
    if (m_cam_enabled) m_caBasicService->setNextCAMDCC(m_parameters_map[m_current_state].tx_inter_packet_time);
    if (m_cpm_enabled) m_cpBasicService->setNextCAMDCC(m_parameters_map[m_current_state].tx_inter_packet_time);
    if (m_vam_enabled) m_vruBasicService->setNextCAMDCC(m_parameters_map[m_current_state].tx_inter_packet_time);
    */

    do
    {
        try
        {
            m_main_cbr_reader.wrapper_start_reading_cbr();
            double currentCbr = m_main_cbr_reader.get_current_cbr();
            if (currentCbr != -1.0f)
            {
                double tx_power = -1;
                long int_pkt_time = -1;
                if (m_cam_enabled)
                {
                    double Ton = m_caBasicService->getTon(); // Milliseconds
                    std::unordered_map<ReactiveState, ReactiveParameters> map = getConfiguration(Ton, currentCbr);
                    if (!map.empty())
                    {
                        tx_power = map[m_current_state].tx_power;
                        int_pkt_time = map[m_current_state].tx_inter_packet_time;
                        setNewTxPower(map[m_current_state].tx_power, m_dissemination_interface);
                        m_caBasicService->setNextCAMDCC(map[m_current_state].tx_inter_packet_time);
                    }
                }

                if (m_cpm_enabled)
                {
                    double Ton = m_cpBasicService->getTon(); // Milliseconds
                    std::unordered_map<ReactiveState, ReactiveParameters> map = getConfiguration(Ton, currentCbr);
                    if (!map.empty())
                    {
                        tx_power = map[m_current_state].tx_power;
                        int_pkt_time = map[m_current_state].tx_inter_packet_time;
                        setNewTxPower(map[m_current_state].tx_power, m_dissemination_interface);
                        m_cpBasicService->setNextCPMDCC (map[m_current_state].tx_inter_packet_time);
                    }
                }

                if (m_vam_enabled)
                {
                    double Ton = m_vruBasicService->getTon(); // Milliseconds
                    std::unordered_map<ReactiveState, ReactiveParameters> map = getConfiguration(Ton, currentCbr);
                    if (!map.empty())
                    {
                        tx_power = map[m_current_state].tx_power;
                        int_pkt_time = map[m_current_state].tx_inter_packet_time;
                        setNewTxPower(map[m_current_state].tx_power, m_dissemination_interface);
                        m_vruBasicService->setNextVAMDCC (map[m_current_state].tx_inter_packet_time);
                    }
                }

                if(m_log_file != "")
                {
                    std::ofstream file;
                    file.open(m_log_file, std::ios::app);

                    auto now_unix = static_cast<double>(get_timestamp_us_realtime())/1000000.0;

                    file << std::fixed << now_unix << "," << static_cast<long int>(get_timestamp_us()-start) << "," << currentCbr << "," << m_current_state << "," << tx_power << "," << int_pkt_time << "\n";
                    file.close();
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(m_dcc_interval));
        }
        catch(const std::exception& e)
        {
            std::cerr << "Error in managing DCC: " << e.what() << std::endl;
            sleep(2);
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
        file << "timestamp_unix_s,timestamp_relative_us,CBR,state,tx_pwr,int_pkt_time\n";
        file.close();
    }
}

void DCC::adaptiveDCCCheckCBR()
{
    bool retry_flag = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(m_T_CBR));
    do
    {
        try
        {
            m_cbr_mutex.lock();
            m_second_cbr_reader.wrapper_start_reading_cbr();
            m_previous_cbr = m_second_cbr_reader.get_current_cbr();
            if (m_previous_cbr == -1) m_previous_cbr = 0;
            m_cbr_mutex.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(m_T_CBR));
        }
        catch(const std::exception& e)
        {
            std::cerr << "Error in managing DCC: " << e.what() << std::endl;
            sleep(2);
            retry_flag = false;
        }
    }
    while(retry_flag);
}

void DCC::functionAdaptive()
{
    if (m_dcc_interval == 0 || m_dissemination_interface == "")
    {
        throw std::runtime_error("DCC not set properly.");
    }
    
    bool retry_flag = true;
    uint64_t start = get_timestamp_us();
    std::this_thread::sleep_for(std::chrono::milliseconds(m_dcc_interval));
    do
    {
        try
        {
            m_cbr_mutex.lock();
            m_main_cbr_reader.wrapper_start_reading_cbr();
            double currentCbr = m_main_cbr_reader.get_current_cbr();
            if (currentCbr != -1.0f)
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
                m_cbr_mutex.unlock();
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

                    auto now_unix = static_cast<double>(get_timestamp_us_realtime())/1000000.0;

                    file << std::fixed << now_unix << "," << static_cast<long int>(get_timestamp_us()-start) << "," << currentCbr << "," << m_CBR_its << "," << m_delta << "\n";
                    file.close();
                }
            }
            else
            {
                m_cbr_mutex.unlock();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(m_dcc_interval));
        }
        catch(const std::exception& e)
        {
            std::cerr << "Error in managing DCC: " << e.what() << std::endl;
            sleep(2);
            retry_flag = false;
        }
    } while (retry_flag);
}

void DCC::adaptiveDCC()
{
    m_dcc_interval = (long) m_dcc_interval / 2;
    m_check_cbr_thread = std::thread(&DCC::adaptiveDCCCheckCBR, this);
    m_adaptive_thread = std::thread(&DCC::functionAdaptive, this);

    if (m_verbose)
    {
        std::cout << "Adaptive DCC thread started" << std::endl;
    }
    if (m_log_file != "")
    {
        std::ofstream file;
        file.open(m_log_file, std::ios::out);
        file << "timestamp_unix_s,timestamp_relative_us,currentCBR,CBRITS,new_delta\n";
        file.close();
    }
}
