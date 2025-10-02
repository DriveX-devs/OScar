#include "DCC.h"
#include <cassert>
#include <stdexcept>
#include <algorithm>


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
    if (m_check_queue_thread.joinable())
    {
        m_check_queue_thread.join();
    }
}

std::unordered_map<DCC::ReactiveState, DCC::ReactiveParameters>
DCC::getConfiguration(double Ton, double currentCBR)
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
    return map;
}

void DCC::setupDCC(unsigned long long dcc_interval, std::string modality, std::string dissemination_interface, float cbr_target, float tolerance, bool verbose, int queue_lenght, int max_lifetime, std::string log_file)
{
    assert(dcc_interval > 0 && dcc_interval <= MAXIMUM_TIME_WINDOW_DCC);
    assert(modality == "reactive" || modality == "adaptive");
    m_dcc_interval = dcc_interval;
    m_dissemination_interface = dissemination_interface;
    m_verbose = verbose;
    m_tolerance = tolerance;
    m_log_file = log_file;
    m_modality = modality;
    m_CBR_target = cbr_target;
    m_queue_lenght = queue_lenght;
    m_lifetime = max_lifetime;
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
    adaptiveDCC();
  }
  else
  {
    reactiveDCC();
  }
  if (m_queue_lenght > 0)
  {
    m_check_queue_thread = std::thread(&DCC::checkQueue, this);
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

    do
    {
        try
        {
            m_main_cbr_reader.wrapper_start_reading_cbr();
            double currentCbr = m_main_cbr_reader.get_current_cbr();
            if (currentCbr != -1.0f)
            {
                float Ton = getTonpp(); // Milliseconds
                std::unordered_map<ReactiveState, ReactiveParameters> map = getConfiguration(Ton, currentCbr);
                long tx_power = map[m_current_state].tx_power;
                long int_pkt_time = map[m_current_state].tx_inter_packet_time;
                setNewTxPower(map[m_current_state].tx_power, m_dissemination_interface);
                updateTgoAfterStateCheck(map[m_current_state].tx_inter_packet_time);

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
                float old_delta = getDelta();
                float new_delta = (1 - m_alpha) * old_delta + delta_offset;

                // Step 4
                if (new_delta > m_delta_max)
                {
                    new_delta = m_delta_max;
                }

                // Step 5
                if (new_delta < m_delta_min)
                {
                    new_delta = m_delta_min;
                }

                updateDelta(new_delta);
                updateTgoAfterDeltaUpdate();

                if(m_log_file != "")
                {
                    std::ofstream file;
                    file.open(m_log_file, std::ios::app);

                    auto now_unix = static_cast<double>(get_timestamp_us_realtime())/1000000.0;

                    file << std::fixed << now_unix << "," << static_cast<long int>(get_timestamp_us()-start) << "," << currentCbr << "," << m_CBR_its << "," << new_delta << "\n";
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

void DCC::updateTgoAfterTransmission()
{
    struct timespec tv;
    clock_gettime (CLOCK_MONOTONIC, &tv);
    int64_t now = (tv.tv_sec * 1e9 + tv.tv_nsec)/1e6;
    float aux;
    m_gate_mutex.lock();
    if (m_Ton_pp / m_delta > 25)
    {
        aux = m_Ton_pp / m_delta;
    }
    else
    {
        aux = 25;
    }
    m_gate_mutex.unlock();

    if (aux > 1000)
    {
        aux = 1000;
    }
    m_gate_mutex.lock();
    m_Tpg_ms = static_cast<float>(now);
    // Compute next time gate will be open
    m_Tgo_ms = m_Tpg_ms + aux;
    m_Toff_ms = aux;
    m_gate_mutex.unlock();
    m_check_queue_cv.notify_all();
}

void DCC::updateTgoAfterDeltaUpdate()
{
    struct timespec tv;
    clock_gettime (CLOCK_MONOTONIC, &tv);
    int64_t now = (tv.tv_sec * 1e9 + tv.tv_nsec)/1e6;
    if (checkGateOpen(now))
    {
        // Update just if the gate is currently closed, otherwise return
        return;
    }
    m_gate_mutex.lock();
    float aux = m_Ton_pp / m_delta;
    aux = aux * ((m_Tgo_ms - now) / (m_Tgo_ms - m_Tpg_ms));
    aux = aux + (now - m_Tpg_ms);
    m_gate_mutex.unlock();
    if (aux < 25)
    {
        aux = 25;
    }
    if (aux > 1000)
    {
        aux = 1000;
    }
    m_gate_mutex.lock();
    m_Tgo_ms = m_Tpg_ms + aux;
    m_Toff_ms = aux;
    m_gate_mutex.unlock();
    m_check_queue_cv.notify_all();
}

bool DCC::checkGateOpen(int64_t now)
{
    m_gate_mutex.lock();
    // Return true if the gate is open now
    bool ret = now - m_last_tx >= m_Toff_ms;
    m_gate_mutex.unlock();
    return ret;
}

void DCC::updateTonpp(ssize_t pktSize)
{
    double bits = pktSize * 8;
    double tx_duration_s = static_cast<double>(bits) / m_bitrate_bps;
    double total_duration_s = tx_duration_s + (68e-6); // 68 µs extra
    m_gate_mutex.lock();
    m_Ton_pp = total_duration_s * 1000.0;
    m_gate_mutex.unlock();
}

float DCC::getTonpp()
{
    m_gate_mutex.lock();
    float ret = m_Ton_pp;
    m_gate_mutex.unlock();
    return ret;
}

void DCC::updateTgoAfterStateCheck(uint32_t Toff)
{
    struct timespec tv;
    clock_gettime (CLOCK_MONOTONIC, &tv);
    int64_t now = (tv.tv_sec * 1e9 + tv.tv_nsec)/1e6;
    m_gate_mutex.lock();
    m_Tgo_ms = now + Toff;
    m_Toff_ms = Toff;
    m_gate_mutex.unlock();
    m_check_queue_cv.notify_all();
}

void 
DCC::cleanQueues(int now)
{
    std::vector<int> to_delete;
    int counter = 0;
    
    for(auto it = m_dcc_queue_dp0.begin(); it != m_dcc_queue_dp0.end(); ++it)
    {
        if (now > (*it).time + m_lifetime)
        {
            m_dropped_by_gate ++;
            to_delete.push_back(counter);
        }
        counter ++;
    }
    for(int i = 0; i < to_delete.size(); i++)
    {
        m_dcc_queue_dp0.erase(m_dcc_queue_dp0.begin() + to_delete[i]);
    }

    to_delete.clear();
    counter = 0;
    for(auto it = m_dcc_queue_dp1.begin(); it != m_dcc_queue_dp1.end(); ++it)
    {
        if (now > (*it).time + m_lifetime)
        {
            m_dropped_by_gate ++;
            to_delete.push_back(counter);
        }
        counter ++;
    }
    for(int i = 0; i < to_delete.size(); i++)
    {
        m_dcc_queue_dp1.erase(m_dcc_queue_dp1.begin() + to_delete[i]);
    }
    
    to_delete.clear();
    counter = 0;
    for(auto it = m_dcc_queue_dp2.begin(); it != m_dcc_queue_dp2.end(); ++it)
    {
        if (now > (*it).time + m_lifetime)
        {
            m_dropped_by_gate ++;
            to_delete.push_back(counter);
        }
        counter ++;
    }
    for(int i = 0; i < to_delete.size(); i++)
    {
        m_dcc_queue_dp2.erase(m_dcc_queue_dp2.begin() + to_delete[i]);
    }
    
    to_delete.clear();
    counter = 0;
    for(auto it = m_dcc_queue_dp3.begin(); it != m_dcc_queue_dp3.end(); ++it)
    {
        if (now > (*it).time + m_lifetime)
        {
            m_dropped_by_gate ++;
            to_delete.push_back(counter);
        }
        counter ++;
    }
    for(int i = 0; i < to_delete.size(); i++)
    {
        m_dcc_queue_dp3.erase(m_dcc_queue_dp3.begin() + to_delete[i]);
    }
    
}

void 
DCC::enqueue(int priority, Packet p)
{
    if (m_queue_lenght == 0)
    {
        m_dropped_by_gate ++;
        return;
    }
    bool inserted = false;
    struct timespec tv;
    clock_gettime (CLOCK_MONOTONIC, &tv);
    int64_t now = (tv.tv_sec * 1e9 + tv.tv_nsec)/1e6;
    m_gate_mutex.lock();
    cleanQueues(now);
    switch(priority)
    {
        case 0:
        if (m_dcc_queue_dp0.size() < m_queue_lenght)
        {
            // The queue is not full, we can accept a new packet
            m_dcc_queue_dp0.push_back(p);
            inserted = true;
        }
        break;
        case 1:
        if (m_dcc_queue_dp1.size()< m_queue_lenght)
        {
            m_dcc_queue_dp1.push_back(p);
            inserted = true;
        }
        break;
        case 2:
        if (m_dcc_queue_dp2.size() < m_queue_lenght)
        {
            m_dcc_queue_dp2.push_back(p);
            inserted = true;
        }
        break;
        case 3:
        if (m_dcc_queue_dp3.size() < m_queue_lenght)
        {
            m_dcc_queue_dp3.push_back(p);
            inserted = true;
        }
        break;
    }
    m_gate_mutex.unlock();
    if (inserted)
    {
        m_check_queue_cv.notify_all();
    }
}

std::tuple<bool, Packet>
DCC::dequeue(int priority)
{
    struct timespec tv;
    clock_gettime (CLOCK_MONOTONIC, &tv);
    int64_t now = (tv.tv_sec * 1e9 + tv.tv_nsec)/1e6;
    m_gate_mutex.lock();
    cleanQueues(now);
    Packet pkt;
    bool found = false;

    if (m_dcc_queue_dp0.size() > 0 && priority >= 0)
    {
        pkt = *m_dcc_queue_dp0.begin();
        m_dcc_queue_dp0.erase(m_dcc_queue_dp0.begin());
        found = true;
    } else if (m_dcc_queue_dp1.size() > 0 && priority >= 1)
    {
        pkt = *m_dcc_queue_dp1.begin();
        m_dcc_queue_dp1.erase(m_dcc_queue_dp1.begin());
        found = true;
    } else if (m_dcc_queue_dp2.size() > 0 && priority >= 2)
    {
        pkt = *m_dcc_queue_dp2.begin();
        m_dcc_queue_dp2.erase(m_dcc_queue_dp2.begin());
        found = true;
    } else if (m_dcc_queue_dp3.size() > 0 && priority >= 3)
    {
        pkt = *m_dcc_queue_dp3.begin();
        m_dcc_queue_dp3.erase(m_dcc_queue_dp3.begin());
        found = true;
    }
    
    m_gate_mutex.unlock();
    return std::tuple<bool, Packet> (found, pkt);
}

void DCC::checkQueue()
{
    std::unique_lock<std::mutex> lock(m_check_queue_mutex);

    while (!m_stop_thread)
    {
        struct timespec tv;
        clock_gettime (CLOCK_MONOTONIC, &tv);
        int64_t now = (tv.tv_sec * 1e9 + tv.tv_nsec)/1e6;

        m_gate_mutex.lock();

        int64_t elapsed = now - m_last_tx;
        int64_t wait_time = (m_Toff_ms >= elapsed) ? (m_Toff_ms - elapsed) : 0;

        bool queues_have_packets = !(m_dcc_queue_dp0.empty() && m_dcc_queue_dp1.empty() && m_dcc_queue_dp2.empty() && m_dcc_queue_dp3.empty());
        
        if (elapsed >= m_Toff_ms && queues_have_packets)
        {
            // Clock is off, perform actions
            lock.unlock();

            // Gate is opened
            // Set priority = 4 (maximum is 3) so that the dequeue will check all the priority queues
            m_gate_mutex.unlock();
            std::tuple<bool, Packet> value = this->dequeue(4);
            bool status = std::get<0>(value);
			if (status)
			{
				Packet pkt_to_send = std::get<1>(value);
				m_send_callback(pkt_to_send);
			}
            else
            {
            }
            lock.lock();
        } 
        else 
        {
            // Wait until Toff expires or Toff changes
            m_gate_mutex.unlock();
            m_check_queue_cv.wait_for(lock, std::chrono::milliseconds(wait_time), [&]{
                std::lock_guard<std::mutex> gate_lock(m_gate_mutex);
                clock_gettime(CLOCK_MONOTONIC, &tv);
                now = (tv.tv_sec * 1e9 + tv.tv_nsec)/1e6;
                elapsed = now - m_last_tx;
                bool are_queues_empty = m_dcc_queue_dp0.empty() && m_dcc_queue_dp1.empty() && m_dcc_queue_dp2.empty() && m_dcc_queue_dp3.empty();
                return (elapsed >= m_Toff_ms && !are_queues_empty) || m_stop_thread;
            });
        }
    }
}

void DCC::setSendCallback(std::function<void(const Packet&)> cb)
{
    m_send_callback = std::move(cb);
}
