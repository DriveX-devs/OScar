#include "GateKeeper.h"
#include <string>
#include <stdexcept>
#include <iostream>

void GateKeeper::updateTgoAfterTransmission()
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
    // std::cout << "AUX 1: " << aux << ", " << m_Ton_pp << ", " << m_delta << std::endl;
    m_gate_mutex.unlock();

    if (aux > 1000)
    {
        aux = 1000;
    }
    // TODO check the update of times
    m_gate_mutex.lock();
    m_Tpg_ms = static_cast<float>(now);
    // Compute next time gate will be open
    m_Tgo_ms = m_Tpg_ms + aux;
    m_gate_mutex.unlock();
}

void GateKeeper::updateTgoAfterDeltaUpdate()
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
    // std::cout << "AUX 2: " << aux << ", " << m_Ton_pp << ", " << m_delta << std::endl;
    m_gate_mutex.lock();
    m_Tgo_ms = m_Tpg_ms + aux;
    m_gate_mutex.unlock();
}

bool GateKeeper::checkGateOpen(int64_t now)
{
    m_gate_mutex.lock();
    // Return true if the gate is open now
    bool ret = now >= m_Tgo_ms;
    m_gate_mutex.unlock();
    return ret;
}

void GateKeeper::updateTonpp(ssize_t pktSize)
{
    double bits = pktSize * 8;
    double tx_duration_s = static_cast<double>(bits) / m_bitrate_bps;
    double total_duration_s = tx_duration_s + (68e-6); // 68 µs extra
    m_gate_mutex.lock();
    m_Ton_pp = total_duration_s * 1000.0;
    m_gate_mutex.unlock();
}

float GateKeeper::getTonpp()
{
    m_gate_mutex.lock();
    float ret = m_Ton_pp;
    m_gate_mutex.unlock();
    return ret;
}