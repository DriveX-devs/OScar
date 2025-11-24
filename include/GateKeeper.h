#include <string>
#include <thread>
#include <mutex>
#include "CBRReader.h"
#include "utils.h"

#ifndef OSCAR_GATEKEEPER_H
#define OSCAR_GATEKEEPER_H

class GateKeeper {
        std::mutex m_gate_mutex;
        float m_Tpg_ms = 0.0;
		float m_Tgo_ms = 0.0;
		float m_Ton_pp = 0.5;
		float m_delta = 0;
        long m_bitrate_bps;
        bool m_adaptive_dcc = false;
        float m_cbr = 0.0;

    public:
        GateKeeper() = default;
        ~GateKeeper() = default;
        void updateDelta(float delta) {m_gate_mutex.lock(); m_delta = delta; m_gate_mutex.unlock();}
        float getDelta() {float delta; m_gate_mutex.lock(); delta = m_delta; m_gate_mutex.unlock(); return delta;}
        void setAdaptiveDCC() {m_adaptive_dcc = true;}
        void setBitRate(long bitrate) {m_bitrate_bps = bitrate;}
        bool getAdaptiveDCC() {return m_adaptive_dcc;}
        void updateTonpp(ssize_t pktSize);
        float getTonpp();
        void updateTgoAfterTransmission();
        void updateTgoAfterDeltaUpdate();
        bool checkGateOpen(int64_t now);
};

#endif // OSCAR_GATEKEEPER_H
