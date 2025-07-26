#ifndef SOCKET_CLIENT_H
#define SOCKET_CLIENT_H

#include <memory>
#include <atomic>
#include <thread>
#include <cstring>
#include <map>
#include <mutex>

#include "etsiDecoderFrontend.h"
#include "LDMmap.h"
#include "utils.h"
#include "MetricSupervisor.h"

#include "gpsc.h"

extern "C" {
	#include "CAM.h"
	#include "DENM.h"
	#include "VAM.h"
    #include "CollectivePerceptionMessage.h"
}

// Maxium possible size of messages which could be received from the socket
#define MSGBUF_MAXSIZ 2048

// GeoNetworking Ethertype value
#define GN_ETHERTYPE 0x8947

typedef struct options{
	std::string gnss_device;
	std::string dissemination_device;
} options_t;

class SocketClient {
	private:
		int m_raw_rx_sock=-1;
		etsiDecoder::decoderFrontend m_decodeFrontend;
		options_t* m_opts_ptr;
		ldmmap::LDMMap *m_db_ptr;

		bool m_enable_security;
		std::string m_logfile_security;

		std::string m_logfile_name;
		FILE *m_logfile_file;

		std::string m_client_id;

        std::map<uint64_t, std::map<uint64_t,uint64_t>> m_recvCPMmap;  //! Structure mapping, for each CV that we have received a CPM from, the CPM's PO ids with the ego LDM's PO ids


        bool m_printMsg; // If 'true' each received message will be printed (default: 'false' - enable only for debugging purposes)

		std::atomic<bool> m_stopflg;

		std::atomic<bool> m_receptionInProgress;

		inline ldmmap::OptionalDataItem<uint8_t> manage_LowfreqContainer(void *decoded_cam_void,uint32_t stationID);
		void manageMessage(uint8_t *buf,size_t bufsize);

		// Reception thread method
		void rxThr(void);

		std::atomic<int> m_unlock_pd_rd;
		std::atomic<int> m_unlock_pd_wr;

		// std::map<std::string,double> m_routeros_rssi; // Auxiliary RouterOS-based device RSSI map (<MAC address>,<RSSI value>)
		// std::mutex m_routeros_rssi_mutex;

		uint8_t m_self_mac[6]={0}; // Self MAC address; if specified, all the received messages coming from this MAC address will be discarded
		bool m_self_mac_set;

		// std::atomic<bool> m_terminate_routeros_rssi_flag;

		bool denm_decoding_enabled;

		VDPGPSClient *m_gpsc_ptr;

        // nl80211 socket info structure to retrieve RSSI values
        nl_sock_info_t m_nl_sock_info;

        // Statistic: total number of received messages
		uint64_t m_received_msg = 0;

        // Metric Supervisor pointer
        MetricSupervisor *m_met_sup_ptr = nullptr;

	public:
		SocketClient(const int &raw_rx_sock,options_t* opts_ptr, ldmmap::LDMMap *db_ptr, std::string logfile_name,bool enable_security, std::string logfile_security);
		
		~SocketClient() = default;

		uint64_t get_received_messages() {return m_received_msg;}

		void setPrintMsg(bool printMsgEnable) {m_printMsg = printMsgEnable;}

		void setClientID(std::string id) {m_client_id=id;}

		void setSelfMAC(uint8_t self_mac[6]) {memcpy(m_self_mac,self_mac,6); m_self_mac_set=true;}

		void enableDENMdecoding() {denm_decoding_enabled=true;}
		void disableDENMdecoding() {denm_decoding_enabled=false;}

		void startReception(void);
		void stopReception(void);

		void setLoggingGNSSClient(VDPGPSClient *gpsc_ptr) {m_gpsc_ptr=gpsc_ptr;}

        void setMetricSupervisor(MetricSupervisor *met_sup_ptr) {m_met_sup_ptr = met_sup_ptr;}
};

#endif // SOCKET_CLIENT_H
