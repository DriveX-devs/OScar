#ifndef AIM_JSONSERVER_H
#define AIM_JSONSERVER_H

// json11 include
#include <atomic>
#include "json11.h"
#include "denBasicService.h"
#include "LDMmap.h" 

class JSONserver {
	public:
		JSONserver(ldmmap::LDMMap *db_ptr, DENBasicService *den_service) :
			m_range_m(m_range_m_default), m_db_ptr(db_ptr), m_den_service(den_service) {m_port=49000; m_thread_running=false;};

		JSONserver(ldmmap::LDMMap *db_ptr, long port, DENBasicService *den_service) :
			m_range_m(m_range_m_default), m_db_ptr(db_ptr), m_den_service(den_service) {m_port=port; m_thread_running=false;};

		~JSONserver() {stopServer();}

		void setServerPort(long port) {m_port = port;}

		void changeContextRange(double range_m) {m_range_m=range_m;}
		double getContextRange(void) {return m_range_m;}

		json11::Json::object make_AIM_json(double lat, double lon);

		bool startServer(void);

		// 'true' is returned if the server was successfully stopped, 'false' is returned if the server termination attemp failed
		bool stopServer();

		// Advanced option: change the backlog queue size for the TCP socket
		// Specifying an invalid size (<= 0) will have no effect
		void changeBacklogQueueSize(int size) {if(size>0) m_backlog_size=size;};

		// The user is not expected to call these functions, which are used internally (they must be public due to the usage of pthread; future work will switch to C++ threads)
		int getSock(void) {return m_sockd;}
		void setJSONThreatRunningStatus(bool status) {m_thread_running=status;}

		void setDENService(DENBasicService *debns_ptr) {m_den_service = debns_ptr;}

		json11::Json::object handleRequest(const json11::Json &request);
		json11::Json::object handleDENMRequest(const json11::Json &request);
	private:
		json11::Json::object make_vehicle_standard(uint64_t stationID, 
			double lat, 
			double lon, 
			uint16_t CAM_tstamp,
			uint64_t GN_tstamp,
			ldmmap::OptionalDataItem<long> car_length_mm,
			ldmmap::OptionalDataItem<long> car_width_mm,
			double speed_ms,
			ldmmap::PHpoints *path_history,
			double relative_dist_m,
			ldmmap::e_StationTypeLDM stationType,
			uint64_t diff_ref_tstamp,
			double heading
			);

		static denData fillDenDataFromJson(const json11::Json &request);

		const double m_range_m_default = 15000.0;
		double m_range_m;
		ldmmap::LDMMap *m_db_ptr;
		DENBasicService *m_den_service;
		long m_port;
		std::atomic<bool> m_thread_running;

		int m_backlog_size = 5;

		int m_sockd = -1;
		pthread_t m_tid = -1;
};

#endif // AIM_JSONSERVER_H
