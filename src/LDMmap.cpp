#include "LDMmap.h"
#include "utils.h"
#include <cmath>
#include <iostream>

#define DEFINE_KEYS(k_up,k_low,stationID) \
		uint32_t k_low = stationID & 0x0000FFFF; \
		uint32_t k_up = (stationID & 0xFFFF0000) >> 16;

#define COMPOSE_KEYS(k_up,k_low) (((uint64_t) k_up) << 16) + k_low;

#define DEG_2_RAD(val) ((val)*M_PI/180.0)

namespace ldmmap
{
	static inline double haversineDist(double lat_a, double lon_a, double lat_b, double lon_b) {
		// 12742000 is the mean Earth radius (6371 km) * 2 * 1000 (to convert from km to m)
		return 12742000.0*asin(sqrt(sin(DEG_2_RAD(lat_b-lat_a)/2)*sin(DEG_2_RAD(lat_b-lat_a)/2)+cos(DEG_2_RAD(lat_a))*cos(DEG_2_RAD(lat_b))*sin(DEG_2_RAD(lon_b-lon_a)/2)*sin(DEG_2_RAD(lon_b-lon_a)/2)));
	}

	LDMMap::LDMMap() {
		m_card = 0;

		m_central_lat = 0.0;
		m_central_lon = 0.0;
	}

	LDMMap::~LDMMap() {
		clear();
	}

	LDMMap::LDMMap_error_t 
	LDMMap::insert(vehicleData_t newVehicleData) {
		LDMMap_error_t retval;

		DEFINE_KEYS(key_upper,key_lower,newVehicleData.stationID);

		if(m_card==UINT64_MAX) {
			return LDMMAP_MAP_FULL;
		}

		std::shared_mutex *mapmutexptr;
		std::unordered_map<uint32_t,returnedVehicleData_t> lowerMap;

		if(m_ldmmap.count(key_upper) == 0) {
			mapmutexptr = new std::shared_mutex();
			lowerMap[key_lower].vehData = newVehicleData;
			lowerMap[key_lower].phData = new PHpoints();

			std::lock_guard<std::shared_mutex> lk(m_mainmapmut);

			m_ldmmap[key_upper] = std::pair<std::shared_mutex*,std::unordered_map<uint32_t,returnedVehicleData_t>>(mapmutexptr,lowerMap);
			m_card++;

			retval = LDMMAP_OK;
		} else {
			if(m_ldmmap[key_upper].second.count(key_lower) == 0) {
				m_card++;
				retval = LDMMAP_OK;
			} else {
				retval = LDMMAP_UPDATED;
			}

			std::lock_guard<std::shared_mutex> lk(*m_ldmmap[key_upper].first);
			m_ldmmap[key_upper].second[key_lower].vehData = newVehicleData;

			// INSERT operation -> create a new PHpoints object
			if(retval == LDMMAP_OK) {
				m_ldmmap[key_upper].second[key_lower].phData = new PHpoints();
			}
		}

		// std::cout << "Updating vehicle: " << newVehicleData.stationID << std::endl;
		m_ldmmap[key_upper].second[key_lower].phData->insert(newVehicleData);

		return retval;
	}

	LDMMap::LDMMap_error_t 
	LDMMap::remove(uint64_t stationID) {
		DEFINE_KEYS(key_upper,key_lower,stationID);

		std::shared_lock<std::shared_mutex> lk(m_mainmapmut);

		if(m_ldmmap.count(key_upper) == 0) {
			return LDMMAP_ITEM_NOT_FOUND;
		} else {
			if(m_ldmmap[key_upper].second.count(key_lower) == 0) {
				return LDMMAP_ITEM_NOT_FOUND;
			} else {
				std::lock_guard<std::shared_mutex> lk(*m_ldmmap[key_upper].first);
				m_ldmmap[key_upper].second.erase(stationID);
				m_card--;
			}
		}

		return LDMMAP_OK;
	}

	LDMMap::LDMMap_error_t 
	LDMMap::lookup(uint64_t stationID,returnedVehicleData_t &retVehicleData) {
		DEFINE_KEYS(key_upper,key_lower,stationID);

		std::shared_lock<std::shared_mutex> lk(m_mainmapmut);

		if(m_ldmmap.count(key_upper) == 0) {
			return LDMMAP_ITEM_NOT_FOUND;
		} else {
			if(m_ldmmap[key_upper].second.count(key_lower) == 0) {
				return LDMMAP_ITEM_NOT_FOUND;
			} else {
				std::shared_lock<std::shared_mutex> lk(*m_ldmmap[key_upper].first);
				retVehicleData = m_ldmmap[key_upper].second[key_lower];
			}
		}

		return LDMMAP_OK;
	}

	LDMMap::LDMMap_error_t 
	LDMMap::rangeSelect(double range_m, double lat, double lon, std::vector<returnedVehicleData_t> &selectedVehicles) {
		std::shared_lock<std::shared_mutex> lk(m_mainmapmut);

		for(auto const& [key, val] : m_ldmmap) {
			// Iterate over the single lower maps
			val.first->lock_shared();

			for(auto const& [keyl, vall] : val.second) {
				if(haversineDist(lat,lon,vall.vehData.lat,vall.vehData.lon)<=range_m) {
					selectedVehicles.push_back(vall);
				}
			}

			val.first->unlock_shared();
		}

		return LDMMAP_OK;
	}

	LDMMap::LDMMap_error_t 
	LDMMap::rangeSelect(double range_m, uint64_t stationID, std::vector<returnedVehicleData_t> &selectedVehicles) {
		returnedVehicleData_t retData;

		// Get the latitude and longitude of the speficied vehicle
		if(lookup(stationID,retData)!=LDMMAP_OK) {
			return LDMMAP_ITEM_NOT_FOUND;
		}

		// Perform a rangeSelect() centered on that latitude and longitude values
		return rangeSelect(range_m,retData.vehData.lat,retData.vehData.lon,selectedVehicles);
	}

	void 
	LDMMap::deleteOlderThan(double time_milliseconds) {
		uint64_t now = get_timestamp_us();

		std::shared_lock<std::shared_mutex> lk(m_mainmapmut);

		for(auto& [key, val] : m_ldmmap) {
			// Iterate over the single lower maps
			val.first->lock();

			for (auto mit=val.second.cbegin();mit!=val.second.cend();) {
				if(((double)(now-mit->second.vehData.timestamp_us))/1000.0 > time_milliseconds) {
					mit = val.second.erase(mit);
					m_card--;
				} else {
					++mit;
				}
			}

			val.first->unlock();
		}
	}

	void 
	LDMMap::deleteOlderThanAndExecute(double time_milliseconds,void (*oper_fcn)(uint64_t,void *),void *additional_args) {
		uint64_t now = get_timestamp_us();

		std::shared_lock<std::shared_mutex> lk(m_mainmapmut);

		for(auto& [key, val] : m_ldmmap) {
			// Iterate over the single lower maps
			val.first->lock();

			for (auto mit=val.second.cbegin();mit!=val.second.cend();) {
				if(((double)(now-mit->second.vehData.timestamp_us))/1000.0 > time_milliseconds) {
					// With respect to deleteOlderThan(), this function will also call oper_fcn() for each deleted entry
					oper_fcn(mit->second.vehData.stationID,additional_args);
					mit = val.second.erase(mit);
					m_card--;
				} else {
					++mit;
				}
			}

			val.first->unlock();
		}
	}

	void
	LDMMap::clear() {
		std::shared_lock<std::shared_mutex> lk(m_mainmapmut);

		for(auto& [key, val] : m_ldmmap) {
			// Iterate over the single lower maps and clear them
			// Delete also the shared mutex objects

			for(auto const& [keyl, vall] : val.second) {
				vall.phData->clear();
				delete vall.phData;
			}

			val.second.clear();
			delete val.first;
		}

		// Clear the upper map
		m_ldmmap.clear();

		// Set the cardinality of the map to 0 again
		m_card = 0;
	}

	void 
	LDMMap::printAllContents(std::string label) {
		std::cout << "[" << label << "] Vehicle IDs: ";

		std::shared_lock<std::shared_mutex> lk(m_mainmapmut);

		for(auto const& [key, val] : m_ldmmap) {
			// Iterate over the single lower maps
			val.first->lock_shared();

			for(auto const& [keyl, vall] : val.second) {
				std::cout << vall.vehData.stationID << ", ";
			}

			val.first->unlock_shared();
		}

		std::cout << std::endl;
	}

	void 
	LDMMap::executeOnAllContents(void (*oper_fcn)(vehicleData_t,void *),void *additional_args) {
		std::shared_lock<std::shared_mutex> lk(m_mainmapmut);

		for(auto const& [key, val] : m_ldmmap) {
			// Iterate over the single lower maps
			val.first->lock_shared();

			for(auto const& [keyl, vall] : val.second) {
				// Execute the callback for every entry
				oper_fcn(vall.vehData,additional_args);
			}

			val.first->unlock_shared();
		}
	}
}