#include "PHpoints.h"
#include <cmath>
#include <iostream> // [TBR]

namespace ldmmap
{
	static inline double haversineDist(double lat_a, double lon_a, double lat_b, double lon_b) {
		// 12742000 is the mean Earth radius (6371 km) * 2 * 1000 (to convert from km to m)
		return 12742000.0*asin(sqrt(sin(DEG_2_RAD_AIM(lat_b-lat_a)/2)*sin(DEG_2_RAD_AIM(lat_b-lat_a)/2)+cos(DEG_2_RAD_AIM(lat_a))*cos(DEG_2_RAD_AIM(lat_b))*sin(DEG_2_RAD_AIM(lon_b-lon_a)/2)*sin(DEG_2_RAD_AIM(lon_b-lon_a)/2)));
	}

	static inline double angDiff(double ang1, double ang2) {
		double angDiff;
		angDiff=ang1-ang2;

		if(angDiff>180) {
			angDiff-=360;
		} else if(angDiff<-180) {
			angDiff+=360;
		}

		return angDiff;
	}

	PHpoints::PHpoints() {
		m_distance_limit = 300.0; // Default: last 300 m
		m_min_dist_m = 1.0; // Default: 1 m
		m_max_dist_m = 20.0; // Default: 20 m
		m_max_heading_diff_degs = 10.0; // Default: 10 degrees 

		m_PHpoints_size = 0;
		m_next_idx = 0;
		m_oldest_idx = 0;

		m_removeOnlyOne = false;
		m_iterateFull = false;

		m_vectorReservedSize = 300; // 300 is the result of ceil(m_distance_limit/m_min_dist_m), i.e. ceil(300/1)

		m_pDataArray.reserve(m_vectorReservedSize); // 300 is the result of ceil(m_distance_limit/m_min_dist_m), i.e. ceil(300/1)
	}

	PHpoints::PHpoints(double distance_limit, double min_dist_m, double max_dist_m, double max_heading_diff_degs) {
		m_distance_limit = distance_limit;
		m_min_dist_m = min_dist_m;
		m_max_dist_m = max_dist_m;
		m_max_heading_diff_degs = max_heading_diff_degs;

		m_PHpoints_size = 0;
		m_next_idx = 0;
		m_oldest_idx = 0;

		m_removeOnlyOne = false;
		m_iterateFull = false;

		m_vectorReservedSize = (int) ceil(m_distance_limit/m_min_dist_m);

		m_pDataArray.reserve(m_vectorReservedSize);
	}

	PHpoints::~PHpoints() {
		clear();
	}

	bool 
	PHpoints::switchRemoveOnlyOne(void) {
		if(m_removeOnlyOne == true) {
			m_removeOnlyOne = false;
		} else {
			m_removeOnlyOne = true;
		}

		return m_removeOnlyOne;
	}

	PHpoints::PHpoints_retval_t 
	PHpoints::insert(vehicleData_t newVehicleData) {
		double point_dist=0;

		if(m_PHpoints_size>=1) {
			// Get the index in the array of the previously saved point
			int prev_idx = m_next_idx == 0 ? m_PHpoints_size-1 : m_next_idx-1;

			point_dist=haversineDist(m_pDataArray[prev_idx].lat,m_pDataArray[prev_idx].lon,newVehicleData.lat,newVehicleData.lon);

			if(point_dist<m_max_dist_m) {
				if(point_dist<m_min_dist_m || fabs(angDiff(m_pDataArray[prev_idx].heading,newVehicleData.heading))<m_max_heading_diff_degs) {
					return PHP_SKIPPED;
				}
			}

			if (m_removeOnlyOne == true) {
				if(m_stored_distance>=m_distance_limit) {
					m_stored_distance-=m_pDataArray[m_oldest_idx].point_distance;
					m_PHpoints_size--;
					m_oldest_idx=(m_oldest_idx+1)%m_vectorReservedSize;
				}
			} else {
				while(m_stored_distance>=m_distance_limit && m_PHpoints_size>1) {
					m_stored_distance-=m_pDataArray[m_oldest_idx].point_distance;
					m_PHpoints_size--;
					m_oldest_idx=(m_oldest_idx+1)%m_vectorReservedSize;
				}
			}

			m_stored_distance+=point_dist;
		}

		if(m_PHpoints_size<m_vectorReservedSize) {
			m_PHpoints_size++;
		}

		m_pDataArray[m_next_idx].lat=newVehicleData.lat;
		m_pDataArray[m_next_idx].lon=newVehicleData.lon;
		m_pDataArray[m_next_idx].elev=newVehicleData.elevation;
		m_pDataArray[m_next_idx].heading=newVehicleData.heading;
		m_pDataArray[m_next_idx].point_distance=point_dist;
        m_pDataArray[m_next_idx].timestamp_us=newVehicleData.timestamp_us;
        m_pDataArray[m_next_idx].speed_ms=newVehicleData.speed_ms;
        m_pDataArray[m_next_idx].detected=newVehicleData.detected;
        m_pDataArray[m_next_idx].CPMincluded=false;
		m_next_idx=(m_next_idx+1)%m_vectorReservedSize;

		return PHP_INSERTED;
	}

	PHpoints::PHpoints_retval_t 
	PHpoints::iterate(PHDataIter_t &PHDataIter, PHData_t *nextPHData) {
		// We need to initizialize some variables when iterate() is called for the first time
		if(PHDataIter.ptrDataArray==NULL) {
			if(m_iterateFull == true) {
				PHDataIter.cyclic_idx=m_next_idx;
				PHDataIter.idx=0;
			} else {
				PHDataIter.cyclic_idx=m_next_idx == 0 ? m_vectorReservedSize-1 : m_next_idx-1;
				PHDataIter.idx=1;
			}

			PHDataIter.ptrDataArray=&m_pDataArray;
			PHDataIter.pDataArraySize=m_PHpoints_size;
		}

		if(PHDataIter.idx<PHDataIter.pDataArraySize) {
			if(PHDataIter.ptrDataArray==NULL) {
				return PHP_ERROR;
			}

			PHDataIter.cyclic_idx=PHDataIter.cyclic_idx == 0 ? m_vectorReservedSize-1 : PHDataIter.cyclic_idx-1;
			PHDataIter.idx++;
			PHDataIter.data=(*PHDataIter.ptrDataArray)[PHDataIter.cyclic_idx];

			if(nextPHData!=NULL) {
				if(PHDataIter.idx<PHDataIter.pDataArraySize-1) {
					*nextPHData=(*PHDataIter.ptrDataArray)[PHDataIter.cyclic_idx == 0 ? m_vectorReservedSize-1 : PHDataIter.cyclic_idx-1];
				} else {
					nextPHData->lat=INVALID_PHDATA;
					nextPHData->lon=INVALID_PHDATA; 
					nextPHData->elev=INVALID_PHDATA;
					nextPHData->heading=INVALID_PHDATA;
					nextPHData->point_distance=INVALID_PHDATA;
				}
			}
			//std::cout << "Next PHData: ["<< PHDataIter.idx <<"] "  << PHDataIter.data.lon << " " << PHDataIter.data.lat << " " << PHDataIter.data.heading << " " << PHDataIter.data.timestamp_us/1000 << " " << PHDataIter.data.CPMincluded << std::endl; // [TBR]
			return PHP_CONTINUE_ITERATION;
		} else {
			//std::cout << "PHpoints::iterate() reached the end of the PH points" << std::endl; // [TBR]
			return PHP_TERMINATE_ITERATION;
		}
	}

	void
	PHpoints::clear() {
		m_pDataArray.clear();

		m_PHpoints_size = 0;
		m_next_idx = 0;
		m_oldest_idx = 0;
	}

    bool PHpoints::setCPMincluded() {
        if(m_PHpoints_size>=1) {
            int last_idx = m_next_idx == 0 ? m_PHpoints_size-1 : m_next_idx-1;
            m_pDataArray[last_idx].CPMincluded=true;
            return true;
        }
        return false;
    }

	PHpoints::PHpoints_retval_t
	PHpoints::insertCPMincluded(vehicleData_t newVehicleData) {
		double point_dist=0;

		if(m_PHpoints_size>=1) {
			// Get the index in the array of the previously saved point
			int prev_idx = m_next_idx == 0 ? m_PHpoints_size-1 : m_next_idx-1;

			point_dist=haversineDist(m_pDataArray[prev_idx].lat,m_pDataArray[prev_idx].lon,newVehicleData.lat,newVehicleData.lon);

			if (m_removeOnlyOne == true) {
				if(m_stored_distance>=m_distance_limit) {
					m_stored_distance-=m_pDataArray[m_oldest_idx].point_distance;
					m_PHpoints_size--;
					m_oldest_idx=(m_oldest_idx+1)%m_vectorReservedSize;
				}
			} else {
				while(m_stored_distance>=m_distance_limit && m_PHpoints_size>1) {
					m_stored_distance-=m_pDataArray[m_oldest_idx].point_distance;
					m_PHpoints_size--;
					m_oldest_idx=(m_oldest_idx+1)%m_vectorReservedSize;
				}
			}

			m_stored_distance+=point_dist;
		}

		if(m_PHpoints_size<m_vectorReservedSize) {
			m_PHpoints_size++;
		}

		m_pDataArray[m_next_idx].lat=newVehicleData.lat;
		m_pDataArray[m_next_idx].lon=newVehicleData.lon;
		m_pDataArray[m_next_idx].elev=newVehicleData.elevation;
		m_pDataArray[m_next_idx].heading=newVehicleData.heading;
		m_pDataArray[m_next_idx].point_distance=point_dist;
		m_pDataArray[m_next_idx].timestamp_us=newVehicleData.timestamp_us;
		m_pDataArray[m_next_idx].speed_ms=newVehicleData.speed_ms;
		m_pDataArray[m_next_idx].detected=newVehicleData.detected;
		m_pDataArray[m_next_idx].CPMincluded=true;
		m_next_idx=(m_next_idx+1)%m_vectorReservedSize;

		return PHP_INSERTED;
	}
}