#ifndef LDMMAP_H
#define LDMMAP_H

#include <unordered_map>
#include <vector>
#include <shared_mutex>
#include "PHpoints.h"
#include "vehicleDataDef.h"

namespace ldmmap {
	class LDMMap {
		public:
	    	typedef enum {
	    		LDMMAP_OK,
	    		LDMMAP_UPDATED,
	    		LDMMAP_ITEM_NOT_FOUND,
	    		LDMMAP_MAP_FULL,
	    		LDMMAP_UNKNOWN_ERROR
	    	} LDMMap_error_t;

	    	typedef struct {
	    		vehicleData_t vehData;
	    		PHpoints *phData;
	    	} returnedVehicleData_t;

	    	LDMMap();
	    	~LDMMap();

	    	// This function clears the whole database (to be used only when the dabatase and its content is not going to be accessed again)
	    	void clear();
	    	// This function inserts or updates a vehicle in the database
	    	// The vehicle data should be passed inside a vehicleData_t structure and vehicles are univocally identified by their stationID
	    	// This function returns LDMMAP_OK is a new vehicle has been inserted, LDMMAP_UPDATED is an existing vehicle entry has been updated,
	    	// LDMMAP_MAP_FULL if the database if full and the insert operation failed (this should never happen, in any case)
	    	LDMMap_error_t insert(vehicleData_t newVehicleData);
	    	// This function removes from the database the vehicle entry with station ID == stationID
	    	// It returns LDMMAP_ITEM_NOT_FOUND if no vehicle with the given stationID was found for removal
	    	// It returns LDMMAP_OK if the vehicle entry was succesfully removed
	    	LDMMap_error_t remove(uint64_t stationID);
	    	// This function returns the vehicle entry with station ID == stationID - the entry data is returned in retVehicleData
	    	// This function returns LDMMAP_ITEM_NOT_FOUND if no vehicle with the given stationID was found for removal, while
	    	// it returns LDMMAP_OK if the retVehicleData structure was properly filled with the requested vehicle data
	    	LDMMap_error_t lookup(uint64_t stationID, returnedVehicleData_t &retVehicleData);
	    	// This function returns a vector of vehicles, including their Path History points, located within a certain radius 
	    	// centered on a given latitude and longitude
	    	// For the time being, this function should always return LDMMAP_OK (i.e. to understand if no vehicles are returned, 
	    	// you should check the size of the selectedVehicles vector)
	    	LDMMap_error_t rangeSelect(double range_m, double lat, double lon, std::vector<returnedVehicleData_t> &selectedVehicles);
	    	// This function is the same as the other method with the same name, but it will return all the vehicles around another
	    	// vehicle (which is also included in the returned vector), given it stationID
	    	// This function may return LDMMAP_ITEM_NOT_FOUND if the specified stationID is not stored inside the database
	    	LDMMap_error_t rangeSelect(double range_m, uint64_t stationID, std::vector<returnedVehicleData_t> &selectedVehicles);
	    	// This function deletes from the database all the entries older than time_milliseconds ms
	    	// The entries are deleted if their age is > time_milliseconds ms if greater_equal == false, 
	    	// or >= time_milliseconds ms if greater_equal == true
	    	// This function performs a full database read operation
	    	void deleteOlderThan(double time_milliseconds);
	    	// This function is a combination of deleteOlderThan() and executeOnAllContents(), calling the open_fcn()
	    	// callback for every deleted entry
	    	void deleteOlderThanAndExecute(double time_milliseconds,void (*oper_fcn)(uint64_t,void *),void *additional_args);
	    	// This function can be used to print all the content of the database
	    	// The stationIDs of the vehicles stored in the LDMMap database will be printed, preceded by an optional string,
	    	// specified with "label"
	    	void printAllContents(std::string label = "");
	    	// This function reads the whole database, and, for each entry, it executes the "oper_fcn" callback
	    	// This callback should return void (i.e. nothing) and have two arguments:
	    	// - a vehicleData_t structure, in which the data stored in each entry will be made available to the callback
	    	// - a void * pointer, by means of which possible additional arguments can be passed to the callback
	    	// The additional arguments to be passed to the callback, each time it is called, can be specified by setting
	    	// void *additional_args to a value different than "nullptr"
	    	// If additional_args == nullptr, also the second argument of each callback call will be nullptr
	    	void executeOnAllContents(void (*oper_fcn)(vehicleData_t,void *),void *additional_args);

	    	// This setter sets two facility (private) variables which are used solely for passing a central 
	    	// latitude and longitude to the vehicle visualizer, extracting them from an LDMMap object
	    	// The same applies to the corresponding getter method
	    	void setCentralLatLon(double lat, double lon) {m_central_lat = lat; m_central_lon = lon;}
	    	std::pair<double,double> getCentralLatLon() {return std::make_pair(m_central_lat,m_central_lon);}

	    	int getCardinality() {return m_card;};

		private:
			// Main database structure
			std::unordered_map<uint32_t,std::pair<std::shared_mutex*,std::unordered_map<uint32_t,returnedVehicleData_t>>> m_ldmmap;
			// Database cardinality (number of entries stored in the database)
			uint64_t m_card;
			// Shared mutex protecting the main database structure
			std::shared_mutex m_mainmapmut;

			// Facility variables representing a central point around which all vehicle entries are ideally located
			// They can be gathered and set using the setCentralLatLon() and getCentralLatLon() methods
			// By default, they are both set to 0.0
			double m_central_lat;
			double m_central_lon;
	};
}

#endif // LDMMAP_H