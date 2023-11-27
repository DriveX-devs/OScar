#ifndef PHPOINTS_H
#define PHPOINTS_H

#include <inttypes.h>
#include <cfloat>
#include <vector>
#include "vehicleDataDef.h"

#define INVALID_PHDATA DBL_MAX
#define PHDATAITER_INITIALIZER(var) ldmmap::PHpoints::PHDataIter_t var = {.ptrDataArray=NULL}

namespace ldmmap {
	class PHpoints {
		public:
			// Information stored for each PH point in this object
			typedef struct PHData {
				double lat;
				double lon;
				double elev;
				double heading;
				double point_distance;
			} PHData_t;

			// Structure used to iterate over the PH points of a vehicle
			typedef struct PHDataIter {
				std::vector<PHData_t> *ptrDataArray;
				PHData_t data;
				int idx;
				int cyclic_idx;
				int pDataArraySize;
			} PHDataIter_t;

			typedef enum {
				PHP_INSERTED,
				PHP_SKIPPED,
				PHP_CONTINUE_ITERATION,
				PHP_TERMINATE_ITERATION,
				PHP_ERROR
			} PHpoints_retval_t;

			PHpoints();
			PHpoints(double distance_limit, double min_dist_m, double max_dist_m, double max_heading_diff_degs);
			~PHpoints();

			PHpoints_retval_t insert(vehicleData_t newVehicleData);
			PHpoints_retval_t iterate(PHDataIter_t &PHDataIter, PHData_t *nextPHData);
			void clear(void);

			// These are the only two parameters which can be set after construcing a PHpoints objects, as it reserves
			// a vector whose size depends on m_distance_limit and m_min_dist_m
			void setPHMaxDist(double max_distance_meters) {m_max_dist_m = max_distance_meters;}
			void setPHMaxHeadingDiff(double max_heading_diff_degs) {m_max_heading_diff_degs = max_heading_diff_degs;}

			// This function returns the value for the newly set "Remove Policy"
			bool switchRemoveOnlyOne(void);

			// This function set the "Iterate Policy" (i.e. sets if "iterate()" should return also the current up-to-date point, or only points from the Path History itself)
			void setIterateFull(bool iterateFull) {m_iterateFull = iterateFull;}

			int getCardinality(void) {return m_PHpoints_size;};

		private:
			double m_distance_limit;
			double m_min_dist_m;
			double m_max_dist_m;
			double m_max_heading_diff_degs;

			// PH points management variables
			int m_next_idx;
			int m_oldest_idx;
			double m_stored_distance;
			int m_PHpoints_size;
			int m_vectorReservedSize;

			std::vector<PHData_t> m_pDataArray;

			// "Remove Policy"
			// Set to "true" for "remove, if needed, at most one point when a new update arrives"
			// Set to "false" for "remove, if needed, any number of points until the stored distance is the minimum one >= distance_limit"
			bool m_removeOnlyOne = false;

			// "Iterate Policy"
			// Set to "true" to iterate over all the PH points, including the most up-to-date point
			// Set to "false" to iterate over only the PH History points (the current point data is not returned)
			bool m_iterateFull = false;
	};
}

#endif // PHPOINTS_H