#include "basicSensorReader.h"
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <math.h>
#include <linux/if.h>
#include <errno.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <iomanip>
#include "LDMmap.h"
#include "utmuts.h"

extern "C" {
    #include "CAM.h"
}

#define DEG_2_RAD_BSR(degs) (degs*(M_PI/180.0))
#define RAD_2_DEG_BSR(rads) (rads*(180.0/M_PI))

#define VO_0x_phi_left_FACTOR 0.00048828125 // [rad]
#define VO_0x_phi_left_OFFSET -0.5 // [rad]
#define VO_0x_phi_right_FACTOR 0.00048828125 // [rad]
#define VO_0x_phi_right_OFFSET -0.5 // [rad]
#define VO_0x_dx_v_FACTOR 0.0625 // [m]
#define VO_0x_dx_v_OFFSET 0 // [m]

#define STANDARD_OBJECT_LENGTH 4.24 // [m]
#define STANDARD_OBJECT_WIDTH 1.81 // [m]
#define BUMPER_TO_SENSOR_DISTANCE 1.54 // [m]

void
BasicSensorReader::readerLoop() {
    struct can_frame radar_msg;
    ssize_t nbytes;
    int cansockfd = m_can_socket;

    // Start reading messages from the CAN bus
    while(m_thread_running) {
        nbytes = read(cansockfd,&radar_msg,sizeof(struct can_frame));

        std::cout << "Bytes: " << nbytes << std::endl;

        if(nbytes<=0) {
            fprintf(stderr,"Error: cannot read CAN message with ID 0x%03X. Details: %s\n",radar_msg.can_id,strerror(errno));
            close(cansockfd);
            exit(EXIT_FAILURE);
        }

        if(radar_msg.can_dlc!=8) {
            fprintf(stderr,"Warning: expected DLC=8, but received DLC=%u\n)\n",radar_msg.can_dlc);
        } else {
            // CAN db version
            // Lambda to extract a raw value from a CAN message depending on the contect read from the CAN DB
            auto extract_raw = [&](const CANSignalInfo &sig, const uint8_t raw_data[8]) -> uint32_t {
                uint32_t raw = 0;
                if (sig.length <= 8) {
                    // single‐byte
                    raw = (raw_data[sig.start_byte] >> sig.shift_right) & sig.mask;
                }
                else if (sig.length <= 16) {
                    // two‐byte
                    uint16_t b0 = raw_data[sig.start_byte];
                    uint16_t b1 = raw_data[sig.start_byte + 1];
                    uint16_t hi = (sig.byte_order == 0) ? b0 : b1;  // 0=Motorola
                    uint16_t lo = (sig.byte_order == 0) ? b1 : b0;
                    raw = ((hi << 8) | lo) & sig.mask;
                }
                else {
                    throw std::runtime_error("Signal too wide for automatic extraction");
                }
                return raw;
            };

            const CANSignalInfo phi_left_sig = m_can_db_sensor_info[phiLeftSignal];
            const CANSignalInfo class_sig = m_can_db_sensor_info[classificationSignal];
            const CANSignalInfo phi_right_sig = m_can_db_sensor_info[phiRightSignal];
            const CANSignalInfo dx_v_sig = m_can_db_sensor_info[distanceSignal];

            uint32_t raw_phi_left = extract_raw(phi_left_sig, radar_msg.data);
            double phi_left = raw_phi_left * phi_left_sig.factor + phi_left_sig.offset;

            uint32_t raw_class = extract_raw(class_sig, radar_msg.data);
            auto classification = static_cast<classification_t>(raw_class);

            uint32_t raw_phi_right = extract_raw(phi_right_sig, radar_msg.data);
            double phi_right = raw_phi_right * phi_right_sig.factor + phi_right_sig.offset;

            uint32_t raw_dx_v = extract_raw(dx_v_sig, radar_msg.data);
            double dx_v = raw_dx_v * dx_v_sig.factor + dx_v_sig.offset;

            if (dx_v > 0.0) {
                //TODO: check if the bumper to sensor distance is not already included in the dx_v (I've seen values lower than 1.54 from the radar)
                dx_v = dx_v - BUMPER_TO_SENSOR_DISTANCE + (STANDARD_OBJECT_LENGTH/2.0);
                double dist_left = dx_v / cos(phi_left);
                double dist_right = dx_v / cos(phi_right);
                double dy_left = dist_left * sin(phi_left);
                double dy_right = dist_right * sin(phi_right);
                double width = dy_right - dy_left;
                double dy_v = dy_left + width / 2.0;
                double dy_c,ego_heading,ego_heading_cart, ego_lat, ego_lon, xDistance, yDistance;
                double gammar=0,kr=0;

                /*
                std::cout << "Radar data: " << std::endl;
                std::cout << "phi_left: " << RAD_2_DEG(phi_left) << std::endl;
                std::cout << "phi_right: " << RAD_2_DEG(phi_right) << std::endl;
                std::cout << "dx_v: " << dx_v << std::endl;
                 */

                ldmmap::vehicleData_t vehicleData;
                //vehicleData.exteriorLights = ldmmap::OptionalDataItem<uint8_t>(false);
                vehicleData.detected = true;

                // TODO check how reliable is the classification
                if (m_enable_classification)
                {
                    if (classification == CAR || classification == SEMI)
                        vehicleData.stationType = ldmmap::StationType_LDM_detectedPassengerCar; // SEMI should be a truck but from tests, cars are classified as SEMI
                    else if (classification == CYCLE)
                        vehicleData.stationType = ldmmap::StationType_LDM_detectedPedestrian;
                    else
                        vehicleData.stationType = ldmmap::StationType_LDM_detectedLightTruck;  // UNKNOWN or NOT_DEFINED
                }
                else
                {
                    vehicleData.stationType = ldmmap::StationType_LDM_detectedPassengerCar;
                }

                vehicleData.perceivedBy = (uint64_t) m_stationID;

                VDPGPSClient::CAM_mandatory_data_t ego_data = m_gpsc_ptr->getCAMMandatoryData();
                for (int i = 0; i < 20; i++) {
                    ego_data = m_gpsc_ptr->getCAMMandatoryData();
                }

                if (ego_data.latitude!= Latitude_unavailable && ego_data.longitude!= Longitude_unavailable) {
                    ego_lat = (double)ego_data.latitude/DOT_ONE_MICRO;
                    ego_lon = (double)ego_data.longitude/DOT_ONE_MICRO;
                }
                else
                {
                    std::cout << "Warning Ego vehicle's position is not available!" << std::endl;
                    continue;
                }

                ego_heading = (double) (ego_data.heading.getValue()) / DECI;

                // ETSI TS 103 324 V2.1.1 (2023-06) demands xDistance and yDistance to be with East as positive x and North as positive y
                ego_heading_cart = DEG_2_RAD_BSR((90-ego_heading)); // The heading from the gps is relative to North --> 90 degrees from East
                dy_c = -dy_v; // Left to the sensor is negative in radar frame but positive in cartesian reference
                // Rotate the object's position from the ego vehicle's frame to the global frame
                xDistance = (double) (dx_v * cos(ego_heading_cart) - dy_c * sin(ego_heading_cart));
                yDistance = (double) (dx_v * sin(ego_heading_cart) + dy_c * cos(ego_heading_cart));


                vehicleData.heading = ego_data.heading.getValue(); // [0.1 degrees]
                vehicleData.xSpeed = ego_data.speed.getValue() * cos(ego_heading_cart); // [0.01 m/s]
                vehicleData.ySpeed = ego_data.speed.getValue() * sin(ego_heading_cart); // [0.01 m/s]
                vehicleData.speed_ms = ego_data.speed.getValue() * 0.01; // [m/s]
                vehicleData.stationID = radar_msg.can_id;

                if (m_verbose)
                {
                    std::string class_str;
                    switch (classification)
                    {
                        case CAR:
                            class_str = "CAR";
                            break;
                        case SEMI:
                            class_str = "SEMI";
                            break;
                        case CYCLE:
                            class_str = "CYCLE";
                            break;
                        case NOT_DEFINED:
                            class_str = "NOT_DEFINED";
                            break;
                        case UNKNOWN:
                            class_str = "UNKNOWN";
                            break;
                    }
                    std::cout << "[SENSOR READER]" << std::endl;
                    std::cout << "    Object "<< vehicleData.stationID <<  " - xDistance: " << xDistance << "m - yDistance: " << yDistance << "m - class: " << class_str << std::endl;
                    std::cout << "    Ego Vehicle - lat: " << std::fixed << std::setprecision(6) << ego_lat
                              << " - lon: " << ego_lon
                              << " - heading: " << ego_heading
                              << std::endl;
                }

                transverse_mercator_t tmerc = UTMUPS_init_UTM_TransverseMercator();
                double lat1, lon1, ego_x, ego_y;
                TransverseMercator_Forward(&tmerc, ego_lon, ego_lat, ego_lon, &ego_x, &ego_y, &gammar, &kr);
                ego_x += xDistance;
                ego_y += yDistance;
                TransverseMercator_Reverse(&tmerc, ego_lon, ego_x, ego_y, &lat1, &lon1, &gammar, &kr);


                vehicleData.lat = lat1;
                vehicleData.lon = lon1;
                vehicleData.elevation = 0;
                //vehicleData.xDistance.setData((long) (dx_v * cos(ego_heading) - dy_c * sin(ego_heading))*CENTI);
                vehicleData.xDistance = (long) (xDistance*CENTI);
                //vehicleData.yDistance.setData((long) (dx_v * sin(ego_heading) + dy_c * cos(ego_heading))*CENTI);
                vehicleData.yDistance = (long) (yDistance*CENTI);
                vehicleData.heading = ego_heading;
                vehicleData.angle = ego_heading * DECI;
                vehicleData.vehicleWidth = ldmmap::OptionalDataItem<long>((long) (width*DECI));
                vehicleData.vehicleLength = ldmmap::OptionalDataItem<long>((long) (STANDARD_OBJECT_LENGTH*DECI));
                //vehicleData.angle = ldmmap::OptionalDataItem<long>((long) RAD_2_DEG(ego_heading)*DECI);

                vehicleData.timestamp_us = computeTimestamp()/NANO_TO_MICRO;
                vehicleData.confidence = 100; //TODO: implement confidence computation
                auto db_retval=m_LDM->insert(vehicleData);

                if(db_retval!=ldmmap::LDMMap::LDMMAP_OK && db_retval!=ldmmap::LDMMap::LDMMAP_UPDATED) {
                    std::cerr << "Warning! Insert on the database for object " << (int) vehicleData.stationID << "failed!" << std::endl;
                }

            }
        }
    }
}

bool BasicSensorReader::startReader() {
    std::cout << "Starting sensor reader on interface " << m_interface << std::endl;
    // Currently, m_can_db_sensor_info.size() = 4 - we currently need to filter for 4 CAN IDs (standard format)
    std::vector<struct can_filter> canfilter(m_can_db_sensor_info.size());

    struct sockaddr_can addr;
    struct ifreq ifr;

    if(!m_can_db_info_set) {
        fprintf(stderr,"Error: trying to start the sensor reader with no CAN db information set.\n");
        return false;
    }

    // Open a new CAN Bus socket
    m_can_socket=socket(PF_CAN,SOCK_RAW,CAN_RAW);
    if(m_can_socket<=0) {
        fprintf(stderr, "Error: cannot create CAN socket. Details: %s\n",strerror(errno));
        exit(EXIT_FAILURE);
    }

    // Get the CAN interface index, starting from the interface name; this is required for binding
    strcpy(ifr.ifr_name,m_interface.c_str());
    ioctl(m_can_socket,SIOCGIFINDEX,&ifr);
    addr.can_family=AF_CAN;
    addr.can_ifindex=ifr.ifr_ifindex;

    // Bind the CAN socket to the CAN interface
    if(bind(m_can_socket,(struct sockaddr *)&addr,sizeof(addr))<0) {
        fprintf(stderr, "Error: cannot bind CAN socket to interface %s. Details: %s\n",m_interface.c_str(),strerror(errno));
        close(m_can_socket);
        exit(EXIT_FAILURE);
    }

    // Prepare a CAN ID filter
    // Filter only for the messages of interest
    // The data of interest should be located inside the messages matching the regex used to read the CAN database (default CAN message names: "Video_Object_0x_B")
    for(int can_id_idx=0;can_id_idx<m_can_db_id_info.size();can_id_idx++) {
        canfilter[can_id_idx].can_id=m_can_db_id_info[can_id_idx];
        canfilter[can_id_idx].can_mask=CAN_SFF_MASK;
    }

    // Set the CAN filter for receiving only the messages of interest
    if(setsockopt(m_can_socket,SOL_CAN_RAW,CAN_RAW_FILTER,canfilter.data(),canfilter.size()*sizeof(struct can_filter))<0) {
        fprintf(stderr,"Error: cannot create CAN ID filter. Details: %s\n",strerror(errno));
        close(m_can_socket);
        m_can_socket=-1;
        return false;
    }
    std::cout << "RADAR sensor reader starting ... " << std::endl;

    m_thread_running=true;
    this->readerLoop();
    return true;

}

bool BasicSensorReader::stopReader() {
    if(m_thread_running==false || m_tid<0) {
        return false;
    }
    close(m_can_socket);
    m_tid=-1;
    m_can_socket=-1;
    m_thread_running=false;
    m_thread.join();
    return true;
}

uint64_t BasicSensorReader::computeTimestamp() {
    int64_t int_tstamp=0;

    struct timespec tv;

    clock_gettime (CLOCK_MONOTONIC, &tv);

    int_tstamp=tv.tv_sec*1e9+tv.tv_nsec;

    return int_tstamp;
}
