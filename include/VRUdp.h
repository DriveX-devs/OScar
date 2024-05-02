#ifndef VRUdp_h
#define VRUdp_h

#include <gps.h>
#include <string>
#include "asn_utils.h"
#include "LDMmap.h"
#include "StationType.h"

extern "C" {
  #include "VAM.h"
}

template <class V = int, class C = int>
class VRUdpValueConfidence
{
    private:
      V m_value;
      C m_confidence;

    public:
      VRUdpValueConfidence() {}
      VRUdpValueConfidence(V value,C confidence):
      m_value(value), m_confidence(confidence) {}

      V getValue() {return m_value;}
      C getConfidence() {return m_confidence;}
      void setValue(V value) {m_value=value;}
      void setConfidence(C confidence) {m_confidence=confidence;}
};

typedef struct VRUdp_PosConfidenceEllipse {
  long semiMajorConfidence;
  long semiMinorConfidence;
  long semiMajorOrientation;
} VRUdp_PosConfidenceEllipse_t;

typedef struct VAM_mandatory_data {
  bool avail;
  VRUdpValueConfidence<> speed;
  long longitude;
  long latitude;
  VRUdpValueConfidence<> altitude;
  VRUdp_PosConfidenceEllipse_t posConfidenceEllipse;
  VRUdpValueConfidence<> longAcceleration;
  VRUdpValueConfidence<> heading;
} VAM_mandatory_data_t;

typedef struct VRUdp_position_latlon {
  double lat,lon,alt;
} VRUdp_position_latlon_t;

typedef struct VRUdp_position_XYZ {
  double x,y,z;
} VRUdp_position_XYZ_t;

typedef struct distance {
  double longitudinal,lateral,vertical;
  StationId_t ID;
  StationType_t station_type;
  bool safe_dist;
} distance_t;

class VRUdp
{
  public:
    VRUdp() {};
    VRUdp(std::string server, long port) : m_server(server), m_port(port) {};
    
    // The connection to the GNSS device is terminated when the object is destroyed
    ~VRUdp() {};
    
    // The method will set up the connection to the GNSS device via gps_open() and gps_stream(), using the server and port stored as private attributes
    void openConnection();

    // This method shall be called after we no longer need to receive new data, and will take care of closing the connection to gpsd
    void closeConnection();

    // Function to retrieve the mandatory data for VAM messages
    VAM_mandatory_data_t getVAMMandatoryData();

    VRUdp_position_latlon_t getPedPosition();
    double getPedSpeedValue();
    double getPedHeadingValue();

    std::vector<distance_t> get_min_distance(ldmmap::LDMMap* LDM);
    
    // convertLatLontoXYZ_ECEF() still does not work as expected - kept for reference but it should not be used unless you know very well what you are doing!
    VRUdp_position_XYZ_t convertLatLontoXYZ_ECEF(VRUdp_position_latlon_t pos_latlon);

    // Working properly
    VRUdp_position_XYZ_t convertLatLontoXYZ_TM(VRUdp_position_latlon_t pos_latlon, double lon0);

    void enableDebugPrints() {m_debug=true;};
    void disableDebugPrints() {m_debug=false;};
  private:
    std::string m_server="localhost";
    long m_port=3000;
    
    struct gps_data_t m_gps_data;

    bool m_debug=false;
};

#endif /* VRUdp_h */
