#ifndef DBCREADER_H
#define DBCREADER_H

// Class for reading a CAN database .dbc file and extracting the relevant information for the parsing of sensor data, required in turn for the dissemination of CPMs
// Currently, the dbcReader does not support CAN signals that use enums (BA_, BA_DEF_, VAL_) or multiplexed signals
// Comments (CM_) will also not be imported

#include <string>
#include <vector>
#include <map>

typedef struct CANdbUserParams {
    std::string sensorMsgNameRegex;
    std::string classificationSignalName;
    std::string phiRightSignalName;
    std::string phiLeftSignalName;
    std::string distanceSignalName;
} CANdbUserParams_t;

// Types of CAN signals that are relevant for the sensor data parsing
typedef enum {
    classificationSignal=0,
    phiLeftSignal=1,
    phiRightSignal=2,
    distanceSignal=3
} CANSignalKind_t;

// Structure to hold generic CAN signal information
struct CANSignalInfo {
    std::string name;
    uint32_t start_bit;
    uint32_t length;
    uint32_t byte_order;  // 0 = Motorola (big endian), 1 = Intel (little endian)
    bool is_signed;
    double factor;
    double offset;
    double min_value;
    double max_value;
    std::string unit;

    // These values are not included in the .dbc file, but are calculated since they are useful for extracting the required information from CAN messages
    uint32_t start_byte;
    uint32_t start_bit_in_byte;
    uint64_t mask;
    uint32_t shift_right;
};

// Structure to hold CAN message information: message name, ID, length and the set of signals included in the message
struct MessageInfo {
    std::string name;
    uint32_t can_id;
    uint32_t size;
    std::map<std::string,CANSignalInfo> signals;
};

// Convenience type aliases for CAN message and signal containers
#define CAN_MESSAGES_MAP_t std::map<std::string,MessageInfo>
#define CAN_SENSOR_SIGNAL_INFO_t std::vector<CANSignalInfo>

class dbcReader {
    public:
        dbcReader() {
            m_parsed = false;
            m_user_params.sensorMsgNameRegex = m_default_CAN_msg_regex;
            m_user_params.classificationSignalName = m_default_signal_names.at(classificationSignal);
            m_user_params.phiLeftSignalName = m_default_signal_names.at(phiLeftSignal);
            m_user_params.phiRightSignalName = m_default_signal_names.at(phiRightSignal);
            m_user_params.distanceSignalName = m_default_signal_names.at(distanceSignal);
        }

        // This method can be used to set non-default user parameters starting from a CANdbUserParams_t structure
        void setUserParams(const CANdbUserParams_t& params) {
            m_user_params = params;
        }

        // This method can be used to set non-default user parameters starting from an INI file
        bool setUserParamsIni(std::string filename);

        // Key method to load the content of a .dbc file into the m_messages internal structure
        // All the other methods cannot be called without returning an error if parseDBC() has not been called before
        bool parseDBC(const std::string& filename);

        // Method to get a full CAN database structure for manual usage
        bool getFullCANMap(CAN_MESSAGES_MAP_t &can_map) {
            if(m_parsed == true) {
                can_map=m_messages;
            }
            return m_parsed;
        }

        // Method that returns the list of CAN message IDs related to sensor data
        std::vector<uint32_t> getSensorObjectCANIDs(bool &error);
        // Method that returns a vector of CAN signals related to sensor data,
        // inside the CAN messages which IDs are returned by getSensorObjectCANIDs()
        // Each element of the vector includes the offset, factor and position in the CAN message
        // The current implementation assumes that all messages containing sensor data include
        // the same set of signals that identify sensed objects, i.e., classification, distance, phy_left (angle
        // between sensor and left-most edge of the sensed object), phy_right (angle between sensor and right-most
        // edge of the sensed object)
        CAN_SENSOR_SIGNAL_INFO_t getSignalInfoSensorObject(bool &error);
        // Debug method that prints the information related to the set of relevant CAN signals for CPM dissemination
        // (which information is stored in CAN_SENSOR_SIGNAL_INFO_t)
        // As it does not access any internal attribute, it is a static method that can be called like this:
        // dbcReader::printSignalInfoSensorData(sigInfo)
        static void printSignalInfoSensorData(CAN_SENSOR_SIGNAL_INFO_t sigInfo);
        // Debug method that prints each CAN message information, as read from the CAN database file
        void printCANdb(bool &error);

    private:
        // Method that returns true if signal_name is equal to target_signal, or if it is equal except for any suffix separated with "-" or "_",
        // or if it is equal except for any prefix separated with "-" or "_".
        // For instance, if "target_signal" is "classification", this method will return:
        // - signal_name: "classification" -> true
        // - signal_name: "classification_test" -> true
        // - signal_name: "classification-test" -> true
        // - signal_name "classificationtest" -> false
        // - signal_name "test_classification" -> true
        // - signal_name "test-classification" -> true
        // - signal_name "testclassification" -> false
        // - signal_name "test#classification" -> false
        // - signal_name "classification#test" -> false
        bool signalMatches(const std::string& signal_name, const std::string& target_signal);
        // Method that determines the start byte, mask and required right shift for extracting a given information from a CAN message
        bool calculateSignalExtraction(CANSignalInfo& signal);
        // Method that parses CAN message information from a BO_ line of .dbc file
        void parseMessage(const std::string& line);
        // Method that parses CAN signal information from an SG_ line of .dbc file
        bool parseSignal(const std::string& line);

        CAN_MESSAGES_MAP_t m_messages;
        const std::map<int,std::string> m_default_signal_names = {
                {classificationSignal, "classification"},
                {phiLeftSignal, "phi_left"},
                {phiRightSignal, "phi_right"},
                {distanceSignal, "dx_v"}};
        const std::string m_default_CAN_msg_regex = "^Video_Object_\\d{2}_B$";
        CANdbUserParams_t m_user_params;
        bool m_parsed = false;
};

#endif //DBCREADER_H