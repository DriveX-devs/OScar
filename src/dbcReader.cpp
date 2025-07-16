#include <iostream>
#include <fstream>
#include <regex>
#include "dbcReader.h"
#include "INIReader.h"

bool dbcReader::signalMatches(const std::string& signal_name, const std::string& target_suffix) {
    return signal_name.length() > target_suffix.length() &&
           signal_name.substr(signal_name.length() - target_suffix.length()) == target_suffix &&
           signal_name.substr(signal_name.length() - target_suffix.length() - 1, 1) == "_";
}

bool dbcReader::calculateSignalExtraction(CANSignalInfo& signal) {
    // Return an error for unsupported signal lengths
    // For the time being, we consider standard CAN messages with a maximum length of 8 B
    if (signal.length == 0 || signal.length > 64) {
        std::cerr << "[ERROR] Signal length must be between 1 and 64 bits.\n" << std::endl;
        return false;
    }

    // Figure out which bytes the signal spans
    int first_bit = signal.start_bit;
    int last_bit  = signal.start_bit + signal.length - 1;
    int first_byte = first_bit/8;
    int last_byte = last_bit/8;
    signal.start_byte = first_byte;
    signal.start_bit_in_byte = first_bit%8;
    int byte_count = (last_byte - first_byte) + 1;

    // Build the mask for exactly 'length' bits (use 64‑bit math, with the "ULL" specifier, in case length == 64)
    if (signal.length == 64) {
        signal.mask = 0xFFFFFFFFFFFFFFFFULL;
    } else {
        signal.mask = (1ULL << signal.length) - 1ULL;
    }

    // Compute how many bits we have to shift right after concatenating those bytes into a 64-bit uint64_t
    if (signal.byte_order == 0) {
        // Motorola / big‑endian
        // We read in 'byte_count' bytes as (data[first_byte] << ((byte_count-1)*8)) | (data[first_byte+1] << ((byte_count-2)*8)) | [...]
        // Then we need to shift right so that the actual 'length'‑bit field sits at bit‑0 of our uint64_t:
        // bits_to_drop = (byte_count * 8) - ((start_bit_in_byte%8) + length)
        signal.shift_right = byte_count * 8 - (signal.start_bit_in_byte + signal.length);
    } else {
        // Intel / little-endian
        signal.shift_right = signal.start_bit_in_byte;
    }

    return true;
}

void dbcReader::parseMessage(const std::string& line) {
    std::regex msg_regex(R"(BO_\s+(\d+)\s+(\w+):\s+(\d+)\s+)");
    std::smatch matches;

    if (std::regex_search(line, matches, msg_regex)) {
        MessageInfo msg;
        // First match: CAN ID
        msg.can_id = std::stoul(matches[1].str());
        // Second match before the colon ":": CAN message name
        msg.name = matches[2].str();
        // Third match: CAN message length (# of data bytes)
        msg.size = std::stoul(matches[3].str());

        m_messages[msg.name] = msg;
    }
}

bool dbcReader::parseSignal(const std::string& line) {
    std::regex sig_regex("SG_\\s+(\\w+)\\s+:\\s+(\\d+)\\|(\\d+)@(\\d+)([\\+\\-])\\s+\\(([\\d\\.\\-\\+e]+),([\\d\\.\\-\\+e]+)\\)\\s+\\[([\\d\\.\\-\\+e]+)\\|([\\d\\.\\-\\+e]+)\\]\\s+\"([^\"]*)\"");
    std::smatch matches;

    // The regex matches the following:
    // SG_\\s+ - Literal "SG_" followed by whitespace
    // (\\w+) - Capture group 1: Signal name (word characters)
    // \\s+:\\s+ - Whitespace, colon, whitespace
    // (\\d+) - Capture group 2: Start bit position
    // \\| - Literal pipe character
    // (\\d+) - Capture group 3: Signal length in bits
    // @ - Literal @ symbol
    // (\\d+) - Capture group 4: Byte order (0=Motorola, 1=Intel)
    //  ([\\+\\-]) - Capture group 5: Sign (+ or -)
    // \\s+\\( - Whitespace and opening parenthesis
    // ([\\d\\.\\-\\+e]+) - Capture group 6: Factor (scaling factor) - specifying "e" let us match the scientific notation
    //, - Literal comma
    // ([\\d\\.\\-\\+e]+) - Capture group 7: Offset - specifying "e" let us match any scientific notation
    // \\)\\s+\\[ - Closing parenthesis, whitespace, opening bracket
    // ([\\d\\.\\-\\+e]+) - Capture group 8: Minimum value
    // \\| - Literal pipe
    // ([\\d\\.\\-\\+e]+) - Capture group 9: Maximum value
    // \\]\\s+\" - Closing bracket, whitespace, opening quote
    // ([^\"]*) - Capture group 10: string with the measurement unit (anything except quotes)
    // \" - Closing quote

    if (std::regex_search(line, matches, sig_regex)) {
        CANSignalInfo signal;
        signal.name = matches[1].str();
        signal.start_bit = std::stoul(matches[2].str());
        signal.length = std::stoul(matches[3].str());
        signal.byte_order = std::stoul(matches[4].str());
        signal.is_signed = (matches[5].str() == "-");
        signal.factor = std::stod(matches[6].str());
        signal.offset = std::stod(matches[7].str());
        signal.min_value = std::stod(matches[8].str());
        signal.max_value = std::stod(matches[9].str());
        signal.unit = matches[10].str();

        // Calculate the remaining values useful for data extraction from CAN messages, i.e., start_byte,
        // start_bit_in_byte, mask, required right shift (shift_right)
        if(!calculateSignalExtraction(signal)) {
            return false;
        }

        // Find the last message that was parsed (signals follow their message)
        // This lets us "assign" the current signal to the right CAN message
        if (!m_messages.empty()) {
            // rebegin() returns a reverse iterator pointing to the last element of the map
            // "second" takes the value (while "first" would take the key, i.e., the message name)
            auto& last_msg = m_messages.rbegin()->second;
            // This should modify the original m_message map; therefore, we are taking a reference with "auto&"
            last_msg.signals[signal.name] = signal;
        }
    }

    return true;
}

bool dbcReader::setUserParamsIni(std::string filename) {
    INIReader reader(filename);

    if (reader.ParseError() < 0) {
        std::cerr << "[ERROR] Cannot load specified INI file "<< filename << "\n";
        return false;
    }

    m_user_params.sensorMsgNameRegex = reader.Get("CAN message names", "sensor_message_regex", m_default_CAN_msg_regex);
    m_user_params.classificationSignalName = reader.Get("CAN signal names", "classification", m_default_signal_names.at(classificationSignal));
    m_user_params.phiLeftSignalName = reader.Get("CAN signal names", "phi_left", m_default_signal_names.at(phiLeftSignal));
    m_user_params.phiRightSignalName = reader.Get("CAN signal names", "phi_right", m_default_signal_names.at(phiRightSignal));
    m_user_params.distanceSignalName = reader.Get("CAN signal names", "distance", m_default_signal_names.at(distanceSignal));

    return true;
}

// Main method to parse a CAN database file
bool dbcReader::parseDBC(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: could not open file " << filename << std::endl;
        return false;
    }

    // Read the CAN .dbc file line by line
    std::string line;
    while (std::getline(file, line)) {
        // Remove leading/trailing whitespace
        line.erase(0, line.find_first_not_of(" \t"));
        line.erase(line.find_last_not_of(" \t") + 1);

        // Skip any empty line
        if (line.empty()) continue;

        // BO_ -> CAN message
        if (line.find("BO_") == 0) {
            parseMessage(line);
        } else if (line.find("SG_") == 0) {
            // _SG -> CAN signal
            // Check the return value as signal parsing may fail
            if(!parseSignal(line)) {
                return false;
            }
        }
    }

    file.close();

    // m_parsed is used by the other methods to determine if parseDBC() was called first
    m_parsed = true;

    return true;
}

std::vector<uint32_t> dbcReader::getSensorObjectCANIDs(bool &error) {
    std::vector<uint32_t> can_ids;
    // Regex that defines the name of the sensor data CAN messages
    std::regex pattern(m_user_params.sensorMsgNameRegex);

    if(!m_parsed) {
        std::cerr << "[ERROR] Attempted to get sensor object information without parsing a CAB database first." << std::endl;
        error = true;
    } else {
        // Loop over all the elements of the m_messages map
        for (const auto& [msg_name, msg] : m_messages) {
            if (std::regex_match(msg_name, pattern)) {
                // Take the IDs of the messages matching the regex
                can_ids.push_back(msg.can_id);
            }
        }
        error = false;
    }

    if(can_ids.empty()) {
        std::cerr << "[ERROR] No sensor data CAN messages matching the specified regex \"" << m_user_params.sensorMsgNameRegex  << "\" found in the CAN database." << std::endl;
        error = true;
    }

    return can_ids;
}

CAN_SENSOR_SIGNAL_INFO_t dbcReader::getSignalInfoSensorObject(bool &error) {
    CAN_SENSOR_SIGNAL_INFO_t can_sensor_signal_info;

    if(!m_parsed) {
        std::cerr << "[ERROR] Attempted to get CAN db signal information without parsing a CAB database first." << std::endl;
        error = true;
        return can_sensor_signal_info;
    } else {
        error = false;
    }

    can_sensor_signal_info.resize(m_default_signal_names.size());

    // Take as reference the first video/sensor object message starting from the regex that defines the name of the sensor data CAN messages
    // IMPORTANT: we assume here that all sensor object messages use the same signal format
    std::regex pattern(m_user_params.sensorMsgNameRegex);
    auto it = std::find_if(m_messages.begin(), m_messages.end(),[&pattern](const auto& msg_pair) {return std::regex_match(msg_pair.first, pattern);});

    MessageInfo first_msg;
    // At least one CAN message has been found which name matches the regex -> take the first message
    if (it != m_messages.end()) {
        first_msg = it->second;
        error = false;
    } else {
        // No CAN message names found matching the regex -> return an error
        error = true;
        return can_sensor_signal_info;
    }

    // Get the relevant CAN signal information and store it into a CAN_SENSOR_SIGNAL_INFO_t vector (that will be used by the sensor reader to get the required information for CPM dissemination)
    // The CAN_SENSOR_SIGNAL_INFO_t vector contains CAN signal information with names that are modified to match the generic names used by the sensor reader, i.e., classification, phi_left, phi_right, distance
    uint8_t found_signals = 0;
    enum SignalBits {
        CLASSIFICATION_BIT = 0,
        PHI_LEFT_BIT = 1,
        PHI_RIGHT_BIT = 2,
        DX_V_BIT = 3
    };
    constexpr uint8_t ALL_SIGNALS = (1 << CLASSIFICATION_BIT) | (1 << PHI_LEFT_BIT) | (1 << PHI_RIGHT_BIT) | (1 << DX_V_BIT);

    for (const auto &sig_pair: first_msg.signals) {
        if(signalMatches(sig_pair.first, m_user_params.classificationSignalName)) {
            can_sensor_signal_info[classificationSignal] = sig_pair.second;
            can_sensor_signal_info[classificationSignal].name = "classification";
            found_signals |= (1 << CLASSIFICATION_BIT);
        } else if(signalMatches(sig_pair.first, m_user_params.phiLeftSignalName)) {
            can_sensor_signal_info[phiLeftSignal] = sig_pair.second;
            can_sensor_signal_info[phiLeftSignal].name = "phi_left";
            found_signals |= (1 << PHI_LEFT_BIT);
        } else if(signalMatches(sig_pair.first, m_user_params.phiRightSignalName)) {
            can_sensor_signal_info[phiRightSignal] = sig_pair.second;
            can_sensor_signal_info[phiRightSignal].name = "phi_right";
            found_signals |= (1 << PHI_RIGHT_BIT);
        } else if(signalMatches(sig_pair.first, m_user_params.distanceSignalName)) {
            can_sensor_signal_info[distanceSignal] = sig_pair.second;
            can_sensor_signal_info[distanceSignal].name = "distance";
            found_signals |= (1 << DX_V_BIT);
        }
    }

    if (found_signals != ALL_SIGNALS) {
        std::cerr << "[ERROR] The CAN db does not include all the needed signals for CPM encoding. Expected bit mask: 0x" << std::hex << static_cast<unsigned>(ALL_SIGNALS) << ", current bit mask: 0x" << static_cast<unsigned>(found_signals) << std::dec << std::endl;
        error = true;
    }

    return can_sensor_signal_info;
}

void dbcReader::printCANdb(bool &error) {
    if(!m_parsed) {
        std::cerr << "[ERROR] Attempted to print CAN db information without parsing a CAB database first." << std::endl;
        error = true;
        return;
    } else {
        error = false;
    }

    std::cout << "[DEBUG] CAN messages information:\n";

    // Print the information related to each CAN message
    for (auto& [msg_name, msg] : m_messages) {
        std::cout << "Message: " << msg_name << "\n";
        std::cout << "CAN ID: 0x" << std::hex << msg.can_id
                  << std::dec << " (" << msg.can_id << ")\n";
        std::cout << "Size: " << msg.size << " bytes\n\n";

        std::cout << "Signals:\n";
        for (const auto& sig_pair : msg.signals) {
            const auto& sig = sig_pair.second;
            std::cout << "  " << sig_pair.first << ":\n";
            std::cout << "    Start bit: " << sig.start_bit << "\n";
            std::cout << "    Length: " << sig.length << " bits\n";
            std::cout << "    Byte order: " << (sig.byte_order == 0 ? "Motorola (big endian)" : "Intel (little endian)") << "\n";
            std::cout << "    Factor: " << sig.factor << "\n";
            std::cout << "    Offset: " << sig.offset << "\n";
            std::cout << "    Start byte: " << sig.start_byte << "\n";
            std::cout << "    Mask: 0x" << std::hex << sig.mask << std::dec << "\n";
            std::cout << "    Shift right: " << sig.shift_right << "\n";
            std::cout << "    Unit: " << sig.unit << "\n\n";
        }
        std::cout << "----------------------------------------\n\n";
    }
}

void dbcReader::printSignalInfoSensorData(CAN_SENSOR_SIGNAL_INFO_t sigInfo) {
    std::cout << "[DEBUG] Sensor Signal Information:\n";

    for (const auto& sig : sigInfo) {
        std::cout << "Signal: " << sig.name << "\n";
        std::cout << "  Start bit: " << sig.start_bit << "\n";
        std::cout << "  Length: " << sig.length << " bits\n";
        std::cout << "  Byte order: " << (sig.byte_order == 0 ? "Motorola" : "Intel") << "\n";
        std::cout << "  Factor: " << sig.factor << "\n";
        std::cout << "  Offset: " << sig.offset << "\n";
        std::cout << "  Start byte: " << sig.start_byte << "\n";
        std::cout << "  Mask: 0x" << std::hex << sig.mask << std::dec << "\n";
        std::cout << "  Shift right: " << sig.shift_right << "\n";
        std::cout << "  Unit: " << sig.unit << "\n\n";
    }
}