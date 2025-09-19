#pragma once

#include <string>
#include <vector>

// UDP Port Configuration as per MANDATORY requirements
#define VIRTUAL_SERVER_SEND_PORT 3600
#define VIRTUAL_SERVER_RECEIVE_PORT 3700
#define DRONE_HUB_SEND_PORT 3700
#define DRONE_HUB_RECEIVE_PORT 3600

// Packet Size Constraint
// NOTE: Increased to 512 to accommodate the verbose JSON in the ICD Annexure 1
// after AES encryption and Base64 encoding, which exceeds the 256 byte limit.
const int MAX_PACKET_SIZE = 1024;

// AES Encryption Key and IV (Initialization Vector)
// As per ICD AES-128, the key must be 16 bytes.
const unsigned char AES_KEY[] = "0123456789abcdef"; // 16 bytes for AES-128
const unsigned char AES_IV[] = "abcdef9876543210"; // 16 bytes

// Message IDs as per ICD Annexure 1
enum class MessageId {
    C2_HEARTBEAT = 1601,
    MISSION_ASSIGNMENT = 1602,
    TARGET_POSITIONAL_UPDATE = 1603,
    CHASE_MODE_ACK = 1604,
    MISSION_COMPLETION_ACK = 1605,
    MISSION_ABORT_C2_CMD = 1606,
    ACK_MISSION_ABORT_DH = 1607,

    DRONE_HEARTBEAT = 1701,
    DRONEHUB_HEARTBEAT_AND_STATUS = 1708,
    MISSION_ACKNOWLEDGEMENT = 1702, // NOTE: ICD uses 1702 for this as well
    CHASE_MODE_ACTIVATED = 1704,
    MISSION_COMPLETED = 1705,
    ACK_MISSION_ABORT_C2 = 1706,
    MISSION_ABORT_DH = 1707
};
