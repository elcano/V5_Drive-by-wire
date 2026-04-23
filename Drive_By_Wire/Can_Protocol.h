#pragma once
/**
 * current CAN Protocol (4-23-19)
 * used for identifying source of signals, must be used
 * for high and low-level boards
 */

#define RCStatus_CANID 0x50
#define HiStatus_CANID 0x100
#define GoalReached_CANID 0x101
#define LowStatus_CANID 0x200
#define SetTime_CANID 0x250
#define SetOrigin_CANID 0x251
#define RCDrive_CANID 0x300
#define HiDrive_CANID 0x350
#define Actual_CANID 0x400
#define LiDAR_CANID 0x420
#define Sonar_CANID 0x440
#define CameraObstacl_CANID 0x460
#define CameraCone_CANID 0x480
#define CameraRiEdge_CANID 0x4A0
#define CameraLeEdge_CANID 0x4A1
// Vehicle / waypoint positions (E/N cm, int32)
#define VehiclePosition_CANID 0x4C0
// 0x4C1-0x4DF reserved for individual waypoint positions
// Header for log file
#define Header_CANID 0x700  
// ID 0x701-0x71F for logged items
#define Log_CANID  0x701   

