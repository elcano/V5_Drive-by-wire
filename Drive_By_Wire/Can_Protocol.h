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

// ---- Log message IDs (0x700-0x70A per wiki) ----
// See https://www.elcanoproject.org/wiki/Communication
#define Header_CANID         0x700   // session start; per-session bitmap planned
#define LogTime_CANID        0x701   // uint32 ms (LE)
#define LogRC_CANID          0x702   // RC channel pulse widths in microseconds
#define LogOp_CANID          0x703   // operator panel: joystick + switches + commanded
#define LogAuto_CANID        0x704   // what DBW received from Nav (mirrors 0x350 + 0x100)
#define LogDesired_CANID     0x705   // desired speed/brake/mode/angle from active source
#define LogThrottle_CANID    0x706   // actual speed, throttle PWM, drive mode
#define LogBrakes_CANID      0x707   // brake on, brake voltage
#define LogSteer_CANID       0x708   // actual angle, R/L column sensors
#define LogPosition_CANID    0x709   // vehicle position (E/N cm); emitted by Nav, not DBW
#define LogFinalize_CANID    0x70A   // end-of-row marker, CPU utilization %
// 0x70B-0x71F reserved for future log expansion
#define Log_CANID            LogTime_CANID  // backward-compat alias

