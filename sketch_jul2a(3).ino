/*
  LeArm bus-servo controller for MaixCAM2 vision.

  This sketch is UART-only. The old two-wire fallback and GPIO pulse mode are removed.

  Official LeArm ESP32 serial course uses:
    Serial.begin(9600, SERIAL_8N1, PA5, PA4);

  Official Config.h:
    PA5 = GPIO33
    PA4 = GPIO32
    BUS_TX = GPIO12
    BUS_RX = GPIO35
    BUS_EN = GPIO14
    IO_BLE_CTL = GPIO25
    IO_LED = GPIO26

  Wiring:
    MaixCAM2 A21 / UART4_TX -> LeArm PA5 / GPIO33 / ESP32 RX
    MaixCAM2 A22 / UART4_RX <- LeArm PA4 / GPIO32 / ESP32 TX
    MaixCAM2 GND            -> LeArm GND

  Text protocol:
    Maix -> ESP32:
      RING,SMALL,x,y,score
      RING,BIG,x,y,score
      RING,BLACK,x,y,score
      TARGET,x,y,score
      RING,BIG,x,y,w,h,score
      TARGET,x,y,w,h,score

    ESP32 -> Maix:
      READY_UART
      RX_CMD
      RING_OK
      NEED_RING
      BUSY
      NEED_TARGET
      TARGET_OK
      BUSY_PLACE
      WAIT_F4
      NEED_REGRAB
      REGRAB_OK
      DONE
      TARGET_TIMEOUT
      BAD_CMD
*/

#include <Arduino.h>

static const char *APP_VERSION = "esp32-uart-f4-flow-20260707-178-box-id5-300-hold";

// ---------------- Maix UART ----------------
static const int PA5_PIN = 33;
static const int PA4_PIN = 32;
static const int MAIX_UART_RX_PIN = PA5_PIN;  // ESP32 RX, LeArm PA5
static const int MAIX_UART_TX_PIN = PA4_PIN;  // ESP32 TX, LeArm PA4
static const uint32_t MAIX_UART_BAUD = 9600;
static const uint8_t UART_CMD_MAX = 96;
static const uint32_t UART_STATUS_PERIOD_MS = 1000;

// ---------------- Board pins ----------------
static const int BUS_TX_PIN = 12;
static const int BUS_RX_PIN = 35;
static const int BUS_EN_PIN = 14;
static const int BLE_POWER_PIN = 25;
static const int DEBUG_LED_PIN = 26;

// ---------------- Serial ports ----------------
static const uint32_t BUS_BAUD = 115200;
static const uint32_t F4_UART_BAUD = 115200;
// Schematic labels IO17/TX2 and IO16/RX2 on the SDA/SCL header.
// Arduino begin() order is RX first, then TX.
static const int F4_UART_RX_PIN = 16;  // ESP32 IO16/RX2 <- F4 TX
static const int F4_UART_TX_PIN = 17;  // ESP32 IO17/TX2 -> F4 RX
// Official LeArm ESP32 examples use UART1 for the bus-servo link.
HardwareSerial BusSerial(1);
HardwareSerial F4Serial(2);

// ---------------- LeArm bus-servo protocol ----------------
static const uint8_t LOBOT_SERVO_FRAME_HEADER = 0x55;
static const uint8_t LOBOT_SERVO_MOVE_TIME_WRITE = 1;
static const uint8_t SERVO_COUNT = 6;
static const int BUS_WRITE_LEVEL = HIGH;

struct Pose {
  uint16_t p[SERVO_COUNT];
};

// Pose order:
//   base/yaw, shoulder, elbow, wrist pitch, wrist roll, claw
// Real bus-servo IDs:
//   ID6 = base/yaw, ID5 = shoulder, ID4 = elbow, ID3 = wrist pitch,
//   ID2 = wrist roll, ID1 = claw.
static const uint8_t BASE_SERVO_ID = 6;
static const uint8_t SHOULDER_SERVO_ID = 5;
static const uint8_t ELBOW_SERVO_ID = 4;
static const uint8_t WRIST_PITCH_SERVO_ID = 3;
static const uint8_t WRIST_ROLL_SERVO_ID = 2;
static const uint8_t CLAW_SERVO_ID = 1;
static const uint8_t POSE_TO_BUS_ID[SERVO_COUNT] = {
  BASE_SERVO_ID,
  SHOULDER_SERVO_ID,
  ELBOW_SERVO_ID,
  WRIST_PITCH_SERVO_ID,
  WRIST_ROLL_SERVO_ID,
  CLAW_SERVO_ID
};

// Calibrated points from the current project. Base angles are shifted from the
// old 500-centered layout to the new 300-centered initial layout.
static const uint16_t PICK_BASE_P = 315;
static const uint16_t TARGET_BASE_P = 500;
static const uint16_t GREEN_TARGET_BASE_P = 800;
static const uint16_t HOME_BASE_P = 315;
static const int16_t PICK_TO_TARGET_DELTA_P = TARGET_BASE_P - PICK_BASE_P;
// Box order while rotating from 800 -> 1000:
//   800..866:  white box = GOOD
//   867..933:  black box = BAD
//   934..1000: last box  = REVIEW
// After the second F4 command, place at the center of the selected window.
static const uint16_t BOX_SCAN_START_BASE_P = 800;
static const uint16_t BOX_SCAN_END_BASE_P = 1000;
static const uint16_t BOX_APPROACH_BASE_P = 825;
static const uint16_t BOX_PRE_PLACE_ID5_P = 300;
static const uint16_t BOX_PRE_PLACE_ID4_P = 0;
static const uint16_t BOX_SCAN_STEP_P = 15;
static const uint16_t BOX_SCAN_STEP_MS = 900;
static const uint16_t BOX_SCAN_SETTLE_MS = 120;
static const uint16_t BOX_GOOD_SCAN_START_P = 800;
static const uint16_t BOX_GOOD_SCAN_END_P = 866;
static const uint16_t BOX_BAD_SCAN_START_P = 867;
static const uint16_t BOX_BAD_SCAN_END_P = 933;
static const uint16_t BOX_REVIEW_SCAN_START_P = 934;
static const uint16_t BOX_REVIEW_SCAN_END_P = 1000;
static const uint16_t BOX_BASE_P = BOX_SCAN_START_BASE_P;
static const int16_t PICK_LEFT_BASE_OFFSET_P = 35;
static const int16_t PICK_CENTER_BASE_OFFSET_P = 0;
static const int16_t PICK_RIGHT_BASE_OFFSET_P = -35;

static const uint16_t OBSERVE_SHOULDER_P = 500;
static const uint16_t OBSERVE_ELBOW_P = 235;
static const uint16_t HOME_ELBOW_P = 245;
static const uint16_t OBSERVE_WRIST_PITCH_P = 300;
static const uint16_t GRIP_LIFT_ELBOW_P = 320;
static const uint16_t ROTATE_HOLD_ELBOW_P = 334;
static const uint16_t TARGET_DROP_ELBOW_P = 225;
static const uint16_t GRIP_LIFT_WRIST_PITCH_P = 300;
static const uint16_t OBSERVE_WRIST_ROLL_P = 500;
static const uint16_t GRIP_WRIST_ROLL_P = 500;
static const uint16_t CLAW_OPEN_P = 300;
static const uint16_t CLAW_CLOSE_P = 700;

static const Pose HOME_POSE = {
  {HOME_BASE_P, OBSERVE_SHOULDER_P, HOME_ELBOW_P, OBSERVE_WRIST_PITCH_P, OBSERVE_WRIST_ROLL_P, CLAW_OPEN_P}
};

static const Pose ABOVE_PICK_POSE = {
  {PICK_BASE_P, OBSERVE_SHOULDER_P, OBSERVE_ELBOW_P, OBSERVE_WRIST_PITCH_P, OBSERVE_WRIST_ROLL_P, CLAW_OPEN_P}
};

static const Pose ABOVE_PICK_HOLD_POSE = {
  {PICK_BASE_P, OBSERVE_SHOULDER_P, GRIP_LIFT_ELBOW_P, GRIP_LIFT_WRIST_PITCH_P, GRIP_WRIST_ROLL_P, CLAW_CLOSE_P}
};

static const Pose PICK_POSE = {
  {PICK_BASE_P, 500, 314, OBSERVE_WRIST_PITCH_P, GRIP_WRIST_ROLL_P, CLAW_OPEN_P}
};

static const Pose PICK_CLOSE_POSE = {
  {PICK_BASE_P, 500, 314, OBSERVE_WRIST_PITCH_P, GRIP_WRIST_ROLL_P, CLAW_CLOSE_P}
};

static const Pose PICK_LIFT_POSE = {
  {PICK_BASE_P, 500, GRIP_LIFT_ELBOW_P, GRIP_LIFT_WRIST_PITCH_P, GRIP_WRIST_ROLL_P, CLAW_CLOSE_P}
};

static const Pose ABOVE_TARGET_POSE = {
  {TARGET_BASE_P, 500, OBSERVE_ELBOW_P, GRIP_LIFT_WRIST_PITCH_P, GRIP_WRIST_ROLL_P, CLAW_CLOSE_P}
};

static const Pose ROTATE_TARGET_POSE = {
  {TARGET_BASE_P, 500, ROTATE_HOLD_ELBOW_P, GRIP_LIFT_WRIST_PITCH_P, GRIP_WRIST_ROLL_P, CLAW_CLOSE_P}
};

static const Pose GREEN_ROTATE_POSE = {
  {GREEN_TARGET_BASE_P, 500, ROTATE_HOLD_ELBOW_P, GRIP_LIFT_WRIST_PITCH_P, GRIP_WRIST_ROLL_P, CLAW_CLOSE_P}
};

static const Pose BOX_ROTATE_APPROACH_POSE = {
  {BOX_APPROACH_BASE_P, 500, ROTATE_HOLD_ELBOW_P, GRIP_LIFT_WRIST_PITCH_P, GRIP_WRIST_ROLL_P, CLAW_CLOSE_P}
};

static const Pose BOX_PRE_PLACE_POSE = {
  {BOX_APPROACH_BASE_P, BOX_PRE_PLACE_ID5_P, BOX_PRE_PLACE_ID4_P, GRIP_LIFT_WRIST_PITCH_P, GRIP_WRIST_ROLL_P, CLAW_CLOSE_P}
};

static const Pose TARGET_POSE = {
  {TARGET_BASE_P, 500, TARGET_DROP_ELBOW_P, OBSERVE_WRIST_PITCH_P, GRIP_WRIST_ROLL_P, CLAW_CLOSE_P}
};

static const Pose TARGET_OPEN_POSE = {
  {TARGET_BASE_P, 500, TARGET_DROP_ELBOW_P, OBSERVE_WRIST_PITCH_P, GRIP_WRIST_ROLL_P, CLAW_OPEN_P}
};

static const Pose TARGET_WAIT_POSE = {
  {TARGET_BASE_P, 500, 265, OBSERVE_WRIST_PITCH_P, GRIP_WRIST_ROLL_P, CLAW_OPEN_P}
};

static const Pose TARGET_CLOSE_POSE = {
  {TARGET_BASE_P, 500, TARGET_DROP_ELBOW_P, OBSERVE_WRIST_PITCH_P, GRIP_WRIST_ROLL_P, CLAW_CLOSE_P}
};

static const Pose ABOVE_BOX_POSE = {
  {BOX_BASE_P, 500, OBSERVE_ELBOW_P, GRIP_LIFT_WRIST_PITCH_P, GRIP_WRIST_ROLL_P, CLAW_CLOSE_P}
};

static const Pose ROTATE_BOX_POSE = {
  {BOX_BASE_P, 500, GRIP_LIFT_ELBOW_P, GRIP_LIFT_WRIST_PITCH_P, GRIP_WRIST_ROLL_P, CLAW_CLOSE_P}
};

static const Pose BOX_POSE = {
  {BOX_BASE_P, 500, 314, OBSERVE_WRIST_PITCH_P, GRIP_WRIST_ROLL_P, CLAW_CLOSE_P}
};

static const Pose BOX_OPEN_POSE = {
  {BOX_BASE_P, 500, 314, OBSERVE_WRIST_PITCH_P, GRIP_WRIST_ROLL_P, CLAW_OPEN_P}
};

static const uint16_t MOVE_MS = 900;
static const uint16_t PICK_APPROACH_MS = 2800;
static const uint16_t PICK_RECENTER_MS = 2600;
static const uint16_t PICK_VISION_SETTLE_MS = 1000;
static const uint16_t PICK_FORWARD_MS = 1000;
static const uint16_t PICK_CLOSE_MS = 900;
static const uint16_t ROTATE_WITH_OBJECT_MS = 1800;
static const uint16_t ROTATE_SETTLE_MS = 500;
static const uint16_t GRIP_MS = 550;
static const uint16_t GRIP_HOLD_MS = 1100;
static const uint16_t PICK_LIFT_MS = 700;
static const uint16_t TARGET_DIRECT_WAIT_MS = 1000;
static const uint16_t MOTION_SPEED_SCALE_NUM = 3;
static const uint16_t MOTION_SPEED_SCALE_DEN = 2;
static const uint32_t ACK_HOLD_MS = 140;
static const uint32_t DONE_HOLD_MS = 1800;
static const uint32_t TARGET_TIMEOUT_MS = 60000;
static const uint32_t PICK_TRACK_TIMEOUT_MS = 9000;
static const uint8_t PICK_TRACK_PASSES = 8;
static const uint8_t PICK_GRIP_CONFIRM_FRAMES = 1;

// Vision input is 224x224. X mainly adjusts base yaw.
// Y reach correction is intentionally conservative; flip the sign if your camera
// orientation makes near/far motion opposite.
static const int VISION_CENTER_X = 112;
static const int VISION_CENTER_Y = 112;
// Camera-to-claw offset calibration. The claw is about 9.5 cm from the camera;
// at the current camera height this is roughly 6.5 image pixels per cm.
// When picking, the object should align to this claw projection point, not to
// the camera center.
static const float CAMERA_TO_CLAW_CM = 9.5f;
static const float PICK_GRASP_FORWARD_CORRECTION_CM = 5.5f;
static const float PICK_EFFECTIVE_CLAW_CM = 15.0f;
static const float PICK_IMAGE_PX_PER_CM = 6.5f;
static const int PICK_GRIPPER_TARGET_X = 112;
static const int PICK_GRIPPER_TARGET_Y = 125;
static const int PICK_GRASP_X_TOL_PX = 55;
static const int PICK_GRASP_Y_TOL_PX = 45;
static const int PICK_GRASP_BOTTOM_MIN_Y = 115;
static const float PICK_GRASP_DISTANCE_TOL_CM = 2.0f;
static const int RING_IMAGE_CENTER_X = 112;
static const int RING_IMAGE_X_DEADBAND = 18;
static const float PICK_BASE_GAIN_P_PER_PX = -1.2f;
static const int16_t PICK_BASE_MAX_OFFSET_P = 80;
static const int16_t PICK_FORWARD_BASE_PUSH_P = 0;
static const float TARGET_BASE_GAIN_P_PER_PX = -1.2f;
static const int16_t TARGET_BASE_MAX_OFFSET_P = 80;
static const float PICK_REACH_GAIN_P_PER_PX = 0.65f;
static const int16_t PICK_REACH_MAX_OFFSET_P = 100;
static const int16_t PICK_REACH_FORWARD_BIAS_P = 0;
static const int16_t PICK_CLOSE_ID5_FORWARD_P = 135;
static const int16_t PICK_CLOSE_ID4_LIFT_P = 8;
static const int16_t PICK_RETRACT_ID5_BACK_P = 335;
static const int16_t PICK_RETRACT_ID4_DOWN_P = 125;
static const int16_t PICK_RETRACT_ID3_DOWN_P = 180;
static const uint16_t POST_ROTATE_ID5_P = 400;
static const int16_t POST_ROTATE_ID4_DOWN_P = 300;
static const int16_t TARGET_KEEP_ID3_LIFT_P = 0;
static const int16_t F4_REGRAB_VISION_ID3_DOWN_P = 175;
static const int16_t GREEN_VISION_ID3_DOWN_P = 100;
static const int16_t GREEN_VISION_ID4_DOWN_P = 45;
static const uint16_t GREEN_VISION_ID5_P = 625;
static const float TARGET_REACH_GAIN_P_PER_PX = 0.18f;
static const int16_t TARGET_REACH_MAX_OFFSET_P = 25;

static const bool ENABLE_HOME_POSE = true;
static const bool HOME_BASE_ONLY = false;
static const bool BOOT_BASE_TEST = false;
static const int16_t BOOT_BASE_TEST_OFFSET_P = 25;
static const uint16_t BOOT_BASE_TEST_MS = 900;
static const bool BOOT_CLAW_TEST = false;

enum PickZone : uint8_t {
  PICK_ZONE_NONE = 0,
  PICK_ZONE_LEFT,
  PICK_ZONE_CENTER,
  PICK_ZONE_RIGHT
};

struct VisionPoint {
  bool valid;
  int x;
  int y;
  float score;
  int w;
  int h;
  float cameraDistanceCm;
  bool hasSize;
};

enum LinkStatus : uint8_t {
  ST_BOOT = 0,
  ST_READY,
  ST_RX_CMD,
  ST_RING_OK,
  ST_NEED_RING,
  ST_PICK_UNREACHABLE,
  ST_BAD_CMD,
  ST_NEED_TARGET,
  ST_NEED_GREEN_TARGET,
  ST_TARGET_OK,
  ST_TARGET_TIMEOUT,
  ST_BUSY,
  ST_BUSY_PLACE,
  ST_WAIT_F4,
  ST_WAIT_F4_REGRAB,
  ST_WAIT_F4_BOX,
  ST_NEED_REGRAB,
  ST_REGRAB_OK,
  ST_DONE
};

enum ArmWorkflowState : uint8_t {
  WF_IDLE = 0,
  WF_WAIT_START_F4,
  WF_RING_ACK,
  WF_PICK_ABOVE,
  WF_WAIT_PICK_RING,
  WF_PICK_TRACK_ACK,
  WF_PICK_RECENTER,
  WF_PICK_DOWN,
  WF_PICK_SETTLE,
  WF_PICK_CLOSE,
  WF_PICK_LIFT,
  WF_PICK_HOLD,
  WF_ROTATE_TARGET,
  WF_AFTER_ROTATE_RETRACT,
  WF_TARGET_LOWER,
  WF_WAIT_TARGET,
  WF_TARGET_ACK,
  WF_PLACE_ABOVE,
  WF_PLACE_DOWN,
  WF_PLACE_OPEN,
  WF_PLACE_UP,
  WF_WAIT_F4,
  WF_REGRAB_VISION_LOWER,
  WF_WAIT_REGRAB,
  WF_REGRAB_ACK,
  WF_REGRAB_ABOVE,
  WF_REGRAB_DOWN,
  WF_REGRAB_SETTLE,
  WF_REGRAB_CLOSE,
  WF_REGRAB_LIFT,
  WF_REGRAB_HOLD,
  WF_GREEN_ROTATE,
  WF_GREEN_LOWER,
  WF_WAIT_GREEN_TARGET,
  WF_GREEN_TARGET_ACK,
  WF_GREEN_PLACE_DOWN,
  WF_GREEN_PLACE_OPEN,
  WF_GREEN_PLACE_UP,
  WF_WAIT_BOX_F4,
  WF_FINAL_REGRAB_DOWN,
  WF_FINAL_REGRAB_SETTLE,
  WF_FINAL_REGRAB_CLOSE,
  WF_FINAL_REGRAB_LIFT,
  WF_SECOND_F4_ROTATE,
  WF_BOX_SCAN,
  WF_BOX_TARGET_ACK,
  WF_BOX_ABOVE,
  WF_BOX_DOWN,
  WF_BOX_OPEN,
  WF_BOX_UP,
  WF_HOME_DONE,
  WF_DONE_HOLD,
  WF_ABORT_HOME,
  WF_ABORT_HOLD
};

LinkStatus linkStatus = ST_BOOT;
bool statusDirty = true;
uint32_t lastStatusMs = 0;
uint32_t lastHeartbeatMs = 0;
String uartRxLine;

bool busy = false;
ArmWorkflowState workflowState = WF_IDLE;
uint32_t workflowDeadlineMs = 0;
uint32_t workflowWaitStartMs = 0;
PickZone workflowPickZone = PICK_ZONE_CENTER;
VisionPoint workflowRingPoint = {false, 0, 0, 0.0f};
VisionPoint workflowTargetPoint = {false, 0, 0, 0.0f};
VisionPoint workflowRegrabPoint = {false, 0, 0, 0.0f};
uint8_t workflowPickTrackCount = 0;
uint8_t workflowPickGripOkCount = 0;

static const uint8_t ARM_LINK_SOF0 = 0xA5;
static const uint8_t ARM_LINK_SOF1 = 0x5A;
static const uint8_t ARM_LINK_VERSION = 0x01;
static const uint8_t ARM_LINK_EOF = 0x6B;
static const uint8_t ARM_LINK_MAX_PAYLOAD = 48;
static const uint8_t ARM_LINK_MIN_FRAME = 10;
static const uint8_t ARM_LINK_MAX_FRAME = ARM_LINK_MIN_FRAME + ARM_LINK_MAX_PAYLOAD;
static const uint8_t ARM_LINK_STAGE_CMD_LEN = 14;
static const uint8_t ARM_LINK_STAGE_DONE_LEN = 10;
static const uint8_t ARM_LINK_ACK_LEN = 7;
static const uint8_t ARM_LINK_NACK_LEN = 9;

static const uint8_t ARM_LINK_CMD_HELLO = 0x01;
static const uint8_t ARM_LINK_CMD_HEARTBEAT = 0x02;
static const uint8_t ARM_LINK_CMD_MOVE_TO_WEIGHT = 0x20;
static const uint8_t ARM_LINK_CMD_MOVE_TO_LDC = 0x21;
static const uint8_t ARM_LINK_CMD_SORT_RESULT = 0x22;
static const uint8_t ARM_LINK_CMD_HOME = 0x23;
static const uint8_t ARM_LINK_CMD_STOP = 0x24;
static const uint8_t ARM_LINK_CMD_STAGE_DONE = 0x30;
static const uint8_t ARM_LINK_CMD_ACK = 0x80;
static const uint8_t ARM_LINK_CMD_NACK = 0x81;

static const uint8_t ARM_LINK_STAGE_NONE = 0;
static const uint8_t ARM_LINK_STAGE_PICK_BELT_TO_WEIGHT = 1;
static const uint8_t ARM_LINK_STAGE_WEIGHT_TO_LDC = 2;
static const uint8_t ARM_LINK_STAGE_LDC_TO_SORT_BIN = 3;
static const uint8_t ARM_LINK_STAGE_HOME = 4;
static const uint8_t ARM_LINK_STAGE_STOP_SAFE = 5;

static const uint8_t ARM_LINK_BIN_NONE = 0;
static const uint8_t ARM_LINK_BIN_GOOD = 1;
static const uint8_t ARM_LINK_BIN_BAD = 2;
static const uint8_t ARM_LINK_BIN_REVIEW = 3;

static const uint8_t ARM_LINK_STATE_IDLE = 0;
static const uint8_t ARM_LINK_STATE_MOVING = 1;
static const uint8_t ARM_LINK_STATE_HOLDING_PART = 2;
static const uint8_t ARM_LINK_STATE_DONE = 3;
static const uint8_t ARM_LINK_STATE_STOPPED = 4;
static const uint8_t ARM_LINK_STATE_FAULT = 5;

static const uint8_t ARM_LINK_RESULT_OK = 0;
static const uint8_t ARM_LINK_ERROR_CRC = 1;
static const uint8_t ARM_LINK_ERROR_FRAME_LENGTH = 2;
static const uint8_t ARM_LINK_ERROR_CMD_UNKNOWN = 3;
static const uint8_t ARM_LINK_ERROR_PAYLOAD_LENGTH = 4;
static const uint8_t ARM_LINK_ERROR_STATE_NOT_ALLOWED = 5;
static const uint8_t ARM_LINK_ERROR_BUSY = 6;
static const uint8_t ARM_LINK_ERROR_CYCLE_MISMATCH = 7;
static const uint8_t ARM_LINK_ERROR_MOTION_TIMEOUT = 8;
static const uint8_t ARM_LINK_ERROR_GRIP_FAILED = 9;
static const uint8_t ARM_LINK_ERROR_SERVO_FAULT = 10;
static const uint8_t ARM_LINK_ERROR_LIMIT_OR_ESTOP = 11;

struct F4StageCommand {
  uint16_t cycleId;
  uint8_t stageId;
  uint8_t partType;
  uint8_t modelResult;
  uint8_t targetBin;
  uint32_t timeoutMs;
  uint8_t motionProfile;
  uint16_t flags;
};

struct F4Frame {
  uint8_t command;
  uint8_t payloadLength;
  uint16_t sequence;
  uint8_t payload[ARM_LINK_MAX_PAYLOAD];
};

uint8_t f4RxBuffer[ARM_LINK_MAX_FRAME];
uint8_t f4RxLen = 0;
uint16_t f4TxSeq = 1;
uint16_t activeF4CycleId = 0;
uint8_t activeF4StageId = ARM_LINK_STAGE_PICK_BELT_TO_WEIGHT;
uint8_t activeF4Command = 0;
uint8_t activeF4TargetBin = 0;
uint32_t activeF4StartMs = 0;
uint32_t activeF4TimeoutMs = TARGET_TIMEOUT_MS;
bool pendingF4SafetyDone = false;
uint16_t pendingF4SafetyCycleId = 0;
uint8_t pendingF4SafetyStageId = ARM_LINK_STAGE_NONE;
uint32_t pendingF4SafetyStartMs = 0;
uint16_t workflowBoxScanBaseP = BOX_SCAN_START_BASE_P;
uint16_t workflowBoxScanEndP = BOX_SCAN_END_BASE_P;
uint32_t lastF4DiagMs = 0;


void startPoseAsync(const Pose &pose, uint16_t durationMs, uint16_t extraDelayMs);
Pose workflowF4RegrabVisionPose();
Pose workflowGreenTargetPose(const Pose &pose);
Pose workflowGreenVisionPoseWithClaw(uint16_t claw);
Pose workflowBoxPose(const Pose &pose);
void enterWaitStartF4();
uint16_t boxScanStartForTargetBin(uint8_t targetBin);
uint16_t boxScanEndForTargetBin(uint8_t targetBin);
uint16_t boxCenterForTargetBin(uint8_t targetBin);
bool boxScanBaseInTargetBin(uint8_t targetBin, uint16_t base);
void acceptBoxTargetWorkflow(const VisionPoint &targetPoint);


bool f4TargetBinValid(uint8_t targetBin) {
  return targetBin == ARM_LINK_BIN_GOOD ||
         targetBin == ARM_LINK_BIN_BAD ||
         targetBin == ARM_LINK_BIN_REVIEW;
}


uint16_t f4CommandDetail(const F4StageCommand &cmd) {
  return (uint16_t)(((uint16_t)cmd.stageId << 8) | cmd.targetBin);
}


bool f4IsMoveToLdcCommand(const F4Frame &frame, const F4StageCommand &cmd) {
  return frame.command == ARM_LINK_CMD_MOVE_TO_LDC &&
         cmd.stageId == ARM_LINK_STAGE_WEIGHT_TO_LDC &&
         cmd.targetBin == ARM_LINK_BIN_NONE;
}


bool f4IsMoveToWeightCommand(const F4Frame &frame, const F4StageCommand &cmd) {
  return frame.command == ARM_LINK_CMD_MOVE_TO_WEIGHT &&
         cmd.stageId == ARM_LINK_STAGE_PICK_BELT_TO_WEIGHT &&
         cmd.targetBin == ARM_LINK_BIN_NONE;
}


bool f4IsSortResultCommand(const F4Frame &frame, const F4StageCommand &cmd) {
  return frame.command == ARM_LINK_CMD_SORT_RESULT &&
         cmd.stageId == ARM_LINK_STAGE_LDC_TO_SORT_BIN &&
         f4TargetBinValid(cmd.targetBin);
}


void logF4Diag(const char *tag, uint8_t value = 0) {
  uint32_t now = millis();
  if (now - lastF4DiagMs < 1000) {
    return;
  }
  lastF4DiagMs = now;
  Serial.print(tag);
  Serial.print(",byte=0x");
  if (value < 16) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
  Serial.print('\n');
}


const char *statusText(LinkStatus status) {
  switch (status) {
    case ST_BOOT: return "BOOT";
    case ST_READY: return "READY_UART";
    case ST_RX_CMD: return "RX_CMD";
    case ST_RING_OK: return "RING_OK";
    case ST_NEED_RING: return "NEED_RING";
    case ST_PICK_UNREACHABLE: return "PICK_UNREACHABLE";
    case ST_BAD_CMD: return "BAD_CMD";
    case ST_NEED_TARGET: return "NEED_TARGET";
    case ST_NEED_GREEN_TARGET: return "NEED_GREEN_TARGET";
    case ST_TARGET_OK: return "TARGET_OK";
    case ST_TARGET_TIMEOUT: return "TARGET_TIMEOUT";
    case ST_BUSY: return "BUSY";
    case ST_BUSY_PLACE: return "BUSY_PLACE";
    case ST_WAIT_F4: return "WAIT_F4";
    case ST_WAIT_F4_REGRAB: return "WAIT_F4_REGRAB";
    case ST_WAIT_F4_BOX: return "WAIT_F4_SECOND";
    case ST_NEED_REGRAB: return "NEED_REGRAB";
    case ST_REGRAB_OK: return "REGRAB_OK";
    case ST_DONE: return "DONE";
    default: return "UNKNOWN";
  }
}


void setStatus(LinkStatus status) {
  if (linkStatus != status) {
    linkStatus = status;
    statusDirty = true;
  }
}


void sendStatus(bool force = false) {
  uint32_t now = millis();
  if (!force && !statusDirty && now - lastStatusMs < UART_STATUS_PERIOD_MS) {
    return;
  }

  Serial.print(statusText(linkStatus));
  Serial.print('\n');
  statusDirty = false;
  lastStatusMs = now;
}


void sendTrace(const char *text) {
  Serial.print(text);
  Serial.print('\n');
}


void initMaixUart() {
  Serial.begin(MAIX_UART_BAUD, SERIAL_8N1, MAIX_UART_RX_PIN, MAIX_UART_TX_PIN);
  uartRxLine = "";
  delay(50);
}


bool fetchUartCommand(String &line) {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      uartRxLine.trim();
      if (uartRxLine.length() > 0) {
        line = uartRxLine;
        uartRxLine = "";
        return true;
      }
      uartRxLine = "";
    } else if ((uint8_t)c >= 32 && (uint8_t)c < 127) {
      uartRxLine += c;
      if (uartRxLine.length() > UART_CMD_MAX) {
        uartRxLine = "";
        setStatus(ST_BAD_CMD);
        sendStatus(true);
      }
    }
  }

  return false;
}


void serviceHeartbeat() {
  uint32_t now = millis();
  if (now - lastHeartbeatMs >= 500) {
    lastHeartbeatMs = now;
    digitalWrite(DEBUG_LED_PIN, !digitalRead(DEBUG_LED_PIN));
  }
}


uint16_t readU16Le(const uint8_t *buffer) {
  return (uint16_t)(((uint16_t)buffer[0]) | ((uint16_t)buffer[1] << 8));
}


uint32_t readU32Le(const uint8_t *buffer) {
  return ((uint32_t)buffer[0]) |
         ((uint32_t)buffer[1] << 8) |
         ((uint32_t)buffer[2] << 16) |
         ((uint32_t)buffer[3] << 24);
}


void writeU16Le(uint8_t *buffer, uint16_t value) {
  buffer[0] = (uint8_t)(value & 0xFF);
  buffer[1] = (uint8_t)((value >> 8) & 0xFF);
}


uint16_t armLinkCrc16(const uint8_t *data, uint16_t length) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = (uint16_t)((crc << 1) ^ 0x1021);
      } else {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
}


uint16_t buildArmLinkFrame(uint8_t command, uint16_t sequence, const uint8_t *payload, uint8_t payloadLen, uint8_t *out) {
  if (payloadLen > ARM_LINK_MAX_PAYLOAD) {
    return 0;
  }

  out[0] = ARM_LINK_SOF0;
  out[1] = ARM_LINK_SOF1;
  out[2] = ARM_LINK_VERSION;
  out[3] = command;
  out[4] = payloadLen;
  writeU16Le(&out[5], sequence);
  for (uint8_t i = 0; i < payloadLen; i++) {
    out[7 + i] = payload[i];
  }

  uint16_t crc = armLinkCrc16(&out[2], (uint16_t)(5 + payloadLen));
  uint8_t crcOffset = (uint8_t)(7 + payloadLen);
  writeU16Le(&out[crcOffset], crc);
  out[crcOffset + 2] = ARM_LINK_EOF;
  return (uint16_t)(ARM_LINK_MIN_FRAME + payloadLen);
}


bool parseArmLinkFrame(const uint8_t *frame, uint8_t frameLen, F4Frame &out) {
  if (frameLen < ARM_LINK_MIN_FRAME || frameLen > ARM_LINK_MAX_FRAME) {
    return false;
  }
  if (frame[0] != ARM_LINK_SOF0 || frame[1] != ARM_LINK_SOF1 || frame[2] != ARM_LINK_VERSION) {
    return false;
  }

  uint8_t payloadLen = frame[4];
  if (payloadLen > ARM_LINK_MAX_PAYLOAD || frameLen != ARM_LINK_MIN_FRAME + payloadLen) {
    return false;
  }
  if (frame[frameLen - 1] != ARM_LINK_EOF) {
    return false;
  }

  uint8_t crcOffset = (uint8_t)(7 + payloadLen);
  uint16_t receivedCrc = readU16Le(&frame[crcOffset]);
  uint16_t calculatedCrc = armLinkCrc16(&frame[2], (uint16_t)(5 + payloadLen));
  if (receivedCrc != calculatedCrc) {
    return false;
  }

  out.command = frame[3];
  out.payloadLength = payloadLen;
  out.sequence = readU16Le(&frame[5]);
  for (uint8_t i = 0; i < payloadLen; i++) {
    out.payload[i] = frame[7 + i];
  }
  return true;
}


bool decodeF4StageCommand(const F4Frame &frame, F4StageCommand &cmd) {
  if (frame.payloadLength != ARM_LINK_STAGE_CMD_LEN) {
    return false;
  }

  cmd.cycleId = readU16Le(&frame.payload[0]);
  cmd.stageId = frame.payload[2];
  cmd.partType = frame.payload[3];
  cmd.modelResult = frame.payload[4];
  cmd.targetBin = frame.payload[5];
  cmd.timeoutMs = readU32Le(&frame.payload[6]);
  cmd.motionProfile = frame.payload[10];
  cmd.flags = readU16Le(&frame.payload[11]);
  return true;
}


uint8_t armStateForF4() {
  if (workflowState == WF_IDLE || workflowState == WF_WAIT_START_F4) {
    return ARM_LINK_STATE_IDLE;
  }
  if (workflowState == WF_WAIT_F4 || workflowState == WF_WAIT_BOX_F4) {
    return ARM_LINK_STATE_DONE;
  }
  if (workflowState == WF_ABORT_HOME || workflowState == WF_ABORT_HOLD) {
    return ARM_LINK_STATE_STOPPED;
  }
  if (workflowState == WF_REGRAB_CLOSE || workflowState == WF_REGRAB_LIFT || workflowState == WF_REGRAB_HOLD ||
      workflowState == WF_GREEN_ROTATE || workflowState == WF_GREEN_LOWER ||
      workflowState == WF_WAIT_GREEN_TARGET || workflowState == WF_GREEN_TARGET_ACK ||
      workflowState == WF_GREEN_PLACE_DOWN || workflowState == WF_GREEN_PLACE_OPEN ||
      workflowState == WF_GREEN_PLACE_UP ||
      workflowState == WF_FINAL_REGRAB_CLOSE || workflowState == WF_FINAL_REGRAB_LIFT ||
      workflowState == WF_SECOND_F4_ROTATE ||
      workflowState == WF_BOX_SCAN || workflowState == WF_BOX_TARGET_ACK ||
      workflowState == WF_BOX_ABOVE || workflowState == WF_BOX_DOWN) {
    return ARM_LINK_STATE_HOLDING_PART;
  }
  return ARM_LINK_STATE_MOVING;
}


void writeF4Frame(uint8_t command, const uint8_t *payload, uint8_t payloadLen) {
  uint8_t frame[ARM_LINK_MAX_FRAME];
  uint16_t len = buildArmLinkFrame(command, f4TxSeq++, payload, payloadLen, frame);
  if (len > 0) {
    F4Serial.write(frame, len);
    F4Serial.flush();
  }
}


void sendF4Ack(const F4Frame &frame, uint16_t cycleId, uint8_t acceptedStatus = 0) {
  uint8_t payload[ARM_LINK_ACK_LEN];
  writeU16Le(&payload[0], cycleId);
  writeU16Le(&payload[2], frame.sequence);
  payload[4] = frame.command;
  payload[5] = acceptedStatus;
  payload[6] = armStateForF4();
  writeF4Frame(ARM_LINK_CMD_ACK, payload, ARM_LINK_ACK_LEN);
}


void sendF4Nack(const F4Frame &frame, uint16_t cycleId, uint8_t errorCode, uint16_t detail = 0) {
  uint8_t payload[ARM_LINK_NACK_LEN];
  writeU16Le(&payload[0], cycleId);
  writeU16Le(&payload[2], frame.sequence);
  payload[4] = frame.command;
  payload[5] = errorCode;
  payload[6] = armStateForF4();
  writeU16Le(&payload[7], detail);
  writeF4Frame(ARM_LINK_CMD_NACK, payload, ARM_LINK_NACK_LEN);
}


void sendF4StageDone(uint16_t cycleId, uint8_t stageId, uint8_t result, uint16_t detailCode, uint16_t elapsedMs) {
  uint8_t payload[ARM_LINK_STAGE_DONE_LEN];
  writeU16Le(&payload[0], cycleId);
  payload[2] = stageId;
  payload[3] = result;
  writeU16Le(&payload[4], detailCode);
  writeU16Le(&payload[6], elapsedMs);
  writeU16Le(&payload[8], 0);
  writeF4Frame(ARM_LINK_CMD_STAGE_DONE, payload, ARM_LINK_STAGE_DONE_LEN);
}


bool takeF4Frame(F4Frame &out) {
  while (F4Serial.available()) {
    uint8_t value = (uint8_t)F4Serial.read();

    if (f4RxLen == 0 && value != ARM_LINK_SOF0) {
      logF4Diag("F4_RX_NONFRAME", value);
      continue;
    }
    if (f4RxLen == 1 && value != ARM_LINK_SOF1) {
      logF4Diag("F4_RX_BAD_SOF1", value);
      f4RxLen = 0;
      if (value == ARM_LINK_SOF0) {
        f4RxBuffer[f4RxLen++] = value;
      }
      continue;
    }

    if (f4RxLen < ARM_LINK_MAX_FRAME) {
      f4RxBuffer[f4RxLen++] = value;
    } else {
      f4RxLen = 0;
      continue;
    }

    if (f4RxLen >= 5) {
      uint8_t payloadLen = f4RxBuffer[4];
      if (payloadLen > ARM_LINK_MAX_PAYLOAD) {
        logF4Diag("F4_RX_BAD_LEN", payloadLen);
        f4RxLen = 0;
        continue;
      }
      uint8_t expectedLen = (uint8_t)(ARM_LINK_MIN_FRAME + payloadLen);
      if (f4RxLen == expectedLen) {
        bool ok = parseArmLinkFrame(f4RxBuffer, f4RxLen, out);
        f4RxLen = 0;
        if (ok) {
          return true;
        }
        logF4Diag("F4_RX_BAD_FRAME", payloadLen);
      }
    }
  }

  return false;
}


void handleF4Frame(const F4Frame &frame) {
  F4StageCommand cmd;

  if (frame.command == ARM_LINK_CMD_HELLO || frame.command == ARM_LINK_CMD_HEARTBEAT) {
    sendF4Ack(frame, 0);
    return;
  }

  if (frame.command == ARM_LINK_CMD_MOVE_TO_WEIGHT ||
      frame.command == ARM_LINK_CMD_MOVE_TO_LDC ||
      frame.command == ARM_LINK_CMD_SORT_RESULT) {
    if (!decodeF4StageCommand(frame, cmd)) {
      sendF4Nack(frame, 0, ARM_LINK_ERROR_PAYLOAD_LENGTH);
      return;
    }
    Serial.print("F4_RX_CMD,cmd=0x");
    Serial.print(frame.command, HEX);
    Serial.print(",stage=");
    Serial.print(cmd.stageId);
    Serial.print(",model=");
    Serial.print(cmd.modelResult);
    Serial.print(",bin=");
    Serial.print(cmd.targetBin);
    Serial.print('\n');

    if (workflowState == WF_WAIT_START_F4 || workflowState == WF_IDLE) {
      if (!f4IsMoveToWeightCommand(frame, cmd)) {
        sendTrace("F4_START_CMD_NEED_0x20_STAGE1");
        sendF4Nack(frame, cmd.cycleId, ARM_LINK_ERROR_STATE_NOT_ALLOWED, f4CommandDetail(cmd));
        return;
      }
      activeF4CycleId = cmd.cycleId;
      activeF4StageId = cmd.stageId;
      activeF4Command = frame.command;
      activeF4TargetBin = ARM_LINK_BIN_NONE;
      activeF4StartMs = millis();
      activeF4TimeoutMs = cmd.timeoutMs > 0 ? (uint32_t)cmd.timeoutMs : TARGET_TIMEOUT_MS;
      busy = true;
      requestPickRingFromMaix();
      sendF4Ack(frame, cmd.cycleId);
      sendTrace("F4_STAGE1_START_OK");
      sendStatus(true);
      return;
    }

    if (workflowState == WF_WAIT_BOX_F4) {
      if (!f4IsSortResultCommand(frame, cmd)) {
        sendTrace("F4_SECOND_CMD_NEED_0x22_STAGE3_BIN");
        sendF4Nack(frame, cmd.cycleId, ARM_LINK_ERROR_STATE_NOT_ALLOWED, f4CommandDetail(cmd));
        return;
      }
      if (activeF4CycleId != 0 && cmd.cycleId != activeF4CycleId) {
        sendTrace("F4_SECOND_CMD_CYCLE_MISMATCH");
        sendF4Nack(frame, cmd.cycleId, ARM_LINK_ERROR_CYCLE_MISMATCH, activeF4CycleId);
        return;
      }
      activeF4CycleId = cmd.cycleId;
      activeF4StageId = cmd.stageId;
      activeF4Command = frame.command;
      activeF4TargetBin = cmd.targetBin;
      activeF4StartMs = millis();
      activeF4TimeoutMs = cmd.timeoutMs > 0 ? (uint32_t)cmd.timeoutMs : TARGET_TIMEOUT_MS;
      workflowBoxScanBaseP = boxCenterForTargetBin(cmd.targetBin);
      workflowBoxScanEndP = boxScanEndForTargetBin(cmd.targetBin);
      workflowRegrabPoint = workflowTargetPoint;
      sendTrace("WF_SECOND_F4_REGRAB_DIRECT");
      setStatus(ST_BUSY_PLACE);
      sendF4Ack(frame, cmd.cycleId);
      startPoseAsync(workflowGreenVisionPoseWithClaw(CLAW_CLOSE_P), GRIP_MS, GRIP_HOLD_MS);
      workflowState = WF_FINAL_REGRAB_CLOSE;
      sendStatus(true);
      return;
    }

    if (workflowState != WF_WAIT_F4) {
      sendTrace("F4_CMD_BAD_STATE");
      sendF4Nack(frame, cmd.cycleId, busy ? ARM_LINK_ERROR_BUSY : ARM_LINK_ERROR_STATE_NOT_ALLOWED);
      return;
    }

    if (!f4IsMoveToLdcCommand(frame, cmd)) {
      sendTrace("F4_FIRST_CMD_NEED_0x21_STAGE2");
      sendF4Nack(frame, cmd.cycleId, ARM_LINK_ERROR_STATE_NOT_ALLOWED, f4CommandDetail(cmd));
      return;
    }

    activeF4CycleId = cmd.cycleId;
    activeF4StageId = cmd.stageId;
    activeF4Command = frame.command;
    activeF4TargetBin = ARM_LINK_BIN_NONE;
    activeF4StartMs = millis();
    activeF4TimeoutMs = cmd.timeoutMs > 0 ? (uint32_t)cmd.timeoutMs : TARGET_TIMEOUT_MS;
    sendTrace("F4_FIRST_CMD_OK");
    sendTrace("WF_F4_REGRAB_DIRECT");
    workflowRegrabPoint = {false, 0, 0, 0.0f};
    setStatus(ST_BUSY);
    workflowState = WF_REGRAB_ACK;
    startWorkflowWait(1);
    sendF4Ack(frame, cmd.cycleId);
    sendStatus(true);
    return;
  }

  if (frame.command == ARM_LINK_CMD_HOME) {
    if (!decodeF4StageCommand(frame, cmd)) {
      sendF4Nack(frame, 0, ARM_LINK_ERROR_PAYLOAD_LENGTH);
      return;
    }
    if (cmd.stageId != ARM_LINK_STAGE_HOME || cmd.targetBin != ARM_LINK_BIN_NONE) {
      sendTrace("F4_HOME_BAD_STAGE");
      sendF4Nack(frame, cmd.cycleId, ARM_LINK_ERROR_STATE_NOT_ALLOWED, f4CommandDetail(cmd));
      return;
    }
    pendingF4SafetyDone = true;
    pendingF4SafetyCycleId = cmd.cycleId;
    pendingF4SafetyStageId = ARM_LINK_STAGE_HOME;
    pendingF4SafetyStartMs = millis();
    sendF4Ack(frame, cmd.cycleId);
    busy = true;
    workflowState = WF_ABORT_HOLD;
    startWorkflowWait(1);
    return;
  }

  if (frame.command == ARM_LINK_CMD_STOP) {
    if (!decodeF4StageCommand(frame, cmd)) {
      sendF4Nack(frame, 0, ARM_LINK_ERROR_PAYLOAD_LENGTH);
      return;
    }
    if (cmd.stageId != ARM_LINK_STAGE_STOP_SAFE || cmd.targetBin != ARM_LINK_BIN_NONE) {
      sendTrace("F4_STOP_BAD_STAGE");
      sendF4Nack(frame, cmd.cycleId, ARM_LINK_ERROR_STATE_NOT_ALLOWED, f4CommandDetail(cmd));
      return;
    }
    pendingF4SafetyDone = true;
    pendingF4SafetyCycleId = cmd.cycleId;
    pendingF4SafetyStageId = ARM_LINK_STAGE_STOP_SAFE;
    pendingF4SafetyStartMs = millis();
    sendF4Ack(frame, cmd.cycleId);
    busy = true;
    workflowState = WF_ABORT_HOLD;
    startWorkflowWait(1);
    return;
  }

  sendF4Nack(frame, 0, ARM_LINK_ERROR_CMD_UNKNOWN);
}


void serviceF4Link() {
  F4Frame frame;
  if (takeF4Frame(frame)) {
    handleF4Frame(frame);
  }
}


uint8_t lobotChecksum(uint8_t buf[]) {
  uint16_t sum = 0;
  for (uint8_t i = 2; i < buf[3] + 2; i++) {
    sum += buf[i];
  }
  return (uint8_t)(~sum);
}


void busWriteMode() {
  digitalWrite(BUS_EN_PIN, BUS_WRITE_LEVEL);
  delayMicroseconds(10);
}


void busServoMove(uint8_t id, uint16_t position, uint16_t durationMs) {
  position = constrain(position, 0, 1000);

  uint8_t buf[10];
  buf[0] = LOBOT_SERVO_FRAME_HEADER;
  buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_MOVE_TIME_WRITE;
  buf[5] = (uint8_t)(position & 0xFF);
  buf[6] = (uint8_t)(position >> 8);
  buf[7] = (uint8_t)(durationMs & 0xFF);
  buf[8] = (uint8_t)(durationMs >> 8);
  buf[9] = lobotChecksum(buf);

  busWriteMode();
  BusSerial.write(buf, sizeof(buf));
  BusSerial.flush();
}


uint16_t scaledMotionMs(uint16_t durationMs) {
  uint32_t scaled = ((uint32_t)durationMs * MOTION_SPEED_SCALE_NUM + MOTION_SPEED_SCALE_DEN - 1) / MOTION_SPEED_SCALE_DEN;
  return (uint16_t)constrain(scaled, 1UL, 65535UL);
}


void moveToPose(const Pose &pose, uint16_t durationMs = MOVE_MS, uint16_t extraDelayMs = 80) {
  uint16_t moveMs = scaledMotionMs(durationMs);
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    busServoMove(POSE_TO_BUS_ID[i], pose.p[i], moveMs);
    delay(4);
  }
  delay((uint32_t)moveMs + extraDelayMs);
}


void startWorkflowWait(uint32_t waitMs) {
  workflowDeadlineMs = millis() + waitMs;
}


bool workflowStepDone() {
  return (int32_t)(millis() - workflowDeadlineMs) >= 0;
}


void startPoseAsync(const Pose &pose, uint16_t durationMs = MOVE_MS, uint16_t extraDelayMs = 80) {
  uint16_t moveMs = scaledMotionMs(durationMs);
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    busServoMove(POSE_TO_BUS_ID[i], pose.p[i], moveMs);
    delay(2);
  }
  startWorkflowWait((uint32_t)moveMs + extraDelayMs);
}


void initBusServo() {
  pinMode(BUS_EN_PIN, OUTPUT);
  digitalWrite(BUS_EN_PIN, BUS_WRITE_LEVEL);
  BusSerial.begin(BUS_BAUD, SERIAL_8N1, BUS_RX_PIN, BUS_TX_PIN);
  delay(100);
}


void initF4Uart() {
  F4Serial.begin(F4_UART_BAUD, SERIAL_8N1, F4_UART_RX_PIN, F4_UART_TX_PIN);
  f4RxLen = 0;
  delay(50);
}


void moveHomeIfEnabled(uint16_t durationMs) {
  if (!ENABLE_HOME_POSE) {
    return;
  }
  if (HOME_BASE_ONLY) {
    uint16_t moveMs = scaledMotionMs(durationMs);
    busServoMove(BASE_SERVO_ID, HOME_BASE_P, moveMs);
    delay((uint32_t)moveMs + 80);
  } else {
    moveToPose(HOME_POSE, durationMs);
  }
}


void startHomeAsync(uint16_t durationMs) {
  if (!ENABLE_HOME_POSE) {
    startWorkflowWait(1);
    return;
  }
  if (HOME_BASE_ONLY) {
    uint16_t moveMs = scaledMotionMs(durationMs);
    busServoMove(BASE_SERVO_ID, HOME_BASE_P, moveMs);
    startWorkflowWait((uint32_t)moveMs + 80);
  } else {
    startPoseAsync(HOME_POSE, durationMs);
  }
}


void bootClawTestIfEnabled() {
  if (!BOOT_CLAW_TEST) {
    return;
  }
  busServoMove(1, CLAW_CLOSE_P, 700);
  delay(900);
  busServoMove(1, CLAW_OPEN_P, 700);
  delay(900);
}


void bootBaseTestIfEnabled() {
  if (!BOOT_BASE_TEST) {
    return;
  }

  uint16_t baseA = PICK_BASE_P;
  uint16_t baseB = (uint16_t)constrain((int32_t)PICK_BASE_P + BOOT_BASE_TEST_OFFSET_P, 0, 1000);
  Serial.print("BOOT_BASE_TEST,id=");
  Serial.print(BASE_SERVO_ID);
  Serial.print(",a=");
  Serial.print(baseA);
  Serial.print(",b=");
  Serial.print(baseB);
  Serial.print('\n');

  busServoMove(BASE_SERVO_ID, baseA, 450);
  delay(650);
  busServoMove(BASE_SERVO_ID, baseB, BOOT_BASE_TEST_MS);
  delay((uint32_t)BOOT_BASE_TEST_MS + 350);
  busServoMove(BASE_SERVO_ID, baseA, BOOT_BASE_TEST_MS);
  delay((uint32_t)BOOT_BASE_TEST_MS + 350);
}


PickZone pickZoneFromImageX(int x) {
  if (x < RING_IMAGE_CENTER_X - RING_IMAGE_X_DEADBAND) {
    return PICK_ZONE_LEFT;
  }
  if (x > RING_IMAGE_CENTER_X + RING_IMAGE_X_DEADBAND) {
    return PICK_ZONE_RIGHT;
  }
  return PICK_ZONE_CENTER;
}


uint16_t pickBaseForZone(PickZone zone) {
  int32_t base = PICK_BASE_P;
  if (zone == PICK_ZONE_LEFT) {
    base += PICK_LEFT_BASE_OFFSET_P;
  } else if (zone == PICK_ZONE_RIGHT) {
    base += PICK_RIGHT_BASE_OFFSET_P;
  } else {
    base += PICK_CENTER_BASE_OFFSET_P;
  }
  return (uint16_t)constrain(base, 0, 1000);
}


int16_t visionOffsetFromX(const VisionPoint &point, int targetX, float gain, int16_t maxOffset) {
  if (!point.valid) {
    return 0;
  }
  int32_t offset = (int32_t)((point.x - targetX) * gain);
  return (int16_t)constrain(offset, -maxOffset, maxOffset);
}


int16_t visionOffsetFromY(const VisionPoint &point, int targetY, float gain, int16_t maxOffset) {
  if (!point.valid) {
    return 0;
  }
  int32_t offset = (int32_t)((point.y - targetY) * gain);
  return (int16_t)constrain(offset, -maxOffset, maxOffset);
}


uint16_t pickBaseForVision(PickZone zone, const VisionPoint &point) {
  (void)zone;
  (void)point;
  return PICK_BASE_P;
}


uint16_t targetBaseForVision(const VisionPoint &point) {
  int32_t base = (int32_t)TARGET_BASE_P +
    visionOffsetFromX(point, VISION_CENTER_X, TARGET_BASE_GAIN_P_PER_PX, TARGET_BASE_MAX_OFFSET_P);
  return (uint16_t)constrain(base, 0, 1000);
}


uint16_t greenTargetBaseForVision(const VisionPoint &point) {
  int32_t base = (int32_t)GREEN_TARGET_BASE_P +
    visionOffsetFromX(point, VISION_CENTER_X, TARGET_BASE_GAIN_P_PER_PX, TARGET_BASE_MAX_OFFSET_P);
  return (uint16_t)constrain(base, 0, 1000);
}


Pose poseWithBase(const Pose &pose, uint16_t base) {
  Pose adjusted = pose;
  adjusted.p[0] = base;
  return adjusted;
}


Pose poseWithBaseAndClaw(const Pose &pose, uint16_t base, uint16_t claw) {
  Pose adjusted = pose;
  adjusted.p[0] = base;
  adjusted.p[5] = claw;
  return adjusted;
}


Pose applyReachOffset(const Pose &pose, int16_t offset) {
  Pose adjusted = pose;
  adjusted.p[1] = (uint16_t)constrain((int32_t)adjusted.p[1] + offset, 0, 1000);
  adjusted.p[2] = (uint16_t)constrain((int32_t)adjusted.p[2] - offset, 0, 1000);
  return adjusted;
}


Pose workflowPickPose(const Pose &pose) {
  uint16_t base = pickBaseForVision(workflowPickZone, workflowRingPoint);
  Pose adjusted = poseWithBase(pose, base);
  int16_t reach = visionOffsetFromY(
    workflowRingPoint,
    PICK_GRIPPER_TARGET_Y,
    PICK_REACH_GAIN_P_PER_PX,
    PICK_REACH_MAX_OFFSET_P
  );
  if (workflowRingPoint.valid) {
    reach = (int16_t)constrain(
      (int32_t)reach + PICK_REACH_FORWARD_BIAS_P,
      -PICK_REACH_MAX_OFFSET_P,
      PICK_REACH_MAX_OFFSET_P
    );
  }
  return applyReachOffset(adjusted, reach);
}


Pose workflowPickAimPose(const Pose &pose) {
  uint16_t base = pickBaseForVision(workflowPickZone, workflowRingPoint);
  return poseWithBase(pose, base);
}


uint16_t workflowPickForwardBase() {
  int32_t base = (int32_t)pickBaseForVision(workflowPickZone, workflowRingPoint) + PICK_FORWARD_BASE_PUSH_P;
  return (uint16_t)constrain(base, 0, 1000);
}


Pose workflowPickForwardPose(const Pose &pose) {
  return poseWithBase(pose, workflowPickForwardBase());
}


Pose workflowPickCloseReachPose(uint16_t claw) {
  Pose adjusted = poseWithBase(PICK_POSE, pickBaseForVision(workflowPickZone, workflowRingPoint));
  adjusted.p[1] = (uint16_t)constrain((int32_t)adjusted.p[1] + PICK_CLOSE_ID5_FORWARD_P, 0, 1000);
  adjusted.p[2] = (uint16_t)constrain((int32_t)adjusted.p[2] + PICK_CLOSE_ID4_LIFT_P, 0, 1000);
  adjusted.p[5] = claw;
  return adjusted;
}


Pose workflowPickLiftBackPose() {
  Pose adjusted = workflowPickCloseReachPose(CLAW_CLOSE_P);
  adjusted.p[1] = (uint16_t)constrain((int32_t)adjusted.p[1] - PICK_RETRACT_ID5_BACK_P, 0, 1000);
  adjusted.p[2] = (uint16_t)constrain((int32_t)adjusted.p[2] - PICK_RETRACT_ID4_DOWN_P, 0, 1000);
  adjusted.p[3] = (uint16_t)constrain((int32_t)adjusted.p[3] - PICK_RETRACT_ID3_DOWN_P, 0, 1000);
  return adjusted;
}


Pose workflowPickHoldBackPose() {
  return workflowPickLiftBackPose();
}


Pose workflowPostRotateRetractPose() {
  Pose adjusted = workflowPickLiftBackPose();
  adjusted.p[0] = TARGET_BASE_P;
  adjusted.p[1] = POST_ROTATE_ID5_P;
  adjusted.p[2] = (uint16_t)constrain((int32_t)adjusted.p[2] - POST_ROTATE_ID4_DOWN_P, 0, 1000);
  return adjusted;
}


Pose workflowTargetPose(const Pose &pose) {
  uint16_t base = targetBaseForVision(workflowTargetPoint);
  Pose adjusted = poseWithBase(pose, base);
  int16_t reach = visionOffsetFromY(workflowTargetPoint, VISION_CENTER_Y, TARGET_REACH_GAIN_P_PER_PX, TARGET_REACH_MAX_OFFSET_P);
  return applyReachOffset(adjusted, reach);
}


Pose workflowGreenTargetPose(const Pose &pose) {
  uint16_t base = greenTargetBaseForVision(workflowTargetPoint);
  Pose adjusted = poseWithBase(pose, base);
  int16_t reach = visionOffsetFromY(workflowTargetPoint, VISION_CENTER_Y, TARGET_REACH_GAIN_P_PER_PX, TARGET_REACH_MAX_OFFSET_P);
  return applyReachOffset(adjusted, reach);
}


Pose workflowGreenVisionPose() {
  Pose adjusted = workflowGreenTargetPose(TARGET_POSE);
  adjusted.p[1] = GREEN_VISION_ID5_P;
  adjusted.p[3] = (uint16_t)constrain((int32_t)adjusted.p[3] - GREEN_VISION_ID3_DOWN_P, 0, 1000);
  adjusted.p[2] = (uint16_t)constrain((int32_t)adjusted.p[2] + GREEN_VISION_ID4_DOWN_P, 0, 1000);
  return adjusted;
}


Pose workflowGreenVisionPoseWithClaw(uint16_t claw) {
  Pose adjusted = workflowGreenVisionPose();
  adjusted.p[5] = claw;
  return adjusted;
}


Pose workflowTargetPoseKeepPostRotateArm(const Pose &pose) {
  Pose adjusted = poseWithBase(pose, targetBaseForVision(workflowTargetPoint));
  Pose postRotate = workflowPostRotateRetractPose();
  adjusted.p[1] = postRotate.p[1];
  adjusted.p[2] = postRotate.p[2];
  adjusted.p[3] = (uint16_t)constrain((int32_t)postRotate.p[3] + TARGET_KEEP_ID3_LIFT_P, 0, 1000);
  return adjusted;
}


Pose workflowF4RegrabVisionPose() {
  Pose adjusted = workflowTargetPoseKeepPostRotateArm(TARGET_OPEN_POSE);
  adjusted.p[3] = (uint16_t)constrain((int32_t)adjusted.p[3] - F4_REGRAB_VISION_ID3_DOWN_P, 0, 1000);
  adjusted.p[5] = CLAW_OPEN_P;
  return adjusted;
}


Pose workflowRegrabPose(const Pose &pose) {
  uint16_t base = targetBaseForVision(workflowRegrabPoint);
  Pose adjusted = poseWithBase(pose, base);
  int16_t reach = visionOffsetFromY(workflowRegrabPoint, VISION_CENTER_Y, TARGET_REACH_GAIN_P_PER_PX, TARGET_REACH_MAX_OFFSET_P);
  return applyReachOffset(adjusted, reach);
}


Pose workflowRegrabAboveOpenPose() {
  Pose adjusted = workflowRegrabPose(TARGET_WAIT_POSE);
  adjusted.p[5] = CLAW_OPEN_P;
  return adjusted;
}


Pose workflowStage2DirectRegrabPose(const Pose &pose) {
  return workflowTargetPoseKeepPostRotateArm(pose);
}


Pose workflowStage2DirectRegrabAboveOpenPose() {
  Pose adjusted = workflowStage2DirectRegrabPose(TARGET_WAIT_POSE);
  adjusted.p[5] = CLAW_OPEN_P;
  return adjusted;
}


uint16_t boxScanStartForTargetBin(uint8_t targetBin) {
  switch (targetBin) {
    case ARM_LINK_BIN_GOOD:
      return BOX_GOOD_SCAN_START_P;
    case ARM_LINK_BIN_BAD:
      return BOX_BAD_SCAN_START_P;
    case ARM_LINK_BIN_REVIEW:
      return BOX_REVIEW_SCAN_START_P;
    default:
      return BOX_SCAN_START_BASE_P;
  }
}


uint16_t boxScanEndForTargetBin(uint8_t targetBin) {
  switch (targetBin) {
    case ARM_LINK_BIN_GOOD:
      return BOX_GOOD_SCAN_END_P;
    case ARM_LINK_BIN_BAD:
      return BOX_BAD_SCAN_END_P;
    case ARM_LINK_BIN_REVIEW:
      return BOX_REVIEW_SCAN_END_P;
    default:
      return BOX_SCAN_END_BASE_P;
  }
}


bool boxScanBaseInTargetBin(uint8_t targetBin, uint16_t base) {
  return base >= boxScanStartForTargetBin(targetBin) &&
         base <= boxScanEndForTargetBin(targetBin);
}


uint16_t boxCenterForTargetBin(uint8_t targetBin) {
  uint16_t start = boxScanStartForTargetBin(targetBin);
  uint16_t end = boxScanEndForTargetBin(targetBin);
  return (uint16_t)(((uint32_t)start + end + 1) / 2);
}


uint16_t workflowBoxBaseForVision() {
  int32_t base = workflowBoxScanBaseP;
  if (workflowTargetPoint.valid) {
    base += visionOffsetFromX(
      workflowTargetPoint,
      VISION_CENTER_X,
      TARGET_BASE_GAIN_P_PER_PX,
      TARGET_BASE_MAX_OFFSET_P
    );
  }
  return (uint16_t)constrain(base, BOX_SCAN_START_BASE_P, BOX_SCAN_END_BASE_P);
}


Pose workflowBoxPose(const Pose &pose) {
  Pose adjusted = poseWithBase(pose, workflowBoxBaseForVision());
  if (workflowTargetPoint.valid) {
    int16_t reach = visionOffsetFromY(
      workflowTargetPoint,
      VISION_CENTER_Y,
      TARGET_REACH_GAIN_P_PER_PX,
      TARGET_REACH_MAX_OFFSET_P
    );
    return applyReachOffset(adjusted, reach);
  }
  return adjusted;
}


float pickCameraDistanceCmFromXY(const VisionPoint &point) {
  if (!point.valid) {
    return 0.0f;
  }
  float dx = (float)(point.x - VISION_CENTER_X);
  float dy = (float)(point.y - VISION_CENTER_Y);
  return (float)sqrt(dx * dx + dy * dy) / PICK_IMAGE_PX_PER_CM;
}


bool pickVisionCanGrip(const VisionPoint &point) {
  if (!point.valid) {
    return false;
  }
  int dx = point.x - PICK_GRIPPER_TARGET_X;
  int dy = point.y - PICK_GRIPPER_TARGET_Y;
  int bottom = point.hasSize ? point.y + point.h / 2 : point.y;
  return abs(dx) <= PICK_GRASP_X_TOL_PX &&
         abs(dy) <= PICK_GRASP_Y_TOL_PX &&
         bottom >= PICK_GRASP_BOTTOM_MIN_Y;
}


void sendPickVisionTrace(const char *tag, const VisionPoint &point) {
  Serial.print(tag);
  Serial.print(",x=");
  Serial.print(point.x);
  Serial.print(",y=");
  Serial.print(point.y);
  Serial.print(",dcm=");
  Serial.print(pickCameraDistanceCmFromXY(point), 1);
  Serial.print(",ok=");
  Serial.print(pickVisionCanGrip(point) ? 1 : 0);
  if (point.hasSize) {
    Serial.print(",w=");
    Serial.print(point.w);
    Serial.print(",h=");
    Serial.print(point.h);
    Serial.print(",bottom=");
    Serial.print(point.y + point.h / 2);
  }
  Serial.print('\n');
}


bool parseVisionFields(const String &line, int xStart, VisionPoint &point) {
  point.valid = false;
  point.x = 0;
  point.y = 0;
  point.score = 0.0f;
  point.w = 0;
  point.h = 0;
  point.cameraDistanceCm = 0.0f;
  point.hasSize = false;

  int yComma = line.indexOf(',', xStart);
  if (yComma < 0) {
    return false;
  }
  int scoreComma = line.indexOf(',', yComma + 1);
  if (scoreComma < 0) {
    return false;
  }

  point.x = line.substring(xStart, yComma).toInt();
  point.y = line.substring(yComma + 1, scoreComma).toInt();

  int hComma = line.indexOf(',', scoreComma + 1);
  if (hComma >= 0) {
    int finalComma = line.indexOf(',', hComma + 1);
    if (finalComma < 0) {
      return false;
    }
    point.w = line.substring(scoreComma + 1, hComma).toInt();
    point.h = line.substring(hComma + 1, finalComma).toInt();
    point.score = line.substring(finalComma + 1).toFloat();
    point.hasSize = point.w > 0 && point.h > 0;
  } else {
    point.score = line.substring(scoreComma + 1).toFloat();
  }
  point.valid = true;
  point.cameraDistanceCm = pickCameraDistanceCmFromXY(point);
  return true;
}


bool parseRingCommand(const String &line, String &ringType, PickZone &pickZone, VisionPoint &point) {
  if (!line.startsWith("RING,")) {
    return false;
  }

  int firstComma = line.indexOf(',');
  int secondComma = line.indexOf(',', firstComma + 1);
  if (secondComma < 0) {
    return false;
  }

  ringType = line.substring(firstComma + 1, secondComma);
  ringType.trim();
  if (!(ringType == "SMALL" || ringType == "BIG" || ringType == "BLACK")) {
    return false;
  }

  if (parseVisionFields(line, secondComma + 1, point)) {
    pickZone = pickZoneFromImageX(point.x);
  } else {
    point.valid = false;
    pickZone = PICK_ZONE_CENTER;
  }
  return true;
}


bool parseRegrabCommand(const String &line, String &ringType, VisionPoint &point) {
  if (!line.startsWith("REGRAB,")) {
    return false;
  }

  int firstComma = line.indexOf(',');
  int secondComma = line.indexOf(',', firstComma + 1);
  if (secondComma < 0) {
    return false;
  }

  ringType = line.substring(firstComma + 1, secondComma);
  ringType.trim();
  if (!(ringType == "SMALL" || ringType == "BIG" || ringType == "BLACK")) {
    return false;
  }

  return parseVisionFields(line, secondComma + 1, point);
}


bool parseTargetCommand(const String &line, VisionPoint &point) {
  if (!line.startsWith("TARGET,")) {
    return false;
  }
  return parseVisionFields(line, 7, point);
}


void startRingWorkflow(PickZone pickZone, const VisionPoint &ringPoint) {
  workflowPickZone = pickZone == PICK_ZONE_NONE ? PICK_ZONE_CENTER : pickZone;
  workflowRingPoint = ringPoint;
  workflowTargetPoint = {false, 0, 0, 0.0f};
  workflowRegrabPoint = {false, 0, 0, 0.0f};
  workflowPickTrackCount = 0;
  workflowPickGripOkCount = 0;
  workflowWaitStartMs = 0;
  busy = true;
  workflowState = WF_PICK_TRACK_ACK;
  setStatus(ST_RING_OK);
  startWorkflowWait(ACK_HOLD_MS);
}


void requestPickRingFromMaix() {
  workflowWaitStartMs = millis();
  workflowState = WF_WAIT_PICK_RING;
  setStatus(ST_NEED_RING);
}


void acceptPickRingUpdate(PickZone pickZone, const VisionPoint &ringPoint) {
  workflowPickZone = pickZone == PICK_ZONE_NONE ? PICK_ZONE_CENTER : pickZone;
  workflowRingPoint = ringPoint;
  workflowPickTrackCount++;
  sendPickVisionTrace("PICK_VISION", workflowRingPoint);
  workflowState = WF_PICK_TRACK_ACK;
  setStatus(ST_RING_OK);
  startWorkflowWait(ACK_HOLD_MS);
}


Pose workflowPickDirectClosePose() {
  return workflowPickCloseReachPose(CLAW_CLOSE_P);
}


Pose workflowPickDirectOpenPose() {
  return workflowPickCloseReachPose(CLAW_OPEN_P);
}


void startPickCloseFromLatestVision() {
  sendTrace("WF_PICK_FORWARD_DOWN");
  setStatus(ST_BUSY);
  startPoseAsync(workflowPickDirectOpenPose(), PICK_FORWARD_MS, 250);
  workflowState = WF_PICK_DOWN;
}


void acceptTargetWorkflow(const VisionPoint &targetPoint) {
  workflowTargetPoint = targetPoint;
  workflowState = WF_TARGET_ACK;
  setStatus(ST_TARGET_OK);
  startWorkflowWait(ACK_HOLD_MS);
}


void acceptGreenTargetWorkflow(const VisionPoint &targetPoint) {
  workflowTargetPoint = targetPoint;
  workflowState = WF_GREEN_TARGET_ACK;
  setStatus(ST_TARGET_OK);
  startWorkflowWait(ACK_HOLD_MS);
}


void acceptBoxTargetWorkflow(const VisionPoint &targetPoint) {
  if (!boxScanBaseInTargetBin(activeF4TargetBin, workflowBoxScanBaseP)) {
    sendTrace("WF_BOX_TARGET_IGNORED_BIN_WINDOW");
    setStatus(ST_NEED_TARGET);
    sendStatus(true);
    return;
  }
  workflowTargetPoint = targetPoint;
  workflowState = WF_BOX_TARGET_ACK;
  setStatus(ST_TARGET_OK);
  startWorkflowWait(ACK_HOLD_MS);
}


void requestRegrabFromMaix() {
  sendTrace("WF_F4_NEED_REGRAB_OBJECT");
  workflowWaitStartMs = millis();
  workflowState = WF_WAIT_REGRAB;
  setStatus(ST_NEED_REGRAB);
}


void startRegrabWorkflow(const VisionPoint &point) {
  workflowRegrabPoint = point;
  workflowState = WF_REGRAB_ACK;
  setStatus(ST_REGRAB_OK);
  startWorkflowWait(ACK_HOLD_MS);
}


void finishWorkflowDone() {
  workflowState = WF_DONE_HOLD;
  setStatus(ST_DONE);
  startWorkflowWait(DONE_HOLD_MS);
}


void enterWaitStartF4() {
  busy = false;
  workflowState = WF_WAIT_START_F4;
  activeF4CycleId = 0;
  activeF4StageId = ARM_LINK_STAGE_PICK_BELT_TO_WEIGHT;
  activeF4Command = 0;
  activeF4TargetBin = ARM_LINK_BIN_NONE;
  activeF4TimeoutMs = TARGET_TIMEOUT_MS;
  pendingF4SafetyDone = false;
  pendingF4SafetyCycleId = 0;
  pendingF4SafetyStageId = ARM_LINK_STAGE_NONE;
  workflowBoxScanBaseP = BOX_SCAN_START_BASE_P;
  workflowBoxScanEndP = BOX_SCAN_END_BASE_P;
  setStatus(ST_WAIT_F4);
}


void abortWorkflow(LinkStatus reason) {
  setStatus(reason);
  workflowState = WF_ABORT_HOLD;
  startWorkflowWait(DONE_HOLD_MS);
}


void advanceWorkflow() {
  if (workflowState == WF_IDLE) {
    return;
  }

  if (workflowState == WF_WAIT_START_F4) {
    return;
  }

  if (workflowState == WF_WAIT_TARGET) {
    if (millis() - workflowWaitStartMs > TARGET_TIMEOUT_MS) {
      abortWorkflow(ST_TARGET_TIMEOUT);
    }
    return;
  }

  if (workflowState == WF_WAIT_GREEN_TARGET) {
    uint32_t timeoutStartMs = workflowWaitStartMs;
    uint32_t timeoutMs = TARGET_TIMEOUT_MS;
    if (activeF4Command == ARM_LINK_CMD_MOVE_TO_LDC && activeF4TimeoutMs > 0) {
      timeoutStartMs = activeF4StartMs;
      timeoutMs = activeF4TimeoutMs;
    }
    if (millis() - timeoutStartMs > timeoutMs) {
      if (activeF4Command != 0 || activeF4StageId != ARM_LINK_STAGE_WEIGHT_TO_LDC) {
        sendF4StageDone(
          activeF4CycleId,
          activeF4StageId ? activeF4StageId : ARM_LINK_STAGE_WEIGHT_TO_LDC,
          ARM_LINK_ERROR_MOTION_TIMEOUT,
          ARM_LINK_ERROR_MOTION_TIMEOUT,
          0
        );
      }
      abortWorkflow(ST_TARGET_TIMEOUT);
    }
    return;
  }

  if (workflowState == WF_WAIT_PICK_RING) {
    uint32_t timeoutStartMs = workflowWaitStartMs;
    uint32_t timeoutMs = PICK_TRACK_TIMEOUT_MS;
    if (activeF4Command == ARM_LINK_CMD_MOVE_TO_WEIGHT && activeF4TimeoutMs > 0) {
      timeoutStartMs = activeF4StartMs;
      timeoutMs = activeF4TimeoutMs;
    }
    if (millis() - timeoutStartMs > timeoutMs) {
      sendTrace("WF_PICK_RING_TIMEOUT");
      if (activeF4Command == ARM_LINK_CMD_MOVE_TO_WEIGHT &&
          activeF4StageId == ARM_LINK_STAGE_PICK_BELT_TO_WEIGHT) {
        sendF4StageDone(
          activeF4CycleId,
          ARM_LINK_STAGE_PICK_BELT_TO_WEIGHT,
          ARM_LINK_ERROR_GRIP_FAILED,
          ARM_LINK_ERROR_GRIP_FAILED,
          0
        );
        activeF4Command = 0;
      }
      abortWorkflow(ST_PICK_UNREACHABLE);
    }
    return;
  }

  if (workflowState == WF_WAIT_F4) {
    return;
  }

  if (workflowState == WF_WAIT_BOX_F4) {
    if (millis() - workflowWaitStartMs > TARGET_TIMEOUT_MS) {
      sendTrace("F4_WAIT_BOX_CMD_TIMEOUT");
      abortWorkflow(ST_TARGET_TIMEOUT);
    }
    return;
  }

  if (workflowState == WF_BOX_SCAN) {
    if (millis() - activeF4StartMs > activeF4TimeoutMs) {
      sendF4StageDone(activeF4CycleId, activeF4StageId, ARM_LINK_ERROR_MOTION_TIMEOUT, ARM_LINK_ERROR_MOTION_TIMEOUT, 0);
      abortWorkflow(ST_TARGET_TIMEOUT);
      return;
    }
  }

  if (workflowState == WF_WAIT_REGRAB) {
    if (millis() - activeF4StartMs > activeF4TimeoutMs) {
      sendF4StageDone(activeF4CycleId, activeF4StageId, ARM_LINK_ERROR_MOTION_TIMEOUT, ARM_LINK_ERROR_MOTION_TIMEOUT, 0);
      abortWorkflow(ST_TARGET_TIMEOUT);
    }
    return;
  }

  if (!workflowStepDone()) {
    return;
  }

  switch (workflowState) {
    case WF_RING_ACK:
      sendTrace("WF_PICK_ABOVE");
      setStatus(ST_BUSY);
      startPoseAsync(workflowPickAimPose(ABOVE_PICK_POSE), PICK_APPROACH_MS, PICK_VISION_SETTLE_MS);
      workflowState = WF_PICK_ABOVE;
      break;

    case WF_PICK_ABOVE:
      sendTrace("WF_NEED_RING");
      requestPickRingFromMaix();
      break;

    case WF_PICK_TRACK_ACK:
      if (pickVisionCanGrip(workflowRingPoint)) {
        sendPickVisionTrace("PICK_DIRECT", workflowRingPoint);
        startPickCloseFromLatestVision();
      } else {
        sendPickVisionTrace("PICK_WAIT_WINDOW", workflowRingPoint);
        requestPickRingFromMaix();
      }
      break;

    case WF_PICK_RECENTER:
      sendTrace("WF_NEED_RING");
      requestPickRingFromMaix();
      break;

    case WF_PICK_DOWN:
      sendTrace("WF_PICK_SETTLE");
      startWorkflowWait(250);
      workflowState = WF_PICK_SETTLE;
      break;

    case WF_PICK_SETTLE:
      sendTrace("WF_PICK_CLOSE");
      startPoseAsync(workflowPickDirectClosePose(), GRIP_MS, GRIP_HOLD_MS);
      workflowState = WF_PICK_CLOSE;
      break;

    case WF_PICK_CLOSE:
      sendTrace("WF_PICK_LIFT_BACK");
      startPoseAsync(workflowPickLiftBackPose(), PICK_LIFT_MS, 350);
      workflowState = WF_PICK_LIFT;
      break;

    case WF_PICK_LIFT:
      sendTrace("WF_PICK_HOLD");
      startPoseAsync(workflowPickHoldBackPose(), 800);
      workflowState = WF_PICK_HOLD;
      break;

    case WF_PICK_HOLD:
      sendTrace("WF_ROTATE_TARGET_ID6_ONLY");
      {
        uint16_t rotateMs = scaledMotionMs(ROTATE_WITH_OBJECT_MS);
        busServoMove(BASE_SERVO_ID, TARGET_BASE_P, rotateMs);
        startWorkflowWait((uint32_t)rotateMs + ROTATE_SETTLE_MS);
      }
      workflowState = WF_ROTATE_TARGET;
      break;

    case WF_ROTATE_TARGET:
      sendTrace("WF_POST_ROTATE_RETRACT");
      setStatus(ST_BUSY_PLACE);
      startPoseAsync(workflowPostRotateRetractPose(), 800, 250);
      workflowState = WF_AFTER_ROTATE_RETRACT;
      break;

    case WF_AFTER_ROTATE_RETRACT:
      sendTrace("WF_TARGET_HEAD_DOWN_FOR_VISION");
      setStatus(ST_BUSY_PLACE);
      startPoseAsync(workflowTargetPoseKeepPostRotateArm(TARGET_POSE), 800, 300);
      workflowState = WF_TARGET_LOWER;
      break;

    case WF_TARGET_LOWER:
      sendTrace("WF_DIRECT_PLACE_WAIT_1S");
      workflowTargetPoint = {false, 0, 0, 0.0f};
      setStatus(ST_BUSY_PLACE);
      startWorkflowWait(TARGET_DIRECT_WAIT_MS);
      workflowState = WF_PLACE_DOWN;
      break;

    case WF_TARGET_ACK:
      sendTrace("WF_ALIGN_TARGET_POINT");
      setStatus(ST_BUSY_PLACE);
      startPoseAsync(workflowTargetPoseKeepPostRotateArm(TARGET_POSE), 800, 250);
      workflowState = WF_PLACE_DOWN;
      break;

    case WF_PLACE_ABOVE:
      sendTrace("WF_PLACE_DOWN");
      startPoseAsync(workflowTargetPoseKeepPostRotateArm(TARGET_POSE), 800);
      workflowState = WF_PLACE_DOWN;
      break;

    case WF_PLACE_DOWN:
      sendTrace("WF_RELEASE_ON_TARGET_POINT");
      startPoseAsync(workflowTargetPoseKeepPostRotateArm(TARGET_OPEN_POSE), GRIP_MS, 300);
      workflowState = WF_PLACE_OPEN;
      break;

    case WF_PLACE_OPEN: {
      sendTrace("WF_LIFT_SLIGHT_KEEP_CAMERA");
      Pose aboveOpen = workflowTargetPoseKeepPostRotateArm(TARGET_WAIT_POSE);
      aboveOpen.p[5] = CLAW_OPEN_P;
      startPoseAsync(aboveOpen, 800);
      workflowState = WF_PLACE_UP;
      break;
    }

    case WF_PLACE_UP: {
      sendTrace("WF_STAGE1_DONE_WAIT_F4_REGRAB");
      if (activeF4Command == ARM_LINK_CMD_MOVE_TO_WEIGHT &&
          activeF4StageId == ARM_LINK_STAGE_PICK_BELT_TO_WEIGHT) {
        uint32_t elapsed = millis() - activeF4StartMs;
        if (elapsed > 65535UL) {
          elapsed = 65535UL;
        }
        sendF4StageDone(
          activeF4CycleId,
          ARM_LINK_STAGE_PICK_BELT_TO_WEIGHT,
          ARM_LINK_RESULT_OK,
          0,
          (uint16_t)elapsed
        );
      }
      activeF4Command = 0;
      activeF4StageId = ARM_LINK_STAGE_WEIGHT_TO_LDC;
      activeF4TimeoutMs = TARGET_TIMEOUT_MS;
      setStatus(ST_WAIT_F4_REGRAB);
      workflowWaitStartMs = millis();
      workflowState = WF_WAIT_F4;
      break;
    }

    case WF_REGRAB_VISION_LOWER:
      requestRegrabFromMaix();
      break;

    case WF_REGRAB_ACK:
      sendTrace("WF_F4_REGRAB_OBJECT_ABOVE");
      setStatus(ST_BUSY);
      startPoseAsync(workflowStage2DirectRegrabAboveOpenPose(), 700);
      workflowState = WF_REGRAB_ABOVE;
      break;

    case WF_REGRAB_ABOVE:
      sendTrace("WF_F4_REGRAB_HEAD_DOWN");
      startPoseAsync(workflowStage2DirectRegrabPose(TARGET_OPEN_POSE), 800);
      workflowState = WF_REGRAB_DOWN;
      break;

    case WF_REGRAB_DOWN:
      sendTrace("WF_REGRAB_SETTLE");
      startWorkflowWait(250);
      workflowState = WF_REGRAB_SETTLE;
      break;

    case WF_REGRAB_SETTLE:
      sendTrace("WF_REGRAB_CLOSE");
      startPoseAsync(workflowStage2DirectRegrabPose(TARGET_CLOSE_POSE), GRIP_MS, GRIP_HOLD_MS);
      workflowState = WF_REGRAB_CLOSE;
      break;

    case WF_REGRAB_CLOSE:
      sendTrace("WF_REGRAB_LIFT");
      startPoseAsync(workflowRegrabPose(ROTATE_TARGET_POSE), PICK_LIFT_MS, 350);
      workflowState = WF_REGRAB_LIFT;
      break;

    case WF_REGRAB_LIFT:
      sendTrace("WF_REGRAB_HOLD");
      startPoseAsync(workflowRegrabPose(ROTATE_TARGET_POSE), 500);
      workflowState = WF_REGRAB_HOLD;
      break;

    case WF_REGRAB_HOLD:
      sendTrace("WF_ROTATE_GREEN_TARGET");
      setStatus(ST_BUSY_PLACE);
      startPoseAsync(GREEN_ROTATE_POSE, ROTATE_WITH_OBJECT_MS, ROTATE_SETTLE_MS);
      workflowState = WF_GREEN_ROTATE;
      break;

    case WF_GREEN_ROTATE:
      sendTrace("WF_GREEN_HEAD_DOWN_DIRECT_PLACE");
      startPoseAsync(workflowGreenVisionPose(), 800, 300);
      workflowState = WF_GREEN_LOWER;
      break;

    case WF_GREEN_LOWER:
      sendTrace("WF_GREEN_DIRECT_PLACE");
      workflowTargetPoint = {false, 0, 0, 0.0f};
      setStatus(ST_BUSY_PLACE);
      startPoseAsync(workflowGreenVisionPoseWithClaw(CLAW_OPEN_P), GRIP_MS, 300);
      workflowState = WF_GREEN_PLACE_OPEN;
      break;

    case WF_GREEN_TARGET_ACK:
      sendTrace("WF_ALIGN_GREEN_CENTER");
      setStatus(ST_BUSY_PLACE);
      startPoseAsync(workflowGreenTargetPose(TARGET_POSE), 800, 250);
      workflowState = WF_GREEN_PLACE_DOWN;
      break;

    case WF_GREEN_PLACE_DOWN:
      sendTrace("WF_RELEASE_ON_GREEN_CENTER");
      startPoseAsync(workflowGreenTargetPose(TARGET_OPEN_POSE), GRIP_MS, 300);
      workflowState = WF_GREEN_PLACE_OPEN;
      break;

    case WF_GREEN_PLACE_OPEN: {
      sendTrace("WF_WAIT_F4_SECOND");
      uint32_t elapsed = millis() - activeF4StartMs;
      if (elapsed > 65535UL) {
        elapsed = 65535UL;
      }
      sendF4StageDone(
        activeF4CycleId,
        activeF4StageId ? activeF4StageId : ARM_LINK_STAGE_WEIGHT_TO_LDC,
        ARM_LINK_RESULT_OK,
        0,
        (uint16_t)elapsed
      );
      activeF4Command = 0;
      activeF4StageId = ARM_LINK_STAGE_LDC_TO_SORT_BIN;
      activeF4TimeoutMs = TARGET_TIMEOUT_MS;
      setStatus(ST_WAIT_F4_BOX);
      workflowWaitStartMs = millis();
      workflowState = WF_WAIT_BOX_F4;
      break;
    }

    case WF_FINAL_REGRAB_DOWN:
      sendTrace("WF_SECOND_F4_REGRAB_SETTLE");
      startWorkflowWait(250);
      workflowState = WF_FINAL_REGRAB_SETTLE;
      break;

    case WF_FINAL_REGRAB_SETTLE:
      sendTrace("WF_SECOND_F4_REGRAB_CLOSE");
      startPoseAsync(workflowGreenTargetPose(TARGET_CLOSE_POSE), GRIP_MS, GRIP_HOLD_MS);
      workflowState = WF_FINAL_REGRAB_CLOSE;
      break;

    case WF_FINAL_REGRAB_CLOSE:
      sendTrace("WF_SECOND_F4_REGRAB_LIFT");
      startPoseAsync(workflowGreenTargetPose(ROTATE_TARGET_POSE), PICK_LIFT_MS, 350);
      workflowState = WF_FINAL_REGRAB_LIFT;
      break;

    case WF_FINAL_REGRAB_LIFT:
      sendTrace("WF_SECOND_F4_ROTATE");
      startPoseAsync(BOX_ROTATE_APPROACH_POSE, ROTATE_WITH_OBJECT_MS, ROTATE_SETTLE_MS);
      workflowState = WF_SECOND_F4_ROTATE;
      break;

    case WF_SECOND_F4_ROTATE:
      sendTrace("WF_BOX_ID5_300_BEFORE_PLACE");
      workflowTargetPoint = {false, 0, 0, 0.0f};
      setStatus(ST_BUSY_PLACE);
      startPoseAsync(BOX_PRE_PLACE_POSE, 700, 250);
      workflowState = WF_BOX_TARGET_ACK;
      break;

    case WF_BOX_SCAN: {
      if (workflowBoxScanBaseP >= workflowBoxScanEndP) {
        sendTrace("WF_BOX_SCAN_TIMEOUT");
        sendF4StageDone(
          activeF4CycleId,
          activeF4StageId ? activeF4StageId : ARM_LINK_STAGE_LDC_TO_SORT_BIN,
          ARM_LINK_ERROR_MOTION_TIMEOUT,
          ARM_LINK_ERROR_MOTION_TIMEOUT,
          0
        );
        abortWorkflow(ST_TARGET_TIMEOUT);
        break;
      }
      uint16_t nextBase = workflowBoxScanBaseP + BOX_SCAN_STEP_P;
      if (nextBase > workflowBoxScanEndP) {
        nextBase = workflowBoxScanEndP;
      }
      workflowBoxScanBaseP = nextBase;
      sendTrace("WF_BOX_SCAN_STEP");
      setStatus(ST_NEED_TARGET);
      startPoseAsync(workflowBoxPose(ROTATE_BOX_POSE), BOX_SCAN_STEP_MS, BOX_SCAN_SETTLE_MS);
      workflowState = WF_BOX_SCAN;
      break;
    }

    case WF_BOX_TARGET_ACK:
      sendTrace("WF_BOX_ABOVE");
      setStatus(ST_BUSY_PLACE);
      startPoseAsync(workflowBoxPose(BOX_PRE_PLACE_POSE), 800, 300);
      workflowState = WF_BOX_ABOVE;
      break;

    case WF_BOX_ABOVE:
      sendTrace("WF_BOX_DOWN");
      startPoseAsync(workflowBoxPose(BOX_PRE_PLACE_POSE), 800);
      workflowState = WF_BOX_DOWN;
      break;

    case WF_BOX_DOWN:
      sendTrace("WF_BOX_OPEN");
      {
        Pose boxOpen = workflowBoxPose(BOX_PRE_PLACE_POSE);
        boxOpen.p[5] = CLAW_OPEN_P;
        startPoseAsync(boxOpen, GRIP_MS, 300);
      }
      workflowState = WF_BOX_OPEN;
      break;

    case WF_BOX_OPEN: {
      sendTrace("WF_BOX_DONE_HOLD_POSE");
      startWorkflowWait(1);
      workflowState = WF_BOX_UP;
      break;
    }

    case WF_BOX_UP: {
      sendTrace("WF_HOME_DONE");
      uint32_t elapsed = millis() - activeF4StartMs;
      if (elapsed > 65535UL) {
        elapsed = 65535UL;
      }
      sendF4StageDone(
        activeF4CycleId,
        activeF4StageId ? activeF4StageId : ARM_LINK_STAGE_LDC_TO_SORT_BIN,
        ARM_LINK_RESULT_OK,
        0,
        (uint16_t)elapsed
      );
      startHomeAsync(1000);
      workflowState = WF_HOME_DONE;
      break;
    }

    case WF_HOME_DONE:
      finishWorkflowDone();
      break;

    case WF_DONE_HOLD:
      enterWaitStartF4();
      break;

    case WF_ABORT_HOLD:
      startHomeAsync(1000);
      workflowState = WF_ABORT_HOME;
      break;

    case WF_ABORT_HOME:
      if (pendingF4SafetyDone) {
        uint32_t elapsed = millis() - pendingF4SafetyStartMs;
        if (elapsed > 65535UL) {
          elapsed = 65535UL;
        }
        sendF4StageDone(
          pendingF4SafetyCycleId,
          pendingF4SafetyStageId,
          ARM_LINK_RESULT_OK,
          0,
          (uint16_t)elapsed
        );
      }
      enterWaitStartF4();
      break;

    default:
      enterWaitStartF4();
      break;
  }
}


void handleMaixCommand(const String &line) {
  if (line == "PING") {
    sendStatus(true);
    return;
  }

  digitalWrite(DEBUG_LED_PIN, !digitalRead(DEBUG_LED_PIN));
  setStatus(ST_RX_CMD);
  sendStatus(true);

  if (line == "HOME" || line == "RESET") {
    sendTrace("WF_FORCE_HOME");
    setStatus(ST_BUSY);
    sendStatus(true);
    busy = true;
    workflowState = WF_ABORT_HOLD;
    startWorkflowWait(1);
    return;
  }

  String ringType;
  PickZone pickZone = PICK_ZONE_CENTER;
  VisionPoint point = {false, 0, 0, 0.0f};

  if (parseRingCommand(line, ringType, pickZone, point)) {
    if (workflowState == WF_WAIT_PICK_RING) {
      acceptPickRingUpdate(pickZone, point);
      sendStatus(true);
      return;
    }
    if (workflowState == WF_WAIT_START_F4) {
      setStatus(ST_WAIT_F4);
      sendStatus(true);
      return;
    }
    if (busy || workflowState != WF_IDLE) {
      setStatus(ST_BUSY);
      sendStatus(true);
      return;
    }
    startRingWorkflow(pickZone, point);
    sendStatus(true);
    return;
  }

  if (parseRegrabCommand(line, ringType, point)) {
    if (workflowState == WF_WAIT_REGRAB) {
      startRegrabWorkflow(point);
      sendStatus(true);
    } else {
      setStatus(ST_BAD_CMD);
      sendStatus(true);
    }
    return;
  }

  if (parseTargetCommand(line, point)) {
    if (workflowState == WF_WAIT_GREEN_TARGET) {
      acceptGreenTargetWorkflow(point);
      sendStatus(true);
    } else if (workflowState == WF_BOX_SCAN) {
      acceptBoxTargetWorkflow(point);
      sendStatus(true);
    } else if (workflowState == WF_WAIT_TARGET) {
      acceptTargetWorkflow(point);
      sendStatus(true);
    } else {
      setStatus(ST_BAD_CMD);
      sendStatus(true);
    }
    return;
  }

  setStatus(ST_BAD_CMD);
  sendStatus(true);
}


void setup() {
  delay(800);

  pinMode(BLE_POWER_PIN, OUTPUT);
  digitalWrite(BLE_POWER_PIN, LOW);
  pinMode(DEBUG_LED_PIN, OUTPUT);
  digitalWrite(DEBUG_LED_PIN, LOW);

  initMaixUart();
  initBusServo();
  initF4Uart();

  Serial.print("VER:");
  Serial.print(APP_VERSION);
  Serial.print('\n');

  setStatus(ST_BOOT);
  sendStatus(true);

  bootClawTestIfEnabled();
  moveHomeIfEnabled(1200);
  bootBaseTestIfEnabled();

  enterWaitStartF4();
  sendStatus(true);
}


void loop() {
  serviceHeartbeat();
  serviceF4Link();

  String maixLine;
  if (fetchUartCommand(maixLine)) {
    handleMaixCommand(maixLine);
  }

  advanceWorkflow();
  sendStatus();
  delay(1);
}
