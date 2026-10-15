#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/u_int8.h>
#include "secrets.h"

#define BOT_ID 0                 // the only line that differs between bots

const int IN1 = 26, IN2 = 27;    // left:  PWM on IN1 = forward
const int IN3 = 32, IN4 = 33;    // right: PWM on IN3 = forward
const float V_MAX = 0.7;         // m/s at full PWM, a GUESS until measured
const float HALF_TRACK = 0.07;   // m, half the wheel spacing, a GUESS
const uint32_t WATCHDOG_MS = 500;

const uint8_t MPU = 0x68;
const float TURN_PWM = 0.7;              // top spin speed for turns, a GUESS
const float TURN_PWM_MIN = 0.45;         // slowest spin near the target; below stall the turn stops short, a GUESS
const float K_TURN = 1.5;                // spin speed per rad still to go, a GUESS
const float TURN_SCALE = 0.87;           // commanded angle x this; 15 Oct video: ~13 deg over per 90 deg turn
const float K_HEADING = 1.0;             // drive correction per rad of heading error, a GUESS
const uint32_t TURN_TIMEOUT_MS = 5000;   // give up on a turn that never finishes

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_subscription_t sub_cmd_vel, sub_turn, sub_drive;
rcl_publisher_t pub_state;
rclc_executor_t executor;
geometry_msgs__msg__Twist cmd_vel_msg;
std_msgs__msg__Float32 turn_msg, drive_msg;
std_msgs__msg__UInt8 state_msg;
char ns[8];
uint32_t last_cmd_ms = 0;

enum State { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state;

// what the bot is doing; this is what /botN/state publishes
enum Mode : uint8_t { IDLE = 0, TURNING = 1, DRIVING = 2 };
Mode mode = IDLE;
bool manual = false;             // DRIVING because of cmd_vel (teleop), not drive
float turn_goal = 0, turn_dir = 0, drive_ref = 0, drive_speed = 0;
uint32_t turn_start_ms = 0, last_drive_ms = 0;

float gz_bias = 0;               // deg/s, measured at boot
float heading = 0;               // rad since boot, counter-clockwise positive
uint32_t last_imu_us = 0;

#define EVERY_MS(MS, X) do { static uint32_t t = 0; if (millis() - t > (MS)) { X; t = millis(); } } while (0)

// ---------- motors ----------

void set_wheel(int fwd, int rev, float u) {
  u = constrain(u, -1.0f, 1.0f);
  int pwm = (int)(fabsf(u) * 255);
  if (u >= 0) { analogWrite(rev, 0); analogWrite(fwd, pwm); }
  else        { analogWrite(fwd, 0); analogWrite(rev, pwm); }
}

const float RAMP_STEP = 0.05;    // max PWM change per 10 ms tick: 0 to full in ~0.2 s
float tgt_l = 0, tgt_r = 0, cur_l = 0, cur_r = 0;

float ramp(float cur, float tgt) {
  if (fabsf(tgt) <= fabsf(cur) && cur * tgt >= 0) return tgt;  // slowing or stopping: at once
  if (cur * tgt < 0) cur = 0;                                   // reversing: through zero first
  return cur + constrain(tgt - cur, -RAMP_STEP, RAMP_STEP);     // speeding up: one step
}

void update_wheels() {
  cur_l = ramp(cur_l, tgt_l);  cur_r = ramp(cur_r, tgt_r);
  set_wheel(IN1, IN2, cur_l);  set_wheel(IN3, IN4, cur_r);
}

// ---------- gyro ----------

void mpu_write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}

bool read_gz(float &dps) {
  Wire.beginTransmission(MPU); Wire.write(0x47);              // GYRO_ZOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint16_t)MPU, (uint8_t)2) != 2) return false;
  uint8_t hi = Wire.read(), lo = Wire.read();
  dps = (int16_t)((hi << 8) | lo) / 65.5f;                    // +-500 deg/s range
  return true;
}

void imu_init() {
  Wire.begin(21, 22);
  Wire.setClock(400000);
  mpu_write(0x6B, 0);       // wake up
  mpu_write(0x1A, 3);       // low-pass filter ~44 Hz
  mpu_write(0x1B, 0x08);    // gyro range +-500 deg/s
  delay(100);
  float sum = 0, g; int n = 0;
  for (int i = 0; i < 200; i++) { if (read_gz(g)) { sum += g; n++; } delay(5); }   // ~1 s, bot must be still
  gz_bias = n ? sum / n : 0;
  Serial.printf("gyro bias %+.3f deg/s from %d samples\n", gz_bias, n);
  last_imu_us = micros();
}

void imu_update() {
  uint32_t now = micros();
  float dt = (now - last_imu_us) * 1e-6f;
  last_imu_us = now;
  float g;
  if (read_gz(g)) heading += (g - gz_bias) * DEG_TO_RAD * dt;
}

// ---------- behaviour ----------

void stop_all() { mode = IDLE; manual = false; tgt_l = 0; tgt_r = 0; }

void control() {
  uint32_t now = millis();
  if (state != AGENT_CONNECTED) { stop_all(); return; }
  if (mode == TURNING) {
    float err = turn_goal - heading;
    if (err * turn_dir <= 0 || now - turn_start_ms > TURN_TIMEOUT_MS) stop_all();   // reached, or gave up
    else {                                                                           // spin in place
      float s = constrain(K_TURN * fabsf(err), TURN_PWM_MIN, TURN_PWM);              // slow down near target
      tgt_l = -turn_dir * s;  tgt_r = turn_dir * s;
    }
  } else if (mode == DRIVING && manual) {
    if (now - last_cmd_ms > WATCHDOG_MS) stop_all();      // wheels already set by on_cmd_vel
  } else if (mode == DRIVING) {
    if (now - last_drive_ms > WATCHDOG_MS) stop_all();
    else {
      float base = drive_speed / V_MAX;
      float corr = K_HEADING * (drive_ref - heading);     // drifted left -> corr < 0 -> steer right
      tgt_l = base - corr;  tgt_r = base + corr;
    }
  }
}

void on_cmd_vel(const void *msgin) {
  const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;
  float v = m->linear.x, w = m->angular.z;
  mode = DRIVING; manual = true;                          // teleop overrides any turn or drive
  tgt_l = (v - w * HALF_TRACK) / V_MAX;
  tgt_r = (v + w * HALF_TRACK) / V_MAX;
  last_cmd_ms = millis();
}

void on_turn(const void *msgin) {
  float a = ((const std_msgs__msg__Float32 *)msgin)->data;   // rad, relative, CCW positive
  if (a == 0) return;
  manual = false;
  turn_goal = heading + a * TURN_SCALE;
  turn_dir = (a > 0) ? 1 : -1;
  turn_start_ms = millis();
  mode = TURNING;                                         // a new turn replaces an old one
}

void on_drive(const void *msgin) {
  if (mode == TURNING) return;                            // ignored until the turn finishes
  if (mode != DRIVING || manual) drive_ref = heading;     // fresh heading for a new drive
  manual = false;
  mode = DRIVING;
  drive_speed = ((const std_msgs__msg__Float32 *)msgin)->data;   // m/s, negative = reverse
  last_drive_ms = millis();
}

// ---------- micro-ROS ----------

bool create_entities() {
  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "bot", ns, &support) != RCL_RET_OK) return false;
  if (rclc_subscription_init_best_effort(&sub_cmd_vel, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel") != RCL_RET_OK) return false;
  if (rclc_subscription_init_default(&sub_turn, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "turn") != RCL_RET_OK) return false;
  if (rclc_subscription_init_best_effort(&sub_drive, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "drive") != RCL_RET_OK) return false;
  if (rclc_publisher_init_best_effort(&pub_state, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "state") != RCL_RET_OK) return false;
  executor = rclc_executor_get_zero_initialized_executor();
  if (rclc_executor_init(&executor, &support.context, 3, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &sub_cmd_vel, &cmd_vel_msg, &on_cmd_vel, ON_NEW_DATA) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &sub_turn, &turn_msg, &on_turn, ON_NEW_DATA) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &sub_drive, &drive_msg, &on_drive, ON_NEW_DATA) != RCL_RET_OK) return false;
  return true;
}

void destroy_entities() {
  rmw_context_t *ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(ctx, 0);
  rcl_publisher_fini(&pub_state, &node);
  rcl_subscription_fini(&sub_cmd_vel, &node);
  rcl_subscription_fini(&sub_turn, &node);
  rcl_subscription_fini(&sub_drive, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  Serial.begin(115200);
  for (int p : {IN1, IN2, IN3, IN4}) { pinMode(p, OUTPUT); analogWrite(p, 0); }
  imu_init();               // before WiFi, while the bot is still on the floor
  snprintf(ns, sizeof ns, "bot%d", BOT_ID);
  char ssid[] = WIFI_SSID, pass[] = WIFI_PASS;
  Serial.println("joining wifi...");
  set_microros_wifi_transports(ssid, pass, IPAddress(AGENT_IP), AGENT_PORT);
  WiFi.setSleep(false);   // modem sleep delays incoming packets past our 100 ms ping timeout
  Serial.printf("wifi up, ip %s\n", WiFi.localIP().toString().c_str());
  state = WAITING_AGENT;
}

void loop() {
  State before = state;
  switch (state) {
    case WAITING_AGENT:
      EVERY_MS(500, state = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) ? AGENT_AVAILABLE : WAITING_AGENT);
      break;
    case AGENT_AVAILABLE:
      state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) destroy_entities();
      break;
    case AGENT_CONNECTED:
      EVERY_MS(1000, state = (rmw_uros_ping_agent(100, 3) == RMW_RET_OK) ? AGENT_CONNECTED : AGENT_DISCONNECTED);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        EVERY_MS(100, { state_msg.data = mode; (void)rcl_publish(&pub_state, &state_msg, NULL); });
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }
  if (state != before) Serial.printf("state %d -> %d\n", before, state);
  imu_update();
  EVERY_MS(10, { control(); update_wheels(); });
}
