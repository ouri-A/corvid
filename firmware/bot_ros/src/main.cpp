#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include "secrets.h"

#define BOT_ID 0                 // the only line that differs between bots

const int IN1 = 26, IN2 = 27;    // left:  PWM on IN1 = forward
const int IN3 = 32, IN4 = 33;    // right: PWM on IN3 = forward
const float V_MAX = 0.7;         // m/s at full PWM, a GUESS until measured
const float HALF_TRACK = 0.07;   // m, half the wheel spacing, a GUESS
const uint32_t WATCHDOG_MS = 500;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_subscription_t sub;
rclc_executor_t executor;
geometry_msgs__msg__Twist msg;
char ns[8];
uint32_t last_cmd_ms = 0;

enum State { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state;

#define EVERY_MS(MS, X) do { static uint32_t t = 0; if (millis() - t > (MS)) { X; t = millis(); } } while (0)

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

void on_cmd_vel(const void *msgin) {
  const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;
  float v = m->linear.x, w = m->angular.z;
  tgt_l = (v - w * HALF_TRACK) / V_MAX;
  tgt_r = (v + w * HALF_TRACK) / V_MAX;
  last_cmd_ms = millis();
}

bool create_entities() {
  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "bot", ns, &support) != RCL_RET_OK) return false;
  if (rclc_subscription_init_best_effort(&sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel") != RCL_RET_OK) return false;
  executor = rclc_executor_get_zero_initialized_executor();
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &sub, &msg, &on_cmd_vel, ON_NEW_DATA) != RCL_RET_OK) return false;
  return true;
}

void destroy_entities() {
  rmw_context_t *ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(ctx, 0);
  rcl_subscription_fini(&sub, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  Serial.begin(115200);
  for (int p : {IN1, IN2, IN3, IN4}) { pinMode(p, OUTPUT); analogWrite(p, 0); }
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
      if (state == AGENT_CONNECTED) rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }
  if (state != before) Serial.printf("state %d -> %d\n", before, state);
  if (state != AGENT_CONNECTED || millis() - last_cmd_ms > WATCHDOG_MS) { tgt_l = 0; tgt_r = 0; }
  EVERY_MS(10, update_wheels());
}
