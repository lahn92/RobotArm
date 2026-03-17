#include <Arduino.h>
#include <ACAN_T4.h>

// --- CONFIGURATION ---
const float GEAR_RATIO = 1.0;
const float COORDS_PER_MOTOR_REV = 16384.0;
const uint16_t SPEED_RPM = 1500; 
const uint8_t ACCEL = 200;

// Function declarations
void parseCommand(String cmd);
void sendMKS_Absolute(int motor_id, float joint_angle_deg);
void sendMKS_Enable(int motor_id, int state);
void sendMKS_Zero(int motor_id);

void setup() {
  Serial.begin(2000000); 
  pinMode(13, OUTPUT); // Enable the onboard LED for debugging
  
  ACAN_T4_Settings settings(500000);
  uint32_t errorCode = ACAN_T4::can1.begin(settings);
  
  if (errorCode != 0) {
    Serial.print("CAN Error: ");
    Serial.println(errorCode);
  }
}

void loop() {
  // 1. Listen for Python commands
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    parseCommand(data);
  }

  // 2. CRITICAL FIX: Drain the CAN RX Buffer
  // We must read the MKS replies to prevent the CAN mailbox from overflowing and freezing.
  CANMessage rxMsg;
  while (ACAN_T4::can1.receive(rxMsg)) {
    // We just discard the replies to keep the bus clean and fast
  }
}

void parseCommand(String cmd) {
  int firstComma = cmd.indexOf(',');
  int secondComma = cmd.indexOf(',', firstComma + 1);
  
  if (firstComma > 0) {
    // Flash the LED so we know Python successfully talked to the Teensy!
    digitalWrite(13, HIGH); 
    
    char type = cmd.charAt(0);
    
    // For commands with 2 parameters (like E,1,1 or J,1,45.5)
    if (secondComma > 0) {
      int id_or_pin = cmd.substring(firstComma + 1, secondComma).toInt();
      float value = cmd.substring(secondComma + 1).toFloat();
      
      if (type == 'J') sendMKS_Absolute(id_or_pin, value);
      else if (type == 'G') digitalWrite(id_or_pin, (int)value);
      else if (type == 'E') sendMKS_Enable(id_or_pin, (int)value);
    } 
    // For commands with 1 parameter (like Z,1)
    else {
      int id = cmd.substring(firstComma + 1).toInt();
      if (type == 'Z') sendMKS_Zero(id);
    }
    
    // Turn LED off
    digitalWrite(13, LOW); 
  }
}

void sendMKS_Absolute(int motor_id, float joint_angle_deg) {
  float motor_angle_deg = joint_angle_deg * GEAR_RATIO;
  int32_t target_coords = (int32_t)((motor_angle_deg / 360.0) * COORDS_PER_MOTOR_REV);
  
  CANMessage msg;
  msg.id = motor_id;
  msg.ext = false; 
  msg.len = 8;
  
  msg.data[0] = 0xF5; 
  msg.data[1] = (SPEED_RPM >> 8) & 0xFF;
  msg.data[2] = SPEED_RPM & 0xFF;
  msg.data[3] = ACCEL;
  msg.data[4] = (target_coords >> 16) & 0xFF;
  msg.data[5] = (target_coords >> 8) & 0xFF;
  msg.data[6] = target_coords & 0xFF;
  
  uint32_t sum = motor_id;
  for(int i = 0; i < 7; i++) {
    sum += msg.data[i];
  }
  msg.data[7] = sum & 0xFF;
  
  ACAN_T4::can1.tryToSend(msg);
}

void sendMKS_Enable(int motor_id, int state) {
  CANMessage msg;
  msg.id = motor_id;
  msg.ext = false;
  msg.len = 3;
  msg.data[0] = 0xF3; 
  msg.data[1] = state ? 0x01 : 0x00; 
  msg.data[2] = (motor_id + msg.data[0] + msg.data[1]) & 0xFF; 
  ACAN_T4::can1.tryToSend(msg);
}

void sendMKS_Zero(int motor_id) {
  CANMessage msg;
  msg.id = motor_id;
  msg.ext = false;
  msg.len = 2;
  msg.data[0] = 0x92; 
  msg.data[1] = (motor_id + 0x92) & 0xFF; 
  ACAN_T4::can1.tryToSend(msg);
}