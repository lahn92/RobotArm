#include <Arduino.h>
#include <FlexCAN_T4.h>

// Use CAN2
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can0;

// --- CONFIGURATION ---
const float GEAR_RATIO = 25.0;
const float COORDS_PER_MOTOR_REV = 16384.0;
const uint8_t ACCEL = 200;

// --- GPIO CONFIGURATION ---
const int GPIO_OUTPUT_PINS[] = {2, 3, 4, 5, 7, 8, 9, 10, 11, 12};
const int NUM_GPIO_OUTPUTS = 10;

const int GPIO_INPUT_PINS[] = {14, 15, 16, 17, 18, 19, 20, 21, 24, 25};
const int NUM_GPIO_INPUTS = 10;
int lastInputStates[10] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1};

// --- FUNCTION DECLARATIONS ---
void parseCommand(String cmd);
void sendMKS_Absolute(int id, float joint_angle_deg, int speed_rpm);
void sendMKS_Enable(int id, int state);
void sendMKS_Zero(int id);

// --- CAN RECEIVE CALLBACK (Buffer Drain) ---
void canSniff(const CAN_message_t &msg)
{
  // Discard replies to keep bus clear
}

void setup()
{
  Serial.begin(115200);
  pinMode(13, OUTPUT);

  // WAKE UP THE TRANSCEIVER
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW);

  // INITIALIZE GPIO OUTPUTS
  for (int i = 0; i < NUM_GPIO_OUTPUTS; i++)
  {
    pinMode(GPIO_OUTPUT_PINS[i], OUTPUT);
    digitalWrite(GPIO_OUTPUT_PINS[i], LOW);
  }

  // INITIALIZE GPIO INPUTS
  for (int i = 0; i < NUM_GPIO_INPUTS; i++)
  {
    pinMode(GPIO_INPUT_PINS[i], INPUT_PULLUP);
  }

  // INITIALIZE CAN2
  Can0.begin();
  Can0.setBaudRate(500000);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.onReceive(canSniff);
}

void loop()
{
  // 1. Process CAN messages
  Can0.events();

  // 2. Listen for Python commands
  if (Serial.available())
  {
    String data = Serial.readStringUntil('\n');
    parseCommand(data);
  }

  // 3. Monitor physical inputs and report to Python
  for (int i = 0; i < NUM_GPIO_INPUTS; i++)
  {
    int currentState = digitalRead(GPIO_INPUT_PINS[i]);

    if (currentState != lastInputStates[i])
    {
      lastInputStates[i] = currentState;
      Serial.print("I,");
      Serial.print(GPIO_INPUT_PINS[i]);
      Serial.print(",");
      Serial.println(currentState);
    }
  }
}

// --- PYTHON COMMAND PARSER ---
void parseCommand(String cmd)
{
  int firstComma = cmd.indexOf(',');
  int secondComma = cmd.indexOf(',', firstComma + 1);
  int thirdComma = cmd.indexOf(',', secondComma + 1);

  if (firstComma > 0)
  {
    digitalWrite(13, HIGH);
    char type = cmd.charAt(0);

    if (secondComma > 0)
    {
      int id_or_pin = cmd.substring(firstComma + 1, secondComma).toInt();

      if (type == 'J')
      {
        float angle = cmd.substring(secondComma + 1, thirdComma > 0 ? thirdComma : cmd.length()).toFloat();
        int speed = thirdComma > 0 ? cmd.substring(thirdComma + 1).toInt() : 1500;
        sendMKS_Absolute(id_or_pin, angle, speed);
      }
      else if (type == 'G')
      {
        digitalWrite(id_or_pin, (int)cmd.substring(secondComma + 1).toFloat());
      }
      else if (type == 'E')
      {
        sendMKS_Enable(id_or_pin, (int)cmd.substring(secondComma + 1).toFloat());
      }
    }
    else
    {
      if (type == 'Z')
        sendMKS_Zero(cmd.substring(firstComma + 1).toInt());
    }
    digitalWrite(13, LOW);
  }
}

// --- MKS CAN TRANSLATORS ---
void sendMKS_Enable(int id, int state)
{
  CAN_message_t msg;
  msg.id = id;
  msg.flags.extended = 0;
  msg.len = 3;
  msg.buf[0] = 0xF3;
  msg.buf[1] = state ? 0x01 : 0x00;
  msg.buf[2] = (id + msg.buf[0] + msg.buf[1]) & 0xFF;
  Can0.write(msg);
}

void sendMKS_Zero(int id)
{
  CAN_message_t msg;
  msg.id = id;
  msg.flags.extended = 0;
  msg.len = 2;
  msg.buf[0] = 0x92;
  msg.buf[1] = (id + msg.buf[0]) & 0xFF;
  Can0.write(msg);
}

void sendMKS_Absolute(int id, float joint_angle_deg, int speed_rpm)
{
  float motor_angle_deg = joint_angle_deg * GEAR_RATIO;
  int32_t target_coords = (int32_t)((motor_angle_deg / 360.0) * COORDS_PER_MOTOR_REV);

  CAN_message_t msg;
  msg.id = id;
  msg.flags.extended = 0;
  msg.len = 8;

  msg.buf[0] = 0xF5;
  msg.buf[1] = (speed_rpm >> 8) & 0xFF;
  msg.buf[2] = speed_rpm & 0xFF;
  msg.buf[3] = ACCEL;
  msg.buf[4] = (target_coords >> 16) & 0xFF;
  msg.buf[5] = (target_coords >> 8) & 0xFF;
  msg.buf[6] = target_coords & 0xFF;

  uint32_t sum = id;
  for (int i = 0; i < 7; i++)
    sum += msg.buf[i];
  msg.buf[7] = sum & 0xFF;

  Can0.write(msg);
}