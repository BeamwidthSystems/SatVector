/*
========================================================
SATVECTOR ROTATOR CONTROLLER
Manual Jog + Target Tracking + Zero/Tare
========================================================
*/

#include <ArduinoBLE.h>

/*
--------------------------------------------------------
Stepper Driver Pins
--------------------------------------------------------
*/

#define AZ_STEP_PIN 9
#define AZ_DIR_PIN 8

#define EL_STEP_PIN 7
#define EL_DIR_PIN 6

/*
Set steps per degree. Default value is 480.0
172,800 steps per revolution
*/

const float STEPS_PER_DEG = 480.0;

/*
--------------------------------------------------------
BLE UUIDs
These must match the webpage
--------------------------------------------------------
*/

#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEService service(SERVICE_UUID);

BLECharacteristic rxChar(
  RX_UUID,
  BLEWrite | BLEWriteWithoutResponse,
  20
);

BLECharacteristic txChar(
  TX_UUID,
  BLENotify,
  20
);

/*
--------------------------------------------------------
Position state variables
--------------------------------------------------------
*/

volatile long azSteps = 0;
volatile long elSteps = 0;

long targetAzSteps = 0;
long targetElSteps = 0;

int i = 0;

/*
--------------------------------------------------------
Command identifiers
--------------------------------------------------------
*/

#define CMD_SET_TARGET 1
#define CMD_JOG        2
#define CMD_ZERO       3

#define AXIS_AZ 1
#define AXIS_EL 2

#define DIR_POS 1
#define DIR_NEG 2

/*
--------------------------------------------------------
Status timing
--------------------------------------------------------
*/

unsigned long lastStatus = 0;

/*
========================================================
SETUP
========================================================
*/

void setup()
{
  Serial.begin(115200);

  pinMode(AZ_STEP_PIN,OUTPUT);
  pinMode(AZ_DIR_PIN,OUTPUT);

  pinMode(EL_STEP_PIN,OUTPUT);
  pinMode(EL_DIR_PIN,OUTPUT);

  if(!BLE.begin())
  {
    while(1);
  }

  BLE.setLocalName("AZEL-Rotator");

  BLE.setAdvertisedService(service);

  service.addCharacteristic(rxChar);
  service.addCharacteristic(txChar);

  BLE.addService(service);

  rxChar.setEventHandler(BLEWritten,onCommand);

  BLE.advertise();
}

/*
========================================================
MAIN LOOP
========================================================
*/

void loop()
{
  BLE.poll();

  updateAxis(
    AZ_DIR_PIN,
    AZ_STEP_PIN,
    &azSteps,
    targetAzSteps
  );

  updateAxis(
    EL_DIR_PIN,
    EL_STEP_PIN,
    &elSteps,
    targetElSteps
  );

  if(millis() - lastStatus > 200)
  {
    sendStatus();
    lastStatus = millis();
  }
}

/*
========================================================
BLE COMMAND HANDLER
========================================================
*/

void onCommand(BLEDevice central, BLECharacteristic characteristic)
{
  const uint8_t *d = characteristic.value();

  uint8_t cmd = d[0];

  if(cmd == CMD_SET_TARGET)
  {
    uint16_t az10 = (d[1] << 8) | d[2];
    uint16_t el10 = (d[3] << 8) | d[4];

    float azDeg = az10 / 10.0;
    float elDeg = el10 / 10.0;

    targetAzSteps = azDeg * STEPS_PER_DEG;
    targetElSteps = elDeg * STEPS_PER_DEG;
  }

  else if(cmd == CMD_ZERO)
  {
    azSteps = 0;
    elSteps = 0;

    targetAzSteps = 0;
    targetElSteps = 0;
  }

  else if(cmd == CMD_JOG)
  {
    uint8_t axis = d[1];
    uint8_t dir  = d[2];

    jogAxis(axis,dir);
  }
}

/*
========================================================
MOTION CONTROLLER
========================================================
*/

void updateAxis(
  int dirPin,
  int stepPin,
  volatile long *pos,
  long target
)
{
  long error = target - *pos;

  // Handle 360 wrap for AZ axis
  if (dirPin == AZ_DIR_PIN) {

    if (error > 86400)
        error -= 172800;

    if (error < -86400)
        error += 172800;
  }

  if (error == 0)
    return;

  bool dir = error > 0;

  digitalWrite(dirPin, dir);

  digitalWrite(stepPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(stepPin, LOW);

  delayMicroseconds(500);

  if (dir)
    (*pos)++;
  else
    (*pos)--;
}
/*
========================================================
MANUAL JOG (0.5 Degree Increments)
========================================================
*/

void jogAxis(uint8_t axis, uint8_t dir)
{
  // Define how many steps = 0.5 degrees
  const long JOG_STEPS = 240; 
  
  bool direction = (dir == DIR_POS);

  if(axis == AXIS_AZ)
  {
    // Update the target so the main loop moves the motors
    if(direction) targetAzSteps += JOG_STEPS;
    else targetAzSteps -= JOG_STEPS;
    
    // Handle Azimuth 360 wrap logic for the new target
    if (targetAzSteps >= 172800) targetAzSteps -= 172800;
    if (targetAzSteps < 0) targetAzSteps += 172800;
  }
  else
  {
    // Update Elevation target
    if(direction) targetElSteps += JOG_STEPS;
    else targetElSteps -= JOG_STEPS;
    
    // Set maximum negative elevation value to -20deg and positive value to 90deg
    // 90 degrees * 480 steps = 43200
    if (targetElSteps > 43200) targetElSteps = 43200;
    if (targetElSteps < -9600) targetElSteps = -9600;
  }
  
  Serial.print("Manual Jog Initiated. New Target Az: ");
  Serial.println(targetAzSteps);
}

/*
========================================================
SEND STATUS TO SATVECTOR
========================================================
*/

void sendStatus()
{
  long normalizedAz = azSteps;
  while (normalizedAz < 0) normalizedAz += 172800;
  while (normalizedAz >= 172800) normalizedAz -= 172800;

  float azDeg = (float)normalizedAz / STEPS_PER_DEG;
  
  float elDeg = (float)elSteps / STEPS_PER_DEG;

  int16_t az10 = (int16_t)(azDeg * 10.0);
  int16_t el10 = (int16_t)(elDeg * 10.0);

  uint8_t pkt[5];
  pkt[0] = 0x10; // Status Identifier

  // Split into High/Low bytes
  pkt[1] = (az10 >> 8) & 0xFF;
  pkt[2] = az10 & 0xFF;
  pkt[3] = (el10 >> 8) & 0xFF;
  pkt[4] = el10 & 0xFF;

  txChar.writeValue(pkt, 5);
}