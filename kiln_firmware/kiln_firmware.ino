#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "Adafruit_MCP9601.h"
#include "TemperatureController.h"

// profile support
#define MAX_PROFILE_STEPS 20  // adjust based on your needs and available memory
struct ProfileStep {
  double targetTemperature;     // in °C
  unsigned long rampMinutes;    // minutes to ramp to this temp
  unsigned long dwellMinutes;   // minutes to hold this temp
};
ProfileStep profile[MAX_PROFILE_STEPS];
size_t profileStepCount = 0; // number of total steps in current profile
// Profile state variables
size_t currentStep = 0;
unsigned long stepStartTime = 0;
bool inDwellPhase = false;
bool profileComplete = false;
int pidInputSetting = 2; // combine top and bottom TC temps for current kiln temp 
double setpoint = 0;
double initialStepTemp = 0.0;


// lora and SD log interval
#define LOG_INTERVAL_MS 5000

// PID loop frequency
#define CONTROL_INTERVAL_MS 1000

// pins
#define RADIO_RESET 30
#define RELAY_1 29
#define RELAY_2 28
#define RELAY_3 27


#define PWM_PERIOD_SECONDS 9.0
#define PWM_PERIOD         9000

#define LORA_SERIAL Serial4  // Use hardware Serial1 for LoRa

Adafruit_MCP9601 tc1, tc2, tc3;
File logFile;

char rxBuf[100];

char logfilename[11] = "LOG000.csv";

uint32_t lastLogTime = 0;
float setpoints[3] = {0, 0, 0};

struct tc_status {
  uint8_t status;
  float hot;
  float cold;
};

struct tc_status tc1_stat; // todo: which is this measureing
struct tc_status tc2_stat;
struct tc_status tc3_stat;

// Create a controller with tuning values (adjust as needed)
TemperatureController tempControl;
unsigned long lastControlTime = 0;

void setup() {
  Serial.begin(115200);
  LORA_SERIAL.begin(115200);
  Wire.begin();

  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);
  pinMode(RELAY_3, OUTPUT);
  digitalWrite(RELAY_1, LOW);
  digitalWrite(RELAY_2, LOW);
  digitalWrite(RELAY_3, LOW);

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD Card initialization failed!");
  }

  Serial.println("Configuring radio");

  pinMode(RADIO_RESET, OUTPUT);
  digitalWrite(RADIO_RESET, LOW);
  delay(100);
  digitalWrite(RADIO_RESET, HIGH);
  delay(100);
  LORA_SERIAL.println("AT+ADDRESS=1");
  loraReadTimeout(1000);
  LORA_SERIAL.println("AT+NETWORKID=1");
  loraReadTimeout(1000);
  LORA_SERIAL.println("AT+PARAMETER=10,7,1,7");
  loraReadTimeout(1000);


  Serial.println("Configuring TCs");

  if ( init_mcp(tc1, 0x60, tc1_stat) < 0 ) {
    Serial.println("Error init TC1");
    //return;
  }
  if ( init_mcp(tc2, 0x61, tc2_stat) < 0 ) {
    Serial.println("Error init TC2");
    //return;
  }
  if ( init_mcp(tc3, 0x64, tc3_stat) < 0 ) {
    Serial.println("Error init TC3");
    //return;
  }

  int lognum = 0;
  Serial.println("Configuring filename...");
  while ( SD.exists(logfilename) && lognum < 999) {
    Serial.print("log "); Serial.print(lognum); Serial.println(" exists");
    char num[10];
    lognum++;
    sprintf(num, "%03d", lognum);
    memcpy(&logfilename[3], num, 3);
    Serial.println(logfilename);
    delay(1);
  }

}

void loop() {
  // check for any incoming command on radio
  loraRxCheckWait(100);

  uint32_t currentTime = millis();

  // do logging to SD card and radio
  if (currentTime - lastLogTime >= LOG_INTERVAL_MS) {
    lastLogTime = currentTime;
    logData();
    sendLoRaData();

  }

  // do PID control of active profile
  currentTime = millis();
  if (profileComplete || profileStepCount == 0) {
    // Profile is done or not loaded
    return;

  }
  // is it time for a PID update
  else if ( currentTime - lastControlTime > CONTROL_INTERVAL_MS) {
    lastControlTime = currentTime;
    unsigned long now = millis();
    unsigned long elapsedTime = now - stepStartTime;

    ProfileStep& step = profile[currentStep];
    unsigned long rampTimeMs = step.rampMinutes * 60UL * 1000UL;
    unsigned long dwellTimeMs = step.dwellMinutes * 60UL * 1000UL;


    if (!inDwellPhase) {
      // We're in the ramping phase
      if (elapsedTime < rampTimeMs) {
        setpoint = calculateSetpoint(initialStepTemp, step.targetTemperature, rampTimeMs, elapsedTime);
      } else {
        // Start dwell phase
        inDwellPhase = true;
        stepStartTime = now;
        setpoint = step.targetTemperature;
      }
    } else {
      // We're in the dwell phase
      setpoint = step.targetTemperature;

      if (elapsedTime >= dwellTimeMs) {
        // Move to next step
        currentStep++;
        inDwellPhase = false;
        stepStartTime = now;

        if (currentStep >= profileStepCount) {
          endProfile();
          return;
        }

        initialStepTemp = readKilnTemperature();
      }
    }

    Serial.print("   Setpoint (deg. C) = "); Serial.println(setpoint);

    // Apply control, prrofile is still running
    tempControl.setSetpoint(setpoint);
    double currentTemp = readKilnTemperature();

    Serial.print("   Current kiln (deg. C) = "); Serial.println(currentTemp);
    
    double duty = tempControl.update(currentTemp);

    Serial.print("   PID output = "); Serial.println(duty);
    
    setpoints[0] = duty;
    setpoints[1] = duty;
    setpoints[2] = duty;
    handlePWM();
  }

  //  // Debug
  //  Serial.print("Step ");
  //  Serial.print(currentStep);
  //  Serial.print(inDwellPhase ? " (Dwell)" : " (Ramp)");
  //  Serial.print(" | Temp: ");
  //  Serial.print(currentTemp);
  //  Serial.print("°C | Setpoint: ");
  //  Serial.print(setpoint);
  //  Serial.print("°C | Duty: ");
  //  Serial.println(duty);

  //  delay(1000);  // 1-second control loop
}

void endProfile() {
  Serial.println("Stopping profile");
  profileComplete = true;
  setpoints[0] = 0; // turn off all elements
  setpoints[1] = 0;
  setpoints[2] = 0;
  handlePWM(); // shut off elements
  Serial.println("Profile complete.");
  logDone();
}

double calculateSetpoint(double initialTemp, double finalTemp, unsigned long targetTimeMs, unsigned long currentTimeMs) {
  if (currentTimeMs >= targetTimeMs) {
    return finalTemp;
  }

  double progress = (double)currentTimeMs / (double)targetTimeMs;
  return initialTemp + (finalTemp - initialTemp) * progress;
}

// load a preset from SD card by name
void loadPreset(const char * filename, ProfileStep *profile) {
  profileStepCount = 0; // reset global step count

  if (readTemperatureProfile(filename, profile, profileStepCount)) {
    Serial.println("Profile loaded:");
    for (size_t i = 0; i < profileStepCount; ++i) {
      Serial.print("Step ");
      Serial.print(i);
      Serial.print(": Temp = ");
      Serial.print(profile[i].targetTemperature);
      Serial.print(" °C, Ramp = ");
      Serial.print(profile[i].rampMinutes);
      Serial.print(" min, Dwell = ");
      Serial.print(profile[i].dwellMinutes);
      Serial.println(" min");
    }
  } else {
    Serial.println("Failed to load profile.");
  }
}

void startProfile() {
    currentStep = 0;
    inDwellPhase = false;
    stepStartTime = millis();
    profileComplete = false;
    initialStepTemp = readKilnTemperature();  // lock in true start temp
    Serial.println("Profile started.");
}

void loraRxCheckWait(int timeout) {
  unsigned long then = millis() + timeout;

  int rxpos = 0;

  while ( millis() < then ) {
    if ( LORA_SERIAL.available() ) {
      rxBuf[rxpos] = LORA_SERIAL.read();
      Serial.write(rxBuf[rxpos]);
      rxpos++;
    }
  }

  if (rxpos > 9) { // if we ogt more than 9 bytes then a good reception is likely
    // check payload length (7th byte)
    // only support 3 letter commands!
    if ( rxBuf[7] == '3' ) {

      char inCmd[4] = {rxBuf[9], rxBuf[10], rxBuf[11], 0};
      if ( strcmp(inCmd, "off") == 0 ) { // all off, stop profile
        endProfile();
        printlnLora("Stopping profile");
      } else if (strcmp(inCmd, "ls1") == 0) { // load and start preset 1
        printlnLora("loading profile bisque1");
        loadPreset("bisque1.txt",profile);
        printlnLora("loaded profile bisque1 !");
        startProfile();
      } else if (strcmp(inCmd, "ls2") == 0) { // load and start preset 1
        printlnLora("loading profile bisque2");
        loadPreset("bisque2.txt",profile);
        printlnLora("loaded profile bisque 2!");
        startProfile();
      } else if ( strcmp(inCmd, "utt") == 0) { // use top peep hole TC temperature as current kiln temp
        pidInputSetting = 0; // 0 = top temp only
      } else if ( strcmp(inCmd, "ubt") == 0) { // use top peep hole TC temperature as current kiln temp
        pidInputSetting = 1; // 1 = bottom temp only
      } else if ( strcmp(inCmd, "uct") == 0) { // use combined top and bottom peep hole TC temperature as current kiln temp
        pidInputSetting = 2;
      } 
      
      else {
        Serial.print("Uknown 3 character command: ");
        Serial.println(rxBuf);
        printLora("Unknown command: ");
        printlnLora(inCmd);
      }
    } else {
        Serial.print("Unknown command: ");
        Serial.println(rxBuf);
        printlnLora("command was not 3 chars and was not recognized");
      }
  }
}

double average(double v1, double v2)
{
  return (v1 + v2) / 2.0;
}

double readKilnTemperature()
{
  switch( pidInputSetting ){
    case 0:
      return tc2_stat.hot;
      break;
    case 1:
      return tc1_stat.hot;
      break;
    case 2:
      return average(tc1_stat.hot, tc2_stat.hot);
      break;
    default:
      return average(tc1_stat.hot, tc2_stat.hot);
      break;
  }
}

bool readTemperatureProfile(const char* filename, ProfileStep* profile, size_t& stepCount) {
  File file = SD.open(filename, FILE_READ);
  if (!file) {
    Serial.print("Failed to open profile file: ");
    Serial.println(filename);
    stepCount = 0;
    return false;
  }

  stepCount = 0;

  while (file.available() && stepCount < MAX_PROFILE_STEPS) {
    String line = file.readStringUntil('\n');
    line.trim();  // Remove leading/trailing whitespace

    if (line.length() == 0 || line.startsWith("#")) {
      continue;
    }

    int firstColon = line.indexOf(':');
    int secondColon = line.indexOf(':', firstColon + 1);

    if (firstColon == -1 || secondColon == -1) {
      Serial.print("Invalid format in line: ");
      Serial.println(line);
      continue;
    }

    double temp = line.substring(0, firstColon).toFloat();
    unsigned long ramp = line.substring(firstColon + 1, secondColon).toInt();
    unsigned long dwell = line.substring(secondColon + 1).toInt();

    profile[stepCount].targetTemperature = temp;
    profile[stepCount].rampMinutes = ramp;
    profile[stepCount].dwellMinutes = dwell;
    stepCount++;
  }

  file.close();
  return (stepCount > 0);
}

void loraReadTimeout(int mils) {
  unsigned long then = millis() + mils;
  char c;
  while ( millis() < then) {
    if (LORA_SERIAL.available()) {
      Serial.write(c = LORA_SERIAL.read());
      if ( c == '\n') break; // assume this is the tail end of "+OK\r\n"
    }
  }
}

void readTC(Adafruit_MCP9601 &mcp, struct tc_status &stat) {
  stat.status = mcp.getStatus();
  stat.hot = mcp.readThermocouple();
  stat.cold = mcp.readAmbient();
}

int init_mcp(Adafruit_MCP9601 &mcp, uint8_t i2c_address, struct tc_status &stat) {
  /* Initialise the driver with I2C_ADDRESS and the default I2C bus. */
  if (! mcp.begin(i2c_address)) {
    Serial.println("Sensor not found. Check wiring!");
    while (1);
  }

  Serial.println("Found MCP9601!");

  mcp.setADCresolution(MCP9600_ADCRESOLUTION_14);
  Serial.print("ADC resolution set to ");
  switch (mcp.getADCresolution()) {
    case MCP9600_ADCRESOLUTION_18:   Serial.print("18"); break;
    case MCP9600_ADCRESOLUTION_16:   Serial.print("16"); break;
    case MCP9600_ADCRESOLUTION_14:   Serial.print("14"); break;
    case MCP9600_ADCRESOLUTION_12:   Serial.print("12"); break;
  }
  Serial.println(" bits");

  mcp.setThermocoupleType(MCP9600_TYPE_K);
  Serial.print("Thermocouple type set to ");
  switch (mcp.getThermocoupleType()) {
    case MCP9600_TYPE_K:  Serial.print("K"); break;
    case MCP9600_TYPE_J:  Serial.print("J"); break;
    case MCP9600_TYPE_T:  Serial.print("T"); break;
    case MCP9600_TYPE_N:  Serial.print("N"); break;
    case MCP9600_TYPE_S:  Serial.print("S"); break;
    case MCP9600_TYPE_E:  Serial.print("E"); break;
    case MCP9600_TYPE_B:  Serial.print("B"); break;
    case MCP9600_TYPE_R:  Serial.print("R"); break;
  }
  Serial.println(" type");

  mcp.setFilterCoefficient(1);
  Serial.print("Filter coefficient value set to: ");
  Serial.println(mcp.getFilterCoefficient());

  mcp.enable(true);

  delay(100);

  readTC(mcp, stat);
  Serial.print("Status: ");
  Serial.println(stat.status, HEX);

  if (stat.status & MCP9601_STATUS_OPENCIRCUIT) {
    Serial.println("Thermocouple open!");
    return -1; // don't continue, since there's no thermocouple
  }
  if (stat.status & MCP9601_STATUS_SHORTCIRCUIT) {
    Serial.println("Thermocouple shorted to ground!");
    return -1; // don't continue, since the sensor is not working
  }

  Serial.println(F("------------------------------"));
  return 0;
}

void handlePWM() {
  uint32_t phase = millis() % PWM_PERIOD;
  digitalWrite(RELAY_1, phase < (setpoints[0] * PWM_PERIOD));
  digitalWrite(RELAY_2, phase < (setpoints[1] * PWM_PERIOD));
  digitalWrite(RELAY_3, phase < (setpoints[2] * PWM_PERIOD));
}

void logDone() {
  logFile = SD.open(logfilename, FILE_WRITE);
  if (logFile) {
    logFile.print("# ");
    logFile.print(millis());
    logFile.println(": Profile complete!");
    logFile.close();
  }
}

void logData() {
  readTC(tc1, tc1_stat);
  readTC(tc2, tc2_stat);
  readTC(tc3, tc3_stat);

  logFile = SD.open(logfilename, FILE_WRITE);
  if (logFile) {
    logFile.print(millis()); logFile.print(",");
    logFile.print(setpoint); logFile.print(",");
    logFile.print(tc1_stat.cold); logFile.print(",");
    logFile.print(tc2_stat.cold); logFile.print(",");
    logFile.print(tc3_stat.cold); logFile.print(",");
    logFile.print(tc1_stat.hot); logFile.print(",");
    logFile.print(tc2_stat.hot); logFile.print(",");
    logFile.print(tc3_stat.hot); logFile.print(",");
    logFile.print(setpoints[0]); logFile.print(",");
    logFile.print(setpoints[1]); logFile.print(",");
    logFile.println(setpoints[2]);
    logFile.close();
  }
}

void sendLoRaData() {
  uint8_t txbuf[120];
  uint8_t databuf[100];

  int len = sprintf(databuf, "%d, %6.2f, %6.2f, %6.2f, %6.2f, %2.0f, %2.0f, %2.0f\n",
                    millis(),
                    setpoint,
                    tc1_stat.hot,
                    tc2_stat.hot,
                    tc3_stat.hot,
                    setpoints[0] * PWM_PERIOD_SECONDS,
                    setpoints[1] * PWM_PERIOD_SECONDS,
                    setpoints[2] * PWM_PERIOD_SECONDS);

  // send data to node address 2
  int paylen = sprintf(txbuf, "AT+SEND=2,%d,%s\r\n", len, databuf);
  LORA_SERIAL.write(txbuf, paylen);
  //Serial.println("Sending LORA:");
  //Serial.write(txbuf, paylen);
  loraReadTimeout(200);
}

void printLora(char *message)
{
  uint8_t txbuf[120];
  uint8_t databuf[100];

  int len = sprintf(databuf, "%s", message);
  //Serial.print("Length of print message was: "); Serial.println(len);

  // send data to node address 2
  int paylen = sprintf(txbuf, "AT+SEND=2,%d,%s\r\n", len, databuf);
  LORA_SERIAL.write(txbuf, paylen);
  //Serial.write(txbuf, paylen);
  loraReadTimeout(200);
}

void printlnLora(char *message)
{
  uint8_t txbuf[120];
  uint8_t databuf[100];

  int len = sprintf(databuf, "%s\n", message);
  //Serial.print("Length of println message was: "); Serial.println(len);

  // send data to node address 2
  int paylen = sprintf(txbuf, "AT+SEND=2,%d,%s\r\n", len, databuf);
  LORA_SERIAL.write(txbuf, paylen);
  //Serial.write(txbuf, paylen);
  loraReadTimeout(200);
}
