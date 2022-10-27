#include <TMCStepper.h>
#include <TMCStepper_UTILITY.h>

// @author Michael Hoffer <info@michaelhoffer.de>
// Requires TMCStepper library
#include <TMCStepper.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <Wire.h>

#define CS_PIN1   8 // CS1 chip select
#define CS_PIN2   9 // CS1 chip select
#define CS_PIN3  10 // CS2 chip select
#define MOSI_PIN 11 // SDI/MOSI (ICSP: 4, Uno: 11, Mega: 51)
#define MISO_PIN 12 // SDO/MISO (ICSP: 1, Uno: 12, Mega: 50)
#define SCK_PIN  13 // CLK/SCK  (ICSP: 3, Uno: 13, Mega: 52)

#define R_SENSE   0.075f //TMC5160: 0.075

// initialize motor drivers
TMC5160Stepper driver_0 = TMC5160Stepper(CS_PIN1, R_SENSE, MOSI_PIN, MISO_PIN, SCK_PIN);
TMC5160Stepper driver_1 = TMC5160Stepper(CS_PIN2, R_SENSE, MOSI_PIN, MISO_PIN, SCK_PIN);
TMC5160Stepper driver_2 = TMC5160Stepper(CS_PIN3, R_SENSE, MOSI_PIN, MISO_PIN, SCK_PIN);

TMC5160Stepper* driver[3];

// -----------------------------------------------------------------------------

#define VERSION    "SPI-MOTOR-DRIVER-MODULE v0.1.0"
#define MSG_SIZE        32
#define BAUD_RATE   115200

// -----------------------------------------------------------------------------

#define READ_STATUS_ERROR_TIMEOUT  -2
#define READ_STATUS_ERROR_OVERFLOW -1
#define READ_STATUS_READING         0
#define READ_STATUS_READY           1

// -----------------------------------------------------------------------------

#define CMD_VERSION         'v'    // get version info
#define CMD_REBOOT          'r'    // reboots the device
#define CMD_SAVE            's'    // save values to eeprom
#define CMD_RMS_CURRENT     'c'    // set current [range of uint32_t]
#define CMD_MICROSTEPS      'm'    // set microsteps  [range of uint8_t]
#define CMD_PWM_MODE        'p'    // enable/disable pwm mode [0,1]
#define CMD_LIST_SETTINGS   'l'    // list settings

#define CHANNEL_ERROR       '!'    // e.g. prefix !re defines error event with content 're'
#define CHANNEL_SELECT      '#'    // e.g. prefix #32 defines channel 32

#define LF                 '\n'    // line feed aka. newline

// -----------------------------------------------------------------------------

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

uint64_t read_timestamp    = 0;
uint64_t now               = 0;
uint8_t  buffer[MSG_SIZE]; 
uint8_t  buffer_len        = 0;
bool     msg_done          = false;
uint32_t cnt               = 0;
int8_t   read_status       = 0;

int8_t read() {
    while (Serial.available()) {
      uint8_t c = Serial.read();
      buffer[cnt++] = c;
      if ((c == LF) || (cnt == sizeof(buffer) - 1)) {
        buffer[cnt] = '\0';
        buffer_len = cnt;
        cnt = 0;
        return (buffer[buffer_len-1] == LF)?READ_STATUS_READY:READ_STATUS_ERROR_OVERFLOW;
      }
    }
    return READ_STATUS_READING;
}

// -----------------------------------------------------------------------------

struct DriverSettings {
  bool pwm_mode_enabled = true;
  uint16_t microsteps    = 16;
  uint16_t rms_current  = 0;
};

DriverSettings settings[3];

void load_setting(uint8_t axis) {
  //int address = &settings[axis];
  int address = axis * sizeof(settings[axis]);
  EEPROM.get(address, settings[axis]);
}

void save_setting(uint8_t axis) {
  //int address = &settings[axis];
  int address = axis * sizeof(settings[axis]);

  EEPROM.put(address, settings[axis]);
}

void apply_setting(uint8_t axis) {
  driver[axis]->begin();
  driver[axis]->toff(4); //off time
  driver[axis]->blank_time(24); //blank time
  driver[axis]->en_pwm_mode(settings[axis].pwm_mode_enabled); // enable extremely quiet stepping
  driver[axis]->microsteps(settings[axis].microsteps);        // microsteps (up to 256)
  driver[axis]->rms_current(settings[axis].rms_current);      // motor current in mA RMS
  digitalWrite(CS_PIN1, LOW);
  digitalWrite(CS_PIN2, LOW);
  digitalWrite(CS_PIN3, LOW);
}

void setup() {

  Serial.begin(BAUD_RATE);

  driver[0] = &driver_0;
  driver[1] = &driver_1;
  driver[2] = &driver_2;

  load_setting(X_AXIS);
  load_setting(Y_AXIS);
  load_setting(Z_AXIS);

  /*
  if(settings[0].rms_current == 65535) {
     
  }
  */
  
  apply_setting(X_AXIS);
  apply_setting(Y_AXIS);
  apply_setting(Z_AXIS);

}

void loop() {

  if((read_status = read()) == READ_STATUS_READY) {

    read_timestamp = now;

    if(buffer[0]==CMD_VERSION) {
      Serial.println(VERSION);

      Serial.print(CMD_VERSION);
      Serial.println(":ok");

      return;
    }

    if(buffer[0]==CMD_REBOOT) {

      Serial.print(CMD_REBOOT);
      Serial.println(":ok");
 
      wdt_enable(WDTO_60MS);
      return;
    }

    if(buffer[0]==CMD_SAVE) {
      
      save_setting(X_AXIS);
      save_setting(Y_AXIS);
      save_setting(Z_AXIS);

      Serial.print(CMD_SAVE);
      Serial.println(":ok");
        
      
      return;
    }

    if(buffer[0]==CMD_LIST_SETTINGS) {
      Serial.println(CMD_LIST_SETTINGS);

      Serial.print("cmd 'p:0:[VALUE]', x-axis:en-pwm:");
      Serial.println(settings[X_AXIS].pwm_mode_enabled);
      Serial.print("cmd 'm:0:[VALUE]', x-axis:microsteps:");
      Serial.println(settings[X_AXIS].microsteps);
      Serial.print("cmd 'c:0:[VALUE]', x-axis:rms-current (mA):");
      Serial.println(settings[X_AXIS].rms_current);

      Serial.print("cmd 'p:1:[VALUE]', y-axis:en-pwm:");
      Serial.println(settings[Y_AXIS].pwm_mode_enabled);
      Serial.print("cmd 'm:1:[VALUE]', y-axis:microsteps:");
      Serial.println(settings[Y_AXIS].microsteps);
      Serial.print("cmd 'c:1:[VALUE]', y-axis:rms-current (mA):");
      Serial.println(settings[Y_AXIS].rms_current);

      Serial.print("cmd 'p:2:[VALUE]', z-axis:en-pwm:");
      Serial.println(settings[Z_AXIS].pwm_mode_enabled);
      Serial.print("cmd 'm:2:[VALUE]', z-axis:microsteps:");
      Serial.println(settings[Z_AXIS].microsteps);
      Serial.print("cmd 'c:2:[VALUE]', z-axis:rms-current (mA):");
      Serial.println(settings[Z_AXIS].rms_current);

      Serial.print(CMD_LIST_SETTINGS);
      Serial.println(":ok");
        

      return;
    }

    if(buffer[0]==CMD_PWM_MODE) {
      // p:0:0 -> x-off
      // p:0:1 -> x-on
      // p:1:0 -> y-off
      // p:1:1 -> y-on

      char* cmdToken  = strtok(buffer, ":");
      char* axisToken = strtok(NULL, ":");
      char* flagToken = strtok(NULL, ":");

      if(cmdToken==NULL) {
        Serial.println("ERROR: cannot parse msg.");
        return;
      }

      int64_t axis = atol(axisToken);

      if(axis < 0 || axis > 2) Serial.println("ERROR: axis out of range [0,1,2].");

      Serial.print("axis: ");
      Serial.print((long)axis);

      if(flagToken==NULL) {
        Serial.println("ERROR: cannot parse msg. Enable/Disable flag missing.");

        Serial.print(CMD_PWM_MODE);
        Serial.println(":ERROR");
        
        return;
      }

      int64_t flag = atol(flagToken);
      if(flag < 0 || flag > 1) Serial.println("ERROR: flag out of range [0,1].");

      Serial.print(", flag: ");
      Serial.println((long)flag);

      driver[axis]->en_pwm_mode(flag);
      settings[axis].pwm_mode_enabled = flag;
      
      Serial.print(CMD_PWM_MODE);
      Serial.println(":ok");
        
      return;
     
    }

    if(buffer[0]==CMD_RMS_CURRENT) {

      // c:0:100 -> x 100mA
      // c:1:230 -> y 230mA
      
      char* cmdToken  = strtok(buffer, ":");
      char* axisToken = strtok(NULL, ":");
      char* valueToken = strtok(NULL, ":");

      if(cmdToken==NULL) {
        Serial.println("ERROR: cannot parse msg.");
        return;
      }

      int64_t axis = atol(axisToken);

      if(axis < 0 || axis > 2) Serial.println("ERROR: axis out of range [0,1,2].");

      Serial.print("axis: ");
      Serial.print((long)axis);

      if(valueToken==NULL) {
        Serial.println("ERROR: cannot parse msg. Current missing.");

        Serial.print(CMD_RMS_CURRENT);
        Serial.println(":error");
        
        return;
      }

      int64_t value = atol(valueToken);
      if(value < 0) Serial.println("ERROR: value out of range [0, int64_t].");

      Serial.print(", rms-current(mA): ");
      Serial.println((long)value);

      driver[axis]->rms_current(value);
      settings[axis].rms_current = value;

      Serial.print(CMD_RMS_CURRENT);
      Serial.println(":ok");
        
      return;
    }

    if(buffer[0]==CMD_MICROSTEPS) {

      // m:0:8   -> x 8   microsteps
      // m:1:256 -> y 256 microsteps

      char* cmdToken  = strtok(buffer, ":");
      char* axisToken = strtok(NULL, ":");
      char* valueToken = strtok(NULL, ":");

      if(cmdToken==NULL) {
        Serial.println("ERROR: cannot parse msg.");
        return;
      }

      int64_t axis = atol(axisToken);

      if(axis < 0 || axis > 2) Serial.println("ERROR: axis out of range [0,1,2].");

      Serial.print("axis: ");
      Serial.print((long)axis);

      if(valueToken==NULL) {
        Serial.println("ERROR: cannot parse msg. Microsteps missing.");
        Serial.print(CMD_MICROSTEPS);
        Serial.println(":error");
        return;
      }

      int64_t value = atol(valueToken);
      if(value < 0||value>256) Serial.println("ERROR: microsteps out of range [1, 2, 4, 8, 16,...,256].");

      Serial.print(", microsteps: ");
      Serial.println((long)value);

      driver[axis]->microsteps(value);
      settings[axis].microsteps = value;

      Serial.print(CMD_MICROSTEPS);
      Serial.println(":ok");

      return;
    }

  }

  if(read_status == READ_STATUS_ERROR_OVERFLOW) {
    Serial.print("!re:");
    Serial.println(READ_STATUS_ERROR_OVERFLOW);
  } else if (read_status == READ_STATUS_ERROR_TIMEOUT) {
    Serial.print("!re:");
    Serial.println(READ_STATUS_ERROR_TIMEOUT);
  }


}
