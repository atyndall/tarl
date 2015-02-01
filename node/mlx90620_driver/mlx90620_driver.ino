/**
 * MLX90260 Arduino Interface
 * Based on code from http://forum.arduino.cc/index.php/topic,126244.0.html
 */
//#define __ASSERT_USE_STDERR

//#include <assert.h>
#include <math.h>
#include <Wire.h>
#include "SimpleTimer.h" // http://playground.arduino.cc/Code/SimpleTimer

// Configurable options
int REFRESH_FREQ            = -1;    // Refresh rate of sensor in Hz, must be power of 2 (0 = 0.5Hz) -1 means configured at startup
const int POR_CHECK_FREQ    = 2000; // Time in milliseconds to check if MLX reset has occurred
const int PIR_INTERRUPT_PIN = 0;    // D2 on the Arduino Uno

// Configuration constants
#define PIXEL_LINES     4
#define PIXEL_COLUMNS   16
#define BYTES_PER_PIXEL 2
#define EEPROM_SIZE     255
#define NUM_PIXELS      (PIXEL_LINES * PIXEL_COLUMNS)

// EEPROM helpers
#define E_READ(X)       (EEPROM_DATA[X])
#define E_WRITE(X, Y)   (EEPROM_DATA[X] = (Y))

// Bit fiddling helpers
#define BYTES2INT(H, L)     ( ((H) << 8) + (L) )
#define UBYTES2INT(H, L)    ( ((unsigned int)(H) << 8) + (unsigned int)(L) )
#define BYTE2INT(B)         ( ((int)(B) > 127) ? ((int)(B) - 256) : (int)(B) )
#define E_BYTES2INT(H, L)   ( BYTES2INT(E_READ(H), E_READ(L)) )
#define E_UBYTES2INT(H, L)  ( UBYTES2INT(E_READ(H), E_READ(L)) )
#define E_BYTE2INT(X)       ( BYTE2INT(E_READ(X)) )

// I2C addresses
#define ADDR_EEPROM   0x50
#define ADDR_SENSOR   0x60

// I2C commands
#define CMD_SENSOR_READ         0x02
#define CMD_SENSOR_WRITE_CONF   0x03
#define CMD_SENSOR_WRITE_TRIM   0x04

// Addresses in the sensor RAM (see Table 9 in spec)
#define SENSOR_PTAT             0x90
#define SENSOR_CPIX             0x91
#define SENSOR_CONFIG           0x92

// Addresses in the EEPROM (see Tables 5 & 7 in spec)
#define EEPROM_A_I_00             0x00 // A_i(0,0) IR pixel individual offset coefficient (ends at 0x3F)
#define EEPROM_B_I_00             0x40 // B_i(0,0) IR pixel individual offset coefficient (ends at 0x7F)
#define EEPROM_DELTA_ALPHA_00     0x80 // Delta-alpha(0,0) IR pixel individual offset coefficient (ends at 0xBF)
#define EEPROM_A_CP               0xD4 // Compensation pixel individual offset coefficients
#define EEPROM_B_CP               0xD5 // Individual Ta dependence (slope) of the compensation pixel offset
#define EEPROM_ALPHA_CP_L         0xD6 // Sensitivity coefficient of the compensation pixel (low)
#define EEPROM_ALPHA_CP_H         0xD7 // Sensitivity coefficient of the compensation pixel (high)
#define EEPROM_TGC                0xD8 // Thermal gradient coefficient
#define EEPROM_B_I_SCALE          0xD9 // Scaling coefficient for slope of IR pixels offset
#define EEPROM_V_TH_L             0xDA // V_TH0 of absolute temperature sensor (low)
#define EEPROM_V_TH_H             0xDB // V_TH0 of absolute temperature sensor (high)
#define EEPROM_K_T1_L             0xDC // K_T1 of absolute temperature sensor (low)
#define EEPROM_K_T1_H             0xDD // K_T1 of absolute temperature sensor (high)
#define EEPROM_K_T2_L             0xDE // K_T2 of absolute temperature sensor (low)
#define EEPROM_K_T2_H             0xDF // K_T2 of absolute temperature sensor (high)
#define EEPROM_ALPHA_0_L          0xE0 // Common sensitivity coefficient of IR pixels (low)
#define EEPROM_ALPHA_0_H          0xE1 // Common sensitivity coefficient of IR pixels (high)
#define EEPROM_ALPHA_0_SCALE      0xE2 // Scaling coefficient for common sensitivity
#define EEPROM_DELTA_ALPHA_SCALE  0xE3 // Scaling coefficient for individual sensitivity
#define EEPROM_EPSILON_L          0xE4 // Emissivity (low)
#define EEPROM_EPSILON_H          0xE5 // Emissivity (high)
#define EEPROM_TRIMMING_VAL       0xF7 // Oscillator trimming value

// Config flag locations
#define CFG_TA    8
#define CFG_IR    9  
#define CFG_POR   10

// Global variables
unsigned int PTAT;              // Proportional to absolute temperature value
int CPIX;                       // Compensation pixel

int IRDATA[NUM_PIXELS];         // Infrared raw data
byte EEPROM_DATA[EEPROM_SIZE];  // EEPROM dump

float ta;                       // Absolute chip temperature / ambient chip temperature (degrees celsius)
float emissivity;               // Emissivity compensation
float k_t1;                     // K_T1 of absolute temperature sensor
float k_t2;                     // K_T2 of absolute temperature sensor
float da0_scale;                // Scaling coefficient for individual sensitivity
float alpha_const;              // Common sensitivity coefficient of IR pixels and scaling coefficient for common sensitivity

int v_th;                       // V_TH0 of absolute temperature sensor
int a_cp;                       // Compensation pixel individual offset coefficients
int b_cp;                       // Individual Ta dependence (slope) of the compensation pixel offset
int tgc;                        // Thermal gradient coefficient
int b_i_scale;                  // Scaling coefficient for slope of IR pixels offset

float alpha_ij[NUM_PIXELS];     // Individual pixel sensitivity coefficient
int a_ij[NUM_PIXELS];           // Individual pixel offset
int b_ij[NUM_PIXELS];           // Individual pixel offset slope coefficient

char hpbuf[2];                  // Hex printing buffer
int res;                        // Error code storage

float temp[NUM_PIXELS];         // Final calculated temperature values in degrees celsius

SimpleTimer timer;              // Allows timed callbacks for temp functions

void(* reset_arduino) (void) = 0;   // Creates function to reset Arduino

// Stores references to the 3 timers used in the program
int ir_timer;
int ta_timer;
int por_timer;

volatile bool pir_motion_detected = false;

/*
// Send assertion failures over serial
void __assert(const char *__func, const char *__file, int __lineno, const char *__sexp) {
    // transmit diagnostic informations through serial link. 
    Serial.println(__func);
    Serial.println(__file);
    Serial.println(__lineno, DEC);
    Serial.println(__sexp);
    Serial.flush();
    // abort program execution.
    abort();
}*/

// Basic assertion failure function
void assert(boolean a) {
  if (!a) Serial.println("ASSFAIL");
}

// Takes byte value and will output 2 character hex representation on serial
void print_hex(byte b) {
  hpbuf[0] = (b >> 4) + 0x30;
  if (hpbuf[0] > 0x39) hpbuf[0] +=7;

  hpbuf[1] = (b & 0x0f) + 0x30;
  if (hpbuf[1] > 0x39) hpbuf[1] +=7;

  Serial.print(hpbuf);
}

// Will read memory from the given sensor address and convert it into an integer
int _sensor_read_int(byte read_addr) {
  Wire.beginTransmission(ADDR_SENSOR);
  Wire.write(CMD_SENSOR_READ);
  Wire.write(read_addr);
  Wire.write(0x00); // address step (0)
  Wire.write(0x01); // number of reads (1)
  res = Wire.endTransmission(false); // we must use the repeated start here
  if (res != 0) return -1;
  
  Wire.requestFrom(ADDR_SENSOR, 2); // technically the 1 read takes up 2 bytes

  int LSB, MSB;
  int i = 0;
  while( Wire.available() ) {
    i++;

    if (i > 2) {
      return -1; // Returned more bytes than it should have
    }

    LSB = Wire.read();
    MSB = Wire.read(); 
  }

  return UBYTES2INT(MSB, LSB); // rearrange int to account for endian difference (TODO: check)
}

// Will read a configuration flag bit specified by flag_loc from the sensor config
bool _sensor_read_config_flag(int flag_loc) {
  int cur_cfg = _sensor_read_int(SENSOR_CONFIG);
  return (bool)(cur_cfg & ( 1 << flag_loc )) >> flag_loc;
}

// Reads Proportional To Absolute Temperature (PTAT) value
int sensor_read_ptat() {
  return _sensor_read_int(SENSOR_PTAT);
}

// Reads compensation pixel
int sensor_read_cpix() {
  return _sensor_read_int(SENSOR_CPIX);
}

// Reads POR flag
bool sensor_read_por() {
  return _sensor_read_config_flag(CFG_POR); // POR is 10th bit
}

// Read Ta measurement flag
bool sensor_read_ta_measure() {
  return _sensor_read_config_flag(CFG_TA);
}

// Read IR measurement flag
bool sensor_read_ir_measure() {
  return _sensor_read_config_flag(CFG_IR);
}

// Reads all raw IR data from sensor into IRDATA variable
boolean sensor_read_irdata() {
  int i = 0;

  // Due to wire library buffer limitations, we can only read up to 32 bytes at a time
  // Thus, the request has been split into multiple different requests to get the full 128 values
  // Each pixel value takes up two bytes (???) thus NUM_PIXELS * 2
  for (int line = 0; line < PIXEL_LINES; line++) {     
    Wire.beginTransmission(ADDR_SENSOR);
    Wire.write(CMD_SENSOR_READ);
    Wire.write(line);
    Wire.write(0x04);
    Wire.write(0x10);
    res = Wire.endTransmission(false); // use repeated start to get answer
    
    if (res != 0) return false;  
    
    Wire.requestFrom(ADDR_SENSOR, PIXEL_COLUMNS * BYTES_PER_PIXEL);
    
    byte PIX_LSB, PIX_MSB;
    
    for(int j = 0; j < PIXEL_COLUMNS; j++) {
      if (!Wire.available()) return false;
      
      // We read two bytes
      PIX_LSB = Wire.read();
      PIX_MSB = Wire.read();
      
      IRDATA[i] = BYTES2INT(PIX_MSB, PIX_LSB);
      i++;
    }
  }

  return true;
}

// Will send a command and the provided most significant and least significant bit 
// with the appropriate check bit added
// Returns the Wire success/error code
boolean _sensor_write_check(byte cmd, byte check, byte lsb, byte msb) {
  Wire.beginTransmission(ADDR_SENSOR);
  Wire.write(cmd);          // Send the command
  Wire.write(lsb - check);  // Send the least significant byte check
  Wire.write(lsb);          // Send the least significant byte
  Wire.write(msb - check);  // Send the most significant byte check
  Wire.write(msb);          // Send the most significant byte
  return Wire.endTransmission() == 0;
}

// See datasheet: 9.4.2 Write configuration register command
// See datasheet: 8.2.2.1 Configuration register (0x92)
// Check byte is 0x55 in this instance
boolean sensor_write_conf() {
  byte cfg_MSB = B01110100;
  //              ||||||||
  //              |||||||*--- Ta measurement running (read only)
  //              ||||||*---- IR measurement running (read only)
  //              |||||*----- POR flag cleared
  //              ||||*------ I2C FM+ mode enabled
  //              ||**------- Ta refresh rate (2 byte code, 2Hz hardcoded)
  //              |*--------- ADC high reference
  //              *---------- NA

  byte cfg_LSB = B00001110;
  //              ||||||||
  //              ||||****--- 4 byte IR refresh rate (4 byte code, 1Hz default)
  //              ||**------- NA
  //              |*--------- Continuous measurement mode
  //              *---------- Normal operation mode

  switch(REFRESH_FREQ) {
  case 0: // 0.5Hz
    cfg_LSB = B00001111;
    break;
  case 2:
    cfg_LSB = B00001101;
    break;
  case 4:
    cfg_LSB = B00001100;
    break;
  case 8:
    cfg_LSB = B00001011;
    break;
  case 16:
    cfg_LSB = B00001010;
    break;
  case 32:
    cfg_LSB = B00001001;
    break;
  case 64:
    cfg_LSB = B00001000;
    break;
  case 128:
    cfg_LSB = B00000111;
    break;
  case 256:
    cfg_LSB = B00000110;
    break;
  case 512:
    cfg_LSB = B00000000; // modes 5 to 0 are all 512Hz
    break;
  }

  return _sensor_write_check(CMD_SENSOR_WRITE_CONF, 0x55, cfg_LSB, cfg_MSB);
}

// See datasheet: 9.4.3 Write trimming command
// Check byte is 0xAA in this instance
boolean sensor_write_trim() {
  return _sensor_write_check(CMD_SENSOR_WRITE_TRIM, 0xAA, E_READ(EEPROM_TRIMMING_VAL), 0x00);
}

// Reads EEPROM memory into global variable
boolean eeprom_read_all() {
  int i = 0;
  // Due to wire library buffer limitations, we can only read up to 32 bytes at a time
  // Thus, the request has been split into 4 different requests to get the full 128 values
  for(int j = 0; j < EEPROM_SIZE; j = j + 32) {
    Wire.beginTransmission(ADDR_EEPROM);
    Wire.write( byte(j) );
    res = Wire.endTransmission();

    if (res != 0) return false;
    
    Wire.requestFrom(ADDR_EEPROM, 32);

    i = j;
    while( Wire.available() ) { // slave may send less than requested
      byte b = Wire.read(); // receive a byte as character
      E_WRITE(i, b);
      i++;
    }
  }

  if (i < EEPROM_SIZE) { // If we didn't get the whole EEPROM
    return false;
  }

  return true;
}

// Writes various calculation values from EEPROM into global variables
void calculate_init() {
  v_th = E_BYTES2INT(EEPROM_V_TH_H, EEPROM_V_TH_L);
  k_t1 = E_BYTES2INT(EEPROM_K_T1_H, EEPROM_K_T1_L) / 1024.0;
  k_t2 = E_BYTES2INT(EEPROM_K_T2_H, EEPROM_K_T2_L) / 1048576.0;

  a_cp = E_BYTE2INT(EEPROM_A_CP);
  b_cp = E_BYTE2INT(EEPROM_B_CP);
  tgc  = E_BYTE2INT(EEPROM_TGC);

  b_i_scale = E_READ(EEPROM_B_I_SCALE);

  emissivity = E_UBYTES2INT(EEPROM_EPSILON_H, EEPROM_EPSILON_L) / 32768.0;

  da0_scale = pow(2, -E_READ(EEPROM_DELTA_ALPHA_SCALE));
  alpha_const = (float)E_UBYTES2INT(EEPROM_ALPHA_0_H, EEPROM_ALPHA_0_L) * pow(2, -E_READ(EEPROM_ALPHA_0_SCALE));

  for (int i = 0; i < NUM_PIXELS; i++){
    float alpha_var = (float)E_READ(EEPROM_DELTA_ALPHA_00 + i) * da0_scale;
    alpha_ij[i] = (alpha_const + alpha_var);

    a_ij[i] = E_BYTE2INT(EEPROM_A_I_00 + i);
    b_ij[i] = E_BYTE2INT(EEPROM_B_I_00 + i);
  }
}

// Calculates the absolute chip temperature from the proportional to absolute temperature (PTAT)
float calculate_ta() {
  float ptat = (float)sensor_read_ptat();
  assert(ptat != -1);
  return (-k_t1 + 
      sqrt(
        square(k_t1) - 
        ( 4 * k_t2 * (v_th-ptat) )
      )
    ) / (2*k_t2) + 25;
}

// Calculates the final temperature value for each pixel and stores it in temp array
void calculate_temp() {
  float v_cp_off_comp = (float) CPIX - (a_cp + (b_cp/pow(2, b_i_scale)) * (ta - 25));

  for (int i = 0; i < NUM_PIXELS; i++){
    float alpha_ij_v = alpha_ij[i];
    int a_ij_v = a_ij[i];
    int b_ij_v = b_ij[i];

    float v_ir_tgc_comp = IRDATA[i] - (a_ij_v + (float)(b_ij_v/pow(2, b_i_scale)) * (ta - 25)) - (((float)tgc/32)*v_cp_off_comp);
    float v_ir_comp = v_ir_tgc_comp / emissivity;
    temp[i] = sqrt(sqrt((v_ir_comp/alpha_ij_v) + pow((ta + 273.15),4))) - 273.15;
  }

}

// Prints all of EEPROM as hex
void print_eeprom() {
  Serial.print("EEPROM ");
  for(int i = 0; i < EEPROM_SIZE; i++) {
    print_hex(E_READ(i));
  }
  Serial.println();
}

// Prints a serial "packet" containing IR data
void print_packet(unsigned long cur_time) {
  Serial.print("START ");
  Serial.println(cur_time);

  Serial.print("MOVEMENT ");
  Serial.println(pir_motion_detected);

  for(int i = 0; i<NUM_PIXELS; i++) {
    Serial.print(temp[i]);

    if ((i+1) % PIXEL_COLUMNS == 0) {
      Serial.println();
    } else {
      Serial.print("\t");
    }
  }

 Serial.print("STOP ");
 Serial.println(millis());
 Serial.flush();
}

// Prints info about driver, build and configuration
void print_info() {
  Serial.println("INFO START");
  Serial.println("DRIVER MLX90620");

  Serial.print("BUILD ");
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.println(__TIME__);

  Serial.print("IRHZ ");
  Serial.println(REFRESH_FREQ);
  Serial.println("INFO STOP");
}

// Runs functions necessary to initialize the temperature sensor
void initialize() {
  assert(eeprom_read_all());
  assert(sensor_write_trim());
  assert(sensor_write_conf());

  calculate_init();

  ta_loop();
}

// Calculates absolute temperature
void ta_loop() {
  ta = calculate_ta();
}

// Checks if the sensor as been reset, and if so, re-runs the initialize functions
void por_loop() {
  if (!sensor_read_por()) { // there has been a reset
    initialize();
  }
}

// Runs functions necessary to compute and output the temperature data
void ir_loop() {
  unsigned long cur_time = millis();

  assert(sensor_read_irdata());

  CPIX = sensor_read_cpix();
  assert(CPIX != -1);

  calculate_temp();

  print_packet(cur_time);

  pir_motion_detected = false;
}

// Configures timers to poll IR and other data periodically
void activate_timers() {
  float hz = REFRESH_FREQ;

  if (REFRESH_FREQ == 0) {
    hz = 0.5;
  }

  // Calculate how many milliseconds each timer should run for
  // based upon the configured refresh rate of the IR data and 
  // absolute temperature data
  long irlen = (1/hz) * 1000;
  long talen = (1/2.0) * 1000;

  if (talen < irlen) {
    talen = irlen;
  }

  ir_timer = timer.setInterval(irlen, ir_loop);
  ta_timer = timer.setInterval(talen, ta_loop);
  por_timer = timer.setInterval(POR_CHECK_FREQ, por_loop);

  attachInterrupt(PIR_INTERRUPT_PIN, pir_motion, RISING);
}

// Disables timers to poll IR and other data periodically
void deactivate_timers() {
  timer.disable(ir_timer);
  timer.deleteTimer(ir_timer);

  timer.disable(ta_timer);
  timer.deleteTimer(ta_timer);

  timer.disable(por_timer);
  timer.deleteTimer(por_timer);

  detachInterrupt(PIR_INTERRUPT_PIN);
}

void pir_motion() {
  pir_motion_detected = true;
}

// Configure libraries and sensors at startup
void setup() {
  pinMode(2, INPUT);

  Wire.begin();
  Serial.begin(115200);

  Serial.println();
  Serial.print("INIT ");
  Serial.println(millis());

  while (REFRESH_FREQ == -1) { // If no freq set, wait for conf over serial
    serialEvent();
    delay(200);
  }

  print_info();
  initialize();

  Serial.print("ACTIVE ");
  Serial.println(millis());
  Serial.flush();
}

char manualLoop = 0;

// Triggered when serial data is sent to Arduino. Used to trigger basic actions.
void serialEvent() {
  while (Serial.available()) {
    char in = (char)Serial.read();
    if (in == '\r' || in == '\n') continue;

    switch (in) {
    case 'R':
    case 'r':
      reset_arduino();
      break;

    case 'I':
    case 'i':
      print_info();
      break;

    case 'T':
    case 't':
      activate_timers();
      break;

    case 'O':
    case 'o':
      deactivate_timers();
      break;

    case 'P':
    case 'p':
      if (manualLoop == 16) { // Run ta_loop every 16 manual iterations
        ta_loop();
        manualLoop = 0;
      }

      ir_loop();

      manualLoop++;
      break;

    case 'f':
    case 'F':
      if (REFRESH_FREQ == -1) {
        REFRESH_FREQ = Serial.parseInt();
      } else {
        Serial.println("FREQ ALREADY SET");
      }
      break;

    default:
      Serial.println("UNKNOWN COMMAND");
    }
  }
}

void loop() {
  timer.run();
}