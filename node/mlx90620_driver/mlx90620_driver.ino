/**
 * MLX90260 Arduino Interface
 */
//#define __ASSERT_USE_STDERR

//#include <assert.h>
#include <math.h>
#include <Wire.h>
#include "SimpleTimer.h" // http://playground.arduino.cc/Code/SimpleTimer


// Configuration constants
#define PIXEL_LINES     4
#define PIXEL_COLUMNS   16
#define BYTES_PER_PIXEL 2
#define EEPROM_SIZE     255
#define NUM_PIXELS      (PIXEL_LINES * PIXEL_COLUMNS)

// EEPROM helpers
#define e_read(X)       (EEPROM_DATA[X])
#define e_write(X, Y)   (EEPROM_DATA[X] = (Y))

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

// Config flags
#define CFG_TA    8
#define CFG_IR    9  
#define CFG_POR   10

const int REFRESH_FREQ = 1;


unsigned int PTAT;              // Proportional to absolute temperature value
int CPIX;                       // Compensation pixel

int IRDATA[NUM_PIXELS];         // Infrared raw data
byte EEPROM_DATA[EEPROM_SIZE];  // EEPROM dump
boolean EEPROM_DUMPED;          // If EEPROM dump has previously occurred

float ta;                       // Absolute chip temperature / ambient chip temperature (degrees celsius)
float to;
float emissivity;
float k_t1;
float k_t2;
float da0_scale;
float alpha_const;

int v_th;
int a_cp;
int b_cp;
int tgc;
int b_i_scale;

float alpha_ij[NUM_PIXELS];
int a_ij[NUM_PIXELS];
int b_ij[NUM_PIXELS];

char hpbuf[2];          // Hex printing buffer
int res;                // Error code storage

float temp[NUM_PIXELS];

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

void assert(boolean a) {
  //if (!a) Serial.println("ASSFAIL");
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

  return ((unsigned int)MSB << 8) + LSB; // rearrange int to account for endian difference (TODO: check)
}

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
      
      IRDATA[i] = (PIX_MSB << 8) + PIX_LSB;
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
  return _sensor_write_check(CMD_SENSOR_WRITE_TRIM, 0xAA, e_read(EEPROM_TRIMMING_VAL), 0x00);
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
      //EEPROM_DATA[i] = b;
      e_write(i, b);
      i++;
    }
  }

  if (i < EEPROM_SIZE) { // If we didn't get the whole EEPROM
    return false;
  }

  EEPROM_DUMPED = true;
  return true;
}

void calculate_init() {
  v_th = (e_read(EEPROM_V_TH_H) <<8) + e_read(EEPROM_V_TH_L);
  k_t1 = ((e_read(EEPROM_K_T1_H) <<8) + e_read(EEPROM_K_T1_L))/1024.0;
  k_t2 =((e_read(EEPROM_K_T2_H) <<8) + e_read(EEPROM_K_T2_L))/1048576.0;

  a_cp = e_read(EEPROM_A_CP);
  if(a_cp > 127){
    a_cp = a_cp - 256;
  }
  b_cp = e_read(EEPROM_B_CP);
  if(b_cp > 127){
    b_cp = b_cp - 256;
  }
  tgc = e_read(EEPROM_TGC);
  if(tgc > 127){
    tgc = tgc - 256;
  }

  b_i_scale = e_read(EEPROM_B_I_SCALE);

  emissivity = (((unsigned int)e_read(EEPROM_EPSILON_H) << 8) + e_read(EEPROM_EPSILON_L))/32768.0;

  da0_scale = pow(2, -e_read(EEPROM_DELTA_ALPHA_SCALE));
  alpha_const = (float)(((unsigned int)e_read(EEPROM_ALPHA_0_H) << 8) + (unsigned int)e_read(EEPROM_ALPHA_0_L)) * pow(2, -e_read(EEPROM_ALPHA_0_SCALE));

  for (int i=0; i<64; i++){
    float alpha_var = (float)e_read(EEPROM_DELTA_ALPHA_00 + i) * da0_scale;
    alpha_ij[i] = (alpha_const + alpha_var);

    a_ij[i] = e_read(EEPROM_A_I_00 + i);
    if(a_ij[i] > 127){
      a_ij[i] = a_ij[i] - 256;
    }

    b_ij[i] = e_read(EEPROM_B_I_00 + i);
    if(b_ij[i] > 127){
      b_ij[i] = b_ij[i] - 256;
    }
  }
}

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

void calculate_temp() {
  float v_cp_off_comp = (float) CPIX - (a_cp + (b_cp/pow(2, b_i_scale)) * (ta - 25)); //this is needed only during the to calculation, so I declare it here.
  
  //Serial.print("IRCLEAN ");
  for (int i=0; i<64; i++){
    float alpha_ij_v = alpha_ij[i];
    int a_ij_v = a_ij[i];
    int b_ij_v = b_ij[i];

    float v_ir_tgc_comp = IRDATA[i] - (a_ij_v + (float)(b_ij_v/pow(2, b_i_scale)) * (ta - 25)) - (((float)tgc/32)*v_cp_off_comp);
    float v_ir_comp = v_ir_tgc_comp / emissivity;                  //removed to save SRAM, since emissivity in my case is equal to 1. 
    temp[i] = sqrt(sqrt((v_ir_comp/alpha_ij_v) + pow((ta + 273.15),4))) - 273.15;
    //Serial.print(temp);
    //Serial.print("\t");
  }

  //Serial.println();
}


void print_eeprom() {
  Serial.print("EEPROM ");
  for(int i = 0; i < EEPROM_SIZE; i++) {
    print_hex(e_read(i));
  }
  Serial.println();
}


void ta_loop() {
  ta = calculate_ta();
}

void por_loop() {
  if (!sensor_read_por()) { // there has been a reset
    assert(sensor_write_conf());
  }
}

void ir_loop() {
  assert(sensor_read_irdata());

  CPIX = sensor_read_cpix();
  assert(CPIX != -1);

  calculate_temp();

  Serial.println("START");

  Serial.print("IRCLEAN ");
  for(int i = 0; i<64; i++) {
    Serial.print(temp[i]);
    Serial.print("\t");
  }
  Serial.println();

 Serial.println("STOP");
}


SimpleTimer timer;

void setup() {
  Wire.begin(); // join i2c bus (address optional for master)
  Serial.begin(115200);

  Serial.println("INIT");

  Serial.println("DRIVER MLX90620");
  Serial.print("BUILD ");
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.println(__TIME__);

  assert(eeprom_read_all());
  assert(sensor_write_trim());
  assert(sensor_write_conf());

  calculate_init();

  ta = calculate_ta();


  float hz = REFRESH_FREQ;

  if (REFRESH_FREQ == 0) {
    hz = 0.5;
  }

  long irlen = (1/hz) * 1000;
  long talen = (1/2.0) * 1000;

  if (talen < irlen) {
    talen = irlen;
  }

  timer.setInterval(irlen, ir_loop);
  timer.setInterval(talen, ta_loop);
  timer.setInterval(2000, por_loop);

  Serial.println("ACTIVE");
}


void loop() {
  timer.run();
}