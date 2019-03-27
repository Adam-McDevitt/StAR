
/*

  - script used for line following
  - uses an fsm to function
  - used for debugging
  - author: ilie, pieris

  Motor movement directions

              + | -

  motor 1:    <-|->
  motor 2:    fw|bw
  motor 3:    <-|->
  motor 4:    fw|bw


*/


// Libraries used for motor movement
#include <SDPArduino.h>
#include <Wire.h>
// #include <Rotary.h>

// Libraries used for sesnor control
#include <QTRSensors.h>

// Variables for Sensors
#define HARDCODE_CALIBRATION false
#define NUM_SENSORS    8     // number of sensors used
#define NUM_BOARDS     4        // number of boards
#define TIMEOUT        2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN0   50
#define EMITTER_PIN1   5
#define EMITTER_PIN2   39
#define EMITTER_PIN3   38
#define PAST           30

//--------------------Globals for motor control-------------------

//PID SHIT
float Kp = 0.01;
float Ki=20;
float Kd = 0.05;
float pasts [PAST] ={0.0};
int ERROR_calib = 3500;

int past_counter=0;

float update_past(int counter,float new_error){
 float sum=0;
 for(int i=0;i<PAST;i++){
   sum+=pasts[i];
 }
 pasts[past_counter]=new_error;
 past_counter= (past_counter+1)%PAST;
 return sum;
 }
 
void clear_past(){
  float sum=0;
  for(int i=0;i<PAST;i++){
    pasts[i]=0;
  }
 past_counter=0;
 }
 

double motors_speed = 0;      // global used for speed of right mototrs
double line_last_error    = 0;      // global that holds last error from fwd sensors
double line_new_error     = 0;
double mottor_adjustment  = 0;      // global used to readjust mottor speed

double right_motors_default_speed = 50;    // global used as default speed of right motors
double left_motors_default_speed  = 50 ;   // global used as default speed of left motors
double right_max_speed = 100   ;           // max speed of right mototrs
double left_max_speed  = 100  ;            // max speed of left motors

double range_left = -1000;
double range_right = 1000;

double motor1_current = 0;
double motor2_current = 0;
double motor3_current = 0;
double motor4_current = 0;

double motor1_new = 0;
double motor2_new = 0;
double motor3_new = 0;
double motor4_new = 0;

int state;                                  //state variable
int state_jump = 0;
// DEBUGGING
int counter = 0;

int sensor_fwd_reading = 0;
int sensor_rgh_reading = 0;
int sensor_lft_reading = 0;
int sensor_bck_reading = 0;

int thresh_turn_pow = 55;
int thresh_fwd_pow  = 50;

int fwd_counter_left  = 0;
int fwd_counter_right = 0;

int user_input = 0;
int input;

bool detect_left = false;
bool detect_right= false;
bool detect_forward=false;


//--------------------Globals for Sensor control-------------------


// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc0((unsigned char[]) {
  52, 53, 51, 49, 47, 45, 43, 41
},
NUM_SENSORS, TIMEOUT, EMITTER_PIN0);

QTRSensorsRC qtrrc1((unsigned char[]) {
  6, 7, 8, 9, 10, 11, 12, 13
},
NUM_SENSORS, TIMEOUT, EMITTER_PIN1);

QTRSensorsRC qtrrc2((unsigned char[]) {
  37, 35, 33, 31, 29, 27, 25, 23
},
NUM_SENSORS, TIMEOUT, EMITTER_PIN2);

QTRSensorsRC qtrrc3((unsigned char[]) {
  36, 34, 32, 30, 28, 26, 24, 22
},
NUM_SENSORS, TIMEOUT, EMITTER_PIN3);

QTRSensorsRC qtrrcs[4];
unsigned int sensorValues[NUM_BOARDS][NUM_SENSORS];


//-------------------------------------------------------------------------------
//                           Movement functions
//-------------------------------------------------------------------------------
void motor_move(int id, double speed) {
  Serial.println("INSIDE motor_move");
  if (speed >= 0) {
    motorForward(id, speed);
    // Serial.print("FWD: id ");
    // Serial.println(id);
    // Serial.print("FWD: speed ");
    // Serial.println(speed);
  }
  else {
    motorBackward(id, abs(speed));
    // Serial.print("BKW: id ");
    // Serial.println(id);
    // Serial.print("BKW: speed ");
    // Serial.println(speed);
  }
  // delay(500);
}



bool check_change_forward() {
  if ((motor2_new >= 0 && motor2_current < 0) || (motor2_current >= 0 && motor2_new < 0)) {
    motor2_current = motor2_new;
    motor4_current = motor4_new;
    return true;
  }
  else if ((abs(abs(motor2_current) - abs(motor2_new)) > 8) || (abs(abs(motor4_current) - abs(motor4_new)) > 8)) {
    motor2_current = motor2_new;
    motor4_current = motor4_new;
    return true;
  }
  else {
    return false;
  }
}

bool check_change_horizontal() {
  if ((motor1_new >= 0 && motor1_current < 0) || (motor3_current >= 0 && motor3_new < 0)) {
    motor1_current = motor1_new;
    motor3_current = motor3_new;
    return true;
  }
  else if ((abs(abs(motor1_current) - abs(motor1_new)) > 8) || (abs(abs(motor3_current) - abs(motor3_new)) > 8)) {
    motor1_current = motor1_new;
    motor3_current = motor3_new;
    return true;
  }
  else {
    return false;
  }
}


bool check_change_rotation() {
  if ((motor1_new >= 0 && motor1_current < 0) || (motor1_current >= 0 && motor1_new < 0)) {
    motor1_current = motor1_new;
    motor3_current = motor3_new;
    return true;
  }
  else if ((abs(abs(motor1_current) - abs(motor1_new)) > 8) || (abs(abs(motor3_current) - abs(motor3_new)) > 8)) {
    motor1_current = motor1_new;
    motor3_current = motor3_new;
    return true;
  }
  else {
    return false;
  }
}

/**
   Function stops all motors
   Updateds new and current globals for movement
*/
void stopAllMotors() {
  motorAllStop();

  motor2_current = 0;
  motor4_current = 0;

  motor1_current = 0;
  motor3_current = 0;

  motor2_new = 0;
  motor4_new = 0;

  motor1_new = 0;
  motor3_new = 0;
}

void stopHorizontal() {
  motorStop(1);
  motorStop(3);

  motor1_current = 0;
  motor3_current = 0;

  motor1_new = 0;
  motor3_new = 0;
}

void stopForward() {
  motorStop(2);
  motorStop(4);

  motor2_current = 0;
  motor4_current = 0;

  motor2_new = 0;
  motor4_new = 0;
}

/**
   Function moves motors forward
   Updateds new and current globals for movement
*/
void move_fwd(double power) {
  stopHorizontal();
  motor_move(2, power);
  motor_move(4, power);
  motor2_current = power;
  motor4_current = power;
}


//-------------------------------------------------------------------------------
//                           SETUP FUNCTION
//-------------------------------------------------------------------------------

void setup() {

  //---------------------- Setting up the motors-----------------

  Serial.begin(9600);  // Serial at given baudrate

  Serial.println("Serial Print Initialized");

  SDPsetup();
  Wire.begin();  // Master of the I2C bus


  // --------------------  Setting up the sensors ---------------
  delay(500);
  pinMode(13, OUTPUT);

  digitalWrite(13, HIGH);                 // turn on Arduino's LED to indicate we are in calibration mode

  qtrrcs[0] = qtrrc0;   // FRONT
  qtrrcs[1] = qtrrc1;   // RIGHT
  qtrrcs[2] = qtrrc2;   // BACK
  qtrrcs[3] = qtrrc3;   // LEFT
  Serial.println("HELLO1");
  for (int i = 0 ; i < NUM_BOARDS ; i++) {
    qtrrcs[i].calibrate();                  // to initialise calibration arrays

  }

  // INSERT HARDCODED VALUES NOW
  qtrrcs[0].calibratedMinimumOn[0] = 156;
  qtrrcs[0].calibratedMinimumOn[1] = 156;
  qtrrcs[0].calibratedMinimumOn[2] = 152;
  qtrrcs[0].calibratedMinimumOn[3] = 156;
  qtrrcs[0].calibratedMinimumOn[4] = 204;
  qtrrcs[0].calibratedMinimumOn[5] = 204;
  qtrrcs[0].calibratedMinimumOn[6] = 208;
  qtrrcs[0].calibratedMinimumOn[7] = 260;

  qtrrcs[1].calibratedMinimumOn[0] = 340;
  qtrrcs[1].calibratedMinimumOn[1] = 272;
  qtrrcs[1].calibratedMinimumOn[2] = 340;
  qtrrcs[1].calibratedMinimumOn[3] = 340;
  qtrrcs[1].calibratedMinimumOn[4] = 340;
  qtrrcs[1].calibratedMinimumOn[5] = 340;
  qtrrcs[1].calibratedMinimumOn[6] = 404;
  qtrrcs[1].calibratedMinimumOn[7] = 544;

  qtrrcs[2].calibratedMinimumOn[0] = 148;
  qtrrcs[2].calibratedMinimumOn[1] = 144;
  qtrrcs[2].calibratedMinimumOn[2] = 144;
  qtrrcs[2].calibratedMinimumOn[3] = 144;
  qtrrcs[2].calibratedMinimumOn[4] = 144;
  qtrrcs[2].calibratedMinimumOn[5] = 192;
  qtrrcs[2].calibratedMinimumOn[6] = 148;
  qtrrcs[2].calibratedMinimumOn[7] = 196;

  qtrrcs[3].calibratedMinimumOn[0] = 56;
  qtrrcs[3].calibratedMinimumOn[1] = 52;
  qtrrcs[3].calibratedMinimumOn[2] = 52;
  qtrrcs[3].calibratedMinimumOn[3] = 52;
  qtrrcs[3].calibratedMinimumOn[4] = 52;
  qtrrcs[3].calibratedMinimumOn[5] = 52;
  qtrrcs[3].calibratedMinimumOn[6] = 56;
  qtrrcs[3].calibratedMinimumOn[7] = 56;

  qtrrcs[0].calibratedMaximumOn[0] = 2500;
  qtrrcs[0].calibratedMaximumOn[1] = 2500;
  qtrrcs[0].calibratedMaximumOn[2] = 2500;
  qtrrcs[0].calibratedMaximumOn[3] = 2500;
  qtrrcs[0].calibratedMaximumOn[4] = 2500;
  qtrrcs[0].calibratedMaximumOn[5] = 2500;
  qtrrcs[0].calibratedMaximumOn[6] = 2500;
  qtrrcs[0].calibratedMaximumOn[7] = 2500;

  qtrrcs[1].calibratedMaximumOn[0] = 2500;
  qtrrcs[1].calibratedMaximumOn[1] = 2500;
  qtrrcs[1].calibratedMaximumOn[2] = 2500;
  qtrrcs[1].calibratedMaximumOn[3] = 2500;
  qtrrcs[1].calibratedMaximumOn[4] = 2500;
  qtrrcs[1].calibratedMaximumOn[5] = 2500;
  qtrrcs[1].calibratedMaximumOn[6] = 2500;
  qtrrcs[1].calibratedMaximumOn[7] = 2500;

  qtrrcs[2].calibratedMaximumOn[0] = 2500;
  qtrrcs[2].calibratedMaximumOn[1] = 2500;
  qtrrcs[2].calibratedMaximumOn[2] = 2500;
  qtrrcs[2].calibratedMaximumOn[3] = 2500;
  qtrrcs[2].calibratedMaximumOn[4] = 2500;
  qtrrcs[2].calibratedMaximumOn[5] = 2500;
  qtrrcs[2].calibratedMaximumOn[6] = 2500;
  qtrrcs[2].calibratedMaximumOn[7] = 2500;

  qtrrcs[3].calibratedMaximumOn[0] = 2372;
  qtrrcs[3].calibratedMaximumOn[1] = 1760;
  qtrrcs[3].calibratedMaximumOn[2] = 1380;
  qtrrcs[3].calibratedMaximumOn[3] = 1492;
  qtrrcs[3].calibratedMaximumOn[4] = 1444;
  qtrrcs[3].calibratedMaximumOn[5] = 2320;
  qtrrcs[3].calibratedMaximumOn[6] = 1512;
  qtrrcs[3].calibratedMaximumOn[7] = 1800;

  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration
  state = 1;
  // delay(1000); // Evil
}

void loop() {

  //------------------- Sensors values ---------------

  // FRONT
  unsigned int position_front = qtrrcs[0].readLine(sensorValues[0]);
  Serial.print(position_front); // comment this line out if you are using raw values
  Serial.print(' ');
  // RIGHT
  unsigned int position_right_raw = qtrrcs[1].readLine(sensorValues[1]);
  unsigned int position_right = 7000 - position_right_raw;
  Serial.print(position_right); // comment this line out if you are using raw values
  Serial.print(' ');
  // BACK
  unsigned int position_back = qtrrcs[2].readLine(sensorValues[2]);
  Serial.print(position_back); // comment this line out if you are using raw values
  Serial.print(' ');
  //LEFT
  unsigned int position_left = qtrrcs[3].readLine(sensorValues[3]);
  Serial.print(position_left); // comment this line out if you are using raw values
  Serial.print(' ');
  Serial.println(' ');
  delay(50);
  //------------------- Motors FSM -------------------
  sensor_fwd_reading = position_front;
  sensor_rgh_reading = position_right;
  sensor_lft_reading = position_left;
  sensor_bck_reading = position_back;


  //PID FOR FORWARD
  double motors_speed_current = motors_speed;
  line_new_error = sensor_fwd_reading - ERROR_calib;
  mottor_adjustment = Kp * line_new_error + Kd * (line_new_error - line_last_error);
  line_last_error = line_new_error;
  motors_speed = mottor_adjustment;

  // make sure you don't go over 100
  if (motors_speed > 100) {
    motors_speed = 100;
  }
  else if (motors_speed < -100) {
    motors_speed = -100;
  }

