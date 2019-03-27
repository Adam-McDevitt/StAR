
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
#include <MotorPIDs.h>

// CONSTATSFOR ROTARY ENCODER
#define ROTARY_SLAVE_ADDRESS 5;
#define PAST 150;
MotorPIDs* test=new MotorPIDs();

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
#define DEFAULT_MOTOR_SPEED 4.5

//--------------------Globals for motor control-------------------

//PID SHIT
float Kp = 2.0/3500.0;
float Ki=0;
float Kd = 2.0/35000.0;
float pasts [PAST] ={0.0};
int ERROR_calib = 3500;

int past_counter=0;

float update_past(float new_error){
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

double range_left = -3500;
double range_right = 3500;

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
QTRSensors qtrrc0;
QTRSensors qtrrc1;
QTRSensors qtrrc2;
QTRSensors qtrrc3;
QTRSensors qtrrcs[4];
unsigned int sensorValues[NUM_BOARDS][NUM_SENSORS];

//-------------------------------------------------------------------------------
//                             PID FUNCTIONS
//-------------------------------------------------------------------------------
void resetPID(){
  line_new_error=0;
  line_last_error=0;
  clear_past();
}

void updateForwardPID(){
  double motors_speed_current = motors_speed;
  line_new_error = sensor_fwd_reading;
  mottor_adjustment = Kp * line_new_error + Kd * (line_new_error - line_last_error)+Ki*update_past(line_new_error);
  line_last_error = line_new_error;
  motors_speed = mottor_adjustment;

  // make sure you don't go over 100
  if (motors_speed > 100) {
    motors_speed = 100;
  }
  else if (motors_speed < -100) {
    motors_speed = -100;
  }
  Serial.print("Kp: ");
  Serial.print(Kp);
  Serial.print(", ");
  Serial.print("Kd: ");
  Serial.print(Kd);
//  Serial.print(", ");
//  Serial.print("Ki: ");
//  Serial.println(Ki);
  Serial.println(); 
  Serial.print("error = ");
  Serial.println(line_new_error);
  Serial.print("motors_speed = ");
  Serial.println(motors_speed);
  Serial.println();
  Serial.println();
}


void updateLeftPID(){
  double motors_speed_current = motors_speed;
  line_new_error = sensor_lft_reading;
  mottor_adjustment = Kp * line_new_error + Kd * (line_new_error - line_last_error)+Ki*update_past(line_new_error);
  line_last_error = line_new_error;
  motors_speed = mottor_adjustment;

  // make sure you don't go over 100
  if (motors_speed > 100) {
    motors_speed = 100;
  }
  else if (motors_speed < -100) {
    motors_speed = -100;
  }
  Serial.print("Kp: ");
  Serial.print(Kp);
  Serial.print(", ");
  Serial.print("Kd: ");
  Serial.print(Kd);
//  Serial.print(", ");
//  Serial.print("Ki: ");
//  Serial.println(Ki);
  Serial.println(); 
  Serial.print("error = ");
  Serial.println(line_new_error);
  Serial.print("motors_speed = ");
  Serial.println(motors_speed);
  Serial.println();
  Serial.println();
}

void updateRightPID(){
  double motors_speed_current = motors_speed;
  line_new_error = sensor_rgh_reading;
  mottor_adjustment = Kp * line_new_error + Kd * (line_new_error - line_last_error)+Ki*update_past(line_new_error);
  line_last_error = line_new_error;
  motors_speed = mottor_adjustment;

  // make sure you don't go over 100
  if (motors_speed > 100) {
    motors_speed = 100;
  }
  else if (motors_speed < -100) {
    motors_speed = -100;
  }
  Serial.print("Kp: ");
  Serial.print(Kp);
  Serial.print(", ");
  Serial.print("Kd: ");
  Serial.print(Kd);
//  Serial.print(", ");
//  Serial.print("Ki: ");
//  Serial.println(Ki);
  Serial.println(); 
  Serial.print("error = ");
  Serial.println(line_new_error);
  Serial.print("motors_speed = ");
  Serial.println(motors_speed);
  Serial.println();
  Serial.println();
}

void updateBackPID(){
  double motors_speed_current = motors_speed;
  line_new_error = sensor_bck_reading;
  mottor_adjustment = Kp * line_new_error + Kd * (line_new_error - line_last_error)+Ki*update_past(line_new_error);
  line_last_error = line_new_error;
  motors_speed = mottor_adjustment;

  // make sure you don't go over 100
  if (motors_speed > 100) {
    motors_speed = 100;
  }
  else if (motors_speed < -100) {
    motors_speed = -100;
  }
  Serial.print("Kp: ");
  Serial.print(Kp);
  Serial.print(", ");
  Serial.print("Kd: ");
  Serial.print(Kd);
//  Serial.print(", ");
//  Serial.print("Ki: ");
//  Serial.println(Ki);
  Serial.println(); 
  Serial.print("error = ");
  Serial.println(line_new_error);
  Serial.print("motors_speed = ");
  Serial.println(motors_speed);
  Serial.println();
  Serial.println();
}
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
  qtrrc0.setTypeRC();
  qtrrc0.setSensorPins((const uint8_t[]) {
    52, 53, 51, 49, 47, 45, 43, 41
  },
  NUM_SENSORS);
  qtrrc0.setEmitterPin(EMITTER_PIN0);

  qtrrc1.setTypeRC();
  qtrrc1.setSensorPins((const uint8_t[]) {
    6, 7, 8, 9, 10, 11, 12, 13
  },
  NUM_SENSORS);
  qtrrc1.setEmitterPin(EMITTER_PIN1);

  qtrrc2.setTypeRC();
  qtrrc2.setSensorPins((const uint8_t[]) {
    37, 35, 33, 31, 29, 27, 25, 23
  },
  NUM_SENSORS);
  qtrrc2.setEmitterPin(EMITTER_PIN2);

  qtrrc3.setTypeRC();
  qtrrc3.setSensorPins((const uint8_t[]) {
    36, 34, 32, 30, 28, 26, 24, 22
  },
  NUM_SENSORS);
  qtrrc3.setEmitterPin(EMITTER_PIN3);

  
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
  qtrrcs[0].calibrationOn.minimum[0] = 156;
  qtrrcs[0].calibrationOn.minimum[1] = 156;
  qtrrcs[0].calibrationOn.minimum[2] = 152;
  qtrrcs[0].calibrationOn.minimum[3] = 156;
  qtrrcs[0].calibrationOn.minimum[4] = 204;
  qtrrcs[0].calibrationOn.minimum[5] = 204;
  qtrrcs[0].calibrationOn.minimum[6] = 208;
  qtrrcs[0].calibrationOn.minimum[7] = 260;

  qtrrcs[1].calibrationOn.minimum[0] = 340;
  qtrrcs[1].calibrationOn.minimum[1] = 272;
  qtrrcs[1].calibrationOn.minimum[2] = 340;
  qtrrcs[1].calibrationOn.minimum[3] = 340;
  qtrrcs[1].calibrationOn.minimum[4] = 340;
  qtrrcs[1].calibrationOn.minimum[5] = 340;
  qtrrcs[1].calibrationOn.minimum[6] = 404;
  qtrrcs[1].calibrationOn.minimum[7] = 544;

  qtrrcs[2].calibrationOn.minimum[0] = 148;
  qtrrcs[2].calibrationOn.minimum[1] = 144;
  qtrrcs[2].calibrationOn.minimum[2] = 144;
  qtrrcs[2].calibrationOn.minimum[3] = 144;
  qtrrcs[2].calibrationOn.minimum[4] = 144;
  qtrrcs[2].calibrationOn.minimum[5] = 192;
  qtrrcs[2].calibrationOn.minimum[6] = 148;
  qtrrcs[2].calibrationOn.minimum[7] = 196;

  qtrrcs[3].calibrationOn.minimum[0] = 56;
  qtrrcs[3].calibrationOn.minimum[1] = 52;
  qtrrcs[3].calibrationOn.minimum[2] = 52;
  qtrrcs[3].calibrationOn.minimum[3] = 52;
  qtrrcs[3].calibrationOn.minimum[4] = 52;
  qtrrcs[3].calibrationOn.minimum[5] = 52;
  qtrrcs[3].calibrationOn.minimum[6] = 56;
  qtrrcs[3].calibrationOn.minimum[7] = 56;

  qtrrcs[0].calibrationOn.maximum[0] = 2500;
  qtrrcs[0].calibrationOn.maximum[1] = 2500;
  qtrrcs[0].calibrationOn.maximum[2] = 2500;
  qtrrcs[0].calibrationOn.maximum[3] = 2500;
  qtrrcs[0].calibrationOn.maximum[4] = 2500;
  qtrrcs[0].calibrationOn.maximum[5] = 2500;
  qtrrcs[0].calibrationOn.maximum[6] = 2500;
  qtrrcs[0].calibrationOn.maximum[7] = 2500;

  qtrrcs[1].calibrationOn.maximum[0] = 2500;
  qtrrcs[1].calibrationOn.maximum[1] = 2500;
  qtrrcs[1].calibrationOn.maximum[2] = 2500;
  qtrrcs[1].calibrationOn.maximum[3] = 2500;
  qtrrcs[1].calibrationOn.maximum[4] = 2500;
  qtrrcs[1].calibrationOn.maximum[5] = 2500;
  qtrrcs[1].calibrationOn.maximum[6] = 2500;
  qtrrcs[1].calibrationOn.maximum[7] = 2500;

  qtrrcs[2].calibrationOn.maximum[0] = 2500;
  qtrrcs[2].calibrationOn.maximum[1] = 2500;
  qtrrcs[2].calibrationOn.maximum[2] = 2500;
  qtrrcs[2].calibrationOn.maximum[3] = 2500;
  qtrrcs[2].calibrationOn.maximum[4] = 2500;
  qtrrcs[2].calibrationOn.maximum[5] = 2500;
  qtrrcs[2].calibrationOn.maximum[6] = 2500;
  qtrrcs[2].calibrationOn.maximum[7] = 2500;

  qtrrcs[3].calibrationOn.maximum[0] = 2372;
  qtrrcs[3].calibrationOn.maximum[1] = 1760;
  qtrrcs[3].calibrationOn.maximum[2] = 1380;
  qtrrcs[3].calibrationOn.maximum[3] = 1492;
  qtrrcs[3].calibrationOn.maximum[4] = 1444;
  qtrrcs[3].calibrationOn.maximum[5] = 2320;
  qtrrcs[3].calibrationOn.maximum[6] = 1512;
  qtrrcs[3].calibrationOn.maximum[7] = 1800;

  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  // SETTING UP ROTARY ENCODER
  test->MotorPIDsinit();
  test->setup_PID(4,20,0.8,2);//motor,Kp,Ki,Kd
  test->setup_PID(2,20,0.8,2);//motor,Kp,Ki,Kd
  test->setup_PID(1,20,0.8,2);//motor,Kp,Ki,Kd
  test->setup_PID(3,20,0.8,2);//motor,Kp,Ki,Kd

  Serial.println("Setup is DONE !!!"); 
}
  
void loop() {
  


  //------------------- Sensors values ---------------

  // FRONT
   int position_front = qtrrcs[2].readLineBlack(sensorValues[2]) - ERROR_calib;
//  Serial.print(position_front); // comment this line out if you are using raw values
//  Serial.print(' ');

  
   int position_right_raw = qtrrcs[0].readLineBlack(sensorValues[0]) - ERROR_calib;
   int position_right = -position_right_raw;
//  Serial.print(position_right); // comment this line out if you are using raw values
//  Serial.print(' ');
  // BACK
   int position_back = -(qtrrcs[1].readLineBlack(sensorValues[1]) - ERROR_calib);
//  Serial.print(position_back); // comment this line out if you are using raw values
//  Serial.print(' ');
  //LEFT
   int position_left = qtrrcs[3].readLineBlack(sensorValues[3]) - ERROR_calib;
//  Serial.print(position_left); // comment this line out if you are using raw values
//  Serial.print(' ');
//  Serial.println(' ');

  sensor_fwd_reading = position_front;
  sensor_rgh_reading = position_right;
  sensor_lft_reading = position_left;
  sensor_bck_reading = position_back;

  updateForwardPID();
//  test->setSetpoint(2,-1*DEFAULT_MOTOR_SPEED+0.5*abs(motors_speed));
//  test->setSetpoint(4, -1*DEFAULT_MOTOR_SPEED+0.5*abs(motors_speed));
  test->setSetpoint(1,-1*motors_speed);
  test->setSetpoint(3,motors_speed);

//  test->update_motor_PID(2);
//  test->update_motor_PID(4);
  motorBackward(2,80);
  motorBackward(4,80);
  test->update_motor_PID(1);
  test->update_motor_PID(3);


}
