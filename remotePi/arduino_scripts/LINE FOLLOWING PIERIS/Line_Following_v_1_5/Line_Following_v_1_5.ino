
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

//--------------------Globals for motor control-------------------

double Kp = 0.01;
// double Ki;
double Kd = 0.05;
int ERROR_calib = 3500;

double motors_speed = 0;      // global used for speed of right mototrs
double line_last_error    = 0;      // global that holds last error from fwd sensors
double line_new_error     = 0;
double mottor_adjustment  = 0;      // global used to readjust mottor speed

double right_motors_default_speed = 50;    // global used as default speed of right motors
double left_motors_default_speed  = 50 ;   // global used as default speed of left motors
double right_max_speed = 100   ;           // max speed of right mototrs
double left_max_speed  = 100  ;            // max speed of left motors

double range_left = 2500;
double range_right = 4500;

double motor1_current = 0;
double motor2_current = 0;
double motor3_current = 0;
double motor4_current = 0;

double motor1_new = 0;
double motor2_new = 0;
double motor3_new = 0;
double motor4_new = 0;

// DEBUGGING
int counter = 0;

int sensor_fwd_reading = 0;
int sensor_rgh_reading = 0;
int sensor_lft_reading = 0;
int sensor_bck_reading = 0;

int thresh_turn_pow = 55;
int thresh_fwd_pow  = 60;

int fwd_counter_left  = 0;
int fwd_counter_right = 0;

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
unsigned int sensorValues[NUM_SENSORS];


//-------------------------------------------------------------------------------
//                           Movement functions
//-------------------------------------------------------------------------------
void motor_move(int id, double speed) {
  Serial.println("INSIDE motor_move");
  if (speed >= 0) {
    motorForward(id, speed);
    Serial.print("FWD: id ");
    Serial.println(id);
    Serial.print("FWD: speed ");
    Serial.println(speed);
  }
  else {
    motorBackward(id, abs(speed));
    Serial.print("BKW: id ");
    Serial.println(id);
    Serial.print("BKW: speed ");
    Serial.println(speed);
  }
  // delay(500);
}

void turn_left(int speed_input) {
  // Local vars
  double speed_input_horizontal;
  // make sure you don't go over 100
  if ((thresh_turn_pow + abs(speed_input)) > 100) {
    speed_input_horizontal = 100;
  }
  else {
    speed_input_horizontal = abs(speed_input) + thresh_turn_pow;
  }

  motor1_new = speed_input_horizontal;
  motor3_new = speed_input_horizontal;

  if (check_change_rotation() == true) {

    //Serial.println("ACTUALLY TURNING LEFT WITH SPEED:",speed_input_horizontal)
    motor_move(1,  -1 * speed_input_horizontal);
    motor_move(3,  1 * speed_input_horizontal);

    motor1_current =  -1 * speed_input_horizontal;
    motor3_current =  1 * speed_input_horizontal;
  }
}



void turn_right(int speed_input) {
  // Local vars
  double speed_input_horizontal;
  // make sure you don't go over 100
  if ((thresh_turn_pow + abs(speed_input)) > 100) {
    speed_input_horizontal = 100;
  }
  else {
    speed_input_horizontal = abs(speed_input) + thresh_turn_pow;
  }

  motor1_new = speed_input_horizontal;
  motor3_new = speed_input_horizontal;

  if (check_change_rotation() == true) {
    //Serial.println("ACTUALLY TURNING RIGHT WITH SPEED:",speed_input_horizontal)
    motor_move(1, 1 * speed_input_horizontal);
    motor_move(3, -1 * speed_input_horizontal);

    motor1_current = 1 * speed_input_horizontal;
    motor3_current = -1 * speed_input_horizontal;
  }
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
  qtrrcs[0].calibratedMinimumOn[0] = 356;
  qtrrcs[0].calibratedMinimumOn[1] = 248;
  qtrrcs[0].calibratedMinimumOn[2] = 204;
  qtrrcs[0].calibratedMinimumOn[3] = 248;
  qtrrcs[0].calibratedMinimumOn[4] = 248;
  qtrrcs[0].calibratedMinimumOn[5] = 248;
  qtrrcs[0].calibratedMinimumOn[6] = 248;
  qtrrcs[0].calibratedMinimumOn[7] = 248;

  qtrrcs[1].calibratedMinimumOn[0] = 208;
  qtrrcs[1].calibratedMinimumOn[1] = 208;
  qtrrcs[1].calibratedMinimumOn[2] = 204;
  qtrrcs[1].calibratedMinimumOn[3] = 208;
  qtrrcs[1].calibratedMinimumOn[4] = 208;
  qtrrcs[1].calibratedMinimumOn[5] = 208;
  qtrrcs[1].calibratedMinimumOn[6] = 208;
  qtrrcs[1].calibratedMinimumOn[7] = 280;

  qtrrcs[2].calibratedMinimumOn[0] = 56;
  qtrrcs[2].calibratedMinimumOn[1] = 56;
  qtrrcs[2].calibratedMinimumOn[2] = 56;
  qtrrcs[2].calibratedMinimumOn[3] = 100;
  qtrrcs[2].calibratedMinimumOn[4] = 100;
  qtrrcs[2].calibratedMinimumOn[5] = 100;
  qtrrcs[2].calibratedMinimumOn[6] = 100;
  qtrrcs[2].calibratedMinimumOn[7] = 104;

  qtrrcs[3].calibratedMinimumOn[0] = 148;
  qtrrcs[3].calibratedMinimumOn[1] = 100;
  qtrrcs[3].calibratedMinimumOn[2] = 56;
  qtrrcs[3].calibratedMinimumOn[3] = 100;
  qtrrcs[3].calibratedMinimumOn[4] = 56;
  qtrrcs[3].calibratedMinimumOn[5] = 100;
  qtrrcs[3].calibratedMinimumOn[6] = 100;
  qtrrcs[3].calibratedMinimumOn[7] = 100;

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

  qtrrcs[3].calibratedMaximumOn[0] = 2304;
  qtrrcs[3].calibratedMaximumOn[1] = 1692;
  qtrrcs[3].calibratedMaximumOn[2] = 1392;
  qtrrcs[3].calibratedMaximumOn[3] = 1540;
  qtrrcs[3].calibratedMaximumOn[4] = 1492;
  qtrrcs[3].calibratedMaximumOn[5] = 1696;
  qtrrcs[3].calibratedMaximumOn[6] = 1488;
  qtrrcs[3].calibratedMaximumOn[7] = 2004;

  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration
  // delay(1000); // Evil
}

void loop() {

  //------------------- Sensors values ---------------

  // FRONT
  unsigned int position_front = qtrrcs[0].readLine(sensorValues);
  Serial.print(position_front); // comment this line out if you are using raw values
  Serial.print(' ');
  // // RIGHT
  unsigned int position_right = qtrrcs[1].readLine(sensorValues);
  Serial.print(position_right); // comment this line out if you are using raw values
  Serial.print(' ');
  // BACK
  unsigned int position_back = qtrrcs[2].readLine(sensorValues);
  Serial.print(position_back); // comment this line out if you are using raw values
  Serial.print(' ');
  // LEFT
  unsigned int position_left = qtrrcs[3].readLine(sensorValues);
  Serial.print(position_left); // comment this line out if you are using raw values
  Serial.println(' ');

  delay(50);
  //------------------- Motors FSM -------------------
  sensor_fwd_reading = position_front;
  sensor_rgh_reading = position_right;
  sensor_lft_reading = position_left;
  sensor_bck_reading = position_back;

  //calculate motor speed
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
  
  
  // motors_speed = abs(motors_speed);
  // stopAllMotors();
  // sensor_fwd_reading+=100;
  // Serial.print("sensor_fwd_reading ="  );
  // Serial.println(sensor_fwd_reading);
  // Serial.print("motors_speed = ");
  // Serial.println(motors_speed);
  // if(sensor_fwd_reading == 7000) {
  //   sensor_fwd_reading = 0;
  // }
  
  // stopAllMotors();
  

  //move motors
  if (sensor_fwd_reading < range_left) {
    stopForward();
    Serial.print("TURNING RIGHT WITH SPEED:  ");
    Serial.println(motors_speed);

    motor1_new = motors_speed;
    motor3_new = motors_speed;

    turn_right(motors_speed);
  }
  else if (sensor_fwd_reading > range_right) {
    stopForward();
    Serial.print("TURNING LEFT WITH SPEED: ");
    Serial.println(motors_speed);
    turn_left(motors_speed);
  }
  else if (sensor_fwd_reading >= range_left && sensor_fwd_reading <= range_right) {
    stopHorizontal();
    Serial.print("MOVING FORWARD with: ");
    
    if(sensor_fwd_reading <= 3500) {
      motor2_new = thresh_fwd_pow - motors_speed;
      motor4_new = thresh_fwd_pow + 1.5 * motors_speed;  
    } else if (sensor_fwd_reading > 3500) {
      motor2_new = thresh_fwd_pow + 1.5*motors_speed;
      motor4_new = thresh_fwd_pow - motors_speed;
    }
    
    // motor2_new = thresh_fwd_pow;
    // motor4_new = thresh_fwd_pow;
    
    Serial.println(motor2_new);

    if (check_change_forward() == true) {
      motorStop(1);
      motorStop(3);
      move_fwd(motor2_new);
      
      Serial.println("INSIDE FORWARD");
      Serial.println("\n\n\n\n\n\n\n\n\n\n\n\n\n");

      Serial.print("motor2_new");
      Serial.println(motor2_new);

      Serial.print("motor4_new");
      Serial.println(motor4_new);
    }

  }
  else {
    Serial.println("MOTORS DIDN'T WORK");
  }
  // go back to top (get sensor values)


  // delay(3000);
  // motorAllStop();
  // delay(5000);
}