/*
  Arduino Control Software Block & Motor Control Firmware
  Author: Felipe Orrico Scognamiglio
  For: Oregon State University - Junior Design - Final Project Spydercam18
  Date: 03/01/2021 
  Version: BETA 1.25 - FINAL
*/

/*
  Known Problems:
    - After M2 command, Incremental Coordinate mode will not work properly 
    and C1 will not be able to report correct coordinates
    - Inputting M2 and M6 right after the other has undefined behaviour
    - Current X, Y, Z positions are not available after receiving M2/M6, only 
    after another G0 or G1 is executed. This happens because it is hard to extrapolate
    the current position only based on current thread lengths. Instead, the program will
    update after another G0 or G1 command.
*/


//////////////////////////LCD SETUP//////////////////////////
#include <LiquidCrystal.h>
const int rs = 12, en = 8, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
//////////////////////////LCD SETUP//////////////////////////

/////////////////////////Sensor SETUP////////////////////////
#include <SoftwareSerial.h>
#include "PN532_SWHSU.h"
#include "PN532.h"

SoftwareSerial SWSerial( 10, 11 ); // RX, TX
PN532_SWHSU pn532swhsu( SWSerial );
PN532 nfc( pn532swhsu );

const int sensor = A0;

float sensorVal = 0;
float voltage = 0;
float lightLevel = 0;

/////////////////////////Sensor SETUP////////////////////////


//////////////////////////CONSTANTS//////////////////////////

//ALL PHYSICAL VALUES ARE REPORTED IN INCHES
const double A_PAPER_DISTANCE = 11.583;
const double B_PAPER_DISTANCE = 7.2;
const double C_PAPER_DISTANCE = 7.2;

const double HEIGHT_PYLON_A = 11.8;
const double HEIGHT_PYLON_B = 11.8;
const double HEIGHT_PYLON_C = 11.8;

/*const double PAYLOAD_CENTER_THREAD_DISTANCE_A_x = 3.6;
const double PAYLOAD_CENTER_THREAD_DISTANCE_C_x = 1.85;
const double PAYLOAD_CENTER_THREAD_DISTANCE_C_y = 3.15;
const double PAYLOAD_CENTER_THREAD_DISTANCE_B_x = 1.85;
const double PAYLOAD_CENTER_THREAD_DISTANCE_B_y = 3.15;*/

const double PAYLOAD_CENTER_THREAD_DISTANCE_A_x = 1.9;      // was 3.6 with triangle payload
const double PAYLOAD_CENTER_THREAD_DISTANCE_C_x = 1.36;     // was 1.85 with triangle payload
const double PAYLOAD_CENTER_THREAD_DISTANCE_C_y = 1.9;     // was 3.15 with triangle payload
const double PAYLOAD_CENTER_THREAD_DISTANCE_B_x = 1.36;     // was 1.85 with triangle payload
const double PAYLOAD_CENTER_THREAD_DISTANCE_B_y = 1.9;     // was 3.15 with triangle payload

const double PAPER_HEIGHT = 11.00;
const double PAPER_WIDTH = 8.50;

const double PAPER_DISTANCE_LOW_EDGE = 1.856;

const double HOME_X = 0;
const double HOME_Y = 0;
const double HOME_Z = 6;

const double INCHES_PER_STEP = 0.024662;
const double MAX_STEPS_PER_SEC = 182;

//////////////////////////CONSTANTS//////////////////////////

//////////////////////////VARIABLES//////////////////////////

double CURRENT_PAYLOAD_HEIGHT = 0;
double CURRENT_THREAD_LEN_A = 0;
double CURRENT_THREAD_LEN_B = 0;
double CURRENT_THREAD_LEN_C = 0;
double CURRENT_X = 0;
double CURRENT_Y = 0;

//values that are updated when there is movement
int OPCODE = -1;
double X_ADDRESS_TARGET = -1;
double Y_ADDRESS_TARGET = -1;
double Z_ADDRESS_TARGET = -1;
int F_PERCENT_SPEED = 100;

///M6///
double M6_SAVE_LEN_A = 0;
double M6_SAVE_LEN_B = 0;
double M6_SAVE_LEN_C = 0;
double M6_SAVE_X = -1;
double M6_SAVE_Y = -1;
double M6_SAVE_Z = -1;
double M6_SAVE_Speed = 0;
bool M6_EN = false;

//////////////////////////VARIABLES//////////////////////////

////////////////////////////FLAGS////////////////////////////

bool UNIT_MODE = true; //true = inches, false = mm;
bool ABSOLUTE_COORDINATES = true; //true = absolute, false = incremental
bool ON_MOVE = false;
bool M2_EN = false;

////////////////////////////FLAGS////////////////////////////

///////////////////////////STEPPER///////////////////////////

#include "AccelStepper.h"
#include "MultiStepper.h"

//change pins here to reflect correct pins
#define dirPinA 25
#define stepPinA 23
#define dirPinB 29
#define stepPinB 27
#define dirPinC 33
#define stepPinC 31

#define motorInterfaceType 1

AccelStepper stepperA;// = AccelStepper(motorInterfaceType, stepPinA, dirPinA);
AccelStepper stepperB;// = AccelStepper(motorInterfaceType, stepPinB, dirPinB);
AccelStepper stepperC;// = AccelStepper(motorInterfaceType, stepPinC, dirPinC);

MultiStepper steppers = MultiStepper();

///////////////////////////STEPPER///////////////////////////

///////////////////////G-CODE LIB SETUP//////////////////////
#include "gcode.h" 
#include "Arduino_Firmware_FINAL.h"
#define NumberOfCommands 10
//you can add more more commands here if needed and create a function under G-Code to execute.
struct commandscallback commands[NumberOfCommands] = {{"G0",G0_cmd},{"G1",G1_cmd},{"G20",G20_cmd},{"G21",G21_cmd},{"G90",G90_cmd},{"G91",G91_cmd},{"M2",M2_cmd},{"M6",M6_cmd},{"C1",C1_cmd},{"C2",C2_cmd}};
gcode Commands(NumberOfCommands,commands);
///////////////////////G-CODE LIB SETUP//////////////////////

///////////////////////////G-Code////////////////////////////

void G0_cmd(){

      if (ON_MOVE || M6_EN) {
        //Serial.println("ON_MOVE");
        return;  
      }

        
      double x_coord = CURRENT_X;
      double y_coord = CURRENT_Y;
      double z_coord = CURRENT_PAYLOAD_HEIGHT;

      double x_coord_old = CURRENT_X;
      double y_coord_old = CURRENT_Y;
      double z_coord_old = CURRENT_PAYLOAD_HEIGHT;


      if(Commands.availableValue('X')){
        x_coord = Commands.GetValue('X');
        if (!UNIT_MODE)
          x_coord = mm_to_in(x_coord);
        if (!ABSOLUTE_COORDINATES)
          x_coord += CURRENT_X;
      }
      if(Commands.availableValue('Y')){
        y_coord = Commands.GetValue('Y');
        if (!UNIT_MODE)
          y_coord = mm_to_in(y_coord);
        if (!ABSOLUTE_COORDINATES)
          y_coord += CURRENT_Y;
      }
      if(Commands.availableValue('Z')){
        z_coord = Commands.GetValue('Z');
        if (!UNIT_MODE)
          z_coord = mm_to_in(z_coord);
        if (!ABSOLUTE_COORDINATES)
          z_coord += CURRENT_Z;
      }
      
      int f_speed = 100; //maximum speed

      //checking for boundaries
      if (x_coord > 8.5)
        x_coord = 8.5;
      if (y_coord > 11)
        x_coord = 11;
      if (z_coord > 11)
        z_coord = 11;

      //update global values
      Y_ADDRESS_TARGET = y_coord;
      X_ADDRESS_TARGET = x_coord;
      Z_ADDRESS_TARGET = z_coord;
      F_PERCENT_SPEED = f_speed;
      
      update_lcd(0);
      
      go(x_coord,y_coord,z_coord, f_speed);
}

void G1_cmd(){

      if (ON_MOVE || M6_EN) return;
      
      double x_coord = CURRENT_X;
      double y_coord = CURRENT_Y;
      double z_coord = CURRENT_PAYLOAD_HEIGHT;
      double f_speed_in = 4.5;
      int f_speed = 100; //maximum speed

      if(Commands.availableValue('X')){
        x_coord = Commands.GetValue('X');
        if (!UNIT_MODE)
          x_coord = mm_to_in(x_coord);
        if (!ABSOLUTE_COORDINATES)
          x_coord += CURRENT_X;
      }
      if(Commands.availableValue('Y')){
        y_coord = Commands.GetValue('Y');
        if (!UNIT_MODE)
          y_coord = mm_to_in(y_coord);
        if (!ABSOLUTE_COORDINATES)
          y_coord += CURRENT_Y;
      }
      if(Commands.availableValue('Z')){
        z_coord = Commands.GetValue('Z');
        if (!UNIT_MODE)
          z_coord = mm_to_in(z_coord);
        if (!ABSOLUTE_COORDINATES)
          z_coord += CURRENT_Z;
      }
      if(Commands.availableValue('F')){
        f_speed_in = Commands.GetValue('F');
        if (!UNIT_MODE)
          f_speed_in = mm_to_in(f_speed_in);
      }
      
      //checking for boundaries
      if (x_coord > 8.5)
        x_coord = 8.5;
      if (y_coord > 11)
        x_coord = 11;
      if (z_coord > 11)
        z_coord = 11;
      
     f_speed_in = (f_speed_in/4.5)*100;

      if (f_speed_in > 100)
        f_speed = 100;
      else if (f_speed_in <= 1)
        f_speed = 1;
      else
        f_speed = int(f_speed_in);

      //update global values
      Y_ADDRESS_TARGET = y_coord;
      X_ADDRESS_TARGET = x_coord;
      Z_ADDRESS_TARGET = z_coord;
      F_PERCENT_SPEED = f_speed;

      update_lcd(1);

      go(x_coord, y_coord, z_coord, f_speed);
}

void G20_cmd(){

  if (ON_MOVE || M6_EN) return;
  
  UNIT_MODE = true;
  update_lcd(20);
}

void G21_cmd(){

  if (ON_MOVE || M6_EN) return;
  
  UNIT_MODE = false;
  update_lcd(21);
}

void G90_cmd(){

  if (ON_MOVE || M6_EN) return;
  
  ABSOLUTE_COORDINATES = true;
  update_lcd(90);
}

void G91_cmd(){

  if (ON_MOVE || M6_EN) return;
  
  ABSOLUTE_COORDINATES = false;
  update_lcd(91);
}

void M2_cmd(){
  update_lcd(2);
  if (ON_MOVE){
      stepperA.stop();
      stepperB.stop();
      stepperC.stop();
      
      ON_MOVE = false;
      ABSOLUTE_COORDINATES = true;
      M2_EN = true;
      M6_EN = false;
  }
}

void M6_cmd(){
  update_lcd(6);
  if (!M6_EN){ //m6 first time
    //stop

    //if (ON_MOVE){
    stepperA.stop();
    stepperB.stop();
    stepperC.stop();
    
    ON_MOVE = false;
    ABSOLUTE_COORDINATES = true;
    M6_EN = true;

    double actual_a = stepperA.currentPosition() * INCHES_PER_STEP;
    double actual_b = -(stepperB.currentPosition() * INCHES_PER_STEP);
    double actual_c = stepperC.currentPosition() * INCHES_PER_STEP;

    CURRENT_THREAD_LEN_A += actual_a;
    CURRENT_THREAD_LEN_B += actual_b;
    CURRENT_THREAD_LEN_C += actual_c;

    M6_SAVE_LEN_A = CURRENT_THREAD_LEN_A;
    M6_SAVE_LEN_B = CURRENT_THREAD_LEN_B;
    M6_SAVE_LEN_C = CURRENT_THREAD_LEN_C;
    M6_SAVE_X = X_ADDRESS_TARGET;
    M6_SAVE_Y = Y_ADDRESS_TARGET;
    M6_SAVE_Z = Z_ADDRESS_TARGET;
    
    if (M6_SAVE_X == -1 || M6_SAVE_Y == -1 || M6_SAVE_Z == -1){
      M6_SAVE_X = CURRENT_X;
      M6_SAVE_Y = CURRENT_Y;
      M6_SAVE_Z = CURRENT_PAYLOAD_HEIGHT;
    }
    if (M6_SAVE_X == 0 && M6_SAVE_Y == 0 && M6_SAVE_Z == 0){
      M6_SAVE_X = CURRENT_X;
      M6_SAVE_Y = CURRENT_Y;
      M6_SAVE_Z = CURRENT_PAYLOAD_HEIGHT;
    }
    M6_SAVE_Speed = F_PERCENT_SPEED;

    go_block(HOME_X,HOME_Y,HOME_Z, 100);
  }
  else {

    go_len_block(M6_SAVE_LEN_A,M6_SAVE_LEN_B,M6_SAVE_LEN_C, 100);

    M6_EN = false;
    
    //go(M6_SAVE_X,M6_SAVE_Y,M6_SAVE_Z,M6_SAVE_Speed);

    /*M6_SAVE_LEN_A = 0;
    M6_SAVE_LEN_B = 0;
    M6_SAVE_LEN_C = 0;
    M6_SAVE_X = -1;
    M6_SAVE_Y = -1;
    M6_SAVE_Z = -1;
    M6_SAVE_Speed = 0;*/
  }
}

void C1_cmd(){ //Write value of X,Y,Z to the Serial Connection

  //if (ON_MOVE || M6_EN) return;
  
  Serial.write("X");
  Serial.print(CURRENT_X);
  Serial.write("Y");
  Serial.print(CURRENT_Y);
  Serial.write("Z");
  Serial.print(CURRENT_PAYLOAD_HEIGHT);
  Serial.write("#");
  update_lcd(57);
  delay(500);
}

void C2_cmd() { //Write thread lengths to Serial
  //if (ON_MOVE || M6_EN) return;
  
  Serial.write("A");
  Serial.print(CURRENT_THREAD_LEN_A);
  Serial.write("B");
  Serial.print(CURRENT_THREAD_LEN_B);
  Serial.write("C");
  Serial.print(CURRENT_THREAD_LEN_C);
  Serial.write("#");
  update_lcd(58);
  delay(500);
}

///////////////////////////G-Code////////////////////////////


/////////////////////THREAD CALCULATIONS/////////////////////
double calculate_thread_height_var(double z){
    return HEIGHT_PYLON_A - z;
}

double calculate_thread_length_A_h(double x, double y, double z) {
  double height_created_triangle =  A_PAPER_DISTANCE + 8.5 - x - PAYLOAD_CENTER_THREAD_DISTANCE_A_x;
  double side_created_triangle;
  if ((y - 5.5) < 0)
    side_created_triangle = 5.5 - y;
  else
    side_created_triangle = y - 5.5;

  double trace_length = sqrt(pow(height_created_triangle, 2) + pow(side_created_triangle, 2));

  double thread_length = sqrt(pow(trace_length,2) + pow(calculate_thread_height_var(z),2));

  return thread_length;
}

double calculate_thread_length_B_h(double x, double y, double z) {
  double height_created_triangle =  PAPER_HEIGHT + B_PAPER_DISTANCE - y - PAYLOAD_CENTER_THREAD_DISTANCE_B_y;
  double side_created_triangle =  PAPER_DISTANCE_LOW_EDGE + x - PAYLOAD_CENTER_THREAD_DISTANCE_B_x;

  double trace_length = sqrt(pow(height_created_triangle,2) + pow(side_created_triangle,2));

  //double trace_length = PAPER_DISTANCE_1 - y - PAYLOAD_CENTER_THREAD_DISTANCE_B_y;

  double thread_length = sqrt(pow(trace_length,2) + pow(calculate_thread_height_var(z),2));

  return thread_length;
}

double calculate_thread_length_C_h(double x, double y, double z) {
  double height_created_triangle = C_PAPER_DISTANCE + y - PAYLOAD_CENTER_THREAD_DISTANCE_B_y;  
  double side_created_triangle = PAPER_DISTANCE_LOW_EDGE + x - PAYLOAD_CENTER_THREAD_DISTANCE_C_x;

  double trace_length = sqrt(pow(height_created_triangle,2) + pow(side_created_triangle,2));

  double thread_length = sqrt(pow(trace_length,2) + pow(calculate_thread_height_var(z),2));

  return thread_length;
}

/////////////////////THREAD CALCULATIONS/////////////////////

///////////////////////UNIT CONVERSION///////////////////////
double mm_to_in(double mm){
  return (mm/(25.4));  
}

double in_to_mm(double in){
  return (in *(25.4));  
}
///////////////////////UNIT CONVERSION///////////////////////

void update_lcd(int opcode){
  lcd.clear();
  if (opcode == 0 || opcode == 1){ //G0 or G1 (displays as integer for size)
    lcd.setCursor(0, 0); //1st line 1st char
    lcd.print("OPCODE: G");
    lcd.print(opcode);
    lcd.print(" F:");
    lcd.print(F_PERCENT_SPEED);
    lcd.setCursor(0, 1); //2nd Line 1st char
    double x = X_ADDRESS_TARGET;
    double y = Y_ADDRESS_TARGET;
    if (!UNIT_MODE){ //translate mm to in
        x = in_to_mm(X_ADDRESS_TARGET);
        y = in_to_mm(Y_ADDRESS_TARGET);
    }
    lcd.print("X: ");
    lcd.print(x);
    lcd.setCursor(8, 1); //2nd Line 9th char
    lcd.print("Y: ");
    lcd.print(y);
  }
  else if (opcode == 20){
    lcd.setCursor(0, 0); //1st line 1st char
    lcd.print("OPCODE: G");
    lcd.print(opcode);
    lcd.setCursor(0, 1); //2nd Line 1st char
    lcd.print("Input Mode > IN");
  }
  else if (opcode == 21){
    lcd.setCursor(0, 0); //1st line 1st char
    lcd.print("OPCODE: G");
    lcd.print(opcode);
    lcd.setCursor(0, 1); //2nd Line 1st char
    lcd.print("Input Mode > MM");
  }
  else if (opcode == 90){
    lcd.setCursor(0, 0); //1st line 1st char
    lcd.print("OPCODE: G");
    lcd.print(opcode);
    lcd.setCursor(0, 1); //2nd Line 1st char
    lcd.print("Coord Mode > ABS");
  }
  else if (opcode == 91){
    lcd.setCursor(0, 0); //1st line 1st char
    lcd.print("OPCODE: G");
    lcd.print(opcode);
    lcd.setCursor(0, 1); //2nd Line 1st char
    lcd.print("Coord Mode > INC");
  }
  else if (opcode == 2){
    lcd.setCursor(0, 0); //1st line 1st char
    lcd.print("OPCODE: M");
    lcd.print(opcode);
    lcd.setCursor(0, 1); //2nd Line 1st char
    lcd.print("END PROGRAM");
  }
  else if (opcode == 6){
    lcd.setCursor(0, 0); //1st line 1st char
    lcd.print("OPCODE: G");
    lcd.print(opcode);
    lcd.setCursor(0, 1); //2nd Line 1st char
    lcd.print("TOOL CHANGE");
  }
  else if (opcode == 57){
    lcd.setCursor(0, 0); //1st line 1st char
    lcd.print("Sending Location");
    lcd.setCursor(0, 1); //2nd Line 1st char
    lcd.print("<<<<<<<<>>>>>>>>");
  }
  else if (opcode == 58){
    lcd.setCursor(0, 0); //1st line 1st char
    lcd.print("Sending Threads");
    lcd.setCursor(0, 1); //2nd Line 1st char
    lcd.print("<<<<<<<<>>>>>>>>");
  }
  else {
    lcd.setCursor(0, 0); //1st line 1st char
    lcd.print("ERROR: ");
    lcd.print(opcode);
    lcd.setCursor(0, 1); //2nd Line 1st char
    lcd.print("OPCODE NOT FOUND");
  }
}

//Calculates the number of steps to move from current_ to new_ length of thread.
//If the output is negative, then the motor must reduce the size of the thread instead of increasing it

/////////////////////////CALCULATIONS FOR MOVEMENT/////////////////////////
int calculate_number_steps(double current_, double new_){
  int num_steps = int((new_ - current_)/INCHES_PER_STEP);
  return num_steps;  
}

int go_block(double x, double y, double z, int f_speed) {
  X_ADDRESS_TARGET = x;
  Y_ADDRESS_TARGET = y;
  Z_ADDRESS_TARGET = z;
  F_PERCENT_SPEED = f_speed;
  
  //calculate new necessary thread lengths
  double new_len_a = calculate_thread_length_A_h(x,y,z);
  double new_len_b = calculate_thread_length_B_h(x,y,z);
  double new_len_c = calculate_thread_length_C_h(x,y,z);
  
  //calculate number of steps and direction of movement to move from current pos to new pos.
  int steps_A = calculate_number_steps(CURRENT_THREAD_LEN_A, new_len_a);
  int steps_B = calculate_number_steps(CURRENT_THREAD_LEN_B, new_len_b);
  int steps_C = calculate_number_steps(CURRENT_THREAD_LEN_C, new_len_c);
  

  //Setting important info in stepper classes
  stepperA.setCurrentPosition(0);
  stepperB.setCurrentPosition(0);
  stepperC.setCurrentPosition(0);
  int max_speed = int((MAX_STEPS_PER_SEC/100)*f_speed);
  stepperA.setMaxSpeed(max_speed);
  stepperB.setMaxSpeed(max_speed);
  stepperC.setMaxSpeed(max_speed);
  //move the payload to location
  long steps[] = {steps_A, -steps_B, steps_C};

  lcd.clear();
  lcd.print("Moving to >>>>");
  lcd.setCursor(0, 1);
  lcd.print(x);
  lcd.print(" ");
  lcd.print(y);
  lcd.print(" ");
  lcd.print(z);
  
  steppers.moveTo(steps);

  //move_to(steps_A, steps_B, steps_C);

  //blocks until steppers finish moving
  while(steppers.run()){
  }

  //find actual displacement
  double actual_a = stepperA.currentPosition() * INCHES_PER_STEP;
  double actual_b = -(stepperB.currentPosition() * INCHES_PER_STEP);
  double actual_c = stepperC.currentPosition() * INCHES_PER_STEP;
  
  //after done, update current values
  CURRENT_X = x;
  CURRENT_Y = y;
  CURRENT_PAYLOAD_HEIGHT = z;
  CURRENT_THREAD_LEN_A += actual_a;
  CURRENT_THREAD_LEN_B += actual_b;
  CURRENT_THREAD_LEN_C += actual_c;
  
  return 0;
}

int go_len_block(double a, double b, double c, int f_speed) {
  
  //calculate number of steps and direction of movement to move from current pos to new pos.
  int steps_A = calculate_number_steps(CURRENT_THREAD_LEN_A, a);
  int steps_B = calculate_number_steps(CURRENT_THREAD_LEN_B, b);
  int steps_C = calculate_number_steps(CURRENT_THREAD_LEN_C, c);
  

  //Setting important info in stepper classes
  stepperA.setCurrentPosition(0);
  stepperB.setCurrentPosition(0);
  stepperC.setCurrentPosition(0);
  int max_speed = int((MAX_STEPS_PER_SEC/100)*f_speed);
  stepperA.setMaxSpeed(max_speed);
  stepperB.setMaxSpeed(max_speed);
  stepperC.setMaxSpeed(max_speed);
  //move the payload to location
  long steps[] = {steps_A, -steps_B, steps_C};
  
  steppers.moveTo(steps);

  //blocks until steppers finish moving
  while(steppers.run()){
  }

  //find actual displacement
  double actual_a = stepperA.currentPosition() * INCHES_PER_STEP;
  double actual_b = -(stepperB.currentPosition() * INCHES_PER_STEP);
  double actual_c = stepperC.currentPosition() * INCHES_PER_STEP;
  
  //after done, update current values
  CURRENT_THREAD_LEN_A += actual_a;
  CURRENT_THREAD_LEN_B += actual_b;
  CURRENT_THREAD_LEN_C += actual_c;

  X_ADDRESS_TARGET = 0;
  Y_ADDRESS_TARGET = 0;
  Z_ADDRESS_TARGET = 0;
  
  return 0;
}


int go(double x, double y, double z, int f_speed) {
  X_ADDRESS_TARGET = x;
  Y_ADDRESS_TARGET = y;
  Z_ADDRESS_TARGET = z;
  F_PERCENT_SPEED = f_speed;
  
  //calculate new necessary thread lengths
  double new_len_a = calculate_thread_length_A_h(x,y,z);
  double new_len_b = calculate_thread_length_B_h(x,y,z);
  double new_len_c = calculate_thread_length_C_h(x,y,z);

  /*Serial.println("");
  Serial.print("New Length A: ");
  Serial.println(new_len_a);
  Serial.print("New Length B: ");
  Serial.println(new_len_b);
  Serial.print("New Length C: ");
  Serial.println(new_len_c);

  Serial.println("");
  Serial.print("CurrentLength A: ");
  Serial.println(CURRENT_THREAD_LEN_A);
  Serial.print("CurrentLength B: ");
  Serial.println(CURRENT_THREAD_LEN_B);
  Serial.print("CurrentLength C: ");
  Serial.println(CURRENT_THREAD_LEN_C);*/
  
  //calculate number of steps and direction of movement to move from current pos to new pos.
  int steps_A = calculate_number_steps(CURRENT_THREAD_LEN_A, new_len_a);
  int steps_B = calculate_number_steps(CURRENT_THREAD_LEN_B, new_len_b);
  int steps_C = calculate_number_steps(CURRENT_THREAD_LEN_C, new_len_c);
  

  //Setting important info in stepper classes
  stepperA.setCurrentPosition(0);
  stepperB.setCurrentPosition(0);
  stepperC.setCurrentPosition(0);
  int max_speed = int((MAX_STEPS_PER_SEC/100)*f_speed);
  stepperA.setMaxSpeed(max_speed);
  stepperB.setMaxSpeed(max_speed);
  stepperC.setMaxSpeed(max_speed);
  //stepperA.setAcceleration(max_speed/5);
  //stepperB.setAcceleration(max_speed/5);
  //stepperC.setAcceleration(max_speed/5);
  //move the payload to location
  long steps[] = {steps_A, -steps_B, steps_C};

  lcd.clear();
  lcd.print("Moving to: F");
  lcd.setCursor(0, 1);
  lcd.print(x);
  lcd.print(" ");
  lcd.print(y);
  lcd.print(" ");
  lcd.print(z);
  
  steppers.moveTo(steps);

  //move_to(steps_A, steps_B, steps_C);

  ON_MOVE = true;

  //blocks until steppers finish moving
  while(steppers.run() && ON_MOVE){
    Commands.available();
  }

  //find actual displacement
  double actual_a = stepperA.currentPosition() * INCHES_PER_STEP;
  double actual_b = -(stepperB.currentPosition() * INCHES_PER_STEP);
  double actual_c = stepperC.currentPosition() * INCHES_PER_STEP;

  /*Serial.println("");
  Serial.print("DELTA A:");
  Serial.println(actual_a);
  Serial.print("DELTA B:");
  Serial.println(actual_b);
  Serial.print("DELTA C:");
  Serial.println(actual_c);

  Serial.println("\nFinal Stepper Pos:");
  Serial.print("Stepper A: ");
  Serial.println(stepperA.currentPosition());
  Serial.print("Stepper B: ");
  Serial.println(-stepperB.currentPosition());
  Serial.print("Stepper C: ");
  Serial.println(stepperC.currentPosition());*/

  ON_MOVE = false;
  
  //after done, update current values
  if (!M2_EN && !M6_EN){
    CURRENT_X = x;
    CURRENT_Y = y;
    CURRENT_PAYLOAD_HEIGHT = z;
  }
  if (!M6_EN){
    CURRENT_THREAD_LEN_A += actual_a;
    CURRENT_THREAD_LEN_B += actual_b;
    CURRENT_THREAD_LEN_C += actual_c;
  }
  M2_EN = false;

  X_ADDRESS_TARGET = 0;
  Y_ADDRESS_TARGET = 0;
  Z_ADDRESS_TARGET = 0;

  /*Serial.println("");
  Serial.print("Final Length A: ");
  Serial.println(CURRENT_THREAD_LEN_A);
  Serial.print("Final Length B: ");
  Serial.println(CURRENT_THREAD_LEN_B);
  Serial.print("Final Length C: ");
  Serial.println(CURRENT_THREAD_LEN_C);*/
  
  return 0;
}

/////////////////////////CALCULATIONS FOR MOVEMENT/////////////////////////

void init_values(){ //expects in not mm
  lcd.clear();
  lcd.print("Location:");
  lcd.setCursor(0, 1);
  lcd.print("Z, X, Y");
  while(Serial.available() <= 0){
  }
  double h = Serial.parseFloat();
  double x = Serial.parseFloat();
  double y = Serial.parseFloat();
  
  CURRENT_PAYLOAD_HEIGHT = h;
  
  double La = calculate_thread_length_A_h(x,y,h);
  double Lb = calculate_thread_length_B_h(x,y,h);
  double Lc = calculate_thread_length_C_h(x,y,h);
  
  CURRENT_THREAD_LEN_A = La;
  CURRENT_THREAD_LEN_B = Lb;
  CURRENT_THREAD_LEN_C = Lc;
  CURRENT_X = x;
  CURRENT_Y = y;

  lcd.clear();
  lcd.print("Z: ");
  lcd.print(CURRENT_PAYLOAD_HEIGHT);
  lcd.setCursor(0, 1); //2nd Line 1st char
  lcd.print("X: ");
  lcd.print(CURRENT_X);
  lcd.setCursor(8, 1); //2nd Line 9th char
  lcd.print("Y: ");
  lcd.print(CURRENT_Y);
  
}

void ready_command_lcd(){
  lcd.clear();
  lcd.setCursor(0, 0); //2nd Line 1st char
  lcd.print("    <READY>    ");
  lcd.setCursor(0, 1); //2nd Line 1st char
  lcd.print("   Listening   ");
}

void send_sensor(){

  
  sensorVal = analogRead(sensor);
  lightLevel = (sensorVal/1023)*10;
  Serial.write("L");
  Serial.print(lightLevel);

  boolean success = false;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;                       // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength);
  long test = 0;
  if (success){
    for (uint8_t i=0; i < uidLength; i++)
    {
      //Serial.print(" 0x");Serial.print(uid[i], HEX);
      test = test + uid[i];
      if(i<3){
      test = test * 256;
      }
    }
    Serial.write("R");
    Serial.write(test);
  } else {
    Serial.write("R0");
  }
}

void task_done(){
  X_ADDRESS_TARGET = 0;
  Y_ADDRESS_TARGET = 0;
  Z_ADDRESS_TARGET = 0;
  F_PERCENT_SPEED = 100;
  OPCODE = 1000; //change OPCODE to 1000 when task is done
  //delay(2000);
  ready_command_lcd();

  Serial.write("X");
  Serial.print(CURRENT_X);
  Serial.write("Y");
  Serial.print(CURRENT_Y);
  Serial.write("Z");
  Serial.print(CURRENT_PAYLOAD_HEIGHT);
  Serial.write("A");
  Serial.print(CURRENT_THREAD_LEN_A);
  Serial.write("B");
  Serial.print(CURRENT_THREAD_LEN_B);
  Serial.write("C");
  Serial.print(CURRENT_THREAD_LEN_C);
  
  //send_sensor();
  
  delay(1000);
  
  Serial.write("G"); //let matlab know when it is good to send another command.
  
}

void set_pins(){
  //LCD
  pinMode(2, OUTPUT);  
  pinMode(3, OUTPUT); 
  pinMode(4, OUTPUT); 
  pinMode(5, OUTPUT); 
  pinMode(8, OUTPUT);

  //STEPPERS
  pinMode(25, OUTPUT);
  pinMode(23, OUTPUT);
  pinMode(29, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(31, OUTPUT);

  pinMode(35, OUTPUT);
  pinMode(37, OUTPUT);
  pinMode(39, OUTPUT);
}

void setup() {
  Commands.begin();
  set_pins();
  lcd.begin(16, 2);
  lcd.print("      <FP>      ");
  lcd.setCursor(2, 1);
  lcd.print("Spydercam 18");
  delay(2000);
  lcd.clear();
  lcd.print("      BETA      ");
  lcd.setCursor(0, 1);
  lcd.print("Arduino Firmware");
  delay(2000);
  init_values();
  delay(2000);
  stepperA = AccelStepper(motorInterfaceType, stepPinA, dirPinA);
  stepperB = AccelStepper(motorInterfaceType, stepPinB, dirPinB);
  stepperC = AccelStepper(motorInterfaceType, stepPinC, dirPinC);

  digitalWrite(35, LOW);
  digitalWrite(37, LOW);
  digitalWrite(39, LOW);
  
  steppers.addStepper(stepperA);
  steppers.addStepper(stepperB);
  steppers.addStepper(stepperC);


  //sensor setup
  /*nfc.begin();
  lcd.clear();
  lcd.print("  Enabling NFC  ");
  lcd.setCursor(0, 1);
  lcd.print(" In Progress... ");
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    lcd.clear();
    lcd.print("     ERROR      ");
    lcd.setCursor(0, 1);
    lcd.print("   NFC Failed   ");
    while (1); // Halt
  }

  delay(1000);
  nfc.SAMConfig();*/
  ready_command_lcd();

  //Serial.write("G");
}

void loop() {
  if (Commands.available()) {
    task_done();
  }
}
