/************************************************************
 * PID controller drifferntial drive robot                  *
 * by Tanawin Siriwan                                       *
 * 6203014610060 sec 1                                      *
 * mechatronics engineering techanology                     *
 * College of industrial technologo                         *
 * King mongkut's university of techanology north bangkok   *
*************************************************************/

#include <PID_v1.h>

/*********************RIGHT wheel valiable*****************************/
#define PWM_R 8
#define inA_R 9
#define inB_R 7

#define ENCA_R 2
#define ENCB_R 3
volatile long encoderPos_R = 0;

/*********************LEFT wheel valiable******************************/
#define PWM_L 11
#define inA_L 6
#define inB_L 10

#define ENCA_L 5
#define ENCB_L 4
volatile long encoderPos_L = 0;

/**********************Robot body***********************************/
int wheel = 70;
int dimention_wheel = wheel * 3.14;
int rev_pluse = 360;
int motor_raio = 74.8;
int motor_main_shaft = 1;
int body = 245;

/***********************************Encoder RIGHT***********************************/
int origin_X = 0;
int origin_Y = 0;

//***********************************Position***********************************//
int coorX = 1;
int coorY = 1;

/***********************************Traget point***********************************/
int Targetpoint = 1;

/***********************************RIGHT wheel PID Tuning***********************************/
//Define Variables we'll be connecting to
double input_R = 0, output_R = 0, setpoint_R = 0;

//Specify the links and initial tuning parameters
double kp_R = 0.5, ki_R = 0.006 , kd_R = 0;
PID myPID_R(&input_R, &output_R, &setpoint_R, kp_R, ki_R, kd_R, DIRECT);

/***********************************LEFT wheel PID Tuning***********************************/
//Define Variables we'll be connecting to
double input_L = 0, output_L = 0, setpoint_L = 0;

//Specify the links and initial tuning parameters
double kp_L = 0.5, ki_L = 0.006 , kd_L = 0;
PID myPID_L(&input_L, &output_L, &setpoint_L, kp_L, ki_L, kd_L, DIRECT);

/*******************A-Z val for memory Tragetpoint value for regtangura form****************/
double A = 0.0, B = 0.0, C = 0.0, D = 0.0, E = 0.0, F = 0.0, G = 0.0, H = 0.0, I = 0.0, J = 0.0, K = 0.0, L = 0.0, M = 0.0, N = 0.0, O = 0.0, P = 0.0, Q = 0.0, R = 0.0, S = 0.0, T = 0.0;

/****************************Arry val for memory Tragetpoint value for triangular form******************************************/
double V = 0.0, U = 0.0;
double Set_tri_R[20];
double Set_tri_L[20];

float encoder_R_Driff;
float encoder_L_Driff;

float encoder_R_Error;
float encoder_L_Error;

float encoder_R_Prev;
float encoder_L_Prev;

volatile int state_modeREC = 0;
volatile int state_modeTRI = 0;
volatile int state_mode = 0;

int stat_A = 0;
int stat_B = 0;
int stat_C = 0;

/*********************************************Millis timer*************************************************/
unsigned long period_X = 150;
unsigned long last_time_X = 0;

unsigned long period_Y = 150;
unsigned long last_time_Y = 0;

unsigned long period_Z = 150;
unsigned long last_time_Z = 0;


void setup()
{
  //initialize the variables we're linked to
  Serial.begin(115200);
  pinMode(PWM_R, OUTPUT);
  pinMode(inA_R, OUTPUT);
  pinMode(inB_R, OUTPUT);

  pinMode(ENCA_R, INPUT);
  pinMode(ENCB_R, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA_R), readEncoder_R, RISING);

  pinMode(PWM_L, OUTPUT);
  pinMode(inA_L, OUTPUT);
  pinMode(inB_L, OUTPUT);

  pinMode(ENCA_L, INPUT);
  pinMode(ENCB_L, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA_L), readEncoder_L, RISING);

  pinMode(47, INPUT);
  pinMode(49, INPUT);
  pinMode(51, INPUT);
  pinMode(39, INPUT);
  pinMode(41, INPUT);
  pinMode(43, INPUT);
  pinMode(45, INPUT);
  attachInterrupt(47, readButtonA, CHANGE);
  attachInterrupt(49, readButtonB, CHANGE);
  attachInterrupt(51, readButtonC, CHANGE);

  //turn the PID RIGHT on
  myPID_R.SetMode(AUTOMATIC);
  myPID_R.SetSampleTime(1);
  myPID_R.SetOutputLimits(-100, 100);

  //turn the PID LEFT on
  myPID_L.SetMode(AUTOMATIC);
  myPID_L.SetSampleTime(1);
  myPID_L.SetOutputLimits(-100, 100);
}

void loop() {
  if ( millis() - last_time_X > period_X) {
    last_time_X = millis();
    if (state_modeREC == 1) {                    /*Read button value condition*/
      stat_A = 1 ;                               /*count = 1*/
    }
    //Serial.print(stat_A);Serial.print(",");
  }
  if ( millis() - last_time_Y > period_Y) {      
    last_time_Y = millis();                      
    if (state_modeTRI == 1) {                    /*Read button value condition*/
      stat_B = 1;                                /*count = 1*/
    }
    //Serial.print(stat_B);Serial.print(",");
  }
  if ( millis() - last_time_Z > period_Z) {
    last_time_Z = millis();          
    if (state_mode == 1) {                       /*Read button value condition*/
      stat_C = 1;                                /*count = 1*/
    }
  }
  Serial.print(stat_A); Serial.print(","); Serial.print(stat_B); Serial.println(","); Serial.print(stat_C);
  switch (stat_C) {                              /*Condition robot mode*/
    case 1 : Triangle(); break;                  /*Robot triangle form*/

  }
  switch (stat_B) {                              /*Condition robot mode*/
    case 1 : Rectangle(); break;                 /*Robot rectangular form*/
  }
  switch (stat_A) {                              /*Condition robot mode*/
    case 1 : linear(); break;                    /*Robot linear form*/
  }
}

/*
   Command to comput_R function and comput_L function compute the PID value to Control Motor left and right go to setpoint
   Use the switch case to condition for command the robot go to position x y in a triangular form
*/
void Triangle() {
  comput_R();
  comput_L();

  Serial.println(Targetpoint);
  switch (Targetpoint) {
    case 1 : drive_Tri_45(); break;
    case 2 : driff_drive(); break;
    case 3 : return_drive(); break;
    case 4 : back_drive(); break;
    case 5 : drive_Tri_90(); break;
    case 6 : to_home(); break;
    case 7 : drive_Tri_90_2(); break;
  }
}

/*
   Command to comput_R function and comput_L function compute the PID value to Control Motor left and right go to setpoint
   Use the switch case to condition for command the robot go to position x y in a square form
*/
void Rectangle() {
  comput_R();
  comput_L();

  Serial.println(Targetpoint);
  switch (Targetpoint) {
    case 1 : direct_drive_1(); break;
    case 2 : A90_drive_1(); break;
    case 3 : direct_drive_2(); break;
    case 4 : A90_drive_2(); break;
    case 5 : direct_drive_3(); break;
    case 6 : A90_drive_3(); break;
    case 7 : direct_drive_4(); break;
    case 8 : A90_drive_4(); break;
  }
}

/*
   Command to comput_R function and comput_L function compute the PID value to Control Motor left and right go to setpoint
   Use the switch case to condition for command the robot go to position x y in a linear form
*/

void linear() {
  comput_R();
  comput_L();

  Serial.println(Targetpoint);
  switch (Targetpoint) {
    case 1 : direct_drive_X(); break;
  }
}

void direct_driveX() {
  double a, b, c, d;
  A = pulse_RW(a) + 2000; B = pulse_LW(b) + 2000;
  C = A; D = B;
  if (encoderPos_R >= A && encoderPos_L >= B) {
    Targetpoint++;
  }
} 

/*
   Set a Tragetpoint encoder value for robot go to position x y in a square form
*/

/*
   Traget point 1 start robot go to x = 0 ft, y = 1 ft
   Motor right CW direction
   Motor left CW direction
*/
void direct_drive_1() {
  double a, b, c, d;
  A = pulse_RW(a); B = pulse_LW(b);
  C = A; D = B;
  if (encoderPos_R >= A && encoderPos_L >= B) {
    Targetpoint++;
  }
}

/*
   Traget point 2 robot turn arount 90 degree
   Motor right CCW direction
   Motor left CW direction
*/

void A90_drive_1() {
  double a, b;
  E = A = C - pulse_r90(a) - 4000; F = B = pulse_r90(b) + D + 2500;
  if (encoderPos_R <= A && encoderPos_L >= B) {
    Targetpoint++;
  }
}

/*
   Traget point 3 robot go to x = 1 ft, y = 1 ft
   Motor right CW direction
   Motor left CW direction
*/

void direct_drive_2() {
  double a, b, c, d;
  G = A = pulse_RW(a) + E; H = B = pulse_LW(b) + F;
  if (encoderPos_R >= A && encoderPos_L >= B) {
    Targetpoint++;
  }
}

/*
   Traget point 4 robot turn arount 90 degree
   Motor right CCW direction
   Motor left CW direction
*/

void A90_drive_2() {
  double a, b;
  I = A = G - pulse_r90(a) - 7000; J = B = pulse_r90(b) + H + 6500;
  if (encoderPos_R <= A && encoderPos_L >= B) {
    Targetpoint++;
  }
}

/*
   Traget point 5 robot go to x = 1 ft, y = 0 ft
   Motor right CW direction
   Motor left CW direction
*/

void direct_drive_3() {
  double a, b, c, d;
  K = A = pulse_RW(a) + I; L = B = pulse_LW(b) + J;
  if (encoderPos_R >= A && encoderPos_L >= B) {
    Targetpoint++;
  }
}

/*
   Traget point 6 robot turn arount 90 degree
   Motor right CCW direction
   Motor left CW direction
*/

void A90_drive_3() {
  double a, b;
  M = A = K - pulse_r90(a) - 7000; N = B = pulse_r90(b) + L + 6500;
  if (encoderPos_R <= A && encoderPos_L >= B) {
    Targetpoint++;
  }
}

/*
   Traget point 7 robot go to x = 0 ft, y = 0 ft
   Motor right CW direction
   Motor left CW direction
*/

void direct_drive_4() {
  double a, b, c, d;
  O = A = pulse_RW(a) + M; P = B = pulse_LW(b) + N;
  if (encoderPos_R >= A && encoderPos_L >= B) {
    Targetpoint++;
  }
}

/*
   Traget point 8 robot turn arount 90 degree robot return to origin point
   Motor right CCW direction
   Motor left CW direction
*/

void A90_drive_4() {
  double a, b;
  Q = A = O - pulse_r90(a) - 7000; R = B = pulse_r90(b) + P + 6500;
  if (encoderPos_R <= A && encoderPos_L >= B) {
    Targetpoint++;
  }
}

/**
   End of robot go to position  x y in a square form
*/



/*
   Set a Tragetpoint encoder value for robot go to position x y in a triangular form
*/

/*
   Traget point 1 start robot turn arount 45 degree
   Motor right CCW direction
   Motor left CW direction
*/

void drive_Tri_45() {
  double a, b;
  Set_tri_R[0] = A = - pulse_Freed_45(a) + 7000; Set_tri_L[0] = B = pulse_Freed_45(b);
  if (encoderPos_R <= A && encoderPos_L >= B) {
    Targetpoint++;
  }
}

/*
   Traget point 2 robot go to x = 1 ft, y = 1 ft
   Motor right CW direction
   Motor left CW direction
*/

void driff_drive() {
  double a, b;
  Set_tri_R[1] = A = pulse_Triangle(a) + Set_tri_R[0]; Set_tri_L[1] = B = pulse_Triangle(b) + Set_tri_L[0];
  if (encoderPos_R >= A && encoderPos_L >= B) {
    Targetpoint++;
  }
}

/*
   Traget point 3 robot turn arount 235 degree
   Motor right CCW direction
   Motor left CW direction
*/

void return_drive() {
  double a, b;
  Set_tri_R[2] = A = Set_tri_R[1] - pulse_Freed_return(a) +1000 ; Set_tri_L[2] = B = pulse_Freed_return(b) + Set_tri_L[1] + 200;
  if (encoderPos_R <= A && encoderPos_L >= B) {
    Targetpoint++;
  }
}

/*
   Traget point 4 robot go to x = 1 ft, y = 0 ft
   Motor right CW direction
   Motor left CW direction
*/

void back_drive() {
  double a, b;
  Set_tri_R[3] = A = pulse_RW(a) + Set_tri_R[2]; Set_tri_L[3] = B = pulse_LW(b) + Set_tri_L[2];
  if (encoderPos_R >= A && encoderPos_L >= B) {
    Targetpoint++;
  }
}

/*
   Traget point 5 robot turn arount 90 degree
   Motor right CCW direction
   Motor left CW direction
*/

void drive_Tri_90() {
  double a, b;
  Set_tri_R[4] = A = Set_tri_R[3] - pulse_r90(a) - 3000; Set_tri_L[4] = B = Set_tri_L[3] + pulse_r90(b) + 4000;
  if (encoderPos_R <= A && encoderPos_L >= B) {
    Targetpoint++;
  }
}

/*
   Traget point 6 robot go to x = 0 ft, y = 0 ft
   Motor right CW direction
   Motor left CW direction
*/

void to_home() {
  double a, b;
  Set_tri_R[5] = A = Set_tri_R[4] + pulse_RW(a)-2500;   Set_tri_L[5] = B = Set_tri_L[4] + pulse_LW(b)-2500;
  if (encoderPos_R >= A && encoderPos_L >= B) {
    Targetpoint++;
  }
}

/*
   Traget point 7 robot turn arount 90 degree robot return to origin point
   Motor right CCW direction
   Motor left CW direction
*/

void drive_Tri_90_2() {
  double a, b;
  Set_tri_R[6] = A = Set_tri_R[5] - pulse_r90(a) - 3000; Set_tri_L[6] = B = Set_tri_L[5] + pulse_r90(b) + 5000;
  if (encoderPos_R <= A && encoderPos_L >= B) {
    Targetpoint++;

  }
}

/**
    End of robot go to position x y in a triangular form
*/

/*
   PID motor position control.
*/

void comput_R() {
  setpoint_R = A;                     /*modify to fit motor and encoder right*/
  input_R = encoderPos_R ;            /*data form encoder right*/
  myPID_R.Compute();                  /*calculate output*/
  pwmOut_R(output_R);                 /*drive motor drive module*/
  Serial.print(setpoint_R);           /*mornitor of motor right*/
  Serial.print(" ");
  Serial.println(encoderPos_R);       /*mornitor motor right position*/

}

void comput_L() {
  setpoint_L = B;                     /*modify to fit motor and encoder left*/
  input_L = encoderPos_L ;            /*data form encoder left*/
  myPID_L.Compute();                  /*calculate output*/
  pwmOut_L(output_L);                 /*drive motor drive module*/
  Serial.print(setpoint_L);           /*mornitor of motor left*/
  Serial.print(" ");
  Serial.println(encoderPos_L);       /*mornitor motor right position*/

}

/**********************Motor RIGHT direction**********************/
void pwmOut_R(int out) {
  if (out > 0) {
    analogWrite(PWM_R, out);                  /*drive motor right CW*/
    digitalWrite(inA_R, HIGH);
    digitalWrite(inB_R, LOW);
  }
  else {
    analogWrite(PWM_R, abs(out));             /*drive motor right CCW*/
    digitalWrite(inA_R, LOW);
    digitalWrite(inB_R, HIGH);
  }
}

/**********************Motor left direction**********************/
void pwmOut_L(int out) {
  if (out > 0) {
    analogWrite(PWM_L, out);                   /*drive motor left CW*/
    digitalWrite(inB_L, HIGH);
    digitalWrite(inA_L, LOW);
  }
  else {
    analogWrite(PWM_L, abs(out));              /*drive motor left CCW*/
    digitalWrite(inB_L, LOW);
    digitalWrite(inA_L, HIGH);
  }
}

/*
   Calculate the pluse of encoder right for command robot go to x = 0 ft, y = 1 ft
*/
float pulse_RW(float pulse) {
  int feetXX = coorX * 304;
  float res_pulse = motor_raio * rev_pluse;
  float tick = dimention_wheel / res_pulse;
  pulse = feetXX / tick;
  return pulse;
}

/*
   Calculate the pluse of encoder left for command robot go to x = 0 ft, y = 1 ft
*/
float pulse_LW(float pulse) {
  int feetYY = coorY * 304;
  float res_pulse = motor_raio * rev_pluse;
  float tick = dimention_wheel / res_pulse;
  pulse = feetYY / tick;
  return pulse;
}

/*
   Calculate the pluse of encoder for command robot turn around 90 degree
*/
float pulse_r90(float pulse) {
  float distance = 2 * 3.14 * (body / 2) * 0.13;
  float res_pulse = motor_raio * rev_pluse;
  float tick = dimention_wheel / res_pulse;
  pulse = distance / tick;
  return pulse;
}

/*
   Calculate the pluse of encoder for command robot turn around 45 degree
*/
float pulse_Freed_45(float pulse) {
  int posXX = coorX * 304;
  int posYY = coorY * 304;
  double fee = atan(posXX / posYY);
  double ratio = fee / 360;
  double distance = 2 * 3.14 * (body / 2) * 0.125;
  double res_pulse = motor_raio * rev_pluse;
  double tick = dimention_wheel / res_pulse;
  pulse = distance / tick;
  return pulse;
}

/*
   Calculate the pluse of encoder for command robot turn around 235 degree
*/
float pulse_Freed_return(float pulse) {
  int posXX = coorX * 304;
  int posYY = coorY * 304;
  double fee = atan(posXX / posYY);
  double ratio = fee / 360;
  double distance = 2 * 3.14 * (body / 2) * 0.3;
  double res_pulse = motor_raio * rev_pluse;
  double tick = dimention_wheel / res_pulse;
  pulse = distance / tick;
  return pulse;
}

/*
   Calculate the pluse of encoder left for command robot go to x = 1 ft, y = 1 ft
*/
float pulse_Triangle(float pulse) {
  int posXX = coorX * 304;
  int posYY = coorY * 304;
  int distance = sqrt(pow(posXX, 2) + pow(posYY, 2));
  float res_pulse = motor_raio * rev_pluse;
  float tick = dimention_wheel / res_pulse;
  pulse = distance / tick;
  return pulse;
}

/******************************Interupt switch*********************************/
void readButtonA() {
  state_modeREC = digitalRead(47);
}

void readButtonB() {
  state_modeTRI = digitalRead(49);
}

void readButtonC() {
  state_mode = digitalRead(51);
}

/*
   Encoder motor right
*/
void readEncoder_R() {               /*pulse and direction, direct port reading to save cycles*/
  int b = digitalRead(ENCB_R);
  if (b > 0) {                       /*if b > 0   count ++;*/
    encoderPos_R++;
  }
  else {
    encoderPos_R--;                   /*if b < 0   count --;*/
  }
}

/*
   Encoder motor left
*/
void readEncoder_L() {                 /*pulse and direction, direct port reading to save cycles*/
  int b = digitalRead(ENCB_L);
  if (b > 0) {                         /*if b > 0   count ++;*/
    encoderPos_L++;
  }
  else {
    encoderPos_L--;                    /*if b < 0   count --;*/
  }
}
