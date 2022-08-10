//changing variables
//pid variables
#define KD 1.4
#define KP 0.16
#define KI 0
double kp=KP, ki=KI, kd=KD, integralConst = 0;
#define SampleTime 20
int sampleTime = 20;
//max speed - same means no motor speed diff else adjust diff
#define maxSpeedR 128
#define maxSpeedL 128
//move foward while taking 90 turn
#define fdelay90 0
//duration of 90 right turn
#define rdelay90 80
//duration of 90 left turn
#define ldelay90 80

//counters etc
int checkpoint = 0;


//sensors
#define s1 14
#define s2 15
#define s3 16
#define s4 17
#define s5 18
#define s6 19
#define s7 20
#define s8 21
#define black 0
#define white 1
//sensor data
int sensorValue[] = {0, 0, 0, 0, 0, 0, 0, 0};
int errorMap[] = {2000, 1000, 500, 100, -100, -500, -1000, -2000};
int inputError = 0;

//motors a=right, b=left
#define m1a 9
#define m1b 8
#define m1p 7
#define stb 10
#define m2a 11
#define m2b 12
#define m2p 13

//motor data


//lights
#define led1 22 //green double flash-> checkpoint, single flash-> line change
#define led2 28 //green count 5
#define led3 34 //blue count 4
#define led4 40 //white count 3
#define led5 46 //blue count 2
#define led6 50 //white count 1


//functions
void readSensors() {
  sensorValue[0] = digitalRead(s1); //left
  sensorValue[1] = digitalRead(s2);
  sensorValue[2] = digitalRead(s3);
  sensorValue[3] = digitalRead(s4);
  sensorValue[4] = digitalRead(s5);
  sensorValue[5] = digitalRead(s6);
  sensorValue[6] = digitalRead(s7);
  sensorValue[7] = digitalRead(s8); //right

  int lineColor = black;
  int noOfSensorOnLine = sensorValue[0] + sensorValue[1] + sensorValue[2] + sensorValue[3] + sensorValue[4] + sensorValue[5] + sensorValue[6] + sensorValue[7];
  //find black on white or white on black light
  if ((sensorValue[0] == white) && ((sensorValue[2] == black) || (sensorValue[3] == black) || (sensorValue[4] == black) || (sensorValue[5] == black)) && (sensorValue[7] == white)) {
    //white surface black line
    digitalWrite(led1, HIGH);
    delay(5);
    digitalWrite(led1, LOW);
    lineColor = black;
    errorMap[0] = 2000;
    errorMap[1] = 1000;
    errorMap[2] = 500;
    errorMap[3] = 100;
    errorMap[4] = -100;
    errorMap[5] = -500;
    errorMap[6] = -1000;
    errorMap[7] = -2000;
  } else if ((sensorValue[0] == black) && ((sensorValue[2] == white) || (sensorValue[3] == white) || (sensorValue[4] == white) || (sensorValue[5] == white)) && (sensorValue[7] == black)) {
    //black surface white line
    digitalWrite(led1, HIGH);
    delay(5);
    digitalWrite(led1, LOW);
    lineColor = white;
    errorMap[0] = -2000;
    errorMap[1] = -1000;
    errorMap[2] = -500;
    errorMap[3] = -100;
    errorMap[4] = 100;
    errorMap[5] = 500;
    errorMap[6] = 1000;
    errorMap[7] = 2000;
  }

  //checkpoint
  if ((noOfSensorOnLine > 6 && lineColor == white) || (noOfSensorOnLine < 2 && lineColor == black)) {
    digitalWrite(stb, LOW);
    digitalWrite(led1, HIGH);
    delay(5);
    digitalWrite(led1, LOW);
    delay(5);
    digitalWrite(led1, HIGH);
    delay(5);
    digitalWrite(led1, LOW);
    checkpoint++;
    showCheckpointCount();
  } else if ((noOfSensorOnLine <= 6 && lineColor == white) || (noOfSensorOnLine >= 2 && lineColor == black)) {
    digitalWrite(stb, HIGH);
  } else if ((noOfSensorOnLine < 7 && noOfSensorOnLine >= 5 && lineColor == white) || (noOfSensorOnLine > 1 && noOfSensorOnLine <= 3 && lineColor == black)){
    //90 turns
    if((sensorValue[0]== white && sensorValue[7] == black && lineColor == black) || (sensorValue[0]== black && sensorValue[7] == white && lineColor == white)){
      //right 90
      right90();
    } else if((sensorValue[0]== black && sensorValue[7] == white && lineColor == black) || (sensorValue[0]== white && sensorValue[7] == black && lineColor == white)){
      //left 90
      left90();
    }
  }

  //error calculation
  inputError = sensorValue[0]*errorMap[0] + sensorValue[1]*errorMap[1] + sensorValue[2]*errorMap[2] + sensorValue[3]*errorMap[3] + sensorValue[4]*errorMap[4]  + sensorValue[5]*errorMap[5] + sensorValue[6]*errorMap[6] + sensorValue[7]*errorMap[7];

}

void showCheckpointCount() {
  int countCkp[] = { 0, 0, 0, 0, 0 };
  if (checkpoint == 1) {
    countCkp[0] = 0;
    countCkp[1] = 0;
    countCkp[2] = 0;
    countCkp[3] = 0;
    countCkp[4] = 1;
  } else if (checkpoint == 2) {
    countCkp[0] = 0;
    countCkp[1] = 0;
    countCkp[2] = 0;
    countCkp[3] = 1;
    countCkp[4] = 0;
  } else if (checkpoint == 3) {
    countCkp[0] = 0;
    countCkp[1] = 0;
    countCkp[2] = 0;
    countCkp[3] = 1;
    countCkp[4] = 1;
  } else if (checkpoint == 4) {
    countCkp[0] = 0;
    countCkp[1] = 0;
    countCkp[2] = 1;
    countCkp[3] = 0;
    countCkp[4] = 0;
  } else if (checkpoint == 5) {
    countCkp[0] = 0;
    countCkp[1] = 0;
    countCkp[2] = 1;
    countCkp[3] = 0;
    countCkp[4] = 1;
  } else if (checkpoint == 6) {
    countCkp[0] = 0;
    countCkp[1] = 0;
    countCkp[2] = 1;
    countCkp[3] = 1;
    countCkp[4] = 0;
  } else if (checkpoint == 7) {
    countCkp[0] = 0;
    countCkp[1] = 0;
    countCkp[2] = 1;
    countCkp[3] = 1;
    countCkp[4] = 1;
  } else if (checkpoint == 8) {
    countCkp[0] = 0;
    countCkp[1] = 1;
    countCkp[2] = 0;
    countCkp[3] = 0;
    countCkp[4] = 0;
  } else if (checkpoint == 9) {
    countCkp[0] = 0;
    countCkp[1] = 1;
    countCkp[2] = 0;
    countCkp[3] = 0;
    countCkp[4] = 1;
  } else if (checkpoint == 10) {
    countCkp[0] = 0;
    countCkp[1] = 1;
    countCkp[2] = 0;
    countCkp[3] = 1;
    countCkp[4] = 0;
  } else if (checkpoint == 11) {
    countCkp[0] = 0;
    countCkp[1] = 1;
    countCkp[2] = 0;
    countCkp[3] = 1;
    countCkp[4] = 1;
  } else if (checkpoint == 12) {
    countCkp[0] = 0;
    countCkp[1] = 1;
    countCkp[2] = 1;
    countCkp[3] = 0;
    countCkp[4] = 0;
  } else if (checkpoint == 13) {
    countCkp[0] = 0;
    countCkp[1] = 1;
    countCkp[2] = 1;
    countCkp[3] = 0;
    countCkp[4] = 1;
  } else if (checkpoint == 14) {
    countCkp[0] = 0;
    countCkp[1] = 1;
    countCkp[2] = 1;
    countCkp[3] = 1;
    countCkp[4] = 0;
  } else if (checkpoint == 15) {
    countCkp[0] = 0;
    countCkp[1] = 1;
    countCkp[2] = 1;
    countCkp[3] = 1;
    countCkp[4] = 1;
  } else if (checkpoint == 16) {
    countCkp[0] = 1;
    countCkp[1] = 0;
    countCkp[2] = 0;
    countCkp[3] = 0;
    countCkp[4] = 0;
  } else if (checkpoint == 17) {
    countCkp[0] = 1;
    countCkp[1] = 0;
    countCkp[2] = 0;
    countCkp[3] = 0;
    countCkp[4] = 1;
  } else if (checkpoint == 18) {
    countCkp[0] = 1;
    countCkp[1] = 0;
    countCkp[2] = 0;
    countCkp[3] = 1;
    countCkp[4] = 0;
  } else if (checkpoint == 19) {
    countCkp[0] = 1;
    countCkp[1] = 0;
    countCkp[2] = 0;
    countCkp[3] = 1;
    countCkp[4] = 1;
  } else if (checkpoint == 20) {
    countCkp[0] = 1;
    countCkp[1] = 0;
    countCkp[2] = 1;
    countCkp[3] = 0;
    countCkp[4] = 0;
  } else if (checkpoint == 21) {
    countCkp[0] = 1;
    countCkp[1] = 0;
    countCkp[2] = 1;
    countCkp[3] = 0;
    countCkp[4] = 1;
  } else if (checkpoint == 22) {
    countCkp[0] = 1;
    countCkp[1] = 0;
    countCkp[2] = 1;
    countCkp[3] = 1;
    countCkp[4] = 0;
  } else if (checkpoint == 23) {
    countCkp[0] = 1;
    countCkp[1] = 0;
    countCkp[2] = 1;
    countCkp[3] = 1;
    countCkp[4] = 1;
  } else if (checkpoint == 24) {
    countCkp[0] = 1;
    countCkp[1] = 1;
    countCkp[2] = 0;
    countCkp[3] = 0;
    countCkp[4] = 0;
  }
  digitalWrite(led6, countCkp[0]);
  digitalWrite(led5, countCkp[1]);
  digitalWrite(led4, countCkp[2]);
  digitalWrite(led3, countCkp[3]);
  digitalWrite(led2, countCkp[4]);
}

//motor movement codes
void motors(int right, int left) {
  //right left speed 0 or less
  if (left <= 0 && right <= 0) {
    digitalWrite(m1a, LOW);   //Establishes no direction of Channel A
    digitalWrite(m1b, LOW);   //Disengage the Brake for Channel A
    analogWrite(m1p, 0);      //Spins the motor on Channel A at 0 speed
    //forward @ full speed
    digitalWrite(m2a, LOW);   //Establishes no direction of Channel B
    digitalWrite(m2b, LOW);   //Disengage the Brake for Channel B
    analogWrite(m2p, 0);      //Spins the motor on Channel B at 0 speed
  }
  //right left speed positive
  else if (right > 0 && left > 0) {
    digitalWrite(m1a, HIGH);  //Establishes forward direction of Channel A
    digitalWrite(m1b, LOW);   //Disengage the Brake for Channel A
    analogWrite(m1p, right);  //Spins the motor on Channel A at right speed

    digitalWrite(m2a, HIGH);  //Establishes forward direction of Channel B
    digitalWrite(m2b, LOW);   //Disengage the Brake for Channel B
    analogWrite(m2p, left);   //Spins the motor on Channel B at left speed
  }
  //right speed negative and left speed positive
  else if (right < 0 && left > 0)
  {
    right = abs(right);
    digitalWrite(m1a, LOW);   //Establishes backward direction of Channel A
    digitalWrite(m1b, HIGH);  //Disengage the Brake for Channel A1
    analogWrite(m1p, right);  //Spins the motor on Channel A at negative right speed

    digitalWrite(m2a, HIGH);  //Establishes forward direction of Channel B
    digitalWrite(m2b, LOW);   //Disengage the Brake for Channel B
    analogWrite(m2p, left);   //Spins the motor on Channel B at left speed
  }
  //right speed positive and left speed negative
  else if (right > 0 && left < 0)
  {
    left = abs(left);
    digitalWrite(m1a, HIGH);  //Establishes forward direction of Channel A
    digitalWrite(m1b, LOW);   //Disengage the Brake for Channel A
    analogWrite(m1p, right);  //Spins the motor on Channel A at right speed

    digitalWrite(m2a, LOW);   //Establishes backward direction of Channel B
    digitalWrite(m2b, HIGH);  //Disengage the Brake for Channel B
    analogWrite(m2p, left);   //Spins the motor on Channel B at negative left speed
  }
}

void brake() {
  digitalWrite(stb, HIGH); // brake
  digitalWrite(m1a, HIGH); // Establishes high impedence brake of Channel A
  digitalWrite(m1b, HIGH);
  digitalWrite(m2a, HIGH); //Establishes high impedence brake of Channel B
  digitalWrite(m2b, HIGH);
}

//right 90 turn
void right90() {
  motors(255, 255);
  delay(fdelay90);
  brake();
  motors(255, -255);
  delay(rdelay90);
}

//left 90 turn
void left90() {
  motors(255, 255);
  delay(fdelay90);
  brake();
  motors(-255, 255);
  delay(ldelay90);
}

void setPIDConst(double Kp = KP, double Ki = KI, double Kd = KD) {
  double SampleTimeInSec = ((double)sampleTime) / 1000;;
  kp = Kp;
  ki = Ki * SampleTimeInSec;
  kd = Kd / SampleTimeInSec;
}

void setSampleTime(int newSampleTime = SampleTime) {
  if (newSampleTime > 0){
    sampleTime = newSampleTime;
    double ratio  = (double)newSampleTime / (double)sampleTime;
    ki *= ratio;
    kd /= ratio;
  }
}

void setup() {
  //sensors as input
  pinMode(s1, INPUT);
  pinMode(s2, INPUT);
  pinMode(s3, INPUT);
  pinMode(s4, INPUT);
  pinMode(s5, INPUT);
  pinMode(s6, INPUT);
  pinMode(s7, INPUT);
  pinMode(s8, INPUT);

  //motors as output
  pinMode(m1a, OUTPUT);
  pinMode(m1b, OUTPUT);
  pinMode(m1p, OUTPUT);
  pinMode(stb, OUTPUT);
  pinMode(m2a, OUTPUT);
  pinMode(m2b, OUTPUT);
  pinMode(m2p, OUTPUT);

  //leds as output
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);
  pinMode(led5, OUTPUT);
  pinMode(led6, OUTPUT);

  //motor driver turned off
  digitalWrite(stb, LOW);
  digitalWrite(m1a, LOW);
  digitalWrite(m1b, LOW);
  digitalWrite(m1p, LOW);
  digitalWrite(m2a, LOW);
  digitalWrite(m2b, LOW);
  digitalWrite(m2p, LOW);

  setSampleTime();
  setPIDConst();
}

void loop() {
  readSensors();
  
  //calculate pid value
  integralConst += (ki * inputError);
  int PD = kp * inputError + integralConst - kd * inputError;

  
  //move motors
  if (inputError < 10 && inputError > -10)
    motors(maxSpeedR, maxSpeedL);
  else if (inputError < -10)
    motors(maxSpeedR, maxSpeedL - PD);
  else if (inputError > 10)
    motors(maxSpeedR - PD, maxSpeedL);
}
