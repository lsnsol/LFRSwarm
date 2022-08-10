#define MANUAL 0
#define AUTOMATIC 1
#define DIRECT 0
#define REVERSE 1

int maxi = 255;       // SET MAX SPEED
//int n=0;             // stopping code for end of arena

int lc = LOW;
int sc = HIGH;

double d;
int takeout = 0;
double lasts = 0;
double pos = 0;

unsigned long lastTime;
double Input, Output, Setpoint, lastpd, power_difference;
double ITerm, lastInput;
double kp, ki, kd;
double kp1, ki1, kd1;
int SampleTime = 1; //1 sec
double outMin, outMax;
bool inAuto = false;
unsigned long present = millis();
unsigned long passed = millis();


int pwma = 7;         // tb6612fng configuration
int ain1 = 8;
int ain2 = 9;
int stby = 10;
int bin1 = 11;
int bin2 = 22;
int pwmb = 13;

int led1 = 25; //white  //led lights
int led2 = 31; //red
int led3 = 37; //white
int led4 = 43; //white
int led5 = 49; //white
int led6 = 52; //red

int sensorL3 = 30;    // SENSOR CONFIG
int sensorL2 = 32;
int sensorL1 = 34;
int sensormidL = 36;
int sensormidR = 38;
int sensorR1 = 40;
int sensorR2 = 42;
int sensorR3 = 44;

int countmiddle()                 // stright line
{
  int c = 0;
  if (digitalRead(sensormidL) == lc || digitalRead(sensormidR) == lc)
    c++;
  return c;
}


double count()                      //error mapping 8 sensor
{
  double c = 0;
  d = 0;
  pos = 0;
  if (digitalRead(sensorL3) == lc)          //extreme left error=9000
  {
    c++;
    lasts = 2;
    pos = pos + 9000;
  }

  if (digitalRead(sensorL2) == lc)           //extreme left error=8000
  {
    c++;
    d++;
    lasts = 2;
    pos = pos + 8000;
  }
  if (digitalRead(sensorL1) == lc)          //extreme left error=7000
  {
    c++;
    d++;
    lasts = 1;
    pos = pos + 7000;
  }
  if (digitalRead(sensormidL) == lc)        //extreme left error=5500
  {
    c++;
    d++;
    lasts = 1;
    pos = pos + 5500;
  }
  if (digitalRead(sensormidR) == lc)        //extreme left error=4500
  {
    c++;
    d++;
    lasts = 1;
    pos = pos + 4500;
  }
  if (digitalRead(sensorR1) == lc)          //extreme left error=3000
  {
    c++;
    d++;
    lasts = 1;
    pos = pos + 3000;
  }
  if (digitalRead(sensorR2) == lc)          //extreme left error=2000
  {
    c++;
    d++;
    lasts = 2;
    pos = pos + 2000;
  }
  if (digitalRead(sensorR3) == lc)         //extreme left error=1000
  {
    c++;
    lasts = 2;
    pos = pos + 1000;
  }
  return c;
}


double countturn()                            //error mapping 6 sensor
{
  double c = 0;
  pos = 0;
  if (digitalRead(sensorL2) == lc)            //extreme left error=8000
  {
    c++;
    lasts = 2;
    pos = pos + 8000;
  }
  if (digitalRead(sensorL1) == lc)            //extreme left error=7000
  {
    c++;
    lasts = 1;
    pos = pos + 7000;
  }
  if (digitalRead(sensormidL) == lc)          //extreme left error=5500
  {
    c++;
    lasts = 1;
    pos = pos + 5500;
  }
  if (digitalRead(sensormidR) == lc)           //extreme left error=4500
  {
    c++;
    lasts = 1;
    pos = pos + 4500;
  }
  if (digitalRead(sensorR1) == lc)            //extreme left error=3000
  {
    c++;
    lasts = 1;
    pos = pos + 3000;
  }
  if (digitalRead(sensorR2) == lc)           //extreme left error=2000
  {
    c++;
    lasts = 2;
    pos = pos + 2000;
  }
  return c;
}


void motors(int r, int l)                   // motor movement codes
{
  if (r >= 0 && l >= 0)
  { //forward @ l speed
    digitalWrite(ain1, HIGH); //Establishes forward direction of Channel A
    digitalWrite(ain2, LOW);   //Disengage the Brake for Channel A
    analogWrite(pwma, l);   //Spins the motor on Channel A at full speed
    //forward @ r speed
    digitalWrite(bin1, HIGH); //Establishes forward direction of Channel B
    digitalWrite(bin2, LOW);   //Disengage the Brake for Channel B
    analogWrite(pwmb, r);   //Spins the motor on Channel B at full speed
  }
  else if (r < 0 && l > 0)
  {
    r = abs(r);
    digitalWrite(ain1, LOW); //Establishes backward direction of Channel A
    digitalWrite(ain2, HIGH);   //Disengage the Brake for Channel A
    analogWrite(pwma, l);   //Spins the motor on Channel A at full speed
    //forward @ full speed
    digitalWrite(bin1, HIGH); //Establishes forward direction of Channel B
    digitalWrite(bin2, LOW);   //Disengage the Brake for Channel B
    analogWrite(pwmb, r);   //Spins the motor on Channel B at full speed
  }
  else if (l < 0 && r > 0)
  {
    l = abs(l);
    digitalWrite(ain1, HIGH); //Establishes forward direction of Channel A
    digitalWrite(ain2, LOW);   //Disengage the Brake for Channel A
    analogWrite(pwma, l);   //Spins the motor on Channel A at full speed
    //forward @ full speed
    digitalWrite(bin1, LOW); //Establishes backward direction of Channel B
    digitalWrite(bin2, HIGH);   //Disengage the Brake for Channel B
    analogWrite(pwmb, r);   //Spins the motor on Channel B at full speed
  }
  else if (l < 0 && r < 0)
  {
    l = abs(l);
    r = abs(r);
    digitalWrite(ain1, LOW); //Establishes no direction of Channel A
    digitalWrite(ain2, LOW);   //Disengage the Brake for Channel A
    analogWrite(pwma, l);   //Spins the motor on Channel A at full speed
    //forward @ full speed
    digitalWrite(bin1, LOW); //Establishes no direction of Channel B
    digitalWrite(bin2, LOW);   //Disengage the Brake for Channel B
    analogWrite(pwmb, r);   //Spins the motor on Channel B at full speed
  }
}


void brake(int s)
{
  digitalWrite(stby, HIGH); // brake
  digitalWrite(ain1, HIGH); //Establishes high impedence brake of Channel A
  digitalWrite(ain2, HIGH);
  digitalWrite(bin1, HIGH); //Establishes high impedence brake of Channel B
  digitalWrite(bin2, HIGH);
}

void righttight()                      //90' right turn
{
  motors(255, 255);
  //delay(8);
  motors(230, -70);
  delay(80);
  while (countturn() == 0)
  {
    digitalWrite(ain1, HIGH); //Establishes forward direction of Channel A
    digitalWrite(ain2, LOW);   //Disengage the Brake for Channel A
    analogWrite(pwma, 255);   //Spins the motor on Channel A at full speed
    //forward @ full speed
    digitalWrite(bin1, LOW); //Establishes backward direction of Channel B
    digitalWrite(bin2, HIGH);   //Disengage the Brake for Channel B
    analogWrite(pwmb, 255);   //Spins the motor on Channel B at full speed
    //delay(10);
  }
}

void lefttight()                     //90' left turn
{
  motors(255, 255);
  //delay(8);
  motors(-70, 230);
  delay(80);
  while (countturn() == 0)
  {
    digitalWrite(ain1, LOW); //Establishes backward direction of Channel A
    digitalWrite(ain2, HIGH);   //Disengage the Brake for Channel A
    analogWrite(pwma, 255);   //Spins the motor on Channel A at full speed
    //forward @ full speed
    digitalWrite(bin1, HIGH); //Establishes forward direction of Channel B
    digitalWrite(bin2, LOW);   //Disengage the Brake for Channel B
    analogWrite(pwmb, 255);   //Spins the motor on Channel B at full speed
    //delay(10);
  }
}


void Compute()
{
  if (!inAuto)
    return;
  unsigned long now = millis();
  int timeChange = (now - lastTime);

  if (timeChange >= SampleTime)
  {
    /*Compute all the working error variables*/
    double error = Setpoint - Input;
    ITerm += (ki * error);

    if (ITerm > outMax)
      ITerm = outMax;
    else if (ITerm < outMin)
      ITerm = outMin;
    double dInput = (Input - lastInput);

    /*Compute PID Output*/
    Output = kp * error + ITerm - kd * dInput;

    if (Output > outMax)
      Output = outMax;
    else if (Output < outMin)
      Output = outMin;

    /*Remember some variables for next time*/
    lastInput = Input;
    lastTime = now;
  }
}

void SetTunings(double Kp, double Ki, double Kd)
{
  double SampleTimeInSec = ((double)SampleTime) / 1000;
  kp = Kp;
  ki = Ki * SampleTimeInSec;
  kd = Kd / SampleTimeInSec;
}

void SetSampleTime(int NewSampleTime)
{
  if (NewSampleTime > 0)
  {
    double ratio  = (double)NewSampleTime / (double)SampleTime;
    ki *= ratio;
    kd /= ratio;
    SampleTime = (unsigned long)NewSampleTime;
  }
}

void SetOutputLimits(double Min, double Max)
{
  if (Min > Max) return;
  outMin = Min;
  outMax = Max;

  if (Output > outMax)
    Output = outMax;
  else if (Output < outMin)
    Output = outMin;

  if (ITerm > outMax)
    ITerm = outMax;
  else if (ITerm < outMin)
    ITerm = outMin;
}

void SetMode(int Mode)
{
  bool newAuto = (Mode == AUTOMATIC);
  if (newAuto && !inAuto)
  { /*we just went from manual to auto*/
    Initialize();
  }
  inAuto = newAuto;
}

void Initialize()
{
  lastInput = Input;
  ITerm = Output;
  if (ITerm > outMax) ITerm = outMax;
  else if (ITerm < outMin) ITerm = outMin;
}

void setup()
{
  Serial.begin(9600);

  pinMode(6, OUTPUT); //Motor A pin1
  pinMode(7, OUTPUT); //Motor A pin2
  pinMode(9, OUTPUT); //PWM Motor A
  pinMode(10, OUTPUT); //Brake
  pinMode(11, INPUT); //Motor B pin1
  pinMode(12, INPUT); //Motor B pin2
  pinMode(13, INPUT); //PWM Motor B

  pinMode(30, INPUT); //Sensors //L3
  pinMode(32, INPUT); //L2
  pinMode(34, INPUT); //L1
  pinMode(36, INPUT); //ML
  pinMode(38, INPUT); //MR
  pinMode(40, INPUT); //R1
  pinMode(42, INPUT); //R2
  pinMode(44, INPUT); //R3

  pinMode(25, OUTPUT); //LEDs //white
  pinMode(31, OUTPUT); //red
  pinMode(37, OUTPUT); //white
  pinMode(43, OUTPUT); //white
  pinMode(49, OUTPUT); //white
  pinMode(52, OUTPUT); //red

  digitalWrite(10, HIGH); //Brake disabled

  double c = count();
  Input = pos / c;
  Setpoint = 5000;
  SetSampleTime(18);
  SetOutputLimits(-maxi, maxi);
  SetMode(1);
  kp = (0.16);
  kd = (1.4); // for thinner more; for thicker less = 0. // 1
  ki = 0.0001;
}


void loop()
{
  double c = count();
  Input = pos;

  if (c != 0)
    Input = Input / c;

  if (c >= 4 || d >= 5 || (sensorL2 == lc && sensorR2 == lc)) // For Checkpoint  +
  {
    digitalWrite(stby, LOW);
    brake(1);
    digitalWrite(led6, HIGH);
    digitalWrite(led4, HIGH);
    delay(10);
    digitalWrite(led4, LOW);
    digitalWrite(led6, LOW);
    //  digitalWrite(stby, HIGH); // brake disengaged
    //  n++;
    //  if(n>20)
    //  while(1)
    //  {
    //    digitalWrite(led1,HIGH);
    //    digitalWrite(led2,HIGH);
    //    digitalWrite(led3,HIGH);
    //    digitalWrite(led4,HIGH);
    //    digitalWrite(led5,HIGH);
    //    digitalWrite(led6,HIGH);
    //    digitalWrite(stby, LOW); // brake permanent stop
    //   }
    return;
  }

  if (c == 0 && takeout == 0)
  {
    motors(-10, -10);
    delay(1);
    if (lastpd > 0 && lasts != 1)
    {
      motors(200, -200);
      delay(2);
      return;
    }
    else if (lastpd < 0 && lasts != 1)
    {
      motors(-200, 200);
      delay(2);
      return;
    }

    power_difference = lastpd;
    if (power_difference >= 0)
      motors(maxi - power_difference, maxi);
    else
      motors(maxi, maxi + power_difference);
    delay(10);
    return;
  }
  else if (c == 0 && takeout != 0)
  {
    present = millis();
    if (present - passed > 100)
      return;                                     // Forget the turn;
    if (takeout == 1)
    {
      lefttight();
      takeout = 0;
      return;
    }
    else if (takeout == 2)
    {
      righttight();
      takeout = 0;
      return;
    }
  }


  if (c >= 3)
  {
    if ((digitalRead(sensorL3) == lc) && (digitalRead(sensorL2) == lc && digitalRead(sensorL1) == lc)) // Left 90
    {
      if (c < 5)
        takeout = 1;
      passed = millis();
    }


    if ((digitalRead(sensorL2) == lc) && (digitalRead(sensorL1) == lc && digitalRead(sensormidL) == lc)) // Left 90
    {
      if (c < 5)
        takeout = 1;
      passed = millis();
    }

    else if ((digitalRead(sensorR2) == lc) && (digitalRead(sensorR1) == lc && digitalRead(sensormidR) == lc)) // Right 90
    {
      if (c < 5)
        takeout = 2;
      passed = millis();
    }

    else if ((digitalRead(sensorR3) == lc) && (digitalRead(sensorR2) == lc && digitalRead(sensorR1) == lc)) // Right 90
    {
      if (c < 5)
        takeout = 2;
      passed = millis();
    }

    else if (( digitalRead(sensorL2) == lc) && (digitalRead(sensormidL) == lc || digitalRead(sensormidR) == lc )) // Left 90
    {
      if (c < 5)
        takeout = 1;
      passed = millis();
    }

    else if (( digitalRead(sensorR2) == lc) && (digitalRead(sensormidL) == lc || digitalRead(sensormidR) == lc)) // Right 90
    {
      if (c < 5)
        takeout = 2;
      passed = millis();
    }
  }

  Compute();
  power_difference = Output;
  lastpd = power_difference;
  if (abs(power_difference) < 10)
    motors(255, 255);
  else if (power_difference > 0)
    motors(maxi - power_difference, maxi);
  else if (power_difference < 0)
    motors(maxi, maxi + power_difference);
}
