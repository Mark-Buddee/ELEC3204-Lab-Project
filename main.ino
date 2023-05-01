// Part of this code generates a pair of complemenatry PWM pulse using Timer 1 registers. The register ICR1 is for frequency,  currently of 100  for 10kHz.
// The duty is determined by the register OCR1A and OCR1B

#define PWMA 9
#define PWMB 10
#define ENC1 2
#define ENC2 4
#define BUT0 5
#define BUT1 6
#define BUT2 7

#define CLICKSPERREV 1300.0
#define DIAM 0.012 // in metres
#define PI 3.14159
#define CIRCUM DIAM*PI
#define FLOORHEIGHT 0.2

#define GAIN 100
#define RANGE 0.03

volatile int t = 0;
volatile int T = 1;
volatile int dt;

volatile int direction = 1;
volatile long int clicks = 0;

volatile int desiredFloor;

double output = 0;


//------------------------- setup routine ----------------------------//
void setup()
{
  Serial.begin(9600);
  pinMode(PWMA, OUTPUT);          // output PWMA to Q1
  pinMode(PWMB, OUTPUT);          // output PWMB to Q2
  pinMode(ENC1,INPUT);   
  pinMode(ENC2,INPUT);
  pinMode(BUT0, INPUT);
  pinMode(BUT1, INPUT);
  pinMode(BUT2, INPUT);

  analogWrite(PWMA, 0);          // let PWMA=0
  analogWrite(PWMB, 0);          // let PWMB=0

  TCCR1A = 0; // clear Timer1 control register TCCR1A & B
  TCCR1B = 0;
  TCNT1 = 0; // clear Timer1 counter register

  TCCR1B |= _BV(CS11); //set prescaler=8 by lettin the bit value of CS11=1 in register TCCR1B, so the clock frequency=16MHz/8=2MHz
  ICR1 = 100;//  phase correct PWM. PWM frequency determined by counting up 0-100 and counting down 100-0 in the input compare register (ICR1), so freq=200*0.5us=10kHz 

  attachInterrupt(digitalPinToInterrupt(ENC1),encRise,RISING);
  attachInterrupt(digitalPinToInterrupt(BUT0),but0Rise,RISING);
  attachInterrupt(digitalPinToInterrupt(BUT1),but1Rise,RISING);
  attachInterrupt(digitalPinToInterrupt(BUT2),but2Rise,RISING);
}

//------------------------- main loop ----------------------------//
void loop() 
{
  // Data
  double f;
  if(dt > 7000) {
    f = 0;
  } else {
    f = 1000000/dt;  // Hz
  }
  double actualVelocity = f * direction * CIRCUM / CLICKSPERREV;  // m/s
  double rotations = clicks/CLICKSPERREV;  // revolutions
  double position = rotations * CIRCUM;  // m above starting point

  // Calculation
  int buttonState0 = digitalRead(BUT0);
  int buttonState1 = digitalRead(BUT1);
  int buttonState2 = digitalRead(BUT2);

  if(buttonState0 == HIGH) {
    desiredFloor = 0;
  }
  if(buttonState1 == HIGH) {
    desiredFloor = 1;
  }
  if(buttonState2 == HIGH) {
    desiredFloor = 2;
  }

  // Control
  double desiredVelocity = getDesiredVelocity(desiredFloor, position);
  double vDiff = desiredVelocity - actualVelocity;
  double motorChange = GAIN*vDiff;
  output += motorChange;
  PWM(50 + output);

  // Print debug info to serial monitor
  // "direction = " + direction + ". clicks = " + clicks + ". rotations = " + rotations + ". desiredVelocity = " + desiredVelocity + "
  Serial.println((String)"position = " + position + ". f = " + f + ". actualVelocity = " + actualVelocity + ". motorChange = " + motorChange);
  // Serial.println((String)"desiredFloor = " + desiredFloor);
  delay(100);
}

//------------------------- subroutine PWM generate complementary PWM from OCR1A and OCR1B ----------------------------//
void PWM(double pwm)
{
  double temp = pwm;
  temp = constrain(temp,1,99);

  OCR1A = temp; //duty of PWM for pin9 is from output compare register A 
  TCCR1A |= _BV(COM1A1) | _BV(COM1A0); //set output to low level

  OCR1B = temp;//duty of PWM for pin10 is from output compare register B
  TCCR1A |= _BV(COM1B1); //set output to high level

  TCCR1B |= _BV(WGM13); //
  TCCR1A |= _BV(WGM11); //Set ICR1 phas correct mode
}

// int getDesiredFloor(volatile bool desiredFloors[3], int position);

int getDesiredDirection(int desiredFloor, double position)
{
  double difference = desiredFloor * FLOORHEIGHT - position;
  if(difference > RANGE) {
    return 1;
  } else if(difference < -RANGE) {
    return -1;
  } else {
    return 0;
  }
}

double getDesiredVelocity(int desiredFloor, double position) // We have a function for this in case we want to complicate the function for velocity. ie a non-constant velocity
{
  int desiredDirection = getDesiredDirection(desiredFloor, position);
  return 0.04 * desiredDirection;
}

//------------------------- interrupt subroutines on rising edges ----------------------------//
void encRise()
{
  // Get direction
  if(digitalRead(ENC2) == LOW) {
    direction = 1;
  } else if(digitalRead(ENC2) == HIGH) {
    direction = -1;
  } else {
    Serial.println("Could not read state of ENC2");
    exit(1);
  }

  // Get period of signal
  T = micros();
  dt = T - t;
  t = micros();

  // Update click counter
  clicks += direction;
}

void but0Rise()
{
  desiredFloor = 0;
  Serial.println("bang");
}

void but1Rise()
{
  desiredFloor = 1;
  Serial.println("bang1");
}

void but2Rise()
{
  desiredFloor = 2;
  Serial.println("bang2");
}
