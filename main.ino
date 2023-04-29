// Part of this code generates a pair of complemenatry PWM pulse using Timer 1 registers. The register ICR1 is for frequency,  currently of 100  for 10kHz.
// The duty is determined by the register OCR1A and OCR1B

#define PWMA 9
#define PWMB 10
#define INPIN1 2
#define INPIN2 4

volatile int t = 0;
volatile int T;
volatile int dt;

volatile int direction;
volatile long int clicks = 0;


//------------------------- setup routine ----------------------------//
void setup()
{
  Serial.begin(9600);
  pinMode(PWMA, OUTPUT);          // output PWMA to Q1
  pinMode(PWMB, OUTPUT);          // output PWMB to Q2
  pinMode(INPIN1,INPUT);   
  pinMode(INPIN2,INPUT);

  analogWrite(PWMA, 0);          // let PWMA=0
  analogWrite(PWMB, 0);          // let PWMB=0

  TCCR1A = 0; // clear Timer1 control register TCCR1A & B
  TCCR1B = 0;
  TCNT1 = 0; // clear Timer1 counter register

  TCCR1B |= _BV(CS11); //set prescaler=8 by lettin the bit value of CS11=1 in register TCCR1B, so the clock frequency=16MHz/8=2MHz
  ICR1 = 100;//  phase correct PWM. PWM frequency determined by counting up 0-100 and counting down 100-0 in the input compare register (ICR1), so freq=200*0.5us=10kHz 

  attachInterrupt(digitalPinToInterrupt(INPIN1),edgeRise,RISING);
}

//------------------------- main loop ----------------------------//
void loop() 
{
  PWM(1); //set duty of PWM as 30%

  // Print the direction to the serial monitor
  Serial.print("  direction = ");
  Serial.print(direction);

  // Print the clicks to the serial monitor
  Serial.print("  clicks = ");
  Serial.print(clicks);

  float rotations = clicks/1623.67;
  Serial.print("  rotations = ");
  Serial.print(rotations);

  // Print frequency to the serial monitor
  float f = 1000000/dt;
  Serial.print("  f = ");
  Serial.println(f);
  delay(150);
}

//------------------------- subroutine PWM generate complementary PWM from OCR1A and OCR1B ----------------------------//
void PWM(int pwm)
{
  int temp = (int)pwm;
  temp = constrain(temp,1,99);

  OCR1A = temp; //duty of PWM for pin9 is from output compare register A 
  TCCR1A |= _BV(COM1A1) | _BV(COM1A0); //set output to low level

  OCR1B = temp;//duty of PWM for pin10 is from output compare register B
  TCCR1A |= _BV(COM1B1); //set output to high level

  TCCR1B |= _BV(WGM13); //
  TCCR1A |= _BV(WGM11); //Set ICR1 phas correct mode
}

//------------------------- interrupt subroutine on rising edge of input pin 1 ----------------------------//
void edgeRise() {

  // Get direction
  if(digitalRead(INPIN2) == LOW) {
    direction = 1;
  } else if(digitalRead(INPIN2) == HIGH) {
    direction = -1;
  } else {
    Serial.println("Could not read state of INPIN2");
    exit(1);
  }

  // Get period of signal
  T = micros();
  dt = T - t;
  t = micros();

  // Update click counter
  clicks += direction;
}
