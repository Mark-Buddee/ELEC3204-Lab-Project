//------------------------- definitions ----------------------------//
// Choose which information to output to serial
#define USER            0
#define PID             0
#define DEBUG           1

// Pin numbers
#define ENC1            2
#define ENC2            4
#define BUT0            5
#define BUT1            6
#define BUT2            7
#define PWMA            9
#define PWMB            10
#define LED0            A1
#define LED1            A2
#define LED2            A3
#define SEGA            12
#define SEGB            8
#define SEGC            13
#define SEGD            11

// Physical parameters
#define CLICKS_PER_REV  1300.0        // encoder clicks in one revolution
#define DIAM            0.012         // diameter of motor shaft (m)
#define PI              3.14159       // pi
#define CIRCUMFERENCE   (DIAM * PI)   // circumference of motor shaft
#define FLOOR_HEIGHT    0.15          // height between floors (m)

// Tuning parameters
#define GAIN            120
#define MARGIN_OF_ERROR 0.03            // allowable deviation from desired elevator height (m)
#define TRAVEL_SPEED    0.04          // nominal travel speed of elevator (m/s)

// Magic numbers
#define BAUD_RATE 115200

#define UPWARDS         1
#define STATIONARY      0
#define DOWNWARDS       -1

#define IDLE            -2
#define IN_TRANSIT      -1
#define GROUND          0
#define FLOOR1          1
#define FLOOR2          2

#define GROUND_HEIGHT   (GROUND * FLOOR_HEIGHT)
#define FLOOR1_HEIGHT   (FLOOR1 * FLOOR_HEIGHT)
#define FLOOR2_HEIGHT   (FLOOR2 * FLOOR_HEIGHT)

// Declare variables that are accessed by interrupts
volatile int t0 = 0;
volatile int t1 = 1;
volatile int dt;
volatile int direction = STATIONARY;
volatile long int clicks = 0;

// Initial state
int desiredFloors[3] = {0, 0, 0};
double motorOut = 0;
int lastFloor = GROUND;

// Macros
#define FLOOR(height) (                                                                                                 \
  ((height) >= GROUND_HEIGHT - MARGIN_OF_ERROR) && ((height) <= GROUND_HEIGHT + MARGIN_OF_ERROR) ? GROUND :             \
  ((height) >= FLOOR1_HEIGHT - MARGIN_OF_ERROR) && ((height) <= FLOOR1_HEIGHT + MARGIN_OF_ERROR) ? FLOOR1 :             \
  ((height) >= FLOOR2_HEIGHT - MARGIN_OF_ERROR) && ((height) <= FLOOR2_HEIGHT + MARGIN_OF_ERROR) ? FLOOR2 : IN_TRANSIT  \
)

//------------------------- setup routine ----------------------------//
void setup()
{
  // Initialise serial monitor
  Serial.begin(BAUD_RATE);

  // Initialise input pins
  pinMode(ENC1, INPUT);   
  pinMode(ENC2, INPUT);
  pinMode(BUT0, INPUT);
  pinMode(BUT1, INPUT);
  pinMode(BUT2, INPUT);

  // Initialise user interface output pins
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);  

  pinMode(SEGA, OUTPUT);
  pinMode(SEGB, OUTPUT);
  pinMode(SEGC, OUTPUT);
  pinMode(SEGD, OUTPUT);  

  // Initialise PWM outputs
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  // Initialise timer for PWM
  TCCR1A = 0;             // clear Timer1 control register TCCR1A & B
  TCCR1B = 0;
  TCNT1 = 0;              // clear Timer1 counter register
  TCCR1B |= _BV(CS11);    // set prescaler to 8. So clock frequency=16MHz/8=2MHz
  ICR1 = 100;             // phase correct PWM. PWM frequency determined by counting up 0-100 and counting down 100-0 in the input compare register (ICR1), so freq=200*0.5us=10kHz 

  // Initialise ISR
  attachInterrupt(digitalPinToInterrupt(ENC1),encRise,RISING);
}

//------------------------- main loop ----------------------------//
void loop() 
{
  // Data
  double f_enc = dt > 18000 ? 0 : 1000000/dt;                                    // frequency of encoder signal (Hz)
  double actualVelocity = f_enc * direction * CIRCUMFERENCE / CLICKS_PER_REV;   // vertical velocity of elevator (m/s)
  double rotations = clicks / CLICKS_PER_REV;                                   // revolution count of motor shaft (revs)
  double height = rotations * CIRCUMFERENCE;                                    // height of elevator (m)

  if(digitalRead(BUT0) == HIGH) desiredFloors[GROUND] = 1;
  if(digitalRead(BUT1) == HIGH) desiredFloors[FLOOR1] = 1;
  if(digitalRead(BUT2) == HIGH) desiredFloors[FLOOR2] = 1;

  int floor = FLOOR(height);
  if(floor == GROUND) lastFloor = GROUND;
  if(floor == FLOOR1) lastFloor = FLOOR1;
  if(floor == FLOOR2) lastFloor = FLOOR2;

  // Update user interface
  doKeypadLights(desiredFloors);
  doFloorNumber(lastFloor);

  // Control
  int desiredFloor = getDesiredFloor(desiredFloors, height);
  double desiredVelocity = getDesiredVelocity(desiredFloor, height);
  double vDiff = desiredVelocity - actualVelocity;
  double motorChange = GAIN * vDiff;

  // Output
  motorOut += motorChange;
  PWM(50 + motorOut);

  // Floor is reached
  if(floor == desiredFloor && actualVelocity == 0) {
    Serial.println("Floor reached. Waiting 3 seconds...");
    delay(3000);
    desiredFloors[desiredFloor] = 0;
  }

  // Serial output
  if(USER) {
    Serial.print("Floor number: ");
    Serial.print(floor);
    Serial.print(" Direction: ");
    direction == UPWARDS    ? Serial.print("upwards")   :
    direction == DOWNWARDS  ? Serial.print("downwards") :
                              Serial.print("in transit");
    Serial.print(" Going to: ");
    desiredFloor == GROUND ? Serial.println("Ground")  :
    desiredFloor == FLOOR1 ? Serial.println("Floor 1") :
    desiredFloor == FLOOR2 ? Serial.println("Floor 2") : 
                             Serial.println("Error");
  }
  if(PID) {
    Serial.print("Desired velocity (m/s): ");
    Serial.print(desiredVelocity, 3);
    Serial.print("    Actual velocity (m/s): ");
    Serial.print(actualVelocity, 3);
    Serial.print("    Velocity difference: ");
    Serial.print(vDiff);
    Serial.print("    Motor change: ");
    Serial.print(motorChange, 2);
    Serial.print("    Motor out: ");
    Serial.println(motorOut, 2);    
  }
  if(DEBUG) {
    Serial.print(" f_enc: ");
    Serial.print(f_enc);
    Serial.print(" Height: ");
    Serial.print(height);
    Serial.print(" Ground: ");
    Serial.print(desiredFloors[GROUND]);
    Serial.print(" Floor 1: ");
    Serial.print(desiredFloors[FLOOR1]);
    Serial.print(" Floor 2: ");
    Serial.print(desiredFloors[FLOOR2]);
    Serial.print(" Last floor : ");
    Serial.println(lastFloor);
  }
  delay(100);
}


//------------------------- subroutine PWM generate complementary PWM from OCR1A and OCR1B ----------------------------//
void PWM(double pwm)
{
  double temp = (int)pwm;
  temp = constrain(temp, 0, 100);

  OCR1A = temp;                         //duty of PWM for pin9 is from output compare register A 
  TCCR1A |= _BV(COM1A1) | _BV(COM1A0);  //set output to low level

  OCR1B = temp;                         //duty of PWM for pin10 is from output compare register B
  TCCR1A |= _BV(COM1B1);                //set output to high level

  TCCR1B |= _BV(WGM13);
  TCCR1A |= _BV(WGM11);                 //Set ICR1 phas correct mode
}


//------------------------- various parameter subroutines ----------------------------//
int getDesiredFloor(int desiredFloors[3], double height)
{ 
  // Returns the closest floor that is currently requested. Returns IDLE if none are requested
  // Super janky method but logic checks out to me
  float currentFloor = height / FLOOR_HEIGHT;
  int closestFloor = constrain(round(currentFloor), GROUND, FLOOR2);

  // none are requested 
  if(!desiredFloors[GROUND] && !desiredFloors[FLOOR1] && !desiredFloors[FLOOR2]) {
    return closestFloor; // TODO
  }
  
  // closest floor is requested
  if(desiredFloors[closestFloor]) {
    return closestFloor;
  }

  // second closest is requested
  int secondClosest = currentFloor <  0.5 ? 1 :
                      currentFloor >= 1.5 ? 1 :
                      currentFloor >= 1.0 ? 2 :
                                            0;
  if(desiredFloors[secondClosest]) {
    return secondClosest;
  }

  // only one floor is requested
  return GROUND + FLOOR1 + FLOOR2 - closestFloor - secondClosest;
}

int getDesiredDirection(int desiredFloor, double height)
{
  // Returns the required direction of travel to reach the desired floor
  double difference = desiredFloor * FLOOR_HEIGHT - height;
  if(difference >= MARGIN_OF_ERROR) {
    return UPWARDS;
  }
  if(difference <= -MARGIN_OF_ERROR) {
    return DOWNWARDS;
  }
  return STATIONARY;
}

double getDesiredVelocity(int desiredFloor, double height)
{
  // Returns the desired, signed velocity of the elevator
  int desiredDirection = getDesiredDirection(desiredFloor, height);
  return TRAVEL_SPEED * desiredDirection;
}

//------------------------- user interface subroutines ----------------------------//
void doKeypadLights(int desiredFloors[3]) 
{
  // Ground LED
  if(desiredFloors[GROUND]) {
    digitalWrite(LED0, HIGH);
  } else {
    digitalWrite(LED0, LOW);
  }

  // Floor 1 LED
  if(desiredFloors[FLOOR1]) {
    digitalWrite(LED1, HIGH);
  } else {
    digitalWrite(LED1, LOW);
  }

  // Floor 2 LED
  if(desiredFloors[FLOOR2]) {
    digitalWrite(LED2, HIGH);
  } else {
    digitalWrite(LED2, LOW);
  }
}
void doFloorNumber(int lastFloor)
{
  if(lastFloor == GROUND) {
    digitalWrite(SEGA, HIGH);
    digitalWrite(SEGB, HIGH);
    digitalWrite(SEGC, HIGH);
    digitalWrite(SEGD, HIGH);
    return;
  }
  if(lastFloor == FLOOR1) {
    digitalWrite(SEGA, LOW);
    digitalWrite(SEGB, HIGH);
    digitalWrite(SEGC, HIGH);
    digitalWrite(SEGD, HIGH);
    return;
  }
  if(lastFloor == FLOOR2) {
    digitalWrite(SEGA, HIGH);
    digitalWrite(SEGB, LOW);
    digitalWrite(SEGC, HIGH);
    digitalWrite(SEGD, HIGH);
    return;
  }

}

//------------------------- interrupt subroutine on encoder channel 1 rising edge ----------------------------//
void encRise()
{
  // Get direction
  if(digitalRead(ENC2) == LOW) {
    direction = UPWARDS;
  } else if(digitalRead(ENC2) == HIGH) {
    direction = DOWNWARDS;
  } else {
    Serial.println("Could not read state of ENC2");
    exit(1);
  }

  // Get period of signal
  t1 = micros();
  dt = t1 - t0;
  t0 = micros();

  // Update click counter
  clicks += direction;
}
