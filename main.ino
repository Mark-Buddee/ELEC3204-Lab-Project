//------------------------- definitions ----------------------------//
// Choose which information to output to serial
#define USER            1
#define PID             1
#define DEBUG           1

// Serial monitor
#define BAUD_RATE 115200

// Pin numbers
#define ENC1            2
#define ENC2            4
#define BUT0            5
#define BUT1            6
#define BUT2            7
#define PWMA            9
#define PWMB            10

// Physical parameters
#define CLICKS_PER_REV  1300.0         // encoder clicks in one revolution
#define DIAM            0.012          // diameter of motor shaft (m)
#define PI              3.14159        // pi
#define CIRCUMFERENCE   (DIAM * PI)    // circumference of motor shaft
#define FLOOR_HEIGHT     0.2           // height between floors (m)

// Tuning parameters
#define GAIN            100
#define MARGIN_OF_ERROR 0.03          // allowable deviation from desired elevator height (m)
#define TRAVEL_SPEED    0.04          // nominal travel speed of elevator (m/s)

// Magic numbers
#define UPWARDS         1
#define DOWNWARDS       -1

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
volatile int direction = UPWARDS;
volatile long int clicks = 0;

// Initial state
int desiredFloors[3] = {0, 0, 0};
double motorOut = 0;

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
  pinMode(ENC1,INPUT);   
  pinMode(ENC2,INPUT);
  pinMode(BUT0, INPUT);
  pinMode(BUT1, INPUT);
  pinMode(BUT2, INPUT);

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
  double f_enc = dt > 7000 ? 0 : 1000000/dt;                                    // frequency of encoder signal (Hz)
  double actualVelocity = f_enc * direction * CIRCUMFERENCE / CLICKS_PER_REV;   // vertical velocity of elevator (m/s)
  double rotations = clicks / CLICKS_PER_REV;                                   // revolution count of motor shaft (revs)
  double height = rotations * CIRCUMFERENCE;                                    // height of elevator (m)

  if(digitalRead(BUT0) == HIGH) desiredFloors[GROUND] = 1;
  if(digitalRead(BUT1) == HIGH) desiredFloors[FLOOR1] = 1;
  if(digitalRead(BUT2) == HIGH) desiredFloors[FLOOR2] = 1;

  int floor = FLOOR(height);
  if(floor == GROUND) desiredFloors[GROUND] = 0;
  if(floor == FLOOR1) desiredFloors[FLOOR1] = 0;
  if(floor == FLOOR2) desiredFloors[FLOOR2] = 0;

  // Update user interface
  doKeypadLights(desiredFloors);
  doFloorNumber(floor);

  // Control
  int desiredFloor = getDesiredFloor(desiredFloors, height);
  double desiredVelocity = getDesiredVelocity(desiredFloor, height);
  double vDiff = desiredVelocity - actualVelocity;
  double motorChange = GAIN * vDiff;

  // Output
  motorOut += motorChange;
  PWM(50 + motorOut);

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
    Serial.print(desiredVelocity, 5);
    Serial.print(" Actual velocity (m/s): ");
    Serial.print(actualVelocity, 5);
    Serial.print(" Motor change: ");
    Serial.print(motorChange, 2);
    Serial.print(" Motor out: ");
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
  }
  delay(100);
}

//------------------------- subroutine PWM generate complementary PWM from OCR1A and OCR1B ----------------------------//
void PWM(double pwm)
{
  double temp = (int)pwm;
  temp = constrain(temp,0,100);

  OCR1A = temp;                         //duty of PWM for pin9 is from output compare register A 
  TCCR1A |= _BV(COM1A1) | _BV(COM1A0);  //set output to low level

  OCR1B = temp;                         //duty of PWM for pin10 is from output compare register B
  TCCR1A |= _BV(COM1B1);                //set output to high level

  TCCR1B |= _BV(WGM13);
  TCCR1A |= _BV(WGM11);                 //Set ICR1 phas correct mode
}

//------------------------- various parameter subroutines ----------------------------//
int getDesiredFloor(int desiredFloors[3], int height) {
  return 1;
}

int getDesiredDirection(int desiredFloor, double height)
{
  double difference = desiredFloor * FLOOR_HEIGHT - height;
  if(difference >= MARGIN_OF_ERROR) {
    return 1;
  }
  if(difference <= -MARGIN_OF_ERROR) {
    return -1;
  }
  return 0;
}

double getDesiredVelocity(int desiredFloor, double height)
{
  int desiredDirection = getDesiredDirection(desiredFloor, height);
  return TRAVEL_SPEED * desiredDirection;
}

//------------------------- user interface subroutines ----------------------------//
void doKeypadLights(int desiredFloors[3]);
void doFloorNumber(int floor);

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
