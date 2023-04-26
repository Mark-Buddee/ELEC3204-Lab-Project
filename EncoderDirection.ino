// Define the pins for the quadrature encoder inputs
const int encoderPinA = 2;
const int encoderPinB = 3;

// Define variables to hold the current and previous encoder states
int currentEncoderState = 0;
int previousEncoderState = 0;

void setup() {
  // Set the encoder pins as inputs
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  // Enable pull-up resistors for the encoder pins
  digitalWrite(encoderPinA, HIGH);
  digitalWrite(encoderPinB, HIGH);

  // Set up serial communication
  Serial.begin(9600);
}

void loop() {
  // Read the current state of the encoder pins
  int encoderStateA = digitalRead(encoderPinA);
  int encoderStateB = digitalRead(encoderPinB);

  // Combine the encoder states into a single value
  currentEncoderState = (encoderStateA << 1) | encoderStateB;

  // Determine the direction of the encoder movement based on the change in state
  int direction = 0;
  if (previousEncoderState == 0b00 && currentEncoderState == 0b01) {
    direction = 1;
  } else if (previousEncoderState == 0b01 && currentEncoderState == 0b00) {
    direction = -1;
  }

  // Print the direction to the serial monitor
  Serial.println(direction);

  // Update the previous encoder state
  previousEncoderState = currentEncoderState;
}
