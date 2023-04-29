 // constants won't change. They're used here to set pin numbers:
const int buttonPin0 = 3;  // the number of the pushbutton pin
const int buttonPin1 = 6;
const int buttonPin2 = 7;
//const int ledPin = 13;    // the number of the LED pin

// variables will change:
int buttonState0 = 0;  // variable for reading the pushbutton status
int buttonState1 = 0;
int buttonState2 = 0;
int level = 0;

void setup() {
  // initialize the LED pin as an output:
  //pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin0, INPUT);

   // initialize the LED pin as an output:
  //pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin1, INPUT);

   // initialize the LED pin as an output:
  //pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin2, INPUT);


  Serial.begin(9600);
}

void loop() {
  // read the state of the pushbutton value:
  buttonState0 = digitalRead(buttonPin0);
  buttonState1 = digitalRead(buttonPin1);
  buttonState2 = digitalRead(buttonPin2);

  //Serial.println(buttonState0);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState0 == HIGH) {
    //turn LED on:
    //digitalWrite(ledPin, HIGH);
    level = 0;
  }

  if (buttonState1 == HIGH) {
    //turn LED on:
    //digitalWrite(ledPin, HIGH);
    level = 1;
  }

  if (buttonState2 == HIGH) {
    //turn LED on:
    //digitalWrite(ledPin, HIGH);
    level = 2;
  }

  else {
    // turn LED off:
    //digitalWrite(ledPin, LOW);
  }

  Serial.print("You have selected level:");
  Serial.println(level);

}
