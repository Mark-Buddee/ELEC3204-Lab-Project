// constants won't change. They're used here to set pin numbers:



const int buttonPin0 = 5;  // the number of the pushbutton pin
const int buttonPin1 = 6;
const int buttonPin2 = 7;

const int segpinA = 8;
const int segpinB = 11;
const int segpinC = 10;
const int segpinD = 9;

//const int ledPin = 13;    // the number of the LED pin

// variables will change:
int buttonState0 = 0;  // variable for reading the pushbutton status
int buttonState1 = 0;
int buttonState2 = 0;
volatile int level = 0;





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

  // attachInterrupt(digitalPinToInterrupt(buttonPin0),but0,RISING);
  // attachInterrupt(digitalPinToInterrupt(buttonPin1),but1,RISING);
  // attachInterrupt(digitalPinToInterrupt(buttonPin2),but2,RISING);

}




// void but0(){
//   //here
//   level = 0;
 
// }

// void but1(){
//   level = 1;
 
// }

// void but2(){
//   level = 2;
 
// }




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

    digitalWrite(segpinA, LOW);
    digitalWrite(segpinB, LOW);
    digitalWrite(segpinC, LOW);
    digitalWrite(segpinD, LOW);


  }

  if (buttonState1 == HIGH) {
    //turn LED on:
    //digitalWrite(ledPin, HIGH);
    level = 1;
    digitalWrite(segpinA, HIGH);
    digitalWrite(segpinB, LOW);
    digitalWrite(segpinC, LOW);
    digitalWrite(segpinD, LOW);

  }

  if (buttonState2 == HIGH) {
    //turn LED on:
    //digitalWrite(ledPin, HIGH);
    level = 2;
    digitalWrite(segpinA, LOW);
    digitalWrite(segpinB, HIGH);
    digitalWrite(segpinC, LOW);
    digitalWrite(segpinD, LOW);

  }

  else {
    // turn LED off:
    //digitalWrite(ledPin, LOW);
  }

  Serial.print("You have selected level:");
  Serial.println(level);

  
  //delay(150);
}
