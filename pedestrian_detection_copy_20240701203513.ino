// Define stepper motor control pins
#define stepPin 3    // STEP pin of stepper driver connected to digital pin 3
#define dirPin 4     // DIR pin of stepper driver connected to digital pin 4
#define enablePin 9  // ENABLE pin of stepper driver connected to digital pin 9

void setup() {
  // Set the motor control pins as outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);

  // Enable the stepper motor driver (active low)
  digitalWrite(enablePin, LOW);

  // Initialize serial communication at 9600 baud rate
  Serial.begin(9600);
}

void loop() {
  // Check if data is available to read
  if (Serial.available() > 0) {
    char state = Serial.read();

    if (state == 'S') {
      // Stop the stepper motor if 'S' is received
      digitalWrite(enablePin, HIGH);  // Disable motor driver to stop
    } else if (state == 'G') {
      // Rotate the stepper motor if 'G' is received
      digitalWrite(enablePin, LOW);   // Enable motor driver

      // Set direction (clockwise or counter-clockwise)
      digitalWrite(dirPin, HIGH);  // Set direction to clockwise

      // Step the motor (you may want to adjust the number of steps per loop)
      for (int i = 0; i < 200; i++) {  // 200 steps for one full revolution (adjust as per your motor)
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(500);  // Adjust this delay for desired speed
        digitalWrite(stepPin, LOW);
        delayMicroseconds(500);  // Adjust this delay for desired speed
      }
    }
  }
}
