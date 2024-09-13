#include <LiquidCrystal_I2C.h>  // Include the LiquidCrystal_I2C library

/* Create an instance of the LCD display */
LiquidCrystal_I2C lcd(0x27, 16, 2);

/* Motors Driver */
#define MOTOR_L_IN1       11
#define MOTOR_L_IN2       10
#define MOTOR_R_IN1       9
#define MOTOR_R_IN2       6   
#define speedControlerR   5 
#define speedControlerL   3
#define buzzerPin         4
#define ir_model          2

int speedValue = 255;
int holder = 40;
char theOrder = ' ';
int sensorRead=digitalRead(ir_model);


/* LED Pins */
#define LEDF 13    // Green LED for Forward
#define LEDR 12    // Red LED for Backward
#define LEDO1 8    // Orange LED for Left Turn
#define LEDO2 7    // Orange LED for Right Turn

// LED and motor pin arrays for initialization
const int motorPins[] = {MOTOR_L_IN1, MOTOR_L_IN2, MOTOR_R_IN1, MOTOR_R_IN2, speedControlerR, speedControlerL};
const int ledPins[] = {LEDF, LEDR, LEDO1, LEDO2};

/* Movement Functions */
void MovingForward();
void MovingBack();
void TurningLeft();
void TurningRight();
void Brake();
void forwardRight(int speed);
void forwardLeft(int speed);
void backwardRight(int speed);
void backwardLeft(int speed);
void lcdSpeed();
void lcdMessage(const char* message);
void IR_buzzer();
void updateSpeed(int increment);
void setMotorState(int lIn1, int lIn2, int rIn1, int rIn2);
void IR_buzzer(int sensor);

void setup() {
  // MOTORS
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);
  pinMode(speedControlerR, OUTPUT);
  pinMode(speedControlerL, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(ir_model, INPUT);

  // Initialize LED pins
  pinMode(LEDF, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDO1, OUTPUT);
  pinMode(LEDO2, OUTPUT);

  Serial.begin(9600);
  Serial.println("Motor Control Program Initialized");
  Serial.print("Initial speed: ");
  Serial.println(speedValue);

  // Initialize LCD
  lcd.init();
  lcd.backlight();   // Turn on the LCD backlight
  lcd.setCursor(0, 0);
  lcd.print("ROBOSNAIL READY");
}

void loop() {

  sensorRead=digitalRead(ir_model);
    IR_buzzer(sensorRead);

  // Continuously read the sensor and control the buzzer/motor
  if (Serial.available()) {
    theOrder = Serial.read();
    // Print the received command
    Serial.print("Command received: ");
    Serial.println(theOrder);
    // Only process new commands
    if (theOrder) {
      

      // Process commands
      switch (theOrder) {
        case 'F': MovingForward(); lcdMessage("MOVING FORWARD"); break;
        case 'B': MovingBack(); lcdMessage("MOVING BACKWARD"); break;
        case 'L': TurningLeft(); lcdMessage("TURNING LEFT"); break;
        case 'R': TurningRight(); lcdMessage("TURNING RIGHT"); break;
        case 'K': Brake(); lcdMessage("STOP"); break;
        case 'I': updateSpeed(10); lcdSpeed(); break;
        case 'D': updateSpeed(-10); lcdSpeed(); break;
        case 'E': forwardRight(speedValue - holder); lcdMessage("LITTLE FORWARD RIGHT"); break;
        case 'Q': forwardLeft(speedValue - holder); lcdMessage("LITTLE FORWARD LEFT"); break;
        case 'C': backwardRight(speedValue - holder); lcdMessage("LITTLE BACKWARD RIGHT"); break;
        case 'Z': backwardLeft(speedValue - holder); lcdMessage("LITTLE  BACKWARD LEFT"); break;
      }

      // Limit speed value between 0 and 255
      speedValue = constrain(speedValue, 0, 255);

      // Update motor speed
      analogWrite(speedControlerR, speedValue);
      analogWrite(speedControlerL, speedValue);

    }
  }
}

// Movement functions
void MovingForward() {
  setMotorState(HIGH, LOW, HIGH, LOW);
  digitalWrite(LEDF, HIGH);  // Turn on GREEN LED for FORWARD
  digitalWrite(LEDR, LOW);   // Turn off RED LED
  digitalWrite(LEDO1, LOW);  // Turn off LEFT LED
  digitalWrite(LEDO2, LOW);  // Turn off RIGHT LED
}

void MovingBack() {
  setMotorState(LOW, HIGH, LOW, HIGH);
  digitalWrite(LEDF, LOW);   // Turn off GREEN LED
  digitalWrite(LEDR, HIGH);  // Turn on RED LED for BACKWARD
  digitalWrite(LEDO1, LOW);  // Turn off LEFT LED
  digitalWrite(LEDO2, LOW);  // Turn off RIGHT LED
}

void TurningLeft() {
  setMotorState(LOW, HIGH, HIGH, LOW);
  digitalWrite(LEDF, LOW);   // Turn off GREEN LED
  digitalWrite(LEDR, LOW);   // Turn off RED LED
  digitalWrite(LEDO1, HIGH); // Turn on LEFT LED
  digitalWrite(LEDO2, LOW);  // Turn off RIGHT LED
}

void TurningRight() {
  setMotorState(HIGH, LOW, LOW, HIGH);
  digitalWrite(LEDF, LOW);   // Turn off GREEN LED
  digitalWrite(LEDR, LOW);   // Turn off RED LED
  digitalWrite(LEDO1, LOW);  // Turn off LEFT LED
  digitalWrite(LEDO2, HIGH); // Turn on RIGHT LED
}

void Brake() {
  setMotorState(LOW, LOW, LOW, LOW);
  digitalWrite(LEDF, LOW);   // Turn off all LEDs
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDO1, LOW);
  digitalWrite(LEDO2, LOW);
}

void forwardRight(int speed) {
  analogWrite(MOTOR_R_IN1, speed);
  digitalWrite(MOTOR_L_IN1, HIGH);
  digitalWrite(MOTOR_L_IN2, LOW);
  digitalWrite(MOTOR_R_IN2, LOW);
  digitalWrite(LEDO2, HIGH);  // Turn on RIGHT LED
}

void forwardLeft(int speed) {
  analogWrite(MOTOR_R_IN1, HIGH);
  digitalWrite(MOTOR_L_IN1, speed);
  digitalWrite(MOTOR_L_IN2, LOW);
  digitalWrite(MOTOR_R_IN2, LOW);
  digitalWrite(LEDO1, HIGH);  // Turn on LEFT LED
}

void backwardRight(int speed) {
  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, HIGH);
  analogWrite(MOTOR_R_IN2, speed);
  digitalWrite(LEDO2, HIGH);  // Turn on RIGHT LED
}

void backwardLeft(int speed) {
  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_L_IN1, LOW);
  analogWrite(MOTOR_L_IN2, speed);
  digitalWrite(MOTOR_R_IN2, HIGH);
  digitalWrite(LEDO1, HIGH);  // Turn on LEFT LED
}

// Helper function to set motor states
void setMotorState(int lIn1, int lIn2, int rIn1, int rIn2) {
  digitalWrite(MOTOR_L_IN1, lIn1);
  digitalWrite(MOTOR_L_IN2, lIn2);
  digitalWrite(MOTOR_R_IN1, rIn1);
  digitalWrite(MOTOR_R_IN2, rIn2);
}

// Helper function to update speed
void updateSpeed(int increment) {
  speedValue += increment;
  Serial.print("Speed: ");
  Serial.println(speedValue);
}

// Helper function to print speed on LCD
void lcdSpeed() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Speed: ");
  lcd.print(speedValue);
}

// Helper function to print messages on LCD
void lcdMessage(const char* message) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(message);
}

void IR_buzzer(int sensor) {

  if (sensor == LOW) {  // If object detected (sensor value below threshold)
    digitalWrite(buzzerPin, HIGH);;  // Turn on buzzer
    Brake(); 
    delay(1000);
    MovingBack();
    delay(500);
    Brake(); 


     // Apply brake
  } 
  else 
  {
     digitalWrite(buzzerPin, LOW);;  // Turn on buzzer
}
}

