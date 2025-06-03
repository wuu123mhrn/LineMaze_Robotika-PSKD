#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Motor pins
#define rmf 10  // IN1 (Right Motor Forward)
#define rmb 11  // IN2 (Right Motor Backward)
#define lmf 6   // IN3 (Left Motor Forward)
#define lmb 9   // IN4 (Left Motor Backward)

// Multiplexer pins
#define s0 2
#define s1 3
#define s2 4
#define s3 5
#define sig A6

// Motor speed settings
#define BASE_SPEED 50
#define MAX_SPEED 75
#define TURN_SPEED 50
#define TURN_DELAY 500  // Delay for turns (ms)

// Sensor thresholds (individual for each sensor)
int thresholds[12] = {600, 600, 500, 500, 500, 500, 500, 500, 500, 500, 600, 600};

// Variables
int sensorValues[12]; // Analog readings
int binarySensors[12]; // Binary readings (0 or 1)
unsigned long bin_Sensor; // Binary sensor state as a single value
bool leftActive, frontActive, rightActive; // Group states
String currentAction = "None"; // Track current action for display
float lineError = 0; // Error for line following

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Maze Solver (Centered)"));
  display.display();

  // Set motor pins as output
  pinMode(rmf, OUTPUT);
  pinMode(rmb, OUTPUT);
  pinMode(lmf, OUTPUT);
  pinMode(lmb, OUTPUT);

  // Set multiplexer pins as output
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(sig, INPUT);
}

void loop() {
  readSensors();
  evaluateSensorGroups();
  navigateMaze();
  updateDisplay();
  updateSerial();
  delay(10); // Small delay for stability
}

void readSensors() {
  bin_Sensor = 0;
  for (int i = 0; i < 12; i++) {
    // Select multiplexer channel
    digitalWrite(s0, bitRead(i, 0));
    digitalWrite(s1, bitRead(i, 1));
    digitalWrite(s2, bitRead(i, 2));
    digitalWrite(s3, bitRead(i, 3));

    // Read analog value
    sensorValues[i] = analogRead(sig);
    // Convert to binary based on individual threshold
    binarySensors[i] = (sensorValues[i] > thresholds[i]) ? 1 : 0;
    // Build binary sensor state
    bin_Sensor |= (binarySensors[i] << (11 - i));
  }
}

void evaluateSensorGroups() {
  // Left group: sensors 1, 2, 3 (indices 0, 1, 2)
  leftActive = (binarySensors[0] || binarySensors[1] || binarySensors[2]);

  // Front group: sensors 4, 5, 6, 7, 8, 9 (indices 3, 4, 5, 6, 7, 8)
  frontActive = (binarySensors[3] || binarySensors[4] || binarySensors[5] ||
                 binarySensors[6] || binarySensors[7] || binarySensors[8]);

  // Right group: sensors 10, 11, 12 (indices 9, 10, 11)
  rightActive = (binarySensors[9] || binarySensors[10] || binarySensors[11]);
}

void navigateMaze() {
  // Left-hand rule based on sensor groups
  if (leftActive) { // Prioritize left turn if left is active
    currentAction = "Left";
    turnLeft();
  } else if (frontActive) { // Go straight if front is active and left is not
    currentAction = "Straight";
    goStraight();
  } else if (!leftActive && !frontActive && !rightActive) { // U-turn if no sensors active
    currentAction = "U-Turn";
    uTurn();
  } else { // Default: follow line
    currentAction = "Follow";
    followLine();
  }
}

void goStraight() {
  driveMotors(BASE_SPEED, BASE_SPEED);
}

void turnLeft() {
  driveMotors(-TURN_SPEED, TURN_SPEED); // Spin left
  delay(TURN_DELAY); // Adjust delay based on testing
  driveMotors(0, 0); // Stop briefly
}

void turnRight() {
  driveMotors(TURN_SPEED, -TURN_SPEED); // Spin right
  delay(TURN_DELAY); // Adjust delay based on testing
  driveMotors(0, 0); // Stop briefly
}

void uTurn() {
  driveMotors(TURN_SPEED, -TURN_SPEED); // Spin right for U-turn
  delay(TURN_DELAY * 2); // Longer delay for 180-degree turn
  driveMotors(0, 0); // Stop briefly
}

void followLine() {
  // Line-following logic centered on sensors 6 and 7 (indices 5, 6)
  lineError = 0;
  if (binarySensors[5] && binarySensors[6]) { // Center: sensors 6 and 7 active
    lineError = 0;
  } else if (binarySensors[4]) { // Slightly left: sensor 5
    lineError = -1;
  } else if (binarySensors[7]) { // Slightly right: sensor 8
    lineError = 1;
  } else if (binarySensors[3]) { // More left: sensor 4
    lineError = -2;
  } else if (binarySensors[8]) { // More right: sensor 9
    lineError = 2;
  } else if (binarySensors[2]) { // Far left: sensor 3
    lineError = -3;
  } else if (binarySensors[9]) { // Far right: sensor 10
    lineError = 3;
  } else {
    // No relevant sensors active, maintain straight movement
    driveMotors(BASE_SPEED, BASE_SPEED);
    return;
  }

  // Adjust motor speeds based on error
  int leftSpeed = BASE_SPEED - (lineError * 15); // Reduced multiplier for smoother control
  int rightSpeed = BASE_SPEED + (lineError * 15);
  leftSpeed = constrain(leftSpeed, 0, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);
  driveMotors(leftSpeed, rightSpeed);
}

void driveMotors(int leftSpeed, int rightSpeed) {
  // Left motor
  if (leftSpeed > 0) {
    analogWrite(lmf, leftSpeed);
    analogWrite(lmb, 0);
  } else {
    analogWrite(lmf, 0);
    analogWrite(lmb, -leftSpeed);
  }

  // Right motor
  if (rightSpeed > 0) {
    analogWrite(rmf, rightSpeed);
    analogWrite(rmb, 0);
  } else {
    analogWrite(rmf, 0);
    analogWrite(rmb, -rightSpeed);
  }
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(F("Action: "));
  display.println(currentAction);
  display.print(F("L:"));
  display.print(leftActive ? "1" : "0");
  display.print(F(" F:"));
  display.print(frontActive ? "1" : "0");
  display.print(F(" R:"));
  display.println(rightActive ? "1" : "0");
  display.print(F("Error: "));
  display.println(lineError);
  display.print(F("Sensors: "));
  for (int i = 0; i < 12; i++) {
    display.print(binarySensors[i]);
  }
  display.println();
  display.println(F("Analog:"));
  for (int i = 0; i < 12; i++) {
    display.print(sensorValues[i]);
    if (i < 11) display.print(F(" "));
    if (i == 5) display.println(); // Split into two lines
  }
  display.display();
}

void updateSerial() {
  Serial.print("Action: ");
  Serial.println(currentAction);
  Serial.print("Groups: L=");
  Serial.print(leftActive ? "1" : "0");
  Serial.print(" F=");
  Serial.print(frontActive ? "1" : "0");
  Serial.print(" R=");
  Serial.println(rightActive ? "1" : "0");
  Serial.print("Error: ");
  Serial.println(lineError);
  Serial.print("Sensors: ");
  for (int i = 0; i < 12; i++) {
    Serial.print(binarySensors[i]);
  }
  Serial.print(" (");
  Serial.print(bin_Sensor, BIN);
  Serial.println(")");
  Serial.print("Analog: ");
  for (int i = 0; i < 12; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(" ");
  }
  Serial.println();
}
