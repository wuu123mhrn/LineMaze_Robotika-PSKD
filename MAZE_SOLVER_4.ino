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

// PID constants
#define Kp 5.0
#define Ki 0.1
#define Kd 0.1

// Motor speed settings
#define BASE_SPEED 70
#define MAX_SPEED 80
#define TURN_SPEED 80 
#define FORWARD_SLIGHTLY_MS 150 

// Sensor thresholds
int thresholds[12] = {600, 600, 500, 500, 500, 500, 500, 500, 500, 500, 600, 600};

// Variables
int sensorValues[12];
int binarySensors[12];
unsigned long bin_Sensor;
float error = 0;
float previousError = 0;
float integral = 0;
unsigned long lastTime = 0;
String currentAction = "Line Follow"; 

void setup() {
  Serial.begin(9600);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  
  pinMode(rmf, OUTPUT);
  pinMode(rmb, OUTPUT);
  pinMode(lmf, OUTPUT);
  pinMode(lmb, OUTPUT);
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(sig, INPUT);

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("MAZE BOT"));
  display.display();
  delay(1500);
}

// Fungsi untuk mendeteksi garis di sisi kiri
bool isPathLeft() {
  return (binarySensors[0] || binarySensors[1]); // Cek 2 sensor terluar
}

// Fungsi untuk mendeteksi garis di sisi kanan
bool isPathRight() {
  return (binarySensors[10] || binarySensors[11]); // Cek 2 sensor terluar
}

// Fungsi untuk mendeteksi garis lurus di depan
bool isPathForward() {
  return (binarySensors[3] || binarySensors[4] || binarySensors[5] || binarySensors[6] || binarySensors[7] || binarySensors[8] );
}

// Fungsi untuk melakukan belok kiri hingga sensor tengah mendeteksi garis
void turnLeft() {
  currentAction = "TURN LEFT";
  updateDisplay();
  driveMotors(TURN_SPEED, TURN_SPEED); // Maju sedikit
  delay(FORWARD_SLIGHTLY_MS);
  driveMotors(-TURN_SPEED, TURN_SPEED); // Mulai berputar ke kiri
  while (true) {
    readSensors();
    if (binarySensors[4] && binarySensors[5]) { // Sensor 5 dan 6 mendeteksi garis
      break;
    }
  }
  driveMotors(0, 0); // Berhenti
  delay(100);
}

// Fungsi untuk melakukan belok kanan hingga sensor tengah mendeteksi garis
void turnRight() {
  currentAction = "TURN RIGHT";
  updateDisplay();
  driveMotors(TURN_SPEED, TURN_SPEED); // Maju sedikit
  delay(FORWARD_SLIGHTLY_MS);
  driveMotors(TURN_SPEED, -TURN_SPEED); // Mulai berputar ke kanan
  while (true) {
    readSensors();
    if (binarySensors[4] && binarySensors[5]) { // Sensor 5 dan 6 mendeteksi garis
      break;
    }
  }
  driveMotors(0, 0); // Berhenti
  delay(100);
}

// Fungsi untuk putar balik hingga sensor tengah mendeteksi garis
void turnAround() {
  currentAction = "TURN AROUND";
  updateDisplay();
  driveMotors(TURN_SPEED, -TURN_SPEED); // Mulai berputar ke kanan untuk U-turn
  while (true) {
    readSensors();
    if (binarySensors[4] && binarySensors[5]) { // Sensor 5 dan 6 mendeteksi garis
      break;
    }
  }
  driveMotors(0, 0); // Berhenti
  delay(100);
}

void loop() {
  readSensors();
  bool pathL = isPathLeft();
  bool pathF = isPathForward();
  bool pathR = isPathRight();

  if (bin_Sensor == 0) {
    // Jalan buntu, putar balik
    turnAround();
  } else if (pathL && pathF && !pathR) {
    // Simpang 3 (kiri & lurus) -> Belok kiri
    turnLeft();
  } else if (pathL && !pathF && pathR) {
    // Simpang 3 (kiri & kanan) -> Belok kiri
    turnLeft();
  } else if (!pathL && pathF && pathR) {
    // Simpang 3 (lurus & kanan) -> Lurus
    currentAction = "FORWARD";
    calculateError();
    pidControl();
  } else if (pathL && pathF && pathR) {
    // Simpang 4 -> Belok kiri
    turnLeft();
  } else if (pathL && !pathF && !pathR) {
    // Hanya kiri -> Belok kiri
    turnLeft();
  } else if (!pathL && !pathF && pathR) {
    // Hanya kanan -> Belok kanan
    turnRight();
  } else {
    // Ikuti garis dengan PID
    currentAction = "Line Follow";
    calculateError();
    pidControl();
  }

  updateDisplay();
  delay(10);
}

void readSensors() {
  bin_Sensor = 0;
  for (int i = 0; i < 12; i++) {
    digitalWrite(s0, bitRead(i, 0));
    digitalWrite(s1, bitRead(i, 1));
    digitalWrite(s2, bitRead(i, 2));
    digitalWrite(s3, bitRead(i, 3));
    sensorValues[i] = analogRead(sig);
    binarySensors[i] = (sensorValues[i] > thresholds[i]) ? 1 : 0;
    if (binarySensors[i]) {
      bin_Sensor |= (1UL << (11 - i));
    }
  }
}

void calculateError() {
  // <<< DIUBAH: Blok "jika kehilangan garis" telah dihapus dari sini
  if (bin_Sensor == 0b100000000000) error = 11;
  else if (bin_Sensor == 0b110000000000) error = 10;
  else if (bin_Sensor == 0b010000000000) error = 9;
  else if (bin_Sensor == 0b011000000000) error = 8;
  else if (bin_Sensor == 0b001000000000) error = 7;
  else if (bin_Sensor == 0b001100000000) error = 6;
  else if (bin_Sensor == 0b000100000000) error = 5;
  else if (bin_Sensor == 0b000110000000) error = 4;
  else if (bin_Sensor == 0b000010000000) error = 3;
  else if (bin_Sensor == 0b000011000000) error = 2;
  else if (bin_Sensor == 0b000001000000) error = 1;
  else if (bin_Sensor == 0b000001100000) error = 0;
  else if (bin_Sensor == 0b000000100000) error = -1;
  else if (bin_Sensor == 0b000000110000) error = -2;
  else if (bin_Sensor == 0b000000010000) error = -3;
  else if (bin_Sensor == 0b000000011000) error = -4;
  else if (bin_Sensor == 0b000000001000) error = -5;
  else if (bin_Sensor == 0b000000001100) error = -6;
  else if (bin_Sensor == 0b000000000100) error = -7;
  else if (bin_Sensor == 0b000000000110) error = -8;
  else if (bin_Sensor == 0b000000000010) error = -9;
  else if (bin_Sensor == 0b000000000011) error = -10;
  else if (bin_Sensor == 0b000000000001) error = -11;
 
}

void pidControl() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  float proportional = error;
  integral += error * deltaTime;
  float derivative = (error - previousError) / deltaTime;
  float output = Kp * proportional + Ki * integral + Kd * derivative;

  int leftSpeed = BASE_SPEED - output;
  int rightSpeed = BASE_SPEED + output;

  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  driveMotors(leftSpeed, rightSpeed);
  previousError = error;
}

void driveMotors(int leftSpeed, int rightSpeed) {
  if (leftSpeed > 0) {
    analogWrite(lmf, leftSpeed);
    analogWrite(lmb, 0);
  } else {
    analogWrite(lmf, 0);
    analogWrite(lmb, -leftSpeed);
  }
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
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.print(currentAction); 
  
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.print("Error: ");
  display.print(error);

  display.setCursor(0, 32);
  display.print("L:"); display.print(isPathLeft());
  display.print(" F:"); display.print(isPathForward());
  display.print(" R:"); display.print(isPathRight());
  
  display.setCursor(0, 48);
  for (int i = 0; i < 12; i++) {
    display.print(binarySensors[i]);
  }
  display.display();
}
