#include <SoftwareSerial.h>
SoftwareSerial BTSerial (6,7);

// Pin Definitions
const int FRONT_LEFT_SENSOR = 2;     // Front left IR sensor
const int FRONT_RIGHT_SENSOR = 3;    // Front right IR sensor
const int SIDE_LEFT_SENSOR = 4;      // Side left IR sensor
const int SIDE_RIGHT_SENSOR = 5;     // Side right IR sensor

// Motor Control Pins
const int LEFT_MOTOR_FORWARD = 9;    // Left motor forward pin
const int LEFT_MOTOR_REVERSE = 10;   // Left motor reverse pin
const int RIGHT_MOTOR_FORWARD = 11;  // Right motor forward pin
const int RIGHT_MOTOR_REVERSE = 12;  // Right motor reverse pin

// Movement Speeds
const int BASE_SPEED = 150;          // Base motor speed
const int TURN_SPEED = 100;          // Turning speed when adjusting alignment

// Command Buffer
const int MAX_COMMANDS = 20;         // Maximum number of commands in queue
String commandQueue[MAX_COMMANDS];   // Queue to store commands
int commandCount = 0;                // Number of commands in queue
int currentCommandIndex = 0;         // Current command being executed

// State Variables
bool isMoving = false;               // Current movement state
bool atIntersection = false;         // Flag to indicate intersection detection

void setup() {
  // Sensor Pins Setup
  pinMode(FRONT_LEFT_SENSOR, INPUT);
  pinMode(FRONT_RIGHT_SENSOR, INPUT);
  pinMode(SIDE_LEFT_SENSOR, INPUT);
  pinMode(SIDE_RIGHT_SENSOR, INPUT);
  
  // Motor Pins Setup
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_REVERSE, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_REVERSE, OUTPUT);
  
  // Serial Setup
  Serial.begin(9600);
  Serial.println("Robot Control System Ready");
  BTSerial.begin(9600);
}

// Function to stop motors
void stopMotors() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_REVERSE, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_REVERSE, LOW);
  isMoving = false;
}

// Function to move forward
void moveForward() {
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_REVERSE, LOW);
  digitalWrite(RIGHT_MOTOR_REVERSE, LOW);
  isMoving = true;
}

// Function to turn left
void turnLeft() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_REVERSE, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_REVERSE, LOW);
}

// Function to turn right
void turnRight() {
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_REVERSE, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_REVERSE, HIGH);
}

// Advanced alignment method
void checkAlignment() {
  int frontLeftStatus = digitalRead(FRONT_LEFT_SENSOR);
  int frontRightStatus = digitalRead(FRONT_RIGHT_SENSOR);
  
  // If perfectly aligned (both sensors off the line), return
  if (frontLeftStatus == LOW && frontRightStatus == LOW) {
    return;
  }
  
  // Continuous alignment adjustment
  while (frontLeftStatus != LOW || frontRightStatus != LOW) {
    if (frontLeftStatus == HIGH && frontRightStatus == LOW) {
      // Left sensor on line, right off - turn right
      turnRight();
    } 
    else if (frontLeftStatus == LOW && frontRightStatus == HIGH) {
      // Right sensor on line, left off - turn left
      turnLeft();
    }
    else if (frontLeftStatus == HIGH && frontRightStatus == HIGH) {
      // Both sensors on line - choose a direction
      turnRight();
    }
    
    // Update sensor status
    frontLeftStatus = digitalRead(FRONT_LEFT_SENSOR);
    frontRightStatus = digitalRead(FRONT_RIGHT_SENSOR);
    
    // Small delay to prevent too rapid switching
    delay(20);
  }
  
  // Stop motors once aligned
  stopMotors();
}

// Check for intersection
bool checkIntersection() {
  int sideLeftStatus = digitalRead(SIDE_LEFT_SENSOR);
  int sideRightStatus = digitalRead(SIDE_RIGHT_SENSOR);
  
  // Check if either side sensor detects a path
  return (sideLeftStatus == HIGH || sideRightStatus == HIGH);
}

// Process incoming serial commands
void processSerialCommands() {
  if (Serial.available() > 0) {
    // Clear existing queue
    commandCount = 0;
    currentCommandIndex = 0;
    
    // Read the entire command string
    String commandString = Serial.readStringUntil('\n');
    
    // Split the string by comma
    int lastIndex = 0;
    for (int i = 0; i < commandString.length(); i++) {
      if (commandString.charAt(i) == ',') {
        // Extract command and add to queue
        String command = commandString.substring(lastIndex, i);
        command.trim();
        
        if (commandCount < MAX_COMMANDS) {
          commandQueue[commandCount] = command;
          commandCount++;
        } else {
          break;  // Stop if queue is full
        }
        
        // Update last index
        lastIndex = i + 1;
      }
    }
    
    // Add the last command
    if (lastIndex < commandString.length() && commandCount < MAX_COMMANDS) {
      String lastCommand = commandString.substring(lastIndex);
      lastCommand.trim();
      commandQueue[commandCount] = lastCommand;
      commandCount++;
    }
    
    // Debug print
    Serial.println("Received Commands:");
    for (int j = 0; j < commandCount; j++) {
      Serial.println(commandQueue[j]);
    }
  }
}

// Execute current command with intersection awareness
void executeCurrentCommand() {
  if (currentCommandIndex >= commandCount) {
    // All commands processed
    stopMotors();
    return;
  }
  
  String currentCommand = commandQueue[currentCommandIndex];
  
  // Check for intersection
  if (checkIntersection()) {
    // Intersection detected
    int sideLeftStatus = digitalRead(SIDE_LEFT_SENSOR);
    int sideRightStatus = digitalRead(SIDE_RIGHT_SENSOR);
    
    // Determine available paths
    bool leftPathAvailable = (sideLeftStatus == HIGH);
    bool rightPathAvailable = (sideRightStatus == HIGH);
    
    // Decide action based on current command and available paths
    if (currentCommand == "turnLeft" && leftPathAvailable) {
      // Execute left turn if commanded and path available
      turnLeft();
      // Wait until front sensors are off the line
      while (digitalRead(FRONT_LEFT_SENSOR) == HIGH || 
             digitalRead(FRONT_RIGHT_SENSOR) == HIGH) {
        // Continue turning
      }
      stopMotors();
      delay(500);
      currentCommandIndex++;
      return;
    }
    else if (currentCommand == "turnRight" && rightPathAvailable) {
      // Execute right turn if commanded and path available
      turnRight();
      // Wait until front sensors are off the line
      while (digitalRead(FRONT_LEFT_SENSOR) == HIGH || 
             digitalRead(FRONT_RIGHT_SENSOR) == HIGH) {
        // Continue turning
      }
      stopMotors();
      delay(500);
      currentCommandIndex++;
      return;
    }
    else if (currentCommand == "moveForward") {
      // If intersection is detected but command is moveForward, continue
      // This allows skipping intersections as per your requirement
      checkAlignment();
      moveForward();
      delay(1000);  // Move forward for a set duration
      currentCommandIndex++;
      return;
    }
  }
  
  // Regular command execution for non-intersection scenarios
  if (currentCommand == "moveForward") {
    // Check alignment first
    checkAlignment();
    moveForward();
    delay(1000);  // Move forward for a set duration
  } 
  else if (currentCommand == "turnLeft") {
    turnLeft();
    // Wait until front sensors are off the line
    while (digitalRead(FRONT_LEFT_SENSOR) == HIGH || 
           digitalRead(FRONT_RIGHT_SENSOR) == HIGH) {
      // Continue turning
    }
    stopMotors();
    delay(500);
  } 
  else if (currentCommand == "turnRight") {
    turnRight();
    // Wait until front sensors are off the line
    while (digitalRead(FRONT_LEFT_SENSOR) == HIGH || 
           digitalRead(FRONT_RIGHT_SENSOR) == HIGH) {
      // Continue turning
    }
    stopMotors();
    delay(500);
  } 
  else if (currentCommand == "stop") {
    stopMotors();
  }
  
  // Move to next command
  currentCommandIndex++;
}

void loop() {
  // Process any incoming serial commands
  processSerialCommands();
  
  // Execute current command if any are in queue
  if (commandCount > 0 && currentCommandIndex < commandCount) {
    executeCurrentCommand();
  }
}