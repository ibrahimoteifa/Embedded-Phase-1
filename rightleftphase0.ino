#include <EEPROM.h>

// --- BLUETOOTH SETUP ---
// Using Hardware Serial1 on Arduino Mega
// Connect HM-10 TX to Mega RX1 (Pin 19)
// Connect HM-10 RX to Mega TX1 (Pin 18)
#define btSerial Serial2

const int ENA = 3;   
const int IN1 = 26;  
const int IN2 = 28;  

const int ENB = 2;   
const int IN3 = 22;  
const int IN4 = 24;  

const int irFarLeft   = 30;
const int irNearLeft  = 32;
const int irCenter    = 34;
const int irNearRight = 36;
const int irFarRight  = 38;

const int BLACK = 0; 
const int WHITE = 1; 

// --- SPEED CONTROLS ---
const int baseSpeed = 55; 
const int turnSpeed = 130; 

// --- PD-CONTROLLER PARAMS ---
const float Kp = 20.0;    
const float Kd = 35.0; // Dampens the swing to completely eliminate wiggling

// --- KICKSTART CONTROLS ---
const int boostSpeed = 100; 
const int boostTime  = 40;  
bool isStopped = true;      

int L2, L1, C, R1, R2;
int lastError = 0;             
unsigned long timeLost = 0;    
unsigned long lastTurnTime = 0; // Added to prevent false intersection triggers after turns

// --- ROUTE RECORDING VARIABLES ---
const int MAX_ROUTE_LENGTH = 100;

// Array 1: Every turn the robot actually takes
char route[MAX_ROUTE_LENGTH];
int routeLength = 0;

// Array 2: The mathematically simplified shortest path
char shortRoute[MAX_ROUTE_LENGTH];
int shortRouteLength = 0;

// --- REPLAY & EXPLORATION VARIABLES ---
bool isReplaying = false;
int replayIndex = 0;
bool useLeftHandRule = true; // Determines which logic the robot uses to explore

enum RobotState {
  FOLLOWING_LINE,
  INTERSECTION_DETECTED,
  DEAD_END,
  STOPPED,
  WAITING_AT_GOAL,
  FINISHED
};

RobotState currentState = STOPPED;
RobotState lastSentState = STOPPED; // To track and display state changes via Bluetooth

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(irFarLeft, INPUT); pinMode(irNearLeft, INPUT); pinMode(irCenter, INPUT);
  pinMode(irNearRight, INPUT); pinMode(irFarRight, INPUT);

  Serial.begin(9600);
  btSerial.begin(9600); // Initialize Bluetooth Serial

  Serial.println("Phase 5: Selectable Left/Right Hand Rule Exploration");
  btSerial.println("--- Robot Ready ---");
  btSerial.println("Commands:");
  btSerial.println("L = Start (Left-Hand Rule)");
  btSerial.println("R = Start (Right-Hand Rule)");
  btSerial.println("E = End / Stop");
  btSerial.println("B = Approve Return Trip");

  // --- READ PREVIOUS ROUTES ON STARTUP ---
  
  // 1. Read Raw Route
  int savedRawLength = EEPROM.read(0);
  if (savedRawLength > 0 && savedRawLength < MAX_ROUTE_LENGTH) {
    Serial.print("Previous Raw Route: ");
    btSerial.print("Mem Raw Route: ");
    for (int i = 0; i < savedRawLength; i++) {
      char move = (char)EEPROM.read(1 + i);
      Serial.print(move);
      Serial.print(" ");
      btSerial.print(move);
      btSerial.print(" ");
    }
    Serial.println();
    btSerial.println();
  } else {
    Serial.println("No valid Raw Route found in EEPROM.");
  }

  // 2. Read Shortest Path Route (Stored starting at Address 101)
  int savedShortLength = EEPROM.read(101);
  if (savedShortLength > 0 && savedShortLength < MAX_ROUTE_LENGTH) {
    Serial.print("Previous Short Route: ");
    btSerial.print("Mem Short Route: ");
    for (int i = 0; i < savedShortLength; i++) {
      char move = (char)EEPROM.read(102 + i);
      Serial.print(move);
      Serial.print(" ");
      btSerial.print(move);
      btSerial.print(" ");
    }
    Serial.println();
    btSerial.println();
  } else {
    Serial.println("No valid Short Route found in EEPROM.");
  }
  
  delay(2000); 
  readSensors();
  
  // Start stopped until Bluetooth Start command is sent. 
  currentState = STOPPED; 
}

void loop() {
  checkBluetooth();     // Constantly listen for App commands
  sendStateIfChanged(); // Update App if State changes

  // Halt motor execution if in any of these passive states
  if (currentState == STOPPED || currentState == WAITING_AT_GOAL || currentState == FINISHED) {
    stopMotors();
    return; 
  }

  readSensors();
  determineState();
  executeState();
}

// --- BLUETOOTH FUNCTIONS ---

void checkBluetooth() {
  if (btSerial.available()) {
    char cmd = btSerial.read();
    
    // Ignore invisible newline and carriage return characters sent by apps!
    if (cmd == '\n' || cmd == '\r') {
      return; 
    }
    
    Serial.print("Bluetooth Received: ");
    Serial.println(cmd);
    
    // Command 'L' = Start using Left-Hand Rule
    if (cmd == 'L' || cmd == 'l') { 
      Serial.println(">> PC Monitor: STARTING RUN (LEFT-HAND RULE)"); 
      btSerial.println(">> App: START (LEFT-HAND)");      
      isReplaying = false; 
      useLeftHandRule = true; 
      routeLength = 0;        
      lastTurnTime = millis(); // Give it a buffer against falsely scanning the start box!
      currentState = FOLLOWING_LINE;
    }
    // Command 'R' = Start using Right-Hand Rule
    else if (cmd == 'R' || cmd == 'r') { 
      Serial.println(">> PC Monitor: STARTING RUN (RIGHT-HAND RULE)"); 
      btSerial.println(">> App: START (RIGHT-HAND)");      
      isReplaying = false; 
      useLeftHandRule = false; 
      routeLength = 0;         
      lastTurnTime = millis(); // Give it a buffer against falsely scanning the start box!
      currentState = FOLLOWING_LINE;
    }
    // Command 'E' = End / Stop
    else if (cmd == 'E' || cmd == 'e') {
      Serial.println(">> PC Monitor: END / STOPPING");
      btSerial.println(">> App: END / STOPPING");
      isReplaying = false;
      currentState = STOPPED;
      stopMotors();
    }
    // Command 'B' = Approve Return Journey
    else if (cmd == 'B' || cmd == 'b') {
      if (currentState == WAITING_AT_GOAL) {
        Serial.println(">> PC Monitor: GOAL CONFIRMED! Starting Return Journey...");
        btSerial.println(">> App: Goal Confirmed! Returning...");
        startReturnJourney();
      } else {
        Serial.println(">> PC Monitor: Ignored 'B'. Car is not waiting at the goal.");
        btSerial.println(">> App: Ignored (Not at Goal)");
      }
    }
  }
}

void sendStateIfChanged() {
  if (currentState != lastSentState) {
    btSerial.print("Current State: ");
    switch(currentState) {
      case FOLLOWING_LINE: btSerial.println(isReplaying ? "RETURNING HOME" : "FOLLOWING LINE"); break;
      case INTERSECTION_DETECTED: btSerial.println("INTERSECTION DETECTED"); break;
      case DEAD_END: btSerial.println("DEAD END"); break;
      case STOPPED: btSerial.println("STOPPED"); break;
      case WAITING_AT_GOAL: btSerial.println("WAITING FOR APPROVAL"); break;
      case FINISHED: btSerial.println("FINISHED MAZE"); break;
    }
    lastSentState = currentState;
  }
}

// ---------------------------

void readSensors() {
  L2 = digitalRead(irFarLeft);
  L1 = digitalRead(irNearLeft);
  C  = digitalRead(irCenter);
  R1 = digitalRead(irNearRight);
  R2 = digitalRead(irFarRight);
}

void determineState() {
  int blackCount = 0;
  if (L2 == BLACK) blackCount++;
  if (L1 == BLACK) blackCount++;
  if (C  == BLACK) blackCount++;
  if (R1 == BLACK) blackCount++;
  if (R2 == BLACK) blackCount++;

  if (blackCount > 0) {
    timeLost = 0; 
  }

  if (blackCount >= 3) {
    // Check if 300ms has elapsed since the last turn finish, prevents picking up overshoots or double thick lines as "extra lefts".
    if (millis() - lastTurnTime > 300) {
      currentState = INTERSECTION_DETECTED;
    } else {
      currentState = FOLLOWING_LINE;
    }
  }
  else if (blackCount == 0) {
    currentState = DEAD_END;
  }
  else {
    currentState = FOLLOWING_LINE;
  }
}

void executeState() {
  switch (currentState) {
    case FOLLOWING_LINE: {
      int error = lastError;

      // Perfectly linear scale prevents sudden jerky jumps and tracks angles seamlessly
      if      (L2 == BLACK && L1 == WHITE) error = -4; 
      else if (L2 == BLACK && L1 == BLACK) error = -3; 
      else if (L1 == BLACK && C == WHITE)  error = -2; 
      else if (L1 == BLACK && C == BLACK)  error = -1; 
      else if (R2 == BLACK && R1 == WHITE) error = 4;  
      else if (R2 == BLACK && R1 == BLACK) error = 3;  
      else if (R1 == BLACK && C == WHITE)  error = 2;  
      else if (R1 == BLACK && C == BLACK)  error = 1;  
      else if (C  == BLACK)                error = 0;                
      
      int errorDiff = error - lastError;
      lastError = error; 

      // Apply PD control: 'P' tracks the line, 'D' dampens it actively to kill wiggles
      int correction = (Kp * error) + (Kd * errorDiff);

      int leftPWM  = baseSpeed + correction;
      int rightPWM = baseSpeed - correction;

      bool leftDir = HIGH;
      bool rightDir = HIGH;

      // REVERSE MINIMUM REMOVED: Motors now dynamically brake perfectly
      if (leftPWM < 0) {
        leftDir = LOW;       
        leftPWM = -leftPWM;  
      }
      if (rightPWM < 0) {
        rightDir = LOW;      
        rightPWM = -rightPWM;
      }

      leftPWM  = constrain(leftPWM, 0, 255);
      rightPWM = constrain(rightPWM, 0, 255);

      setMotors(leftPWM, rightPWM, leftDir, rightDir);
      break;
    }

    case INTERSECTION_DETECTED: {
      setMotors(baseSpeed, baseSpeed, HIGH, HIGH);
      
      unsigned long alignStart = millis();
      int leftHits = 0, rightHits = 0, loopCount = 0;
      bool endBubble = false;
      
      while (true) {
        unsigned long elapsed = millis() - alignStart;
        readSensors();
        
        if (elapsed < 80) {
          if (L2 == BLACK) leftHits++;
          if (R2 == BLACK) rightHits++;
          loopCount++;
        }
        
        if (elapsed >= 80) {
          int bCount = 0;
          if (L2 == BLACK) bCount++;
          if (L1 == BLACK) bCount++;
          if (C  == BLACK) bCount++;
          if (R1 == BLACK) bCount++;
          if (R2 == BLACK) bCount++;
          
          if (bCount <= 2) {
            break;
          }
          
          if (elapsed >= 300) {
            if (bCount >= 4) {
              endBubble = true;
            }
            break;
          }
        }
      }
      
      stopMotors();
      
      // --- END BUBBLE LOGIC ---
      if (endBubble) {
        Serial.println("END BUBBLE DETECTED! Destination Reached.");
        btSerial.println(">> END BUBBLE DETECTED!");
        
        saveRouteToEEPROM(); // Calculates shortest path and prints it out.
        
        Serial.println("Waiting for 'B' command to start return journey...");
        btSerial.println(">> Send 'B' to Return!");
        
        currentState = WAITING_AT_GOAL; // Puts the car into pause mode
        break; 
      }
      
      delay(100); 
      
      bool leftDetected  = (leftHits * 5 >= loopCount);
      bool rightDetected = (rightHits * 5 >= loopCount);

      readSensors();
      bool straightDetected = (C == BLACK || L1 == BLACK || R1 == BLACK);

      // --- REPLAY MODE LOGIC (Navigating Back) ---
      if (isReplaying) {
        if (replayIndex < shortRouteLength) {
          char nextMove = shortRoute[replayIndex];
          replayIndex++;
          
          Serial.print("Replaying Turn: "); Serial.println(nextMove);
          btSerial.print(">> Return Turn: "); btSerial.println(nextMove);

          if (nextMove == 'L') {
            tankTurnLeft();
          } else if (nextMove == 'R') {
            tankTurnRight();
          } else if (nextMove == 'S') {
            setMotors(baseSpeed, baseSpeed, HIGH, HIGH);
            delay(150);
            lastTurnTime = millis();
          }
        } else {
          // If we run out of moves, we have safely navigated back to Start!
          Serial.println("RETURN JOURNEY COMPLETE! Back at Start.");
          btSerial.println(">> Back at Start!");
          currentState = FINISHED;
          isReplaying = false;
        }
      } 
      // --- EXPLORATION LOGIC (Solving the maze) ---
      else {
        
        // Check which hand rule the user selected
        if (useLeftHandRule) {
          // --- LEFT HAND RULE --- (Priority: Left -> Straight -> Right)
          if (leftDetected) {
            recordTurn('L'); 
            tankTurnLeft(); 
          } 
          else if (straightDetected) {
            recordTurn('S'); 
            setMotors(baseSpeed, baseSpeed, HIGH, HIGH);
            delay(150); 
            lastTurnTime = millis();
          } 
          else if (rightDetected) {
            recordTurn('R'); 
            tankTurnRight();
          }
        } 
        else {
          // --- RIGHT HAND RULE --- (Priority: Right -> Straight -> Left)
          if (rightDetected) {
            recordTurn('R'); 
            tankTurnRight(); 
          } 
          else if (straightDetected) {
            recordTurn('S'); 
            setMotors(baseSpeed, baseSpeed, HIGH, HIGH);
            delay(150); 
            lastTurnTime = millis();
          } 
          else if (leftDetected) {
            recordTurn('L'); 
            tankTurnLeft();
          }
        }
        
      }
      break;
    }

    case DEAD_END:
      if (timeLost == 0) {
        timeLost = millis();
      }

      // REDUCED FROM 800ms to 150ms!
      // This minimizes false twists and prevents it from falsely registering missed left turns.
      if (millis() - timeLost > 450) {
        if (isReplaying) {
          // If we hit a dead end while replaying, we have arrived backwards at the Start Box
          Serial.println("RETURN JOURNEY COMPLETE! (Reached Start Box)");
          btSerial.println(">> Arrived at Start!");
          currentState = FINISHED;
          isReplaying = false;
          stopMotors();
        } else {
          recordTurn('U'); 
          uTurn();
        }
        timeLost = 0; 
      } 
      else {
        // Keeps the brief 150ms sweep to recover from tiny glitches (bumpy tape)
        if (lastError < 0) {
          setMotors(turnSpeed, turnSpeed, LOW, HIGH); 
        } else {
          setMotors(turnSpeed, turnSpeed, HIGH, LOW); 
        }
      }
      break;
      
    case STOPPED:
    case WAITING_AT_GOAL:
    case FINISHED:
      stopMotors();
      break;
  }
}

// --- RECORDING & OPTIMIZATION FUNCTIONS ---

void recordTurn(char move) {
  if (routeLength < MAX_ROUTE_LENGTH) {
    route[routeLength] = move;
    routeLength++;
    
    // Print locally and transmit via Bluetooth
    Serial.print("Path Recorded: ");
    Serial.println(move);
    btSerial.print(">> Executed Turn: ");
    btSerial.println(move);
  }
}

void simplifyPath() {
  Serial.println("Calculating Shortest Path...");
  btSerial.println("Calculating Shortest Path...");
  
  // 1. Copy the raw route to the short route array
  shortRouteLength = routeLength;
  for(int i = 0; i < routeLength; i++) {
    shortRoute[i] = route[i];
  }

  // 2. Continually look for 'U' turns and simplify the steps around them
  bool simplified = true;
  while(simplified) {
    simplified = false;
    
    for(int i = 1; i < shortRouteLength - 1; i++) {
      if(shortRoute[i] == 'U') {
        char newTurn = '?';
        char x = shortRoute[i-1];
        char y = shortRoute[i+1];

        // Maze substitution mathematical rules based on heading vectors
        // (These are geometry-based, so they work the same for Left & Right hand rules!)
        if(x == 'L' && y == 'L') newTurn = 'S';
        else if(x == 'L' && y == 'S') newTurn = 'R';
        else if(x == 'L' && y == 'R') newTurn = 'U';
        else if(x == 'S' && y == 'L') newTurn = 'R';
        else if(x == 'S' && y == 'S') newTurn = 'U';
        else if(x == 'S' && y == 'R') newTurn = 'L';
        else if(x == 'R' && y == 'L') newTurn = 'U';
        else if(x == 'R' && y == 'S') newTurn = 'L';
        else if(x == 'R' && y == 'R') newTurn = 'S';

        // If a substitution was found, update the array
        if(newTurn != '?') {
          shortRoute[i-1] = newTurn; 
          
          // Shift remaining array left by 2 spaces to delete the 'U' and 'y'
          for(int j = i; j < shortRouteLength - 2; j++) {
            shortRoute[j] = shortRoute[j+2];
          }
          
          shortRouteLength -= 2; 
          simplified = true; 
          break; 
        }
      }
    }
  }
}

void saveRouteToEEPROM() {
  // First calculate the short route
  simplifyPath(); 

  Serial.println("Saving Routes to EEPROM...");
  btSerial.println("Saving to Memory...");
  
  // We use EEPROM.update() instead of write() to prevent unnecessary damage to flash memory life
  
  // 1. Save Raw Route
  EEPROM.update(0, routeLength);
  for (int i = 0; i < routeLength; i++) {
    EEPROM.update(1 + i, route[i]);
  }
  
  // 2. Save Short Route (Starting at address 101 to avoid overlap)
  EEPROM.update(101, shortRouteLength);
  for (int i = 0; i < shortRouteLength; i++) {
    EEPROM.update(102 + i, shortRoute[i]);
  }
  
  // --- PRINT RESULTS TO BLUETOOTH & SERIAL ---
  Serial.print("Raw Route Saved: ");
  for (int i = 0; i < routeLength; i++) { Serial.print(route[i]); Serial.print(" "); }
  Serial.println(); 

  Serial.print("Shortest Route Saved: ");
  btSerial.print(">> Final Short Path: ");
  for (int i = 0; i < shortRouteLength; i++) {
    Serial.print(shortRoute[i]); Serial.print(" ");
    btSerial.print(shortRoute[i]); btSerial.print(" ");
  }
  Serial.println(); btSerial.println();
}

void reverseShortPath() {
  // 1. Reverse the array sequence completely
  for(int i = 0; i < shortRouteLength / 2; i++) {
    char temp = shortRoute[i];
    shortRoute[i] = shortRoute[shortRouteLength - 1 - i];
    shortRoute[shortRouteLength - 1 - i] = temp;
  }

  // 2. Swap L and R (approaching an intersection backward flips directions)
  for(int i = 0; i < shortRouteLength; i++) {
    if(shortRoute[i] == 'L') {
      shortRoute[i] = 'R';
    } else if(shortRoute[i] == 'R') {
      shortRoute[i] = 'L';
    }
    // 'S' (straight) correctly remains straight 
  }
  
  Serial.print("Reversed Return Path: ");
  btSerial.print(">> Configured Return Path: ");
  for(int i = 0; i < shortRouteLength; i++) {
    Serial.print(shortRoute[i]); Serial.print(" ");
    btSerial.print(shortRoute[i]); btSerial.print(" ");
  }
  Serial.println(); btSerial.println();
}

void startReturnJourney() {
  // We no longer call saveRouteToEEPROM() here because it's already generated 
  // and printed the moment it hit the bubble!
  
  // 1. Reconfigure the shortest path for the return journey
  reverseShortPath();
  
  // 2. Set the variables indicating we are replaying a path backwards
  isReplaying = true;
  replayIndex = 0;
  
  // 3. Physically perform the U-Turn to aim backward at the maze
  Serial.println("Reversing off the bubble before U-Turn...");
  btSerial.println(">> Reversing...");
  
  // NEW: Back up off the large finish bubble first so the U-Turn works cleanly 
  setMotors(baseSpeed, baseSpeed, LOW, LOW); // LOW, LOW reverses the motors
  delay(450); // Wait long enough to travel off the bubble so sensors can detect empty space
  stopMotors();
  delay(200); 

  Serial.println("Executing U-Turn to return...");
  btSerial.println(">> Turning around...");
  uTurn();
  
  // 4. Send the car on its way
  currentState = FOLLOWING_LINE;
}

// --- MOTOR CONTROL FUNCTIONS ---

void setMotors(int leftPWM, int rightPWM, bool leftFwd, bool rightFwd) {
  if (isStopped && (leftPWM > 0 || rightPWM > 0)) {
    digitalWrite(IN1, leftFwd ? LOW : HIGH); 
    digitalWrite(IN2, leftFwd ? HIGH : LOW);
    digitalWrite(IN3, rightFwd ? HIGH : LOW); 
    digitalWrite(IN4, rightFwd ? LOW : HIGH);
    
    analogWrite(ENA, boostSpeed);
    analogWrite(ENB, boostSpeed);
    delay(boostTime);
    
    isStopped = false; 
  }
  
  digitalWrite(IN1, leftFwd ? LOW : HIGH); 
  digitalWrite(IN2, leftFwd ? HIGH : LOW);
  digitalWrite(IN3, rightFwd ? HIGH : LOW); 
  digitalWrite(IN4, rightFwd ? LOW : HIGH);
  
  analogWrite(ENA, leftPWM);
  analogWrite(ENB, rightPWM);

  if (leftPWM == 0 && rightPWM == 0) {
    isStopped = true;
  }
}

void stopMotors() {
  setMotors(0, 0, HIGH, HIGH); 
}

void tankTurnLeft() {
  Serial.println("TANK TURN LEFT");
  setMotors(turnSpeed, turnSpeed, LOW, HIGH);
  delay(250); 
  do { readSensors(); } while (C == WHITE); 
  stopMotors();
  delay(100);
  lastTurnTime = millis(); // Refresh timestamp to avoid "extra lefts" false-reads
}

void tankTurnRight() {
  Serial.println("TANK TURN RIGHT");
  setMotors(turnSpeed, turnSpeed, HIGH, LOW); 
  delay(250); 
  do { readSensors(); } while (C == WHITE); 
  stopMotors();
  delay(100);
  lastTurnTime = millis(); // Refresh timestamp to avoid "extra lefts" false-reads
}

void uTurn() {
  Serial.println("CLOCKWISE TANK U-TURN");
  setMotors(turnSpeed, turnSpeed, HIGH, LOW); 
  delay(300); 
  do { readSensors(); } while (C == WHITE); 
  stopMotors();
  delay(100);
  lastTurnTime = millis(); // Refresh timestamp to avoid "extra lefts" false-reads
}