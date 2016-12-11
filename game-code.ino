
/*********************************************************** START GYRO **************************************************/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
/******************************************************** END GYRO *************************************************/


/***************************************************** START SHIFTPWM **********************************************/
const int ShiftPWM_latchPin=8;
// If your LED's turn on if the pin is low, set this to true, otherwise set it to false.
const bool ShiftPWM_invertOutputs = true; 
const bool ShiftPWM_balanceLoad = false;
#include <ShiftPWM.h>   // include ShiftPWM.h after setting the pins!

unsigned char maxBrightness = 200;
unsigned char pwmFrequency = 60;
byte numRegisters = 6;
byte numRGBleds = numRegisters*8/3;
/***************************************************** END SHIFTPWM ************************************************/


/*************************************************** START OUR CODE ************************************************/
// Timing
byte updateThreshold = 100;  // Time between all game logic updates
unsigned long lastUpdate,  // Counter for all game logic updates
  lastWallReset;  // Counter used to tell how long player has been between a pair of walls

// Course setup
byte wallSegments = 16,  // Number of "lanes", also the horizontal resolution of LED strips
  wall[16];  // A byte array that carries the state of each segment
int wallWidth = 1000;  // Wall width in digital coordinates, important for tilt movement
int laneLength = 200,  // Length player travels between walls
  laneWidth = wallWidth/wallSegments;  // Used in lane switching calculations

// Position
byte velocity = 20,  // Player velocity
  gapSize = 0,  // Size of gap in wall
  gapCenter = wallSegments/2;  // Where the center of gap is in wall, starts in middle
int xPos = 0,  // Player position within a single lane
  stepSize = 40,  // How far they move when tilting
  pitch;  // Tilt information provided by gyro sensor readings
/******************************************************* END OUR CODE **********************************************/


/*******************************************************************************************************************/
/********************************************************* SETUP ***************************************************/
/*******************************************************************************************************************/
void setup() {
  Serial.begin(115200);  // Start Serial communication

  //  GYRO SETUP
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      dmpReady = true;
      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR! (if it's going to break, usually the code will be 1)
      // 1 = initial memory load failed, 2 = DMP configuration updates failed
      Serial.println(devStatus);
  }
  // END GYRO SETUP


  //  SHIFTPWM SETUP
  ShiftPWM.SetAmountOfRegisters(numRegisters);  // number of bit shift register circuits being used
  ShiftPWM.SetPinGrouping(1);  // color channel groupings when connecting pins, e.g. 1 = RGB RGB, 2 = RR GG BB, etc.
  ShiftPWM.Start(pwmFrequency, maxBrightness);  // Initialize
  //  END SHIFTPWM SETUP
  
  update();  // Run a cycle of the main game logic

  lastUpdate = millis();  // Set reference time for game logic updates, gets reset every loop
  lastWallReset = millis();  // Similar but for when players pass walls
}


/*******************************************************************************************************************/
/********************************************************* LOOP ****************************************************/
/*******************************************************************************************************************/
void loop() {
  updateGyro();
  // rgbLedRainbow(wallSegments, 5, 5, wallSegments); // For testing, very pretty
  
  // Avoid running game logic too often
  if (millis() - lastUpdate > updateThreshold) {
    lastUpdate = millis();  // Clear counter
    
    // Run game logic to determine which lights should be on or off
    update(); 

    // Actually change state of LEDs using ShiftPWM.
    for (int i=0; i<wallSegments; i++) {
      ShiftPWM.SetRGB(i, 0, 0, wall[i] * 255);
    }
  }
}


/*******************************************************************************************************************/
/******************************************************* HELPERS ***************************************************/
/*******************************************************************************************************************/
void update() {
  // Main loop logic: checks the time since last wall was passed to see how close player is to next wall.
  // This information then dictates the rest of the logic.
  int distanceTravelled = (float)velocity * (float)(millis() - lastWallReset)/1000.0f;
  gapSize = 2 * (distanceTravelled / (laneLength/6));
  
  if (distanceTravelled >= laneLength) {
    // Player has reached wall
    if (abs(wallSegments/2 - gapCenter) < 2) {
      // Made it through the gap!
      resetStage();
    } else {
      // Crashed!
      runEndgame(); // Some blinking red lights
      resetStage();
    }
  } else if (gapSize == 0) {
    // After player passes wall, all lights are off
    refreshWall(0);
  } else {
    // Most common logic, traveling between walls.
    updatePos();  // Updates lateral position to see if player has moved towards gap
    refreshWall(1);
    addGap();
  }
}

void updatePos() {
  // Uses pitch information gathered by the gyro sensor to update player X coordinate.
  // If the X position goes beyond the width of the lane, player moves into adjacent lane.

  // Some manually calibrated ranges of gyro sensor
  if (pitch > 5 && pitch < 150) {
    xPos += stepSize; // Leaning right
  } else if (pitch > 150 && pitch < 250) {
    xPos -= stepSize; // Leaning left
  }
  
  if (abs(xPos) > laneWidth/2) {
    int flip = xPos/abs(xPos);  // Gets 1 or -1 depending on direction
    gapCenter += flip;  // Moves into new lane by setting relative position of gap
    xPos = -xPos + (int)(flip * (float)laneWidth/(float)wallSegments); // Start away from edge in "new lane"
  }
}

void resetStage() {
  // Happens when player has just passed a wall
  lastWallReset = millis();  // Clear timer
  gapCenter = random(3, wallSegments - 3);  // Randomly set new gap
  xPos = 0;  // Reset X position
  refreshWall(0);  // Set all LEDs to turn off
}

void runEndgame() {
  // Plays three rounds of blinking red light
  for (int j=0; j<3; j++) {
    for (int i=0; i<wallSegments; i++) {
      ShiftPWM.SetRGB(i, 255, 0, 0);
    }
    delay(500);
    for (int i=0; i<wallSegments; i++) {
      ShiftPWM.SetRGB(i, 0, 0, 0);
    }
    delay(500); 
  }
}

void refreshWall(int wallValue) {
  // Set all wall values to either 1 (on) or 0 (off). These values are later used by
  // ShiftPWM to control our LED strips.
  for (int j=0; j<wallSegments; j++) {
    wall[j] = wallValue;
  }
}

void addGap() {
  // Using an updated gapSize that corresponds to player distance from the next wall,
  // this function turns off segments where the next gap should be.
  for (int gap=0; gap<gapSize/2; gap++) {
    wall[gapCenter - gap] = 0;
    wall[gapCenter + gap] = 0;
  }
}

void rgbLedRainbow(int numRGBLeds, int delayVal, int numCycles, int rainbowWidth){
  // Displays a rainbow spread over a few LED's (numRGBLeds), which shifts in hue. 
  // The rainbow can be wider then the real number of LED's.

  ShiftPWM.SetAll(0);
  for(int cycle=0;cycle<numCycles;cycle++){ // Loop through the hue shift a number of times (numCycles)
    for(int colorshift=0;colorshift<360;colorshift++){ // Shift over full color range (like the hue slider in Photoshop)
      for(int led=0;led<numRGBLeds;led++){ // Loop over all LED's
        int hue = ((led)*360/(rainbowWidth-1)+colorshift)%360; // Set hue from 0 to 360 from first to last led and shift the hue
        ShiftPWM.SetHSV(led, hue, 255, 255); // Write the HSV values, with saturation and value at maximum
      }
      delay(delayVal); // This delay value determines the speed of hue shift
    } 
  }  
}

void updateGyro() {
  // If programming failed, don't try to do anything
  if (!dmpReady) return;

  // Wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {}

  // Reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // Get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // Check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // Reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));

  // Otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
      // Wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // Read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      // Track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      // Get Yaw, Pitch, Roll information
      #ifdef OUTPUT_READABLE_YAWPITCHROLL
          // Display Euler angles in degrees
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          pitch = ypr[1] * 180/M_PI;  // Extract pitch specifically
      #endif
  }  
}
