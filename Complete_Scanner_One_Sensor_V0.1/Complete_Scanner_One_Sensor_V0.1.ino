#include <stdint.h>
#include <Wire.h>
#include <LIDARLite_v3HP.h>
#include <Servo.h>

#define dirPin 2
#define stepPin 3
//#define stepsPerRevolution 1

Servo servo;  // create servo object to control a servo
LIDARLite_v3HP myLidarLite; //create lidarsensor object

enum rangeType_T
{
    RANGE_NONE,
    RANGE_SINGLE,
    RANGE_CONTINUOUS,
    RANGE_TIMER
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {

    // Initialize stepper pin modes
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    digitalWrite(dirPin, LOW); //LOW == Clockwise
//    Serial.begin(115200);
    Serial.begin(2000000);

    //servo setup
    servo.attach(8);  // attaches the servo on pin 8 to the servo object
    servo.write(58);  // position servo upon startup

        // Initialize Arduino I2C (for communication to LidarLite)
    Wire.begin();
    #ifdef FAST_I2C
        #if ARDUINO >= 157
            Wire.setClock(400000UL); // Set I2C frequency to 400kHz (for Arduino Due)
        #else
            TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
        #endif
    #endif

    // Configure the LidarLite internal parameters so as to lend itself to
    // various modes of operation by altering 'configure' input integer to
    // anything in the range of 0 to 5. See LIDARLite_v3HP.cpp for details.
    myLidarLite.configure(3); 

    //for reset resistor
    digitalWrite(10, HIGH);
    pinMode(10, OUTPUT);

    //send a character to serial to start scanning
    while (!Serial.available()) {}
    
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//initializing variables for stepper position, acceleration of stepper motor, servo and serial data sent to controller
uint16_t pos = 0;
uint32_t accel = 400;
uint32_t accel2 = 900;
uint32_t incomingByte = 0;

int servo_direction = 1;
int servo_pos = 58;

uint8_t  newDistance = 0;
uint16_t distance;

  //uint8_t  c;
    //rangeType_T rangeMode = RANGE_NONE;
    //rangeMode = RANGE_SINGLE;
  // take measurement
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  
  //read serial input for stopping scan
  incomingByte = Serial.read(); 
  //'e' to exit    
  if(incomingByte == 101){
      delay(1000);
      digitalWrite(10, LOW);}    
  //'r' to slow motor before stop  
  if(incomingByte == 114){accel = 1000;}


  //advance the stepper motor
  stepper_advance(accel, accel2);
  pos += 1;
  
  if(pos == 840 || pos == 420){
      pos = 0;
      if(servo_pos != 148 && servo_direction == 1){servo_pos +=1;}
      if(servo_pos == 148){servo_direction = -1; servo_pos -=1;}
      
      //servo going down
      if(servo_pos != 58 && servo_direction == -1){servo_pos -=1;}
      if (servo_pos == 58){servo_direction = 1; servo_pos +=1;}
    }
    servo.write(servo_pos);
    
  // Trigger the next range measurement
  myLidarLite.takeRange();
  // Read new distance data from device registers
  distance = myLidarLite.readDistance();


  //print out the variablees needed for coordinates
  //Serial.write("Distance: ");
  Serial.print(distance);
  Serial.print(",");  

  Serial.print(servo_pos);
  Serial.print(",");
  
  Serial.println(pos);     

  //for accelerating the motor to full speed potential
  if (accel2 > 450){accel2 -= 0.5;}
  if (accel > 0){accel -= 1;}
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void stepper_advance(int ac, int ac2){
  // Spin the stepper motor 1 revolution slowly:
//  for (int i = 0; i < 1; i++) {
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(ac2);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(ac); 
  }


// funciton to continuously pool sensor for measurements
uint8_t distanceContinuous(uint16_t * distance)
{
    uint8_t newDistance = 1;

    // Check on busyFlag to indicate if device is idle
    // (meaning = it finished the previously triggered measurement)
   // if (myLidarLite.getBusyFlag() == 0)
    //{
        // Trigger the next range measurement
        myLidarLite.takeRange();

        // Read new distance data from device registers
        *distance = myLidarLite.readDistance();

        // Report to calling function that we have new data
        //newDistance = 1;
    //}

    return newDistance;
}

//funciton to take single measurement
uint8_t distanceSingle(uint16_t * distance) {
    // 1. Wait for busyFlag to indicate device is idle. This must be
    //    done before triggering a range measurement.
    myLidarLite.waitForBusy();

    // 2. Trigger range measurement.
    myLidarLite.takeRange();

    // 3. Wait for busyFlag to indicate device is idle. This should be
    //    done before reading the distance data that was triggered above.
    myLidarLite.waitForBusy();
    
    // 4. Read new distance data from device registers
    *distance = myLidarLite.readDistance();

    return 1;
}
