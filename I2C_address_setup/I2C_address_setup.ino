#include <stdint.h>
#include <Wire.h>
#include <LIDARLite_v3HP.h>

LIDARLite_v3HP myLidarLite; //create lidarsensor object

#define FAST_I2C

enum rangeType_T
{
    RANGE_NONE,
    RANGE_SINGLE,
    RANGE_CONTINUOUS,
    RANGE_TIMER
};

void setup()
{
    uint8_t dataByte;
    

    // Initialize Arduino serial port (for display of ASCII output to PC)
    Serial.begin(115200);

    // Initialize Arduino I2C (for communication to LidarLite)
    Wire.begin();
    #ifdef FAST_I2C
        #if ARDUINO >= 157
            Wire.setClock(400000UL); // Set I2C frequency to 400kHz (for Arduino Due)
        #else
            TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
        #endif
    #endif
    
    myLidarLite.configure(3);
    uint8_t rea = myLidarLite.read(uint8_t regAddr, uint8_t * dataBytes,
                          uint16_t numBytes, uint8_t lidarliteAddress)
    Serial.println(rea);
}

void loop() {
  
//    uint16_t distance;
//    uint8_t  newDistance = 0;
//   // uint8_t  c;
//   
//    rangeType_T rangeMode = RANGE_NONE;
//    rangeMode = RANGE_SINGLE;
//    
//    newDistance = distanceSingle(&distance);  
//
//    if (newDistance)
//    {
//        Serial.print("Distance: ");
//        Serial.print(distance);     
//    }
//    
//    // Single measurements print once and then stop
//    if (rangeMode == RANGE_SINGLE)
//    {
//        rangeMode = RANGE_NONE;
//    }
}
