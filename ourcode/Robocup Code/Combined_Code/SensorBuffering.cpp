#include "SensorBuffering.h"
#include <Wire.h>  
#include <VL53L1X.h>
#include <VL53L0X.h>
#include <SparkFunSX1509.h>
#include <stdio.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

uint16_t * initCircBuf (circBuf_t *buffer, uint16_t size)
{
	buffer->windex = 0;
	buffer->rindex = 0;
	buffer->size = size;
	buffer->data = 
        (uint16_t *) calloc (size, sizeof(uint16_t));
	return buffer->data;
}
   // Note use of calloc() to clear contents.

// *******************************************************
// writeCircBuf: insert entry at the current windex location,
// advance windex, modulo (buffer size).
void writeCircBuf (circBuf_t *buffer, uint16_t entry)
{
  if (entry > 8100) {  // Only write if entry is 8100 or less
    entry = 1500;
  }
  buffer->data[buffer->windex] = entry;
  buffer->windex++;
  if (buffer->windex >= buffer->size) {
    buffer->windex = 0;
  }
}


// *******************************************************
// readCircBuf: return entry at the current rindex location,
// advance rindex, modulo (buffer size). The function deos not check
// if reading has advanced ahead of writing.
uint16_t readCircBuf (circBuf_t *buffer)
{
	uint16_t entry;
	
	entry = buffer->data[buffer->rindex];
	buffer->rindex++;
	if (buffer->rindex >= buffer->size)
	   buffer->rindex = 0;
    return entry;
}

uint16_t averageCircBuf (circBuf_t * buffer) {
    uint32_t sum = 0;
    int count = 0;

    // Loop through the buffer to calculate the sum and count valid readings
    for (int i = 0; i < buffer->size; i++) {
          sum += buffer->data[i];
          count++;
        }
    // Return average or maxSensorValue if no valid readings were found
    return (count > 0) ? (sum / count) : 0;
}


// *******************************************************
// freeCircBuf: Releases the memory allocated to the buffer data
void freeCircBuf (circBuf_t * buffer)
{
	buffer->windex = 0;
	buffer->rindex = 0;
	buffer->size = 0;
	free (buffer->data);
	buffer->data = NULL;
}

