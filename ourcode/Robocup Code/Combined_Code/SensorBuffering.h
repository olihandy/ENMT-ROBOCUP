#ifndef SENSORBUFFERING_H
#define SENSORBUFFERING_H

#include <stdint.h>
#include <stdlib.h>

// Structure to define a circular buffer for sensor data
typedef struct {
    uint16_t *data;   // Pointer to buffer memory (dynamically allocated)
    uint16_t windex;  // Write index (next position to write)
    uint16_t rindex;  // Read index (next position to read)
    uint16_t size;    // Total size of the buffer
} circBuf_t;

// Function prototypes
uint16_t *initCircBuf(circBuf_t *buffer, uint16_t size);
void writeCircBuf(circBuf_t *buffer, uint16_t entry);
uint16_t readCircBuf(circBuf_t *buffer);
uint16_t averageCircBuf(circBuf_t *buffer);
void freeCircBuf(circBuf_t *buffer);

#endif  // SENSORBUFFERING_H
