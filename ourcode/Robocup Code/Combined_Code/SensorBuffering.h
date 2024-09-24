#ifndef SENSORBUFFERING_H
#define SENSORBUFFERING_H

#include <stdint.h>
#include <stdlib.h>

typedef struct {
    uint32_t *data;   // Pointer to buffer memory
    uint32_t windex;  // Write index
    uint32_t rindex;  // Read index
    uint32_t size;    // Number of entries in the buffer
} circBuf_t;

// Function prototypes
uint32_t *initCircBuf(circBuf_t *buffer, uint32_t size);
void writeCircBuf(circBuf_t *buffer, uint32_t entry);
uint32_t readCircBuf(circBuf_t *buffer);
void freeCircBuf(circBuf_t *buffer);

#endif  // SENSORBUFFERING_H
