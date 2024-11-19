#include "ReverseBuffer.h"


int reverseBuffer[BUFFER_SIZE];
int bufferIndex = 0;


void writeBuffer(int value) {
    reverseBuffer[bufferIndex] = value;

    if (bufferIndex < BUFFER_SIZE - 1) {
        bufferIndex++;
    } else {
        bufferIndex = 0;
    }
}

int readBuffer() {
    if (bufferIndex > 0) {
        bufferIndex--;
    } else {
        bufferIndex = BUFFER_SIZE - 1;
    }

    return reverseBuffer[bufferIndex];
}

void refreshBuffer() {
    bufferIndex = 0;
}