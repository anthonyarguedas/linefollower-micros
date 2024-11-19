#include "ReverseBuffer.h"


int reverseBuffer[BUFFER_SIZE];
int index = 0;


void writeBuffer(int value) {
    reverseBuffer[index] = value;

    if (index < BUFFER_SIZE - 1) {
        index++;
    } else {
        index = 0;
    }
}

int readBuffer() {
    if (index > 0) {
        index--;
    } else {
        index = BUFFER_SIZE - 1;
    }

    return reverseBuffer[index];
}

void refreshBuffer() {
    index = 0;
}