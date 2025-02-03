#ifndef PRIORITY_STACK_H
#define PRIORITY_STACK_H

#include <Arduino.h>

// Node structure for stack
struct Node {
    int value;
    int priority;
    Node* next;
};

// Priority Stack class
class PriorityStack {
private:
    Node* topNode;  // Pointer to the top of the stack
    int size;       // Current size of the stack

public:
    // Constructor
    PriorityStack();

    // Destructor
    ~PriorityStack();

    // Push a value with a priority
    void push(int value, int priority);

    // Pop the highest-priority element
    void pop();

    // Get the top element (highest priority)
    int top();

    // Check if stack is empty
    bool isEmpty();

    // Get current stack size
    int getSize();

    // Print the stack (for debugging)
    void printStack();
};

#endif  // PRIORITY_STACK_H