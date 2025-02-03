#include "prioritystack.h"

// Constructor
PriorityStack::PriorityStack() {
    size = 0;  // Heap starts empty
}

// Push an element (Insert in Heap)
void PriorityStack::push(int value, int priority) {
    if (size >= MAX_SIZE) {
        Serial.println("Heap Overflow! Stack is full.");
        return;
    }

    // Insert at end and heapify up
    heap[size] = {value, priority};
    heapifyUp(size);
    size++;
}

// Pop the highest-priority element (Remove Root)
void PriorityStack::pop() {
    if (isEmpty()) {
        Serial.println("Heap Underflow! Stack is empty.");
        return;
    }

    // Replace root with last element and heapify down
    heap[0] = heap[size - 1];
    size--;
    heapifyDown(0);
}

// Get the top element (Highest Priority)
int PriorityStack::top() {
    if (!isEmpty()) {
        return heap[0].value;
    }
    return -1;  // Return -1 if stack is empty
}

// Check if the stack is empty
bool PriorityStack::isEmpty() {
    return size == 0;
}

// Get the size of the stack
int PriorityStack::getSize() {
    return size;
}

// Heapify Up (for insertions)
void PriorityStack::heapifyUp(int index) {
    while (index > 0) {
        int parent = (index - 1) / 2;
        if (heap[index].priority > heap[parent].priority) {
            // Swap if child is greater than parent
            HeapNode temp = heap[index];
            heap[index] = heap[parent];
            heap[parent] = temp;
            index = parent;  // Move up
        } else {
            break;
        }
    }
}

// Heapify Down (for deletions)
void PriorityStack::heapifyDown(int index) {
    while (true) {
        int leftChild = 2 * index + 1;
        int rightChild = 2 * index + 2;
        int largest = index;

        // Find the largest among root, left, and right
        if (leftChild < size && heap[leftChild].priority > heap[largest].priority) {
            largest = leftChild;
        }
        if (rightChild < size && heap[rightChild].priority > heap[largest].priority) {
            largest = rightChild;
        }

        // If largest is not root, swap and continue
        if (largest != index) {
            HeapNode temp = heap[index];
            heap[index] = heap[largest];
            heap[largest] = temp;
            index = largest;
        } else {
            break;
        }
    }
}

// Print the priority stack (For Debugging)
void PriorityStack::printStack() {
    Serial.println("Priority Stack (Binary Heap):");
    for (int i = 0; i < size; i++) {
        Serial.print("Value: ");
        Serial.print(heap[i].value);
        Serial.print(", Priority: ");
        Serial.println(heap[i].priority);
    }
    Serial.println("----------------------");
}
