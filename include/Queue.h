//
// Created by anaple on 2024/4/25.
//

#ifndef UMS_SDK_QUEUE_H
#define UMS_SDK_QUEUE_H
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

class CircularQueue {
private:
    std::vector<std::string> buffer;
    int head;
    int tail;
    int maxSize;
    bool isFull;

public:
    CircularQueue(int size) : maxSize(size), head(0), tail(0), isFull(false) {
        buffer.resize(size);
    }

    bool enqueue(std::string data);

    bool dequeue(std::string& data);

    bool isEmpty() const;
};
#endif //UMS_SDK_QUEUE_H
