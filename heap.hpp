//
// Created by Brummell, Doug on 10/16/16.
//

#ifndef COURSERA_HEAP_HPP_H
#define COURSERA_HEAP_HPP_H
#include <random>
#include <vector>
#include <unordered_map>
#include <set>
#include <cassert>
#include <unordered_set>
#include <string>
#include <queue>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <limits>

template <typename T>
class Heap {
private:
    set<T> items{};
public:
    Heap(const auto &collection, const auto &comparator) : {};

    void update_score() {};

    auto extract_min() {};

    auto contains() {
        return True;
    };

    auto size() {
        return True;
    };

    auto is_empty() {
        return True;
    }
};




#endif //COURSERA_HEAP_HPP_H
chgPrioirity(PQ, priority): changes the priority (node value) of queue element.
minPrioirty(PQ): removes the top element of the queue.
contains(PQ, queue_element): does the queue contain queue_element.
Insert(PQ, queue_element): insert queue_element into queue
top(PQ):returns the top element of the queue.
size(PQ): return the number of queue_elements.