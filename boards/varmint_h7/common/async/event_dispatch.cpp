/*
 * eventdispatch.cpp
 *
 *  Created on: Dec 19, 2025
 *      Author: philt
 */


#include <async/event_dispatch.h>

struct Slot {
    void* key = nullptr;
    std::coroutine_handle<> h;
};

static Slot slots[8];

void register_awaiter(void* key, std::coroutine_handle<> h) {
    for (auto& s : slots) {
        if (!s.key) {
            s.key = key;
            s.h = h;
            return;
        }
    }
    while (1);
}

void resume_awaiter(void* key) {
    for (auto& s : slots) {
        if (s.key == key) {
            auto h = s.h;
            s.key = nullptr;
            h.resume();
            return;
        }
    }
}


