/*
 * coroutine_task.h
 *
 *  Created on: Dec 19, 2025
 *      Author: philt
 */

#ifndef ASYNC_COROUTINE_TASK_H_
#define ASYNC_COROUTINE_TASK_H_

#include <coroutine>

struct Task {
    struct promise_type {
        Task get_return_object() {
            return Task{
                std::coroutine_handle<promise_type>::from_promise(*this)
            };
        }

        std::suspend_always initial_suspend() noexcept { return {}; }
        std::suspend_always final_suspend() noexcept { return {}; }
        void return_void() noexcept {}
        void unhandled_exception() { while (1); }

        static void* operator new(size_t sz) {
            static uint8_t storage[1024];
            if (sz > sizeof(storage)) while (1);
            return storage;
        }
        static void operator delete(void*) {}
    };

    std::coroutine_handle<promise_type> h;

    void resume() {
        if (!h.done()) h.resume();
    }
};


#endif /* ASYNC_COROUTINE_TASK_H_ */
