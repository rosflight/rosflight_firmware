/*
 * async_mutex.h
 *
 *  Created on: Dec 21, 2025
 *      Author: philt
 */

#ifndef ASYNC_ASYNC_MUTEX_H_
#define ASYNC_ASYNC_MUTEX_H_

#include <coroutine>
#include <atomic>
#include <cstdint>
#include "cmsis_gcc.h"   // or core_cm*.h for SCB

class async_mutex {
public:
    async_mutex() = default;

    async_mutex(const async_mutex&) = delete;
    async_mutex& operator=(const async_mutex&) = delete;

    /* ---------- awaitable lock ---------- */

    class lock_awaiter {
    public:
        explicit lock_awaiter(async_mutex& m) : mutex(m) {}

        bool await_ready() noexcept {
            // Fast path: try to acquire immediately
            bool expected = false;
            return mutex.locked.compare_exchange_strong(
                expected, true, std::memory_order_acquire);
        }

        void await_suspend(std::coroutine_handle<> h) noexcept {
            handle = h;
            mutex.enqueue(this);
        }

        void await_resume() noexcept {}

    private:
        async_mutex& mutex;
        std::coroutine_handle<> handle{};

        friend class async_mutex;
    };

    lock_awaiter lock() noexcept {
        return lock_awaiter{*this};
    }

    /* ---------- unlock ---------- */

    // Must be called when the protected async operation completes.
    // Safe to call from ISR.
    void unlock_from_isr() noexcept {
        lock_awaiter* next = dequeue();
        if (next) {
            // Transfer ownership directly
            SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
            next->handle.resume();
        } else {
            locked.store(false, std::memory_order_release);
        }
    }

private:
    std::atomic<bool> locked{false};

    /* ---------- wait queue (intrusive, single-linked) ---------- */

    lock_awaiter* head{nullptr};
    lock_awaiter* tail{nullptr};

    void enqueue(lock_awaiter* w) noexcept {
        __disable_irq();
        if (!tail) {
            head = tail = w;
        } else {
            tail->next = w;
            tail = w;
        }
        __enable_irq();
    }

    lock_awaiter* dequeue() noexcept {
        __disable_irq();
        auto* w = head;
        if (w) {
            head = w->next;
            if (!head)
                tail = nullptr;
            w->next = nullptr;
        }
        __enable_irq();
        return w;
    }
};




#endif /* ASYNC_ASYNC_MUTEX_H_ */
