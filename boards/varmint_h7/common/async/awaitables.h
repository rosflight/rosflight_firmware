/*
 * awaitables.h
 *
 *  Created on: Dec 19, 2025
 *      Author: philt
 */

#ifndef ASYNC_AWAITABLES_H_
#define ASYNC_AWAITABLES_H_

#include <async/event_dispatch.h>
#include <coroutine>
#include "main.h"

/* EXTI */
struct ExtiAwaitable {
    uint16_t pin;

    bool await_ready() const noexcept { return false; }

    void await_suspend(std::coroutine_handle<> h) noexcept {
        register_awaiter((void*)(uintptr_t)pin, h);
    }

    void await_resume() noexcept {}
};

/* SPI DMA */
struct SpiDmaAwaitable {
    SPI_HandleTypeDef* hspi;
    uint8_t* tx;
    uint8_t* rx;
    uint16_t len;

    bool await_ready() const noexcept { return false; }

    void await_suspend(std::coroutine_handle<> h) noexcept {
        register_awaiter(hspi, h);
        HAL_SPI_TransmitReceive_DMA(hspi, tx, rx, len);
    }

    void await_resume() noexcept {}
};





#endif /* ASYNC_AWAITABLES_H_ */
