/*
 * gpio.h
 *
 *  Created on: Dec 21, 2025
 *      Author: philt
 */

#ifndef DRIVERS_GPIO_H_
#define DRIVERS_GPIO_H_

#include <cstdint>
#include <functional>
#include "stm32h7xx_hal.h"

//#define GPIO_ON(g)      HAL_GPIO_WritePin((g).port, (g).pin, GPIO_PIN_SET)
//#define GPIO_OFF(g)     HAL_GPIO_WritePin((g).port, (g).pin, GPIO_PIN_RESET)
//#define GPIO_TOGGLE(g)  HAL_GPIO_TogglePin((g).port, (g).pin)

struct gpio_t
{
    GPIO_TypeDef* port;
    uint16_t pin;
};

/* Internal helpers (header-only by necessity) */

constexpr GPIO_TypeDef* gpio_port(char p)
{
    return (p == 'A') ? GPIOA :
           (p == 'B') ? GPIOB :
           (p == 'C') ? GPIOC :
           (p == 'D') ? GPIOD :
           (p == 'E') ? GPIOE :
           (p == 'F') ? GPIOF :
           (p == 'G') ? GPIOG :
           (p == 'H') ? GPIOH :
           nullptr;
}

constexpr uint16_t gpio_pin_mask(const char* s,
                                 std::size_t i = 2,
                                 uint16_t v = 0)
{
    return (s[i] == '\0')
        ? static_cast<uint16_t>(1U << v)
        : gpio_pin_mask(
              s,
              i + 1,
              static_cast<uint16_t>(v * 10 + (s[i] - '0'))
          );
}

/**
 * @brief Construct a gpio_t from a string literal like "PG9"
 *
 * Format: "P<port><pin>", e.g. "PA0", "PC13", "PG9"
 *
 * Invalid input yields {nullptr, 0}
 */
constexpr gpio_t gpio(const char* s)
{
    return (s &&
            s[0] == 'P' &&
            gpio_port(s[1]) != nullptr)
        ? gpio_t{ gpio_port(s[1]), gpio_pin_mask(s) }
        : gpio_t{ nullptr, 0 };
}

class Led
{
public:
  void init(gpio_t gpio, uint32_t state)
  {
    gpio_ = gpio;
    set((GPIO_PinState)state);
  }

  void set(GPIO_PinState state) { HAL_GPIO_WritePin(gpio_.port, gpio_.pin, state);}
  void on(void) { HAL_GPIO_WritePin(gpio_.port, gpio_.pin, GPIO_PIN_SET);  }
  void off(void) { HAL_GPIO_WritePin(gpio_.port, gpio_.pin, GPIO_PIN_RESET); }
  void toggle(void) {HAL_GPIO_TogglePin(gpio_.port, gpio_.pin);}

private:
  gpio_t gpio_;

};


#endif /* DRIVERS_GPIO_H_ */
