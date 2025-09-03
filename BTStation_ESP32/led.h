#pragma once

#include <Arduino.h>

class LED {
public:
    LED(uint8_t pin) : m_led_pin(pin) {};

    void on() const { digitalWrite(m_led_pin, HIGH); }
    void off() const { digitalWrite(m_led_pin, LOW); }

private:
    uint8_t m_led_pin;
};
