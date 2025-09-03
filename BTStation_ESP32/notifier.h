#pragma once

#include <optional>
#include "buzzer.h"
#include "led.h"

class Notifier {
public:
    Notifier() = default;
    void init(Buzzer buzzer, LED green_led, LED red_led);

    void notify(uint8_t times = 1, uint16_t durationMs = 300);
    void notifyError(uint8_t times = 4, uint16_t durationMs = 50);
    
    void reset();
    void update(); // Should be called in a loop.

private:
    struct ActionState {
        enum Type { NONE, NOTIFY, NOTIFY_ERROR } type = NONE;
        uint8_t remaining = 0;
        uint32_t onTime = 0;
        uint32_t offTime = 0;
        uint32_t lastMillis = 0;
        bool isOn = false;
        uint32_t frequency = 0;
    } m_action;

private:
    void updateBeepLED(uint32_t now, const std::optional<LED>& led);

private:
    bool m_initialized{ false };
    std::optional<Buzzer> m_buzzer;
    std::optional<LED> m_green_led;
    std::optional<LED> m_red_led;
};