#include <cstdint>

#include "notifier.h"
#include "logger.h"

void Notifier::init(Buzzer buzzer, LED green_led, LED red_led) {
    m_buzzer.emplace(buzzer);
    m_green_led.emplace(green_led);
    m_red_led.emplace(red_led);
    m_initialized = true;
}

void Notifier::notify(uint8_t times, uint16_t durationMs) {
    reset();
    m_action.type = ActionState::NOTIFY;
    m_action.remaining = times;
    m_action.onTime = durationMs;
    m_action.offTime = 500;
    m_action.lastMillis = millis();
    m_action.isOn = false;
    m_action.frequency = 4000;
}

void Notifier::notifyError(uint8_t times, uint16_t durationMs) {
    reset();
    m_action.type = ActionState::NOTIFY_ERROR;
    m_action.remaining = times;
    m_action.onTime = durationMs;
    m_action.offTime = 500;
    m_action.lastMillis = millis();
    m_action.isOn = false;
    m_action.frequency = 500;
}

void Notifier::reset() {
    if (m_initialized) {
        m_buzzer->off();
        m_red_led->off();
        m_green_led->off();
    }
    m_action.type = ActionState::NONE;
    m_action.remaining = 0;
    m_action.isOn = false;
}

void Notifier::update() {
    if (!m_initialized) {
        logError(F("Notifier is not initialized"));
        return;
    }

    if (m_action.remaining == 0) return;

    uint32_t now = millis();
    switch (m_action.type) {
    case ActionState::NOTIFY:
        updateBeepLED(now, m_green_led.value());
        break;

    case ActionState::NOTIFY_ERROR:
        updateBeepLED(now, m_red_led.value());
        break;

    default:
        break;
    }
}

void Notifier::updateBeepLED(uint32_t now, const std::optional<LED>& led) {
    if (!m_action.isOn && (now - m_action.lastMillis >= m_action.offTime)) {
        m_buzzer->on(m_action.frequency);
        led->on();
        m_action.lastMillis = now;
        m_action.isOn = true;
    } 
    else if (m_action.isOn && (now - m_action.lastMillis >= m_action.onTime)) {
        m_buzzer->off();
        led->off();
        m_action.lastMillis = now;
        m_action.isOn = false;
        --m_action.remaining;
    }
}