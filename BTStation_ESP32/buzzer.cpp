#include "logger.h"
#include "buzzer.h"

Buzzer::Buzzer(
    uint8_t pin,
    uint8_t resolution
) : m_pin(pin) {
    if (!ledcAttach(m_pin, 0, resolution)) {
        logError(F("Failed to setup buzzer pin"));
    }
}

Buzzer::~Buzzer() {
    ledcDetach(m_pin);
}

void Buzzer::on(uint32_t frequency) const {
    if (ledcWriteTone(m_pin, frequency) == 0) {
        logError(F("Failed to turn on buzzer"));
    }
}

void Buzzer::off() const {
    if (ledcWriteTone(m_pin, 0) == 0) {
        logError(F("Failed to turn off buzzer"));
    }
}