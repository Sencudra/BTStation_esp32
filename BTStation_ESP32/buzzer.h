#pragma once

#define PWM_FREQ 5000
#define PWM_RESOLUTION 10

class Buzzer {
public:
    Buzzer(
        uint8_t pin,
        uint8_t resolution = PWM_RESOLUTION
    );
    ~Buzzer();

    void on(uint32_t frequency = PWM_FREQ) const;
    void off() const;

    

private:
    uint8_t m_pin;
};