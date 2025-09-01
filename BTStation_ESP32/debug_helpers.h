#pragma once

#include <Arduino.h>

constexpr bool DEBUG_ENABLED =
#ifdef DEBUG
    true;
#else
    false;
#endif

namespace {

    inline void debugLogPrefix(const char* file, int line) {
        Serial.print(file);
        Serial.print(F(":"));
        Serial.print(line);
        Serial.print(F(" "));
    }
        
    inline void debugPrintHex8(uint8_t value) {
        if (value < 0x10) Serial.print('0');
        Serial.print(value, HEX);
    }

}

#define DEBUG_PREFIX() debugLogPrefix(__FILE__, __LINE__)

template<typename First, typename... Rest>
inline void debugLog(First&& first, Rest&&... rest) {
    if constexpr (DEBUG_ENABLED) {
        DEBUG_PREFIX();
        Serial.print(std::forward<First>(first));
        ((Serial.print(' '), Serial.print(std::forward<Rest>(rest))), ...);
        Serial.println();
    }
}

template<typename... Args>
inline void debugError(Args&&... args) {
    if constexpr (DEBUG_ENABLED) {
        debugLog(F("ERROR:"), std::forward<Args>(args)...);
    }
}

template<typename T>
inline void debugLogHex(const __FlashStringHelper* label, T value) {
    static_assert(std::is_integral_v<T>, "debugLogHex requires an integral type");

    if constexpr (DEBUG_ENABLED) {
        DEBUG_PREFIX();
        Serial.print(label);
        Serial.print(F("=0x"));

        constexpr uint8_t digits = sizeof(T) * 2;

        // Print with leading zeros
        for (int i = (digits - 1) * 4; i >= 0; i -= 4) {
            uint8_t nibble = (value >> i) & 0xF;
            Serial.print(nibble, HEX);
        }
        Serial.println();
    }
}

inline void debugLogHexArray(const __FlashStringHelper* label, const uint8_t* data, size_t length) {
    if constexpr (DEBUG_ENABLED) {
        DEBUG_PREFIX();
        Serial.print(label);

        for (size_t i = 0; i < length; i++) {
            Serial.print(F(" "));
            if (data[i] < 0x10) Serial.print(F("0"));
            Serial.print(data[i], HEX);
        }
        Serial.println();
    }
}

inline void debugLogDateTime(const uint8_t* buf) {
    if constexpr (DEBUG_ENABLED) {
        DEBUG_PREFIX();
        Serial.print(F("Time: "));
        Serial.print(buf[0] + 2000); // год
        Serial.print(F("-"));
        if (buf[1] < 10) Serial.print('0');
        Serial.print(buf[1]);        // месяц
        Serial.print(F("-"));
        if (buf[2] < 10) Serial.print('0');
        Serial.print(buf[2]);        // день
        Serial.print(F(" "));
        if (buf[3] < 10) Serial.print('0');
        Serial.print(buf[3]);        // час
        Serial.print(F(":"));
        if (buf[4] < 10) Serial.print('0');
        Serial.print(buf[4]);        // минута
        Serial.print(F(":"));
        if (buf[5] < 10) Serial.print('0');
        Serial.print(buf[5]);        // секунда
        Serial.println();
    }
}