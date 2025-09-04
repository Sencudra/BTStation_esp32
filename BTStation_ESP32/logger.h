#pragma once


#ifndef DEBUG
    #define logDebug(...) (void)0
    #define logError(...) (void)0
    #define logDebugHex(...) (void)0
    #define logDebugHexArray(...) (void)0
    #define logDebugDateTime(...) (void)0
#else
    #include <Arduino.h>
    constexpr bool DEBUG_ENABLED = true;
    #define DEBUG_PREFIX() logDebugPrefix(__FILE__, __LINE__)
    #define logDebug(...) do {DEBUG_PREFIX(); _logDebug(__VA_ARGS__);} while(0)
    #define logError(...) do {DEBUG_PREFIX(); _logError(__VA_ARGS__);} while(0)
    #define logDebugHex(...) do {DEBUG_PREFIX(); _logDebugHex(__VA_ARGS__);} while(0)
    #define logDebugHexArray(...) do {DEBUG_PREFIX(); _logDebugHexArray(__VA_ARGS__);} while(0)
    #define logDebugDateTime(...) do {DEBUG_PREFIX(); _logDebugDateTime(__VA_ARGS__);} while(0)

namespace {

    inline void logDebugPrefix(const char* file, int line) {
        Serial.print(F("line:"));
        Serial.print(line);
        Serial.print(F(" "));
    }
        
    inline void debugPrintHex8(uint8_t value) {
        if (value < 0x10) Serial.print('0');
        Serial.print(value, HEX);
    }

}

template<typename First, typename... Rest>
inline void _logDebug(First&& first, Rest&&... rest) {
    if constexpr (DEBUG_ENABLED) {
        Serial.print(std::forward<First>(first));
        ((Serial.print(' '), Serial.print(std::forward<Rest>(rest))), ...);
        Serial.println();
    }
}

template<typename... Args>
inline void _logError(Args&&... args) {
    if constexpr (DEBUG_ENABLED) {
        _logDebug(F("ERROR:"), std::forward<Args>(args)...);
    }
}

template<typename T>
inline void _logDebugHex(const __FlashStringHelper* label, T value) {
    static_assert(std::is_integral_v<T>, "logDebugHex requires an integral type");

    if constexpr (DEBUG_ENABLED) {
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

inline void _logDebugHexArray(const __FlashStringHelper* label, const uint8_t* data, size_t length) {
    if constexpr (DEBUG_ENABLED) {
        Serial.print(label);

        for (size_t i = 0; i < length; i++) {
            Serial.print(F(" "));
            if (data[i] < 0x10) Serial.print(F("0"));
            Serial.print(data[i], HEX);
        }
        Serial.println();
    }
}

inline void _logDebugDateTime(const uint8_t* buf) {
    if constexpr (DEBUG_ENABLED) {
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

#endif
