
inline uint16_t readUInt16(const uint8_t* buffer) {
    if (buffer == nullptr) return 0;
    return (static_cast<uint16_t>(buffer[0]) << 8) |
           static_cast<uint16_t>(buffer[1]);
}

inline uint32_t readUInt32(const uint8_t* buffer) {
    if (buffer == nullptr) return 0;
    return (static_cast<uint32_t>(buffer[0]) << 24) |
           (static_cast<uint32_t>(buffer[1]) << 16) |
           (static_cast<uint32_t>(buffer[2]) << 8)  |
           static_cast<uint32_t>(buffer[3]);
}