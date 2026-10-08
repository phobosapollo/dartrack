
#ifndef GM6020_H
#define GM6020_H

#include <cstdint>

class Gm6020 {
public:
    explicit Gm6020(uint8_t id);
    ~Gm6020() = default;

    // CAN ID
    uint32_t rxId() const;
    uint32_t txId() const;

    // Feedback
    float angle() const;       // Multi-turn angle (rad)
    float vel() const;         // Angular velocity (rad/s)
    float current() const;     // Feedback current (A)
    float temp() const;        // Temperature (C)
    int16_t velRpm() const;    // Velocity (rpm)

    // Current command (raw protocol value)
    void setCurrent(int16_t current);
    int16_t currentCommand() const;

    // CAN protocol
    void decode(const uint8_t *data);
    void encode(uint8_t *data) const;

private:
    uint8_t id_;
    int16_t current_command_;

    uint16_t raw_angle_;
    int16_t raw_vel_rpm_;
    int16_t raw_current_;
    uint8_t raw_temp_;

    int32_t full_turns_;
    uint16_t last_raw_angle_;
    bool angle_initialized_;
};

#endif // GM6020_H