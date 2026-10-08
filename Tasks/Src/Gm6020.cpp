
#include "Gm6020.h"

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr uint16_t kAngleRange = 8192;
constexpr float kRpmToRadS = 2.0f * kPi / 60.0f;
constexpr float kRawToAmp = 3.0f / 16384.0f;

}

Gm6020::Gm6020(uint8_t id)
    : id_(id),
      current_command_(0),
      raw_angle_(0),
      raw_vel_rpm_(0),
      raw_current_(0),
      raw_temp_(0),
      full_turns_(0),
      last_raw_angle_(0),
      angle_initialized_(false)
{
}

uint32_t Gm6020::rxId() const
{
    return 0x204u + id_;
}

uint32_t Gm6020::txId() const
{
    // Current control protocol (to be verified)
    return (id_ <= 4) ? 0x1FEu : 0x2FEu;
}

void Gm6020::decode(const uint8_t *data)
{
    uint16_t raw = static_cast<uint16_t>(
        (static_cast<uint16_t>(data[0]) << 8) | data[1]
    );

    if (!angle_initialized_)
    {
        last_raw_angle_ = raw;
        angle_initialized_ = true;
    }
    else
    {
        int32_t diff = static_cast<int32_t>(raw)
                     - static_cast<int32_t>(last_raw_angle_);

        if (diff > 4096)
        {
            full_turns_--;
        }
        else if (diff < -4096)
        {
            full_turns_++;
        }

        last_raw_angle_ = raw;
    }

    raw_angle_ = raw;

    raw_vel_rpm_ = static_cast<int16_t>(
        (static_cast<uint16_t>(data[2]) << 8) | data[3]
    );

    raw_current_ = static_cast<int16_t>(
        (static_cast<uint16_t>(data[4]) << 8) | data[5]
    );

    raw_temp_ = data[6];
}

float Gm6020::angle() const
{
    return (static_cast<float>(full_turns_) * kAngleRange
            + static_cast<float>(raw_angle_))
           * (2.0f * kPi / kAngleRange);
}

float Gm6020::vel() const
{
    return static_cast<float>(raw_vel_rpm_) * kRpmToRadS;
}

float Gm6020::current() const
{
    return static_cast<float>(raw_current_) * kRawToAmp;
}

float Gm6020::temp() const
{
    return static_cast<float>(raw_temp_);
}

int16_t Gm6020::velRpm() const
{
    return raw_vel_rpm_;
}

void Gm6020::setCurrent(int16_t current)
{
    current_command_ = current;
}

int16_t Gm6020::currentCommand() const
{
    return current_command_;
}

void Gm6020::encode(uint8_t *data) const
{
    // Caller must provide an 8-byte initialized buffer.
    // ID 1-4: DATA[0:1], [2:3], [4:5], [6:7]
    // ID 5-7: DATA[0:1], [2:3], [4:5]

    if (id_ < 1 || id_ > 7 || data == nullptr)
    {
        return;
    }

    uint8_t index = (id_ <= 4)
        ? static_cast<uint8_t>((id_ - 1) * 2)
        : static_cast<uint8_t>((id_ - 5) * 2);

    uint16_t raw = static_cast<uint16_t>(current_command_);

    data[index] = static_cast<uint8_t>((raw >> 8) & 0xFF);
    data[index + 1] = static_cast<uint8_t>(raw & 0xFF);
}