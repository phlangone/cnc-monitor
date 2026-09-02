#include "platform/Network.h"

#include "Config.h"

#include <ctime>
#include <mbed_mktime.h>

namespace cnc
{

bool WifiStation::connected() const noexcept
{
    return WiFi.status() == WL_CONNECTED;
}

bool WifiStation::connectOnce() const noexcept
{
    return WiFi.begin(config::wifiSsid, config::wifiPassword) == WL_CONNECTED;
}

RtcClock::RtcClock() : ntp_(udp_, "pool.ntp.org", 0)
{
}

bool RtcClock::synchronize() noexcept
{
    ntp_.begin();
    const bool updated = ntp_.update();
    if (updated)
    {
        set_time(ntp_.getEpochTime());
    }
    ntp_.end();
    return updated;
}

bool RtcClock::isoTimestamp(char *const output, const std::size_t outputSize) const noexcept
{
    if (output == nullptr || outputSize == 0U)
    {
        return false;
    }
    const std::time_t now = std::time(nullptr);
    std::tm utc{};
    if (gmtime_r(&now, &utc) == nullptr)
    {
        output[0] = '\0';
        return false;
    }
    return std::strftime(output, outputSize, "%Y-%m-%dT%H:%M:%SZ", &utc) != 0U;
}

MqttTelemetryPublisher::MqttTelemetryPublisher() : client_(transport_)
{
}

void MqttTelemetryPublisher::begin() noexcept
{
    client_.setId(config::mqttClientId);
    client_.setUsernamePassword(config::mqttUsername, config::mqttPassword);
    client_.setTxPayloadSize(config::mqttTxCapacity);
}

bool MqttTelemetryPublisher::connected() noexcept
{
    return client_.connected();
}

bool MqttTelemetryPublisher::connect() noexcept
{
    return client_.connect(config::mqttBroker, config::mqttPort);
}

void MqttTelemetryPublisher::poll() noexcept
{
    if (connected())
    {
        client_.poll();
    }
}

bool MqttTelemetryPublisher::publish(const char *const topic, const char *const payload,
                                     const std::size_t length) noexcept
{
    if (!connected() || topic == nullptr || payload == nullptr || length == 0U)
    {
        return false;
    }
    if (!client_.beginMessage(topic, static_cast<unsigned long>(length)))
    {
        return false;
    }
    const std::size_t written =
        client_.write(reinterpret_cast<const std::uint8_t *>(payload), length);
    const bool completed = client_.endMessage();
    return written == length && completed;
}

int MqttTelemetryPublisher::connectError() noexcept
{
    return client_.connectError();
}

} // namespace cnc
