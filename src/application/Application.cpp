#include "application/Application.h"

#include "Config.h"
#include "domain/TelemetrySerializer.h"
#include "platform/Logger.h"

namespace cnc
{
namespace
{
MonotonicTimePoint monotonicNow() noexcept
{
    const auto elapsed =
        std::chrono::duration_cast<Milliseconds>(rtos::Kernel::Clock::now().time_since_epoch());
    return MonotonicTimePoint{elapsed};
}
} // namespace

Application::Application() noexcept
    : diagnosticLeds_(diagnosticSignals_), kpiTracker_(config::cycleFilterAlpha),
      publishPolicy_(config::publishHeartbeat), machineThread_(osPriorityNormal),
      energyThread_(osPriorityNormal), telemetryThread_(osPriorityNormal),
      diagnosticThread_(osPriorityNormal)
{
}

void Application::start() noexcept
{
    if (started_)
    {
        return;
    }
    started_ = true;

    initializeLogging();
    machineInputs_.begin();
    diagnosticLeds_.begin();
    diagnosticSignals_.set(DiagnosticChannel::System, LedMode::BlinkSlow);

    diagnosticThread_.start(
        [this]()
        {
            runDiagnostics();
        });
    energyThread_.start(
        [this]()
        {
            runEnergyAcquisition();
        });
    machineThread_.start(
        [this]()
        {
            runMachineAcquisition();
        });
    telemetryThread_.start(
        [this]()
        {
            runTelemetry();
        });
}

void Application::runMachineAcquisition()
{
    for (;;)
    {
        const MachineData machine = machineInputs_.read();
        const MonotonicTimePoint now = monotonicNow();
        const KpiData &kpis = kpiTracker_.update(machine, now);
        stateStore_.updateMachine(machine, kpis);
        rtos::ThisThread::sleep_for(config::machineSamplePeriod);
    }
}

void Application::runEnergyAcquisition()
{
    bool initialized = false;

    for (;;)
    {
        if (!initialized)
        {
            initialized = energyReader_.begin();
            if (!initialized)
            {
                logf("Finder 7M initialization failed");
                diagnosticSignals_.set(DiagnosticChannel::Modbus, LedMode::BlinkFast);
                rtos::ThisThread::sleep_for(config::energyPollPeriod);
                continue;
            }
            logf("Finder 7M ready");
        }

        EnergyData energy{};
        if (energyReader_.read(energy))
        {
            stateStore_.updateEnergy(energy);
            diagnosticSignals_.set(DiagnosticChannel::Modbus, LedMode::Pulse);
        }
        else
        {
            logf("Modbus read error");
            diagnosticSignals_.set(DiagnosticChannel::Modbus, LedMode::BlinkFast);
        }
        rtos::ThisThread::sleep_for(config::energyPollPeriod);
    }
}

void Application::runTelemetry()
{
    mqtt_.begin();
    diagnosticSignals_.set(DiagnosticChannel::Wifi, LedMode::BlinkSlow);

    for (std::uint8_t attempt = 0U; attempt < 10U && !wifi_.connected(); ++attempt)
    {
        logf("Connecting to Wi-Fi: %s", config::wifiSsid);
        wifi_.connectOnce();
        if (!wifi_.connected())
        {
            rtos::ThisThread::sleep_for(config::reconnectInterval);
        }
    }

    if (wifi_.connected())
    {
        diagnosticSignals_.set(DiagnosticChannel::Wifi, LedMode::On);
        if (!clock_.synchronize())
        {
            logf("NTP synchronization failed");
        }
    }
    else
    {
        logf("Wi-Fi initial connection timed out");
    }

    auto lastWifiAttempt = monotonicNow() - config::reconnectInterval;
    auto lastMqttAttempt = lastWifiAttempt;

    for (;;)
    {
        const MonotonicTimePoint now = monotonicNow();

        if (!wifi_.connected())
        {
            diagnosticSignals_.set(DiagnosticChannel::Wifi, LedMode::BlinkSlow);
            diagnosticSignals_.set(DiagnosticChannel::Mqtt, LedMode::BlinkSlow);
            if (now - lastWifiAttempt >= config::reconnectInterval)
            {
                lastWifiAttempt = now;
                if (wifi_.connectOnce())
                {
                    diagnosticSignals_.set(DiagnosticChannel::Wifi, LedMode::On);
                    clock_.synchronize();
                }
            }
        }
        else if (!mqtt_.connected() && now - lastMqttAttempt >= config::reconnectInterval)
        {
            lastMqttAttempt = now;
            diagnosticSignals_.set(DiagnosticChannel::Mqtt, LedMode::BlinkSlow);
            if (mqtt_.connect())
            {
                logf("MQTT connected");
                diagnosticSignals_.set(DiagnosticChannel::Mqtt, LedMode::On);
            }
            else
            {
                logf("MQTT connection failed: %d", mqtt_.connectError());
            }
        }

        mqtt_.poll();

        if (mqtt_.connected())
        {
            const SystemSnapshot snapshot = stateStore_.snapshot();
            if (publishPolicy_.shouldPublish(snapshot.machine, now))
            {
                char timestamp[32]{};
                char payload[config::mqttPayloadCapacity]{};
                const bool hasTimestamp = clock_.isoTimestamp(timestamp, sizeof(timestamp));
                const std::size_t length =
                    hasTimestamp ? serializeTelemetry(snapshot, timestamp, payload, sizeof(payload))
                                 : 0U;

                if (length != 0U && mqtt_.publish(config::mqttStateTopic, payload, length))
                {
                    publishPolicy_.markPublished(snapshot.machine, now);
                    logf("Device state published");
                }
                else
                {
                    logf("Telemetry serialization or publication failed");
                }
            }
        }

        rtos::ThisThread::sleep_for(config::telemetryLoopPeriod);
    }
}

void Application::runDiagnostics()
{
    for (;;)
    {
        diagnosticLeds_.update(monotonicNow());
        rtos::ThisThread::sleep_for(config::telemetryLoopPeriod);
    }
}

} // namespace cnc
