#include "domain/PublishPolicy.h"

namespace cnc
{

PublishPolicy::PublishPolicy(const Milliseconds heartbeat) noexcept : heartbeat_(heartbeat)
{
}

bool PublishPolicy::shouldPublish(const MachineData &current,
                                  const MonotonicTimePoint now) const noexcept
{
    const bool stateChanged = current.machineOn != lastPublished_.machineOn ||
                              current.machineReady != lastPublished_.machineReady ||
                              current.doorClosed != lastPublished_.doorClosed ||
                              current.doorOpened != lastPublished_.doorOpened ||
                              current.partClamped != lastPublished_.partClamped ||
                              current.spindleRunning != lastPublished_.spindleRunning;
    const bool heartbeatElapsed = now - lastPublish_ >= heartbeat_;
    return stateChanged || heartbeatElapsed || !hasPublished_;
}

void PublishPolicy::markPublished(const MachineData &published,
                                  const MonotonicTimePoint now) noexcept
{
    lastPublished_ = published;
    lastPublish_ = now;
    hasPublished_ = true;
}

} // namespace cnc
