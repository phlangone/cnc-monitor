#include "platform/Logger.h"

#include "Config.h"
#include "platform/Rtos.h"

#include <chrono>
#include <cstdarg>
#include <cstdio>

namespace cnc
{
namespace
{
#if CNC_LOG_ENABLED
rtos::Mutex serialMutex;
#endif
} // namespace

void initializeLogging() noexcept
{
#if CNC_LOG_ENABLED
    Serial.begin(9600);
#endif
}

void logf(const char *const format, ...) noexcept
{
#if CNC_LOG_ENABLED
    if (format == nullptr)
    {
        return;
    }

    char buffer[160];
    va_list arguments;
    va_start(arguments, format);
    std::vsnprintf(buffer, sizeof(buffer), format, arguments);
    va_end(arguments);

    if (serialMutex.trylock_for(std::chrono::milliseconds{50}))
    {
        Serial.println(buffer);
        serialMutex.unlock();
    }
#else
    (void)format;
#endif
}

} // namespace cnc
