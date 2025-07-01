#include "utils/Timer.h"
#include "utils/Logger.h"

namespace CloudStream {

// Timer implementation is header-only

// ScopedTimer destructor implementation
ScopedTimer::~ScopedTimer() {
    double elapsed = timer_.elapsedMilliseconds();
    Logger::instance().debug("{} took {:.2f} ms", name_, elapsed);
}

} // namespace CloudStream 