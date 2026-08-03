#include "modules/process/runtime/process_cutting_safety.h"

#include <cassert>

using namespace lcnc::process;

int main()
{
    ContourBoundaryHealth health;
    auto result = evaluateContourBoundaryHealth(health);
    assert(!result.success);
    assert(result.error.contains(QStringLiteral("not connected")));

    health.connected = true;
    result = evaluateContourBoundaryHealth(health);
    assert(!result.success);
    assert(result.error.contains(QStringLiteral("Unable to read")));

    health.statusReadable = true;
    health.faultCode = 17;
    result = evaluateContourBoundaryHealth(health);
    assert(!result.success);
    assert(result.error.contains(QStringLiteral("17")));

    health.faultCode = 0;
    health.missingAxes = {QStringLiteral("A")};
    result = evaluateContourBoundaryHealth(health);
    assert(!result.success);
    assert(result.error.contains(QStringLiteral("A")));

    health.missingAxes.clear();
    health.disabledAxes = {QStringLiteral("C")};
    result = evaluateContourBoundaryHealth(health);
    assert(!result.success);
    assert(result.error.contains(QStringLiteral("C")));

    health.disabledAxes.clear();
    result = evaluateContourBoundaryHealth(health);
    assert(result.success);
    assert(result.completion == DeviceCommandCompletion::Succeeded);
    return 0;
}
