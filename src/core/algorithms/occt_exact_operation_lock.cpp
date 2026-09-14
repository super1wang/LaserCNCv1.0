#include "core/algorithms/occt_exact_operation_lock.h"

namespace lcnc {
namespace {

// This mutex deliberately has one out-of-line definition in lcnc_core.
// Header-local or task-local locks do not protect exact OCCT operations made
// by other modules or by a verification task that is still winding down.
std::mutex& exactOperationMutex()
{
    static std::mutex instance;
    return instance;
}

} // namespace

OcctExactOperationLock::OcctExactOperationLock()
    : m_lock(exactOperationMutex())
{
}

OcctExactOperationLock::~OcctExactOperationLock() = default;

} // namespace lcnc
