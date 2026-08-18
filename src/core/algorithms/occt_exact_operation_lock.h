#pragma once

#include <mutex>

namespace lcnc {

/// Serialises OCCT exact distance/boolean algorithms process-wide.  OCCT may
/// lazily update shared topology even when callers own separate algorithm
/// instances, so task-local locks are not sufficient.
class OcctExactOperationLock final
{
public:
    OcctExactOperationLock();
    ~OcctExactOperationLock();

    OcctExactOperationLock(const OcctExactOperationLock&) = delete;
    OcctExactOperationLock& operator=(const OcctExactOperationLock&) = delete;

private:
    std::unique_lock<std::mutex> m_lock;
};

} // namespace lcnc
