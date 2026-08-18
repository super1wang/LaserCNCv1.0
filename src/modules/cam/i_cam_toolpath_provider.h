#pragma once

#include "core/kernel/i_service.h"
#include "modules/cam/contracts/toolpath_export_dto.h"

namespace lcnc::cam {

/**
 * @brief Exports CAM toolpath data as OCC-free snapshots for Process runtime.
 */
class ICamToolpathProvider : public lcnc::IService
{
public:
    ~ICamToolpathProvider() override = default;

    virtual bool hasToolpath() const = 0;
    virtual std::uint64_t toolpathRevision() const = 0;
    /// A detached, read-only-by-contract description of all CAM contours.
    /// It is for list/preflight display only and deliberately carries no
    /// promise that a rapid plan has been committed.
    virtual ToolpathExportSnapshot exportToolpathCatalogSnapshot() const = 0;

    /// The exact current CAM execution result: CAM's committed order, solved
    /// cutting coordinates, and its matching rapid plan.  Consumers receive a
    /// value copy and have no API for changing/reordering/re-solving it.
    virtual ToolpathExportSnapshot exportCommittedExecutionSnapshot() const = 0;
};

} // namespace lcnc::cam
