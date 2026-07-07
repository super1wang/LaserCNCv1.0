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
    virtual ToolpathExportSnapshot exportToolpathSnapshot() const = 0;
    virtual ToolpathExportSnapshot exportToolpathSnapshotForOrder(
        const QVector<std::uint64_t>& orderedContourIds) const = 0;
};

} // namespace lcnc::cam
