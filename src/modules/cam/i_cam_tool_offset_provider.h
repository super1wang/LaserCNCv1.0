#pragma once

#include "core/kernel/i_service.h"

#include <QString>

namespace lcnc::cam {

/**
 * @brief Read-only bridge for displaying a configured tool's rapid offset.
 *
 * CAM owns the rapid geometry and must not include Process tool classes.  The
 * Process module optionally implements this contract from its tool catalogue.
 * A missing tool is deliberately represented by false so CAM draws the exact
 * planned curve instead of inventing a legacy default offset.
 */
class ICamToolOffsetProvider : public lcnc::IService
{
public:
    ~ICamToolOffsetProvider() override = default;

    /// Returns the configured idle-height offset for \p toolName.
    virtual bool rapidDisplayOffsetMm(const QString& toolName, double* offsetMm) const = 0;
};

} // namespace lcnc::cam
