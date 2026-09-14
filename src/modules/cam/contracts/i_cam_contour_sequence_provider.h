#pragma once

#include "core/kernel/i_service.h"
#include "core/project/cam/cam_data_contracts.h"
#include "core/project/cam/layer_contracts.h"

#include <QVector>

#include <cstdint>

namespace lcnc::cam {

/**
 * @brief Immutable authoritative CAM cutting sequence.
 *
 * The sequence is deliberately CAM-owned: Process consumes it to prepare
 * controller commands and the offline simulator consumes the exact same
 * solved order.  Neither consumer is allowed to redefine it.
 */
struct ContourSequenceSnapshot
{
    QVector<ContourId> orderedContourIds;
    CuttingPlanSortStrategy strategy{CuttingPlanSortStrategy::LayerThenContour};
    std::uint64_t revision{0};
};

class ICamContourSequenceProvider : public lcnc::IService
{
public:
    ~ICamContourSequenceProvider() override = default;

    virtual ContourSequenceSnapshot contourSequence() const = 0;
};

} // namespace lcnc::cam
