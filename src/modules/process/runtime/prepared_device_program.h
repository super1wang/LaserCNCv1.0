#pragma once

#include "modules/cam/contracts/toolpath_export_dto.h"
#include "modules/process/tool/tool.h"

#include <functional>
#include <memory>

namespace lcnc::process {

// Captured on the configuration owner thread. No pointers to ToolFactory or
// editable settings survive preparation. Revision is a content identity, not
// controller qualification and not CAM's (binding-only) toolProcessHash.
struct DeviceRunRecipe {
    QByteArray planHash;
    QByteArray contextHash;
    QByteArray revision;
    QString sourceId;
    toml::value processIoProfile{toml::table{}};
    QHash<std::uint64_t, Tool> toolsByContour;
    double feedOverride{1.0};
};

QByteArray deviceRunRecipeHash(const DeviceRunRecipe& recipe);

class PreparedDeviceProgram final {
public:
    static std::shared_ptr<const PreparedDeviceProgram> prepare(
        const cam::ToolpathExportSnapshot& snapshot, const DeviceRunRecipe& recipe,
        std::uint64_t runEpoch, bool realMachine, QString* error);

    const cam::CamMotionPlanSnapshot& plan() const { return m_plan; }
    const MachineAxisLayout& layout() const { return m_layout; }
    const DeviceRunRecipe& recipe() const { return m_recipe; }
    std::uint64_t runEpoch() const { return m_runEpoch; }
    bool realMachine() const { return m_realMachine; }

private:
    PreparedDeviceProgram() = default;
    cam::CamMotionPlanSnapshot m_plan;
    MachineAxisLayout m_layout;
    DeviceRunRecipe m_recipe;
    std::uint64_t m_runEpoch{0};
    bool m_realMachine{false};
};

// Encoding-only boundary. Implementations must stage a finite program without
// starting motion. Structural fences are delivered, never interpreted as IO.
class ExactPlanConsumer {
public:
    virtual ~ExactPlanConsumer() = default;
    virtual bool begin(const PreparedDeviceProgram&, QString*) = 0;
    virtual bool block(const cam::CamMotionBlock&, const Tool&, QString*) = 0;
    virtual bool knot(const cam::CamMotionBlock&, int index, QString*) = 0;
    virtual bool fence(const cam::CamMotionBlock&, const cam::MotionProcessFence&, QString*) = 0;
    virtual bool seal(QString*) = 0;
    virtual void discard() noexcept = 0;
};

bool consumeExactPlan(const PreparedDeviceProgram& program, ExactPlanConsumer& consumer,
                      const std::function<bool()>& cancelled, QString* error);

} // namespace lcnc::process
