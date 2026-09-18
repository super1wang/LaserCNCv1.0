#pragma once

#include "modules/cam/contracts/toolpath_export_dto.h"
#include "modules/process/runtime/frozen_tool_execution_recipe.h"
#include <toml.hpp>

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
    QHash<std::uint64_t, FrozenToolExecutionRecipe> toolsByContour;
    double feedOverride{1.0};
};

QByteArray deviceRunRecipeHash(const DeviceRunRecipe& recipe);

struct PreparedExecutionSection {
    int ordinal{0};
    std::uint64_t contourId{0};
    int firstBlock{0};
    int lastBlock{0};
    std::uint64_t firstBlockId{0};
    std::uint64_t lastBlockId{0};
    QByteArray firstBlockHash;
    QByteArray lastBlockHash;
    QByteArray planHash;
    std::uint64_t runEpoch{0};
};

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
    const QVector<PreparedExecutionSection>& sections() const { return m_sections; }

private:
    PreparedDeviceProgram() = default;
    cam::CamMotionPlanSnapshot m_plan;
    MachineAxisLayout m_layout;
    DeviceRunRecipe m_recipe;
    std::uint64_t m_runEpoch{0};
    bool m_realMachine{false};
    QVector<PreparedExecutionSection> m_sections;
};

// Encoding-only boundary. Implementations must stage a finite program without
// starting motion. Structural fences are delivered, never interpreted as IO.
class ExactPlanConsumer {
public:
    virtual ~ExactPlanConsumer() = default;
    virtual bool begin(const PreparedDeviceProgram&, QString*) = 0;
    virtual bool block(const cam::CamMotionBlock&, const FrozenToolExecutionRecipe&, QString*) = 0;
    virtual bool knot(const cam::CamMotionBlock&, int index, QString*) = 0;
    virtual bool fence(const cam::CamMotionBlock&, const cam::MotionProcessFence&, QString*) = 0;
    virtual bool seal(QString*) = 0;
    virtual void discard() noexcept = 0;
};

bool consumeExactPlan(const PreparedDeviceProgram& program, ExactPlanConsumer& consumer,
                      const std::function<bool()>& cancelled, QString* error);
bool consumeExactSection(const PreparedDeviceProgram& program, int ordinal,
                         ExactPlanConsumer& consumer,
                         const std::function<bool()>& cancelled, QString* error);

} // namespace lcnc::process
