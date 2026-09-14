#pragma once

#include "core/algorithms/cam/laser_toolpath.h"
#include "core/kinematics/machine_topology.h"

#include <QHash>

#include <memory>
#include <vector>

class MachineKinematics;

namespace lcnc {

struct ToolpathKinematicsRequest
{
    const MachineKinematics* machine{nullptr};
    MachiningMode mode{MachiningMode::Planar3Axis};
    MachineModeDefinition definition;
    WorkpieceSetupTransform workpieceSetup;
    HeadToolGeometry headToolGeometry;
    const std::vector<ToolpathPoint>* points{nullptr};
    const SolvedMachinePose* previousPose{nullptr};
};

class IToolpathKinematicsSolver
{
public:
    virtual ~IToolpathKinematicsSolver() = default;
    virtual QString id() const = 0;
    virtual int version() const = 0;
    virtual std::vector<SolvedMachinePose> solve(
        const ToolpathKinematicsRequest& request) const = 0;
};

class ToolpathSolverRegistry
{
public:
    ToolpathSolverRegistry();

    const IToolpathKinematicsSolver* solver(MachiningMode mode) const;
    std::vector<SolvedMachinePose> solve(const ToolpathKinematicsRequest& request) const;

private:
    QHash<MachiningMode, std::shared_ptr<IToolpathKinematicsSolver>> m_solvers;
};

} // namespace lcnc
