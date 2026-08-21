#pragma once

#include "core/algorithms/cam/laser_toolpath.h"
#include "core/kernel/i_service.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/project/cam/cam_data_contracts.h"
#include "modules/cam/contracts/i_cam_collision_configuration_provider.h"
#include "modules/cam/contracts/toolpath_export_dto.h"

#include <Graphic3d_Camera.hxx>
#include <QMap>
#include <QSet>
#include <QString>
#include <QVector>
#include <TopoDS_Shape.hxx>
#include <cstdint>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

namespace lcnc::cam {

struct OfflineSimulationBodySnapshot {
    QString entry;
    TopoDS_Shape shape;
    bool workpiece{false};
};

/** Immutable-at-capture CAM scene consumed only by the offline simulator.
 * Process must never include this OCC-bearing contract.
 */
struct OfflineSimulationSnapshot {
    std::uint64_t revision{0};
    QString error;
    ToolpathExportSnapshot execution;
    LaserToolpath toolpath;
    QList<MachineAxisDef> axes;
    QString kinematicsType;
    gp_Trsf workpieceSetup;
    QMap<QString, QString> shapeAssignments;
    QMap<QString, QString> workpieceMounts;
    QVector<OfflineSimulationBodySnapshot> bodies;
    CollisionConfigurationSnapshot collision;
    gp_Pnt cutterHeadModelPosition{0.0, 0.0, 0.0};
    TopoDS_Shape cutterProxy;
    bool showToolpath{true};
    bool showTravel{false};
    bool showNormals{false};
    bool showMachine{true};
    QSet<QString> visibleMachineEntries;
    bool showRotaryGuides{true};
    bool showCutterHeadGuide{true};
    double normalSampleStep{2.0};
    double collisionClearanceMm{0.0};
    Handle(Graphic3d_Camera) camera;

    bool valid() const noexcept {
        return error.isEmpty() && !axes.isEmpty() && execution.hasEnabledContours();
    }
};

class ICamOfflineSimulationProvider : public lcnc::IService {
  public:
    ~ICamOfflineSimulationProvider() override = default;
    virtual OfflineSimulationSnapshot captureOfflineSimulationSnapshot() const = 0;
    virtual std::uint64_t revision() const = 0;
};

} // namespace lcnc::cam
