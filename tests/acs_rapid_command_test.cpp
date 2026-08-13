#include "core/kinematics/machine_configuration_service.h"
#include "modules/process/device/motion_control/acs_motion_control.h"
#include "modules/process/runtime/acs_text_command_sink.h"
#include "modules/process/runtime/axis_map.h"
#include "modules/process/runtime/machine_pose5.h"
#include "modules/process/runtime/process_runtime_configuration.h"
#include "modules/process/settings/process_settings_service.h"
#include "modules/process/tool/tool.h"

#include <QCoreApplication>
#include <QTemporaryDir>
#include <QTextStream>

#include <string>

namespace {

class InspectableAcsMotionControl final : public ACSMotionControl
{
public:
    using ACSMotionControl::ACSMotionControl;

    const std::string& pendingProgram() const { return m_strCommand; }
};

int fail(const QString& message)
{
    QTextStream(stderr) << message << '\n';
    return 1;
}

lcnc::MachineAxisRuntimeConfig axisConfig(const QString& name,
                                          lcnc::MachineAxisRole role,
                                          int controllerIndex)
{
    lcnc::MachineAxisRuntimeConfig config;
    config.axis.name = name;
    config.axis.role = role;
    config.axis.motionType = role == lcnc::MachineAxisRole::LinearX
            || role == lcnc::MachineAxisRole::LinearY
            || role == lcnc::MachineAxisRole::LinearZ
        ? MachineAxisDef::Linear : MachineAxisDef::Rotary;
    config.controllerIndex = controllerIndex;
    config.highSpeed = 100.0;
    config.acceleration = 100.0;
    config.jerk = 1000.0;
    return config;
}

int occurrenceCount(const std::string& text, const std::string& token)
{
    int count = 0;
    std::string::size_type position = 0;
    while ((position = text.find(token, position)) != std::string::npos) {
        ++count;
        position += token.size();
    }
    return count;
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    QTemporaryDir settingsDirectory;
    if (!settingsDirectory.isValid())
        return fail(QStringLiteral("Could not create temporary settings directory"));

    lcnc::process::ProcessSettingsService settings(settingsDirectory.path());
    if (!settings.initialize())
        return fail(QStringLiteral("Could not initialize Process settings"));
    lcnc::process::ProcessRuntimeConfiguration runtimeConfiguration;
    InspectableAcsMotionControl controller(settings, runtimeConfiguration);

    lcnc::MachineConfigurationService machineConfiguration;
    machineConfiguration.setAxisConfigurations({
        axisConfig(QStringLiteral("X"), lcnc::MachineAxisRole::LinearX, 0),
        axisConfig(QStringLiteral("Y"), lcnc::MachineAxisRole::LinearY, 1),
        axisConfig(QStringLiteral("Z"), lcnc::MachineAxisRole::LinearZ, 2),
        axisConfig(QStringLiteral("A"), lcnc::MachineAxisRole::TableTilt, 3),
        axisConfig(QStringLiteral("C"), lcnc::MachineAxisRole::TableSpin, 4),
    });
    lcnc::MachineAxisLayout layout;
    layout.append(QStringLiteral("X"), lcnc::MachineAxisRole::LinearX);
    layout.append(QStringLiteral("Y"), lcnc::MachineAxisRole::LinearY);
    layout.append(QStringLiteral("Z"), lcnc::MachineAxisRole::LinearZ);
    layout.append(QStringLiteral("A"), lcnc::MachineAxisRole::TableTilt);
    layout.append(QStringLiteral("C"), lcnc::MachineAxisRole::TableSpin);
    const lcnc::process::AxisMap axisMap =
        lcnc::process::AxisMap::from(&machineConfiguration, layout);
    if (!axisMap.isFiveAxis())
        return fail(QStringLiteral("Could not build the five-axis command map"));

    lcnc::process::AcsTextCommandSink sink(&controller, axisMap);
    Tool rapidTool;
    rapidTool.m_dLineVelocity = 100.0;
    rapidTool.m_dLineAcc = 100.0;
    rapidTool.m_dLineJerk = 1000.0;
    rapidTool.m_dCuttingHeight = 0.0;
    rapidTool.m_dCuttingHeightCompensate = 0.0;

    lcnc::process::MachinePose5 first;
    first.mask = lcnc::process::MachinePose5::Bx
        | lcnc::process::MachinePose5::By
        | lcnc::process::MachinePose5::Bz
        | lcnc::process::MachinePose5::Br1
        | lcnc::process::MachinePose5::Br2;
    first.x = 10.0;
    first.y = 20.0;
    first.z = 30.0;
    first.r1 = -15.0;
    first.r2 = 25.0;
    first.r1Name = QStringLiteral("A");
    first.r2Name = QStringLiteral("C");
    lcnc::process::MachinePose5 second = first;
    second.x = 12.0;
    second.z = 28.0;
    second.r1 = -20.0;

    QString error;
    sink.resetProgram();
    if (!sink.beginSegment(first, rapidTool, &error)
        || !sink.lineTo(first, rapidTool, &error)
        || !sink.lineTo(second, rapidTool, &error)) {
        return fail(QStringLiteral("Could not build coordinated rapid program: %1").arg(error));
    }
    sink.endSegment(rapidTool);

    const std::string& program = controller.pendingProgram();
    if (program.find("XSEG/VFJA (0, 1, 2, 3, 4)") == std::string::npos)
        return fail(QStringLiteral("Rapid program does not contain a five-axis XSEG"));
    if (occurrenceCount(program, "LINE/V (0, 1, 2, 3, 4)") != 2)
        return fail(QStringLiteral("Rapid samples were not emitted as coordinate LINE commands"));
    if (program.find("PTP/EV") != std::string::npos)
        return fail(QStringLiteral("Rapid program unexpectedly contains PTP/EV"));
    return 0;
}
