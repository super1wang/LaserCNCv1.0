#include "modules/process/runtime/process_runtime_configuration.h"

#include <QCoreApplication>
#include <QTextStream>

namespace {

int fail(const QString& message)
{
    QTextStream(stderr) << message << '\n';
    return 1;
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    lcnc::process::ProcessRuntimeConfiguration configuration;

    configuration.setEnabledAxes({QStringLiteral("x"), QStringLiteral("A"), QStringLiteral("X"), QStringLiteral(" base ")});
    if (!configuration.isAxisEnabled(Axis::X) || !configuration.isAxisEnabled(Axis::A)
        || configuration.isAxisEnabled(Axis::Y) || configuration.enabledAxes().size() != 2)
        return fail(QStringLiteral("Axis normalization or selection is invalid"));

    configuration.setExtensionAxes({QStringLiteral("x1"), QStringLiteral("U")});
    if (!configuration.isExtensionAxis(QStringLiteral("X1"))
        || !configuration.isExtensionAxis(QStringLiteral("u"))
        || configuration.isExtensionAxis(QStringLiteral("A1")))
        return fail(QStringLiteral("Extension-axis matching is invalid"));

    if (!configuration.simulationMode())
        return fail(QStringLiteral("Simulation default must be enabled"));
    configuration.setSimulationMode(false);
    configuration.setCustomerId("MaiTong");
    configuration.setPermission(PermissionLevel::Factory);
    if (configuration.simulationMode() || configuration.customerId() != "MaiTong"
        || configuration.permission() != PermissionLevel::Factory)
        return fail(QStringLiteral("Runtime configuration update is invalid"));

    return 0;
}
