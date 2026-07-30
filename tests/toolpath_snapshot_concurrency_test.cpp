#include "modules/cam/i_cam_toolpath_provider.h"
#include "modules/process/toolpath/process_toolpath_service.h"

#include <QCoreApplication>
#include <QTextStream>

#include <atomic>
#include <thread>
#include <vector>

namespace {

class SnapshotProvider final : public lcnc::cam::ICamToolpathProvider
{
public:
    bool hasToolpath() const override { return true; }
    std::uint64_t toolpathRevision() const override { return m_revision.load(); }
    bool solveToolpathForOrder(const QVector<std::uint64_t>&) override { return true; }

    lcnc::cam::ToolpathExportSnapshot exportToolpathSnapshot() const override
    {
        const std::uint64_t revision = m_revision.fetch_add(1) + 1;
        lcnc::cam::ToolpathExportSnapshot snapshot;
        snapshot.revision = revision;
        lcnc::cam::ToolpathExportContour contour;
        contour.contourId = revision;
        contour.enabled = true;
        contour.layerEnabled = true;
        contour.pointCount = 1;
        snapshot.contours.append(contour);
        lcnc::cam::ToolpathExportPoint point;
        point.x = static_cast<double>(revision);
        point.machineCoordValid = true;
        snapshot.pointsByContourId.insert(revision, {point});
        return snapshot;
    }

    lcnc::cam::ToolpathExportSnapshot exportToolpathSnapshotForOrder(
        const QVector<std::uint64_t>&) const override
    {
        return exportToolpathSnapshot();
    }

private:
    mutable std::atomic_uint64_t m_revision{0};
};

int fail(const QString& message)
{
    QTextStream(stderr) << message << '\n';
    return 1;
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    auto provider = std::make_shared<SnapshotProvider>();
    lcnc::process::ProcessToolpathService service(provider);
    service.refreshSnapshot();

    std::atomic_bool running{true};
    std::atomic_bool mixedRevision{false};
    std::thread writer([&] {
        for (int iteration = 0; iteration < 200; ++iteration)
            service.refreshSnapshot();
        running.store(false);
    });

    std::vector<std::thread> readers;
    for (int reader = 0; reader < 4; ++reader) {
        readers.emplace_back([&] {
            while (running.load()) {
                const auto snapshot = service.currentSnapshot();
                if (snapshot.contours.size() != 1)
                    continue;
                const auto& contour = snapshot.contours.first();
                const auto points = snapshot.pointsByContourId.value(contour.contourId);
                if (contour.contourId != snapshot.revision
                    || points.size() != 1
                    || points.first().x != static_cast<double>(snapshot.revision)) {
                    mixedRevision.store(true);
                    return;
                }
                const auto plan = service.buildJobPlan();
                if (plan.valid && (plan.contours.size() != 1
                    || plan.contours.first().contour.contourId != plan.revision
                    || plan.contours.first().points.first().x
                        != static_cast<double>(plan.revision))) {
                    mixedRevision.store(true);
                    return;
                }
            }
        });
    }

    writer.join();
    for (auto& reader : readers)
        reader.join();
    if (mixedRevision.load())
        return fail(QStringLiteral("Concurrent toolpath readers observed a mixed revision"));
    return 0;
}
