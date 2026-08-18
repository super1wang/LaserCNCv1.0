#include "core/algorithms/occt_exact_operation_lock.h"

#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>

#include <QCoreApplication>
#include <QByteArray>
#include <QTextStream>

#include <algorithm>
#include <atomic>
#include <thread>
#include <vector>

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

    const TopoDS_Shape first = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();
    const TopoDS_Shape second = BRepPrimAPI_MakeBox(
        gp_Pnt(9.5, 0.0, 0.0), 10.0, 10.0, 10.0).Shape();
    const int threadCount = argc > 1 ? std::max(1, QByteArray(argv[1]).toInt()) : 12;
    const int iterations = argc > 2 ? std::max(1, QByteArray(argv[2]).toInt()) : 24;
    std::atomic_int ready{0};
    std::atomic_bool start{false};
    std::atomic_bool failed{false};
    std::atomic_int holders{0};
    std::atomic_int maximumHolders{0};
    std::vector<std::thread> threads;
    threads.reserve(threadCount);

    for (int threadIndex = 0; threadIndex < threadCount; ++threadIndex) {
        threads.emplace_back([&] {
            ready.fetch_add(1);
            while (!start.load(std::memory_order_acquire))
                std::this_thread::yield();
            try {
                for (int iteration = 0; iteration < iterations; ++iteration) {
                    lcnc::OcctExactOperationLock lock;
                    const int active = holders.fetch_add(1) + 1;
                    int observed = maximumHolders.load();
                    while (active > observed
                           && !maximumHolders.compare_exchange_weak(observed, active)) {
                    }
                    BRepExtrema_DistShapeShape distance(first, second);
                    distance.SetMultiThread(Standard_False);
                    distance.Perform();
                    if (!distance.IsDone())
                        failed.store(true);
                    holders.fetch_sub(1);
                }
            } catch (...) {
                failed.store(true);
            }
        });
    }
    while (ready.load() != threadCount)
        std::this_thread::yield();
    start.store(true, std::memory_order_release);
    for (std::thread& thread : threads)
        thread.join();

    if (failed.load())
        return fail(QStringLiteral("Guarded OCCT exact-distance stress failed"));
    if (maximumHolders.load() != 1 || holders.load() != 0)
        return fail(QStringLiteral("OCCT exact-operation gate allowed concurrent holders"));
    return 0;
}
