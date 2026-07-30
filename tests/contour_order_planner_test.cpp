#include "core/algorithms/cam/contour_order_planner.h"

#include <QCoreApplication>
#include <QTextStream>

#include <algorithm>

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
    using namespace lcnc::cam;

    if (!planContourOrder({}, {}).isEmpty())
        return fail(QStringLiteral("Empty contour input produced output"));

    const QVector<ContourEndpoints> axes = {
        {1, -3.0, -2.0, -1.0, -3.0, -2.0, -1.0},
        {2,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0},
        {3,  3.0,  2.0,  1.0,  3.0,  2.0,  1.0},
    };
    const struct {
        PrimaryAxis axis;
        QVector<std::uint64_t> expected;
    } directionCases[] = {
        {PrimaryAxis::XPos, {1, 2, 3}},
        {PrimaryAxis::XNeg, {3, 2, 1}},
        {PrimaryAxis::YPos, {1, 2, 3}},
        {PrimaryAxis::YNeg, {3, 2, 1}},
        {PrimaryAxis::ZPos, {1, 2, 3}},
        {PrimaryAxis::ZNeg, {3, 2, 1}},
    };
    for (const auto& direction : directionCases) {
        if (planContourOrder(axes, {direction.axis, 0.0}) != direction.expected)
            return fail(QStringLiteral("Direction ordering failed"));
    }

    const QVector<ContourEndpoints> nearest = {
        {10, 0.0, 0.0, 0.0, 9.0, 0.0, 0.0},
        {20, 0.1, 20.0, 0.0, 0.1, 20.0, 0.0},
        {30, 0.2, 10.0, 0.0, 0.2, 10.0, 0.0},
    };
    if (planContourOrder(nearest, {PrimaryAxis::XPos, 0.5})
        != QVector<std::uint64_t>({10, 30, 20})) {
        return fail(QStringLiteral("Bucket nearest-neighbour ordering failed"));
    }

    QVector<ContourEndpoints> identical = {
        {3, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0},
        {1, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0},
        {2, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0},
    };
    const QVector<std::uint64_t> deterministic = {1, 2, 3};
    for (int iteration = 0; iteration < 20; ++iteration) {
        std::rotate(identical.begin(), identical.begin() + 1, identical.end());
        if (planContourOrder(identical, {PrimaryAxis::XPos, 0.5}) != deterministic)
            return fail(QStringLiteral("Equal-coordinate ordering was not deterministic"));
    }

    return 0;
}
