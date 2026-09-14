#include "core/algorithms/kinematics/axis_line_calibration_solver.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace lcnc::kinematics {
namespace {

using Vec3 = std::array<double, 3>;
using Mat3 = std::array<std::array<double, 3>, 3>;

constexpr double kEpsilon = 1e-12;
constexpr double kPi = 3.14159265358979323846;

Vec3 add(const Vec3& a, const Vec3& b)
{
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

Vec3 subtract(const Vec3& a, const Vec3& b)
{
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

Vec3 multiply(const Vec3& value, double scale)
{
    return {value[0] * scale, value[1] * scale, value[2] * scale};
}

double dot(const Vec3& a, const Vec3& b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

Vec3 cross(const Vec3& a, const Vec3& b)
{
    return {a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]};
}

double norm(const Vec3& value)
{
    return std::sqrt(dot(value, value));
}

bool normalize(Vec3* value)
{
    const double length = value ? norm(*value) : 0.0;
    if (!value || !std::isfinite(length) || length <= kEpsilon)
        return false;
    *value = multiply(*value, 1.0 / length);
    return true;
}

bool finite(const Vec3& value)
{
    return std::isfinite(value[0]) && std::isfinite(value[1])
        && std::isfinite(value[2]);
}

bool solve3(Mat3 matrix, Vec3 rhs, Vec3* solution)
{
    if (!solution)
        return false;
    for (int pivot = 0; pivot < 3; ++pivot) {
        int best = pivot;
        for (int row = pivot + 1; row < 3; ++row) {
            if (std::abs(matrix[row][pivot]) > std::abs(matrix[best][pivot]))
                best = row;
        }
        if (std::abs(matrix[best][pivot]) <= kEpsilon)
            return false;
        if (best != pivot) {
            std::swap(matrix[best], matrix[pivot]);
            std::swap(rhs[best], rhs[pivot]);
        }
        for (int row = pivot + 1; row < 3; ++row) {
            const double factor = matrix[row][pivot] / matrix[pivot][pivot];
            for (int column = pivot; column < 3; ++column)
                matrix[row][column] -= factor * matrix[pivot][column];
            rhs[row] -= factor * rhs[pivot];
        }
    }
    for (int row = 2; row >= 0; --row) {
        double value = rhs[row];
        for (int column = row + 1; column < 3; ++column)
            value -= matrix[row][column] * (*solution)[column];
        (*solution)[row] = value / matrix[row][row];
    }
    return finite(*solution);
}

void symmetricEigen(Mat3 matrix, Vec3* values, Mat3* vectors)
{
    Mat3 eigenvectors{{{1.0, 0.0, 0.0},
                       {0.0, 1.0, 0.0},
                       {0.0, 0.0, 1.0}}};
    for (int iteration = 0; iteration < 32; ++iteration) {
        int p = 0;
        int q = 1;
        double largest = std::abs(matrix[p][q]);
        for (int row = 0; row < 3; ++row) {
            for (int column = row + 1; column < 3; ++column) {
                if (std::abs(matrix[row][column]) > largest) {
                    largest = std::abs(matrix[row][column]);
                    p = row;
                    q = column;
                }
            }
        }
        if (largest <= kEpsilon)
            break;
        const double angle = 0.5 * std::atan2(
            2.0 * matrix[p][q], matrix[q][q] - matrix[p][p]);
        const double cosine = std::cos(angle);
        const double sine = std::sin(angle);
        for (int index = 0; index < 3; ++index) {
            const double aip = matrix[index][p];
            const double aiq = matrix[index][q];
            matrix[index][p] = cosine * aip - sine * aiq;
            matrix[index][q] = sine * aip + cosine * aiq;
        }
        for (int index = 0; index < 3; ++index) {
            const double api = matrix[p][index];
            const double aqi = matrix[q][index];
            matrix[p][index] = cosine * api - sine * aqi;
            matrix[q][index] = sine * api + cosine * aqi;
        }
        matrix[p][q] = matrix[q][p] = 0.0;
        for (int row = 0; row < 3; ++row) {
            const double vip = eigenvectors[row][p];
            const double viq = eigenvectors[row][q];
            eigenvectors[row][p] = cosine * vip - sine * viq;
            eigenvectors[row][q] = sine * vip + cosine * viq;
        }
    }
    *values = {matrix[0][0], matrix[1][1], matrix[2][2]};
    *vectors = eigenvectors;
}

double circularCoverage(QVector<double> angles)
{
    if (angles.size() < 2)
        return 0.0;
    for (double& angle : angles) {
        angle = std::fmod(angle, 2.0 * kPi);
        if (angle < 0.0)
            angle += 2.0 * kPi;
    }
    std::sort(angles.begin(), angles.end());
    double largestGap = 0.0;
    for (int index = 1; index < angles.size(); ++index)
        largestGap = std::max(largestGap, angles[index] - angles[index - 1]);
    largestGap = std::max(largestGap,
                          angles.front() + 2.0 * kPi - angles.back());
    return (2.0 * kPi - largestGap) * 180.0 / kPi;
}

} // namespace

AxisLineCalibrationResult fitRotaryAxisLine(
    const QVector<CalibrationSample>& sourceSamples,
    const QString& axisName,
    const AxisLineCalibrationOptions& options)
{
    AxisLineCalibrationResult result;
    QVector<const CalibrationSample*> samples;
    for (const CalibrationSample& sample : sourceSamples) {
        if (sample.accepted
            && sample.targetAxisName.compare(axisName, Qt::CaseInsensitive) == 0
            && finite(sample.measuredReferencePointMcs)) {
            samples.append(&sample);
        }
    }
    if (samples.size() < options.minimumSamples) {
        result.error = QStringLiteral("Axis %1 requires at least %2 accepted samples")
                           .arg(axisName).arg(options.minimumSamples);
        return result;
    }

    const int targetSlot = samples.front()->targetAxisSlot;
    if (targetSlot < 3 || targetSlot > 4) {
        result.error = QStringLiteral("Axis %1 has an invalid rotary feedback slot")
                           .arg(axisName);
        return result;
    }
    QVector<double> actualAngles;
    actualAngles.reserve(samples.size());
    const int fixedRotarySlot = targetSlot == 3 ? 4 : 3;
    double fixedMinimum = std::numeric_limits<double>::infinity();
    double fixedMaximum = -std::numeric_limits<double>::infinity();
    for (const CalibrationSample* sample : samples) {
        if (sample->targetAxisSlot != targetSlot
            || !std::all_of(sample->actualAxes.cbegin(), sample->actualAxes.cend(),
                            [](double value) { return std::isfinite(value); })) {
            result.error = QStringLiteral("Axis %1 samples have inconsistent actual-axis feedback")
                               .arg(axisName);
            return result;
        }
        actualAngles.append(sample->actualAxes[targetSlot] * kPi / 180.0);
        fixedMinimum = std::min(fixedMinimum, sample->actualAxes[fixedRotarySlot]);
        fixedMaximum = std::max(fixedMaximum, sample->actualAxes[fixedRotarySlot]);
    }
    const double actualCoverage = circularCoverage(actualAngles);
    if (actualCoverage < options.minimumAngularCoverageDeg) {
        result.error = QStringLiteral("Axis %1 actual feedback coverage %2 deg is below %3 deg")
                           .arg(axisName).arg(actualCoverage, 0, 'f', 3)
                           .arg(options.minimumAngularCoverageDeg, 0, 'f', 3);
        return result;
    }
    if (fixedMaximum - fixedMinimum > options.maximumFixedRotaryDriftDeg) {
        result.error = QStringLiteral("Axis %1 calibration moved the fixed rotary axis by %2 deg")
                           .arg(axisName).arg(fixedMaximum - fixedMinimum, 0, 'f', 6);
        return result;
    }

    Vec3 mean{};
    for (const CalibrationSample* sample : samples)
        mean = add(mean, sample->measuredReferencePointMcs);
    mean = multiply(mean, 1.0 / samples.size());

    Mat3 covariance{};
    for (const CalibrationSample* sample : samples) {
        const Vec3 centered = subtract(sample->measuredReferencePointMcs, mean);
        for (int row = 0; row < 3; ++row)
            for (int column = 0; column < 3; ++column)
                covariance[row][column] += centered[row] * centered[column];
    }
    Vec3 eigenvalues{};
    Mat3 eigenvectors{};
    symmetricEigen(covariance, &eigenvalues, &eigenvectors);
    int minimumIndex = 0;
    int maximumIndex = 0;
    for (int index = 1; index < 3; ++index) {
        if (eigenvalues[index] < eigenvalues[minimumIndex]) minimumIndex = index;
        if (eigenvalues[index] > eigenvalues[maximumIndex]) maximumIndex = index;
    }
    Vec3 normal{eigenvectors[0][minimumIndex], eigenvectors[1][minimumIndex],
                eigenvectors[2][minimumIndex]};
    if (!normalize(&normal)) {
        result.error = QStringLiteral("Axis %1 samples do not define a stable plane").arg(axisName);
        return result;
    }
    const Vec3 reference = std::abs(normal[2]) < 0.9
        ? Vec3{0.0, 0.0, 1.0} : Vec3{1.0, 0.0, 0.0};
    Vec3 basisU = cross(reference, normal);
    if (!normalize(&basisU)) {
        result.error = QStringLiteral("Axis %1 plane basis is degenerate").arg(axisName);
        return result;
    }
    Vec3 basisV = cross(normal, basisU);
    normalize(&basisV);

    Mat3 normalMatrix{};
    Vec3 rhs{};
    QVector<std::array<double, 2>> projected;
    projected.reserve(samples.size());
    for (const CalibrationSample* sample : samples) {
        const Vec3 centered = subtract(sample->measuredReferencePointMcs, mean);
        const double x = dot(centered, basisU);
        const double y = dot(centered, basisV);
        projected.append({x, y});
        const Vec3 row{2.0 * x, 2.0 * y, 1.0};
        const double value = x * x + y * y;
        for (int i = 0; i < 3; ++i) {
            rhs[i] += row[i] * value;
            for (int j = 0; j < 3; ++j)
                normalMatrix[i][j] += row[i] * row[j];
        }
    }
    Vec3 circle{};
    if (!solve3(normalMatrix, rhs, &circle)) {
        result.error = QStringLiteral("Axis %1 samples do not define a stable circle").arg(axisName);
        return result;
    }
    const double radiusSquared = circle[2] + circle[0] * circle[0]
        + circle[1] * circle[1];
    if (!(radiusSquared > 0.0)) {
        result.error = QStringLiteral("Axis %1 fitted radius is invalid").arg(axisName);
        return result;
    }
    const double radius = std::sqrt(radiusSquared);
    if (radius < options.minimumRadiusMm) {
        result.error = QStringLiteral("Axis %1 fitted radius %2 mm is below %3 mm")
                           .arg(axisName).arg(radius, 0, 'f', 6)
                           .arg(options.minimumRadiusMm, 0, 'f', 6);
        return result;
    }
    const Vec3 center = add(mean, add(multiply(basisU, circle[0]),
                                      multiply(basisV, circle[1])));

    double residualSquared = 0.0;
    double maximumResidual = 0.0;
    QVector<double> geometricAngles;
    geometricAngles.reserve(samples.size());
    QVector<double> residuals;
    residuals.reserve(samples.size());
    for (int index = 0; index < samples.size(); ++index) {
        const double dx = projected[index][0] - circle[0];
        const double dy = projected[index][1] - circle[1];
        // Distance to the fitted 3D circle includes axial/plane error, not
        // only radial error after projection onto that plane.
        const double planeError = dot(
            subtract(samples[index]->measuredReferencePointMcs, center), normal);
        const double residual = std::hypot(std::hypot(dx, dy) - radius, planeError);
        residuals.append(residual);
        residualSquared += residual * residual;
        maximumResidual = std::max(maximumResidual, residual);
        geometricAngles.append(std::atan2(dy, dx));
    }
    const double rms = std::sqrt(residualSquared / samples.size());
    const double coverage = circularCoverage(geometricAngles);
    if (coverage < options.minimumAngularCoverageDeg) {
        result.error = QStringLiteral("Axis %1 angular coverage %2 deg is below %3 deg")
                           .arg(axisName).arg(coverage, 0, 'f', 3)
                           .arg(options.minimumAngularCoverageDeg, 0, 'f', 3);
        return result;
    }
    if (rms > options.maximumRmsResidualMm) {
        result.error = QStringLiteral("Axis %1 RMS residual %2 mm exceeds %3 mm")
                           .arg(axisName).arg(rms, 0, 'f', 6)
                           .arg(options.maximumRmsResidualMm, 0, 'f', 6);
        return result;
    }
    if (maximumResidual > options.maximumResidualMm) {
        result.error = QStringLiteral("Axis %1 maximum residual %2 mm exceeds %3 mm")
                           .arg(axisName).arg(maximumResidual, 0, 'f', 6)
                           .arg(options.maximumResidualMm, 0, 'f', 6);
        return result;
    }

    // A fitted axis line is unsigned. Use increasing real rotary feedback to
    // select the vector sign that predicts the measured point travel.
    double directionScore = 0.0;
    QVector<int> order(samples.size());
    for (int index = 0; index < order.size(); ++index) order[index] = index;
    const int slot = targetSlot;
    if (slot >= 0 && slot < 5) {
        std::sort(order.begin(), order.end(), [&samples, slot](int lhs, int rhs) {
            return samples[lhs]->actualAxes[slot] < samples[rhs]->actualAxes[slot];
        });
        for (int orderIndex = 1; orderIndex < order.size(); ++orderIndex) {
            const CalibrationSample* first = samples[order[orderIndex - 1]];
            const CalibrationSample* second = samples[order[orderIndex]];
            const double angleDelta = second->actualAxes[slot] - first->actualAxes[slot];
            if (std::abs(angleDelta) <= 1e-9)
                continue;
            const Vec3 radial = subtract(first->measuredReferencePointMcs, center);
            const Vec3 expected = cross(normal, radial);
            const Vec3 measured = subtract(second->measuredReferencePointMcs,
                                           first->measuredReferencePointMcs);
            directionScore += dot(expected, measured) * angleDelta;
        }
        if (std::abs(directionScore) <= kEpsilon) {
            result.error = QStringLiteral(
                "Axis %1 direction cannot be determined from actual feedback").arg(axisName);
            return result;
        }
        if (directionScore < 0.0)
            normal = multiply(normal, -1.0);
    }

    AxisLineFit fit;
    fit.axisName = axisName;
    fit.pointMcs = center;
    fit.unitVectorMcs = normal;
    fit.sampleCount = samples.size();
    fit.angularCoverageDeg = coverage;
    fit.fittedRadiusMm = radius;
    fit.rmsResidualMm = rms;
    fit.maxResidualMm = maximumResidual;
    const double largestEigenvalue = std::max(std::abs(eigenvalues[maximumIndex]), kEpsilon);
    fit.conditionMetric = std::abs(eigenvalues[minimumIndex]) / largestEigenvalue;
    const double suggestionThreshold = std::max(
        options.maximumRmsResidualMm,
        options.outlierSuggestionFactor * std::max(rms, kEpsilon));
    for (int index = 0; index < residuals.size(); ++index) {
        if (residuals[index] > suggestionThreshold)
            fit.suggestedRejectedSampleIds.append(samples[index]->sampleId);
    }
    result.success = true;
    result.fit = fit;
    return result;
}

} // namespace lcnc::kinematics
