#include "core/algorithms/kinematics/rtcp_reference_transform.h"
#include "core/algorithms/cam/solved_rapid_geometry.h"
#include "core/kinematics/machine_kinematics.h"
#include "modules/cam/toolpath/travel_solution_cache.h"
#include "modules/process/runtime/rtcp_target_validation.h"

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Vec.hxx>

#include <array>
#include <cmath>
#include <iostream>
#include <limits>

namespace {
constexpr double kRadians = 3.14159265358979323846 / 180.0;

// Independent Rodrigues evaluation, not the OCC transform used by production.
gp_Pnt rotate(const gp_Pnt& p, const gp_Pnt& center, const gp_Dir& axis, double degrees)
{
    const std::array<double, 3> v{p.X() - center.X(), p.Y() - center.Y(), p.Z() - center.Z()};
    const std::array<double, 3> u{axis.X(), axis.Y(), axis.Z()};
    const double c = std::cos(degrees * kRadians), s = std::sin(degrees * kRadians);
    const double dot = u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
    return gp_Pnt(center.X() + c * v[0] + s * (u[1] * v[2] - u[2] * v[1]) + (1 - c) * dot * u[0],
                  center.Y() + c * v[1] + s * (u[2] * v[0] - u[0] * v[2]) + (1 - c) * dot * u[1],
                  center.Z() + c * v[2] + s * (u[0] * v[1] - u[1] * v[0]) + (1 - c) * dot * u[2]);
}

gp_Trsf rotation(const gp_Pnt& center, const gp_Dir& axis, double degrees)
{
    gp_Trsf result;
    result.SetRotation(gp_Ax1(center, axis), degrees * kRadians);
    return result;
}

int fail(const char* message)
{
    std::cerr << message << '\n';
    return 1;
}
} // namespace

int main()
{
    using lcnc::kinematics::tableRtcpReferencePoint;
    const gp_Pnt center(290, -40, -30);
    const gp_Dir b(0, 1, 0), c(0, 0, -1);
    const gp_Trsf table = rotation(center, b, 90) * rotation(center, c, 711.428);
    const gp_Trsf home;

    // 2026-09-11 second contour: the planner's baseline-world sample was
    // paired with axes solved at B=8.213..., C=360. Reproduce both outputs.
    const gp_Pnt rapidLocal(288.7857142857144, -40, -5.051283406946306);
    const double rapidB = 8.213210701737614;
    const gp_Trsf rapidTable = rotation(center, b, rapidB) * rotation(center, c, 360);
    gp_Pnt staleReference;
    if (!tableRtcpReferencePoint(rapidLocal, rapidTable, home, &staleReference)
        || staleReference.Distance(gp_Pnt(285.2340664569653, -40, -5.480643831866075)) > 1e-7)
        return fail("failed to reproduce second-contour stale-world RTCP input");
    lcnc::cam::RapidPose rapid;
    rapid.axes = {292.36227119783723, 40, 5.1337050563559, rapidB, 360};
    const auto originalAxes = rapid.axes;
    rapid.tcpMcsValid = true;
    const gp_Dir rapidNormal(0, 0, 1);
    lcnc::cam_algo::setSolvedRapidWorldGeometry(rapid, rapidLocal, rapidNormal, rapidTable);
    gp_Pnt rapidReference;
    if (!tableRtcpReferencePoint(gp_Pnt(rapid.tcpX, rapid.tcpY, rapid.tcpZ),
            rapidTable, home, &rapidReference)
        || rapidReference.Distance(rapidLocal) > 1e-7
        || gp_Pnt(rapid.tcpX, rapid.tcpY, rapid.tcpZ).Distance(
            gp_Pnt(292.36227119783723, -40, -5.1337050563559)) > 1e-7
        || rapid.axes != originalAxes || rapid.tcpMcsValid)
        return fail("second-contour solved world geometry did not match CAM axes");
    const gp_Pnt rapidController = rotate(rotate(rapidReference, center, c, 360), center, b, rapidB);
    if (rapidController.Distance(gp_Pnt(rapid.axes[0], -rapid.axes[1], -rapid.axes[2])) > 1e-7)
        return fail("second-contour RTCP did not recover CAM predicted axes");
    const gp_Dir expectedNormal = rapidNormal.Transformed(rapidTable);
    if (gp_Dir(rapid.surfaceNormalX, rapid.surfaceNormalY, rapid.surfaceNormalZ)
            .Angle(expectedNormal) > 1e-7)
        return fail("rapid normal was left in the planner posture");
    lcnc::cam_algo::setSolvedRapidWorldGeometry(rapid, rapidLocal, rapidNormal, rapidTable);
    if (gp_Pnt(rapid.tcpX, rapid.tcpY, rapid.tcpZ).Distance(rapidController) > 1e-7)
        return fail("rapid re-export applied carrier transform twice");

    // A cache hit must restore the continuous successor branch as well as
    // rapid geometry, without replacing current metadata or collision proof.
    lcnc::cam::ToolpathExportSnapshot cached;
    cached.revision = 17;
    cached.travelPlan.key.toolpathRevision = 17;
    cached.travelPlan.key.orderHash = 31;
    cached.machineConfigurationFingerprint = QStringLiteral("fixture-machine");
    for (std::uint64_t id : {6u, 7u}) {
        lcnc::cam::ToolpathExportContour contour;
        contour.contourId = id;
        contour.hasLeadIn = true;
        contour.leadInPoint.machineR2 = 360 * id;
        contour.leadInPoint.machineCoordValid = true;
        cached.contours.append(contour);
        cached.pointsByContourId.insert(id, {contour.leadInPoint});
    }
    auto cacheTarget = cached;
    cacheTarget.contours[1].enabled = true;
    cacheTarget.contours[1].contourName = QStringLiteral("current metadata");
    cacheTarget.contours[1].leadInPoint.machineR2 = 0;
    cacheTarget.pointsByContourId[7][0].machineR2 = 0;
    cacheTarget.travelPlan.collision.complete = true;
    if (!lcnc::cam::restoreTravelSolution(cached, cached.travelPlan.key, &cacheTarget)
        || cacheTarget.contours[1].leadInPoint.machineR2 != 2520
        || cacheTarget.pointsByContourId[7][0].machineR2 != 2520
        || !cacheTarget.contours[1].enabled
        || cacheTarget.contours[1].contourName != QStringLiteral("current metadata")
        || !cacheTarget.travelPlan.collision.complete)
        return fail("transition cache did not restore the complete continuous solution");
    cacheTarget.contours[1].rapidOffsetMm += 1;
    cacheTarget.contours[0].leadInPoint.machineR2 = 123;
    if (lcnc::cam::restoreTravelSolution(cached, cached.travelPlan.key, &cacheTarget)
        || cacheTarget.contours[0].leadInPoint.machineR2 != 123)
        return fail("incompatible cache partially overwrote current geometry");
    cacheTarget = cached;
    cacheTarget.pointsByContourId.remove(7);
    if (lcnc::cam::restoreTravelSolution(cached, cached.travelPlan.key, &cacheTarget))
        return fail("incomplete point set accepted by transition cache");

    // 2026-09-10 16:39:31 command_start: live axes and controller MCS agree
    // only after undoing the table carrier, not after sending world XYZ.
    gp_Pnt reference;
    if (!tableRtcpReferencePoint(gp_Pnt(250.0359, -39.9999, -4.1338), table, home, &reference)
        || reference.Distance(gp_Pnt(264.42275743785694, -36.14448896159015, -69.9641)) > 1e-7)
        return fail("logged controller command_start reference mismatch");

    const gp_Pnt world(251.9602805089926, -39.99990454545454, -4.133705056355652);
    const gp_Pnt oldResult = rotate(rotate(world, center, c, 711.428), center, b, 90);
    if (oldResult.Distance(gp_Pnt(315.8662949436444, -45.66980675364131, 7.614804935556953)) > 1e-7)
        return fail("failed to reproduce the logged double-transform fault");
    if (!tableRtcpReferencePoint(world, table, home, &reference)
        || reference.Distance(gp_Pnt(264.42266287728614, -36.14447930471677, -68.03971949100739)) > 1e-7)
        return fail("corrected first target reference mismatch");
    const gp_Pnt corrected = rotate(rotate(reference, center, c, 711.428), center, b, 90);
    if (corrected.Distance(world) > 1e-7 || oldResult.Distance(world) < 60)
        return fail("RTCP target did not recover the independently predicted physical TCP");

    // Exercise the same mounted-workpiece transforms used by CAM export.
    MachineAxisDef base, tiltAxis, spinAxis;
    base.name = QStringLiteral("BASE");
    tiltAxis.name = QStringLiteral("B");
    tiltAxis.parentAxis = base.name;
    tiltAxis.motionType = MachineAxisDef::Rotary;
    tiltAxis.origin = center;
    tiltAxis.direction = b;
    tiltAxis.currentPos = 90;
    spinAxis.name = QStringLiteral("C");
    spinAxis.parentAxis = tiltAxis.name;
    spinAxis.motionType = MachineAxisDef::Rotary;
    spinAxis.origin = center;
    spinAxis.direction = c;
    spinAxis.currentPos = 711.428;
    MachineKinematics mounted;
    mounted.setAxes({base, tiltAxis, spinAxis});
    mounted.mountWorkpiece(QStringLiteral("test-workpiece"), spinAxis.name);
    gp_Trsf installation;
    installation.SetTranslation(gp_Vec(15, 20, 25));
    mounted.setWorkpieceSetupTransform(installation);
    if (!tableRtcpReferencePoint(world, mounted.computeWpcTransform(QStringLiteral("test-workpiece")),
            mounted.computeWpcTransformHome(QStringLiteral("test-workpiece")), &reference)
        || reference.Distance(gp_Pnt(264.42266287728614, -36.14447930471677, -68.03971949100739)) > 1e-7)
        return fail("CAM carrier/home transforms did not preserve the RTCP reference");

    // Different centres, tilted vectors, both rotary signs, multi-turn angles,
    // translated/rotated setup and a workpiece-side linear carrier.
    gp_Trsf setup = rotation(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1), 37);
    setup.SetTranslationPart(gp_Vec(23, -17, 11));
    gp_Trsf linear;
    linear.SetTranslation(gp_Vec(15, -8, 3));
    const gp_Pnt primary(13, -25, 7), slave(20, -19, 12);
    for (const gp_Dir primaryDir : {gp_Dir(0, 1, 0), gp_Dir(0.2, 0.95, 0.24)}) {
        for (const gp_Dir slaveDir : {gp_Dir(0, 0, 1), gp_Dir(0, 0, -1)}) {
            for (double tilt : {-90.0, -37.0, 0.0, 45.0, 90.0}) {
                for (double spin : {-711.428, -90.0, 0.0, 61.0, 711.428}) {
                    const gp_Pnt expected = gp_Pnt(35, -12, 8).Transformed(setup);
                    gp_Pnt posed = rotate(rotate(expected, slave, slaveDir, spin), primary, primaryDir, tilt);
                    posed.Translate(gp_Vec(15, -8, 3));
                    const gp_Trsf current = linear * rotation(primary, primaryDir, tilt)
                        * rotation(slave, slaveDir, spin) * setup;
                    if (!tableRtcpReferencePoint(posed, current, setup, &reference)
                        || reference.Distance(expected) > 1e-7)
                        return fail("carrier/setup separation regression");
                }
            }
        }
    }

    const double nan = std::numeric_limits<double>::quiet_NaN();
    gp_Trsf scaled;
    scaled.SetScale(gp_Pnt(0, 0, 0), 2);
    if (tableRtcpReferencePoint(gp_Pnt(nan, 0, 0), home, home, &reference)
        || tableRtcpReferencePoint(world, scaled, home, &reference)
        || tableRtcpReferencePoint(world, home, home, nullptr))
        return fail("invalid geometry accepted as an RTCP reference");

    const std::array<double, 5> predicted{world.X(), -world.Y(), -world.Z(), 90, 711.428};
    std::array<double, 5> actual{corrected.X(), -corrected.Y(), -corrected.Z(), 90, 711.428};
    double maximum = 0;
    if (!lcnc::process::rtcpAxesAgree(predicted, actual, 0.05, &maximum))
        return fail("corrected axis agreement failed");
    actual[0] += 63.906014;
    if (lcnc::process::rtcpAxesAgree(predicted, actual, 0.05, &maximum))
        return fail("large axis error was bypassed");
    actual = predicted;
    actual[2] = nan;
    if (lcnc::process::rtcpAxesAgree(predicted, actual, 0.05, &maximum)
        || lcnc::process::rtcpAxesAgree(predicted, predicted, nan, &maximum)
        || lcnc::process::rtcpAxesAgree(predicted, predicted, 0, &maximum))
        return fail("non-finite axis output or invalid tolerance was accepted");
    std::cout << "RTCP reference regression passed (two logged fixtures + 100 carrier poses + continuous solution cache)\n";
    return 0;
}
