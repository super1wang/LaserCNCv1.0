#pragma once

#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax3.hxx>

namespace lcnc::cad_algo {

/**
 * @brief 2D sketch point in a local sketch plane coordinate system.
 */
struct SketchPoint2d {
    double x{0.0};
    double y{0.0};
};

/**
 * @brief Local sketch plane and helpers for common world-aligned planes.
 */
struct SketchPlane {
    gp_Ax3 axes;

    /// Return the default XY sketch plane.
    static SketchPlane xy();
    /// Return the default YZ sketch plane.
    static SketchPlane yz();
    /// Return the default ZX sketch plane.
    static SketchPlane zx();
};

/**
 * @brief Build a closed rectangular sketch wire on @p plane.
 */
TopoDS_Wire makeRectangleWire(const SketchPlane& plane,
                              double width,
                              double height,
                              SketchPoint2d center = {});

/**
 * @brief Build a circular sketch wire on @p plane.
 */
TopoDS_Wire makeCircleWire(const SketchPlane& plane,
                           double radius,
                           SketchPoint2d center = {});

/**
 * @brief Build a single open straight-segment wire between two 2D points.
 */
TopoDS_Wire makeLineWire(const SketchPlane& plane,
                         SketchPoint2d start,
                         SketchPoint2d end);

/**
 * @brief Build a circular arc wire defined by start, mid and end 2D points.
 */
TopoDS_Wire makeArcWire(const SketchPlane& plane,
                        SketchPoint2d start,
                        SketchPoint2d mid,
                        SketchPoint2d end);

/**
 * @brief Build a closed regular polygon wire inscribed in @p radius.
 */
TopoDS_Wire makePolygonWire(const SketchPlane& plane,
                            int sides,
                            double radius,
                            SketchPoint2d center = {});

/**
 * @brief Build a planar face from a closed sketch wire.
 */
TopoDS_Face makeFaceFromWire(const TopoDS_Wire& wire);

} // namespace lcnc::cad_algo
