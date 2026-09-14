#pragma once

#include "core/command/commands_api.h"

// =============================================================================
// CAD Phase-2 commands
// =============================================================================

// ─────────────────────────────────────────────────────────────────────────────
// Sketch session commands
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Start a sketch session on the default XY plane.
 */
class CmdNewSketch : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "cad.newSketch";
    explicit CmdNewSketch(IAppContext* ctx);
    void execute() override;
};

/**
 * @brief Finish the active sketch session and prepare it for feature creation.
 */
class CmdFinishSketch : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "cad.finishSketch";
    explicit CmdFinishSketch(IAppContext* ctx);
    void execute() override;
};

/**
 * @brief Cancel the active sketch session without committing any geometry.
 */
class CmdCancelSketch : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "cad.cancelSketch";
    explicit CmdCancelSketch(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

// ─────────────────────────────────────────────────────────────────────────────
// Sketch tool palette commands (only enabled while a sketch is being edited)
// ─────────────────────────────────────────────────────────────────────────────

class CmdSketchPoint : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "cad.sketch.point";
    explicit CmdSketchPoint(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

class CmdSketchLine : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "cad.sketch.line";
    explicit CmdSketchLine(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

class CmdSketchArc : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "cad.sketch.arc";
    explicit CmdSketchArc(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

class CmdSketchCircleTool : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "cad.sketch.circle";
    explicit CmdSketchCircleTool(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

class CmdSketchRectangleTool : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "cad.sketch.rectangle";
    explicit CmdSketchRectangleTool(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

class CmdSketchPolygon : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "cad.sketch.polygon";
    explicit CmdSketchPolygon(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

// ─────────────────────────────────────────────────────────────────────────────
// View modeling aids
// ─────────────────────────────────────────────────────────────────────────────

class CmdToggleCadGrid : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "cad.toggleGrid";
    explicit CmdToggleCadGrid(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

class CmdToggleGridSnap : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "cad.toggleGridSnap";
    explicit CmdToggleGridSnap(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

class CmdSnapNone : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "cad.snapNone";
    explicit CmdSnapNone(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

class CmdSnapVertex : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "cad.snapVertex";
    explicit CmdSnapVertex(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

class CmdSnapEdge : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "cad.snapEdge";
    explicit CmdSnapEdge(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

class CmdSnapFace : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "cad.snapFace";
    explicit CmdSnapFace(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

// ─────────────────────────────────────────────────────────────────────────────
// Parametric primitive creation
// ─────────────────────────────────────────────────────────────────────────────

class CmdCreateBox : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "cad.createBox";
    explicit CmdCreateBox(IAppContext* ctx);
    void execute() override;
};

class CmdCreateCylinder : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "cad.createCylinder";
    explicit CmdCreateCylinder(IAppContext* ctx);
    void execute() override;
};

class CmdCreateSphere : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "cad.createSphere";
    explicit CmdCreateSphere(IAppContext* ctx);
    void execute() override;
};

class CmdCreateCone : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "cad.createCone";
    explicit CmdCreateCone(IAppContext* ctx);
    void execute() override;
};

class CmdCreateTorus : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "cad.createTorus";
    explicit CmdCreateTorus(IAppContext* ctx);
    void execute() override;
};

// ─────────────────────────────────────────────────────────────────────────────
// Transform operations (move / rotate)
// ─────────────────────────────────────────────────────────────────────────────

class CmdMoveShape : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "cad.move";
    explicit CmdMoveShape(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

class CmdRotateShape : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "cad.rotate";
    explicit CmdRotateShape(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

// ─────────────────────────────────────────────────────────────────────────────
// Boolean operations
// ─────────────────────────────────────────────────────────────────────────────

class CmdBoolUnion : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "cad.boolUnion";
    explicit CmdBoolUnion(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

class CmdBoolCut : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "cad.boolCut";
    explicit CmdBoolCut(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

class CmdBoolCommon : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "cad.boolCommon";
    explicit CmdBoolCommon(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

// ─────────────────────────────────────────────────────────────────────────────
// Measurement tools
// ─────────────────────────────────────────────────────────────────────────────

class CmdMeasureDistance : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "cad.measureDist";
    explicit CmdMeasureDistance(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

class CmdMeasureAngle : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "cad.measureAngle";
    explicit CmdMeasureAngle(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

class CmdMeasureArea : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "cad.measureArea";
    explicit CmdMeasureArea(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

// ─────────────────────────────────────────────────────────────────────────────
// Delete entity
// ─────────────────────────────────────────────────────────────────────────────

class CmdDeleteShape : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "cad.deleteShape";
    explicit CmdDeleteShape(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};

// ─────────────────────────────────────────────────────────────────────────────
// Explode / decompose entity into direct sub-shapes (one level down)
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Split the selected compound entity into its direct child sub-shapes.
 *
 * Replaces the selected entity with N child shapes extracted via TopoDS_Iterator
 * (one level of decomposition, no deep recursion). Works on both Machine and
 * Workpiece entities, making it easy to separate machine axis assemblies for
 * individual axis calibration.
 */
class CmdExplodeShape : public CommandBase {
    Q_OBJECT
public:
    inline static const QString Name = "cad.explode";
    explicit CmdExplodeShape(IAppContext* ctx);
    void execute() override;
    bool isEnabled() const override;
};
