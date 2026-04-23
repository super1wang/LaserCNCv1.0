#pragma once

#include "core/command/commands_api.h"

// =============================================================================
// CAD Phase-2 commands
// =============================================================================

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
