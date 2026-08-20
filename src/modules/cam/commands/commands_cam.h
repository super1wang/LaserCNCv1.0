#pragma once

#include "core/command/commands_api.h"

class LaserToolpath;
class WidgetToolpathPanel;
class GraphicsScene;
class QTimer;

#include <AIS_Shape.hxx>
#include <QList>

/**
 * @brief Extract contours from the mounted workpiece and generate laser toolpath.
 *
 * Collects all workpiece entities from the project document, extracts
 * edge/wire contours, discretises them, and displays the toolpath as
 * coloured AIS overlays in the machine 3D view.
 */
class CmdGenerateToolpath : public CommandBase
{
    Q_OBJECT
public:
    explicit CmdGenerateToolpath(IAppContext* ctx);
    static constexpr const char* Name = "cam.generate_toolpath";

    bool isEnabled() const override;
    void execute()   override;
};

/**
 * @brief Interactive pick of the real cutting start on a contour.
 *
 * The closest sampled point becomes points.front(); closed contours are
 * rotated while open contours accept endpoints only.
 */
class CmdSetLeadIn : public CommandBase
{
    Q_OBJECT
public:
    explicit CmdSetLeadIn(IAppContext* ctx);
    static constexpr const char* Name = "cam.set_leadin";

    bool isEnabled() const override;
    void execute()   override;
};

/**
 * @brief Toggle visibility of all toolpath overlays (contours + lead-in lines).
 */
class CmdToolpathPreview : public CommandBase
{
    Q_OBJECT
public:
    explicit CmdToolpathPreview(IAppContext* ctx);
    static constexpr const char* Name = "cam.preview";

    bool isEnabled() const override;
    void execute()   override;
};

/**
 * @brief Apply pending parameters and rebuild only the active contour.
 */
class CmdRecalcToolpath : public CommandBase
{
    Q_OBJECT
public:
    explicit CmdRecalcToolpath(IAppContext* ctx);
    static constexpr const char* Name = "cam.recalc";

    bool isEnabled() const override;
    void execute()   override;
};

/**
 * @brief Pick a workpiece face to use as a machining face (Manual strategy).
 *
 * Enters the view's face-pick mode; the picked face is appended to the
 * manual machining-face set used by ExtractionStrategy::ManualFaceSelection.
 */
class CmdSelectMachiningFace : public CommandBase
{
    Q_OBJECT
public:
    explicit CmdSelectMachiningFace(IAppContext* ctx);
    static constexpr const char* Name = "cam.select_machining_face";

    bool isEnabled() const override;
    void execute()   override;
};

/**
 * @brief Clear all manually picked machining faces.
 */
class CmdClearMachiningFaces : public CommandBase
{
    Q_OBJECT
public:
    explicit CmdClearMachiningFaces(IAppContext* ctx);
    static constexpr const char* Name = "cam.clear_machining_faces";

    bool isEnabled() const override;
    void execute()   override;
};

/// Replaces the CAM manual contour order with the current selection order.
class CmdManualAppendSelectedToCamOrder : public CommandBase
{
    Q_OBJECT
public:
    inline static const QString Name = "cam.manualAppendSelectedToOrder";
    explicit CmdManualAppendSelectedToCamOrder(IAppContext* ctx);
    bool isEnabled() const override;
    void execute() override;
};

/// Applies CAM's automatic contour sorting with the stored primary axis.
class CmdAutoSortCamOrder : public CommandBase
{
    Q_OBJECT
public:
    inline static const QString Name = "cam.autoSortOrder";
    explicit CmdAutoSortCamOrder(IAppContext* ctx);
    bool isEnabled() const override;
    void execute() override;
};

/// Runs collision validation for the current solved toolpath regardless of
/// the automatic collision-detection switch.
class CmdValidateCamCollisions : public CommandBase
{
    Q_OBJECT
public:
    inline static const QString Name = "cam.validateCollisions";
    explicit CmdValidateCamCollisions(IAppContext* ctx);
    bool isEnabled() const override;
    void execute() override;
};

/// Toggles the CAM-owned rapid-travel overlay.
class CmdToggleCamTravelPath : public CommandBase
{
    Q_OBJECT
public:
    inline static const QString Name = "cam.toggleTravelPath";
    explicit CmdToggleCamTravelPath(IAppContext* ctx);
    bool isEnabled() const override;
    void execute() override;
};

/// Toggles CAM-owned sequence-number labels in the view.
class CmdToggleCamContourOrderLabel : public CommandBase
{
    Q_OBJECT
public:
    inline static const QString Name = "cam.toggleContourOrderLabel";
    explicit CmdToggleCamContourOrderLabel(IAppContext* ctx);
    bool isEnabled() const override;
    void execute() override;
};


