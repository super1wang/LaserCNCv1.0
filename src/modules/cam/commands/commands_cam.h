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
 * @brief Recalculate toolpath lead-in geometry using current parameters.
 *
 * Reads length and normal angle from the global LaserToolpath state,
 * recomputes all lead-in edges, and refreshes the 3D display.
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


