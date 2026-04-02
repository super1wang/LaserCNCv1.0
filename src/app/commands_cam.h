#pragma once

#include "app/commands_api.h"

class LaserToolpath;
class WidgetToolpathPanel;
class GraphicsScene;
class QTimer;

#include <AIS_Shape.hxx>
#include <QList>

/**
 * @brief Extract contours from the mounted workpiece and generate laser toolpath.
 *
 * Collects all workpiece entities from the machine document, extracts
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
 * @brief Interactive pick of lead-in entry point on a contour edge.
 *
 * Activates edge-level selection mode on the toolpath contour shapes.
 * The user clicks an edge; the closest point becomes the lead-in entry.
 * After picking, the lead-in line is computed and displayed.
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

/**
 * @brief Continuous machine simulation animation.
 *
 * Plays through all toolpath contour points sequentially, driving
 * the machine axes to their computed IK positions each tick.
 * Supports play / pause / stop, and adjustable speed.
 */
class CmdSimulate : public CommandBase
{
    Q_OBJECT
public:
    explicit CmdSimulate(IAppContext* ctx);
    static constexpr const char* Name = "cam.simulate";

    bool isEnabled() const override;
    void execute()   override;

    /// Control methods (called by ribbon buttons)
    void play();
    void pause();
    void stop();
    void setSpeed(double factor);  ///< 1.0 = normal, 2.0 = 2x, etc.

    bool isPlaying() const;
    bool isPaused()  const;

signals:
    void simulationTick(int contourIndex, int pointIndex, int totalPoints);
    void simulationFinished();

private slots:
    void onTick();

private:
    QTimer* m_timer{nullptr};
    int     m_currentContour{0};
    int     m_currentPoint{0};
    int     m_totalPoints{0};
    double  m_speed{1.0};
    bool    m_playing{false};
    bool    m_paused{false};
};

