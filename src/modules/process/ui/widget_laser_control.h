#pragma once

#include <QWidget>
#include <QList>
#include <QMap>
#include <QString>

#include "core/kinematics/machine_kinematics.h"

/**
 * @brief Right-panel widget shown when the "执行" tab is active.
 *
 * Displays real-time axis positions, jog controls, start/pause/stop
 * buttons, and system status indicators — mirroring the control panel
 * visible in the reference screenshot.
 *
 * Phase 1: UI scaffold with simulated values (all via QTimer).
 * Phase 5: wired to DeviceManager / IMotionController.
 */
class WidgetLaserControl : public QWidget
{
    Q_OBJECT
public:
    explicit WidgetLaserControl(QWidget* parent = nullptr);

    // ── Called from device layer (Phase 5) ────────────────────────────────────
    void setAxisDefinitions(const QList<MachineAxisDef>& axes);
    void updateAxisPosition(const QString& axis, double pos);
    void updateConnectionStatus(bool connected);
    void updateSimulationMode(bool enabled);
    void updateSystemStatus(const QString& status);

signals:
    void startRequested();
    void pauseRequested();
    void stopRequested();
    void eStopRequested();
    void jogRequested(const QString& axis, int direction, int speedLevel);
    void homeRequested();
    void feedOverrideChanged(double factor);

private:
    void buildUi();
    void buildAxisGroup();
    void buildJogGroup();
    void buildProcessGroup();
    void rebuildAxisGroup();
    void rebuildJogGroup();
    void refreshStatusBanner();

    QList<MachineAxisDef> m_axisDefinitions;
    QMap<QString, class QLabel*> m_posLabels;    ///< axis → position label
    class QGroupBox* m_axisGroup{nullptr};
    class QGroupBox* m_jogGroup{nullptr};
    class QLabel*  m_statusLabel{nullptr};
    bool           m_connected{false};
    bool           m_simulationMode{true};
    QString        m_statusText;
    int            m_jogSpeedLevel{1};
};
