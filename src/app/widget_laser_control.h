#pragma once

#include <QWidget>
#include <QMap>
#include <QString>

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
    void updateAxisPosition(const QString& axis, double pos);
    void updateConnectionStatus(bool connected);
    void updateSystemStatus(const QString& status);

signals:
    void startRequested();
    void pauseRequested();
    void stopRequested();
    void eStopRequested();
    void jogRequested(const QString& axis, int direction, int speedLevel);
    void homeRequested();

private slots:
    void onSimTick();     ///< Simulates axis readback in Phase 1

private:
    void buildUi();
    void buildAxisGroup();
    void buildJogGroup();
    void buildProcessGroup();

    QMap<QString, class QLabel*> m_posLabels;    ///< axis → position label
    class QTimer*  m_simTimer{nullptr};
    class QLabel*  m_statusLabel{nullptr};
    double         m_simPos[5]{};                ///< simulated positions X Y Z A C
};
