#pragma once

#include <QWidget>
#include <QList>
#include <QMap>
#include <QString>
#include <QVector>
#include <QElapsedTimer>

#include "core/kinematics/machine_kinematics.h"
#include "modules/process/i_process_facade.h"
#include "modules/process/process_module.h"

/**
 // 中文翻译：执行
 * @brief Right-panel widget shown when the "execute" tab is active.
 *
 * Displays real-time axis positions, jog controls, start/pause/stop
 * buttons, and system status indicators.
 */
class WidgetLaserControl : public QWidget
{
    Q_OBJECT
public:
    explicit WidgetLaserControl(QWidget* parent = nullptr);

    // ── Called from ProcessModule ─────────────────────────────────────────────
    void setAxisDefinitions(const QList<MachineAxisDef>& axes);
    void updateAxisPosition(const QString& axis, double pos);
    void updateConnectionStatus(bool connected);
    void updateSimulationMode(bool enabled);
    void updateSystemStatus(const QString& status);
    void updateRunState(lcnc::ProcessRunState state);
    /// 新加工运行开始时清零进度和计时；暂停恢复不调用此接口。
    void beginProcessingRun();
    void updateProcessingProgress(int completedContours, int totalContours);
    void updateAxisEnabled(const QString& axis, bool enabled);
    void updateDigitalOutput(const QString& outputName, const QString& channel, bool value);
    /// 根据 ProcessModule 推送的描述符列表重建 IO 栏按钮。
    void setDigitalOutputDescriptors(const QList<DigitalOutputDescriptor>& descriptors);
    void appendLogMessage(const QString& level, const QString& message);

signals:
    void startRequested();
    void pauseRequested();
    void resumeRequested();
    void stopRequested();
    void jogRequested(const QString& axis, int direction, int speedLevel, double distance);
    void absoluteMoveRequested(const QString& axis, double position, int speedLevel);
    void continuousJogStarted(const QString& axis, int direction, int speedLevel);
    void continuousJogStopped(const QString& axis);
    void axisEnableToggled(const QString& axis, bool enabled);
    void digitalOutputToggled(const QString& outputName, bool value);

private:
    enum class JogMode {
        Relative,
        Absolute,
        Continuous
    };

    void buildUi();
    void buildAxisGroup();
    void buildJogGroup();
    void buildProcessGroup();
    void buildIoGroup();
    void buildStatusGroup();
    void rebuildAxisGroup();
    void rebuildJogGroup();
    void resetProcessingProgress();
    void startProcessingClock();
    void pauseProcessingClock();
    void refreshProcessingStats();
    void handleMotionPressed(const QString& axis, int direction);
    void handleMotionReleased();
    void emitJogRequest(const QString& axis, int direction);
    void updateJogModeUi();
    void updateAxisButtonStyle(const QString& axis, bool enabled);
    void updateIoButtonStyle(const QString& outputName, bool value);
    void refreshStatusBanner();
    QString stateText(lcnc::ProcessRunState state) const;
    QString logColor(const QString& level) const;

    QList<MachineAxisDef> m_axisDefinitions;
    QMap<QString, class QLabel*> m_posLabels;    ///< axis → position label
    QMap<QString, class QPushButton*> m_axisButtons;
    QMap<QString, class QPushButton*> m_ioButtons;
    class QTabWidget* m_tabs{nullptr};
    class QWidget* m_controlPage{nullptr};
    class QWidget* m_logPage{nullptr};
    class QVBoxLayout* m_controlLayout{nullptr};
    class QTextEdit* m_logView{nullptr};
    class QGroupBox* m_axisGroup{nullptr};
    class QGroupBox* m_jogGroup{nullptr};
    class QGroupBox* m_ioGroup{nullptr};
    class QLabel* m_statusLabel{nullptr};
    class QProgressBar* m_processingProgressBar{nullptr};
    class QLabel* m_processingTimeLabel{nullptr};
    class QLabel* m_totalContoursLabel{nullptr};
    class QLabel* m_completedContoursLabel{nullptr};
    class QTimer* m_processingTimer{nullptr};
    class QPushButton* m_btnRun{nullptr};
    class QPushButton* m_btnPause{nullptr};
    class QPushButton* m_btnResume{nullptr};
    class QPushButton* m_btnStop{nullptr};
    class QStackedLayout* m_runActionStack{nullptr};
    class QDoubleSpinBox* m_jogDistanceSpin{nullptr};
    class QLabel* m_jogValueLabel{nullptr};
    JogMode        m_jogMode{JogMode::Relative};
    bool           m_connected{false};
    bool           m_simulationMode{true};
    lcnc::ProcessRunState m_runState{lcnc::ProcessRunState::Idle};
    QString        m_statusText;
    int            m_jogSpeedLevel{1};
    QString        m_activeJogAxis;
    int            m_activeJogDirection{0};
    int            m_totalContours{0};
    int            m_completedContours{0};
    qint64         m_processingElapsedMs{0};
    QElapsedTimer  m_processingElapsedTimer;
};
