#include "modules/process/ui/widget_laser_control.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QButtonGroup>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QLabel>
#include <QFrame>
#include <QGridLayout>
#include <QLayoutItem>
#include <QSignalBlocker>
#include <QTabWidget>
#include <QTextEdit>
#include <QTimer>

namespace {

void clearLayout(QLayout* layout)
{
    if (!layout)
        return;

    while (QLayoutItem* item = layout->takeAt(0)) {
        if (QLayout* childLayout = item->layout())
            clearLayout(childLayout);

        if (QWidget* widget = item->widget())
            delete widget;

        delete item;
    }
}

} // namespace

WidgetLaserControl::WidgetLaserControl(QWidget* parent)
    : QWidget(parent)
{
    m_jogHoldTimer = new QTimer(this);
    m_jogHoldTimer->setInterval(120);
    connect(m_jogHoldTimer, &QTimer::timeout, this, [this] {
        if (!m_activeJogAxis.isEmpty() && m_activeJogDirection != 0)
            emitJogRequest(m_activeJogAxis, m_activeJogDirection);
    });
    buildUi();
    refreshStatusBanner();
}

void WidgetLaserControl::buildUi()
{
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(4, 4, 4, 4);
    mainLayout->setSpacing(6);

    m_tabs = new QTabWidget(this);
    m_controlPage = new QWidget(m_tabs);
    m_controlLayout = new QVBoxLayout(m_controlPage);
    m_controlLayout->setContentsMargins(4, 4, 4, 4);
    m_controlLayout->setSpacing(6);

    m_logPage = new QWidget(m_tabs);
    auto* logLayout = new QVBoxLayout(m_logPage);
    logLayout->setContentsMargins(4, 4, 4, 4);
    m_logView = new QTextEdit(m_logPage);
    m_logView->setReadOnly(true);
    m_logView->setAcceptRichText(true);
    m_logView->setStyleSheet("QTextEdit { background: #101820; color: #E5E7EB; font-family: Consolas, monospace; font-size: 11px; }");
    logLayout->addWidget(m_logView);

    m_tabs->addTab(m_controlPage, tr("控制"));
    m_tabs->addTab(m_logPage, tr("系统日志"));
    mainLayout->addWidget(m_tabs);

    buildProcessGroup();
    buildAxisGroup();
    buildJogGroup();
    buildIoGroup();
    buildStatusGroup();
    buildDeviceGroup();

    m_controlLayout->addStretch();
}

void WidgetLaserControl::buildAxisGroup()
{
    m_axisGroup = new QGroupBox(tr("轴位置"), this);
    m_controlLayout->addWidget(m_axisGroup);
    rebuildAxisGroup();
}

void WidgetLaserControl::buildJogGroup()
{
    m_jogGroup = new QGroupBox(tr("手动点动"), this);
    m_controlLayout->addWidget(m_jogGroup);
    rebuildJogGroup();
}

void WidgetLaserControl::buildProcessGroup()
{
    auto* group = new QGroupBox(tr("加工控制"), this);
    auto* vlay  = new QVBoxLayout(group);
    auto* row = new QHBoxLayout();

    m_btnRun = new QPushButton(QIcon(":/icons/start.svg"), tr("运行"), group);
    m_btnPause = new QPushButton(QIcon(":/icons/pause.svg"), tr("暂停"), group);
    m_btnResume = new QPushButton(QIcon(":/icons/start.svg"), tr("继续"), group);
    m_btnStop = new QPushButton(QIcon(":/icons/stop.svg"), tr("停止"), group);

    for (auto* button : {m_btnRun, m_btnPause, m_btnResume, m_btnStop})
        button->setMinimumHeight(36);

    m_btnRun->setStyleSheet("background-color: #1B8F4A; color: white; font-weight: 600;");
    m_btnPause->setStyleSheet("background-color: #997a4b; color: white; font-weight: 600;");
    m_btnResume->setStyleSheet("background-color: #2563EB; color: white; font-weight: 600;");
    m_btnStop->setStyleSheet("background-color: #B91C1C; color: white; font-weight: 600;");

    row->addWidget(m_btnRun);
    row->addWidget(m_btnPause);
    row->addWidget(m_btnResume);
    row->addWidget(m_btnStop);
    vlay->addLayout(row);

    connect(m_btnRun, &QPushButton::clicked, this, &WidgetLaserControl::startRequested);
    connect(m_btnPause, &QPushButton::clicked, this, &WidgetLaserControl::pauseRequested);
    connect(m_btnResume, &QPushButton::clicked, this, &WidgetLaserControl::resumeRequested);
    connect(m_btnStop, &QPushButton::clicked, this, &WidgetLaserControl::stopRequested);
    updateRunState(lcnc::ProcessRunState::Idle);

    m_controlLayout->addWidget(group);
}

void WidgetLaserControl::buildIoGroup()
{
    m_ioGroup = new QGroupBox(tr("IO 状态"), this);
    auto* grid = new QGridLayout(m_ioGroup);
    const QStringList names = { tr("激光"), tr("吹气"), tr("夹头"), tr("水冷"), tr("气泵") };
    for (int index = 0; index < names.size(); ++index) {
        const QString name = names.at(index);
        auto* button = new QPushButton(name, m_ioGroup);
        button->setCheckable(true);
        button->setMinimumHeight(30);
        button->setToolTip(tr("点击切换 %1 输出").arg(name));
        m_ioButtons.insert(name, button);
        updateIoButtonStyle(name, false);
        connect(button, &QPushButton::clicked, this, [this, name](bool checked) {
            updateIoButtonStyle(name, checked);
            emit digitalOutputToggled(name, checked);
        });
        grid->addWidget(button, index / 2, index % 2);
    }
    m_controlLayout->addWidget(m_ioGroup);
}

void WidgetLaserControl::buildDeviceGroup()
{
    m_deviceGroup = new QGroupBox(tr("设备状态"), this);
    auto* layout = new QVBoxLayout(m_deviceGroup);

    m_deviceSummaryLabel = new QLabel(tr("等待设备状态刷新"), m_deviceGroup);
    m_deviceSummaryLabel->setWordWrap(true);
    layout->addWidget(m_deviceSummaryLabel);

    m_controlLayout->addWidget(m_deviceGroup);
    refreshDeviceSummary();
}

void WidgetLaserControl::buildStatusGroup()
{
    auto* group = new QGroupBox(tr("状态显示"), this);
    auto* layout = new QVBoxLayout(group);
    m_statusLabel = new QLabel(group);
    m_statusLabel->setAlignment(Qt::AlignCenter);
    m_statusLabel->setWordWrap(true);
    m_statusLabel->setMinimumHeight(36);
    layout->addWidget(m_statusLabel);
    m_controlLayout->addWidget(group);
}

void WidgetLaserControl::setAxisDefinitions(const QList<MachineAxisDef>& axes)
{
    m_axisDefinitions = axes;
    rebuildAxisGroup();
    rebuildJogGroup();
}

void WidgetLaserControl::rebuildAxisGroup()
{
    if (!m_axisGroup)
        return;

    clearLayout(m_axisGroup->layout());
    delete m_axisGroup->layout();

    m_posLabels.clear();

    auto* grid = new QGridLayout(m_axisGroup);
    int row = 0;
    for (const MachineAxisDef& axis : m_axisDefinitions) {
        if (axis.name == QStringLiteral("BASE"))
            continue;

        auto* lbl  = new QLabel(axis.name + ":", m_axisGroup);
        auto* val  = new QLabel("  0.000", m_axisGroup);
        val->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        val->setMinimumWidth(70);
        val->setStyleSheet("font-family: Consolas, monospace; color: #00FF88;");
        auto* unit = new QLabel(axis.motionType == MachineAxisDef::Linear ? "mm" : "°", m_axisGroup);
        grid->addWidget(lbl,  row, 0);
        grid->addWidget(val,  row, 1);
        grid->addWidget(unit, row, 2);
        m_posLabels[axis.name] = val;
        ++row;
    }

    if (row == 0) {
        auto* placeholder = new QLabel(tr("加载机台并配置轴系后显示"), m_axisGroup);
        placeholder->setStyleSheet("color: gray; font-size: 11px;");
        grid->addWidget(placeholder, 0, 0, 1, 3);
    }
}

void WidgetLaserControl::rebuildJogGroup()
{
    if (!m_jogGroup)
        return;

    clearLayout(m_jogGroup->layout());
    delete m_jogGroup->layout();
    m_axisButtons.clear();

    auto* vlay = new QVBoxLayout(m_jogGroup);

    auto* speedRow = new QHBoxLayout();
    speedRow->addWidget(new QLabel(tr("速度:"), m_jogGroup));
    auto* btnSlow = new QPushButton(tr("慢"), m_jogGroup);
    auto* btnMed  = new QPushButton(tr("中"), m_jogGroup);
    auto* btnFast = new QPushButton(tr("快"), m_jogGroup);
    btnSlow->setCheckable(true);
    btnMed->setCheckable(true);
    btnFast->setCheckable(true);
    btnMed->setChecked(m_jogSpeedLevel == 1);
    btnSlow->setChecked(m_jogSpeedLevel == 0);
    btnFast->setChecked(m_jogSpeedLevel == 2);

    auto* speedGroup = new QButtonGroup(m_jogGroup);
    speedGroup->setExclusive(true);
    speedGroup->addButton(btnSlow, 0);
    speedGroup->addButton(btnMed, 1);
    speedGroup->addButton(btnFast, 2);
    connect(speedGroup, &QButtonGroup::idClicked,
            this, [this](int id) { m_jogSpeedLevel = id; });
    for (auto* b : {btnSlow, btnMed, btnFast})
        b->setMaximumWidth(40);
    speedRow->addWidget(btnSlow);
    speedRow->addWidget(btnMed);
    speedRow->addWidget(btnFast);
    speedRow->addStretch();
    vlay->addLayout(speedRow);

    auto* distanceRow = new QHBoxLayout();
    distanceRow->addWidget(new QLabel(tr("距离:"), m_jogGroup));
    m_jogDistanceSpin = new QDoubleSpinBox(m_jogGroup);
    m_jogDistanceSpin->setRange(0.001, 1000.0);
    m_jogDistanceSpin->setDecimals(3);
    m_jogDistanceSpin->setSingleStep(0.1);
    m_jogDistanceSpin->setValue(1.0);
    m_jogDistanceSpin->setSuffix(QStringLiteral(" mm"));
    distanceRow->addWidget(m_jogDistanceSpin);
    vlay->addLayout(distanceRow);

    auto* jogGrid = new QGridLayout();
    int row = 0;
    for (const MachineAxisDef& axis : m_axisDefinitions) {
        if (axis.name == QStringLiteral("BASE"))
            continue;

        auto* btnAxis = new QPushButton(axis.name, m_jogGroup);
        btnAxis->setCheckable(true);
        btnAxis->setChecked(true);
        btnAxis->setMinimumWidth(46);
        auto* btnPlus  = new QPushButton("+", m_jogGroup);
        auto* btnMinus = new QPushButton("-", m_jogGroup);
        btnPlus->setFixedWidth(46);
        btnMinus->setFixedWidth(46);
        btnPlus->setAutoRepeat(false);
        btnMinus->setAutoRepeat(false);

        const QString axisName = axis.name.trimmed().toUpper();
        m_axisButtons.insert(axisName, btnAxis);
        updateAxisButtonStyle(axisName, true);
        connect(btnAxis, &QPushButton::toggled, this, [this, axisName](bool checked) {
            updateAxisButtonStyle(axisName, checked);
            emit axisEnableToggled(axisName, checked);
        });
        connect(btnPlus, &QPushButton::pressed,
            this, [this, axisName]{ startJogHold(axisName, +1); });
        connect(btnPlus, &QPushButton::released,
            this, &WidgetLaserControl::stopJogHold);
        connect(btnMinus, &QPushButton::pressed,
            this, [this, axisName]{ startJogHold(axisName, -1); });
        connect(btnMinus, &QPushButton::released,
            this, &WidgetLaserControl::stopJogHold);

        jogGrid->addWidget(btnAxis,  row, 0);
        jogGrid->addWidget(btnPlus,  row, 1);
        jogGrid->addWidget(btnMinus, row, 2);
        ++row;
    }

    if (row == 0) {
        auto* placeholder = new QLabel(tr("加载机台并配置轴系后显示"), m_jogGroup);
        placeholder->setStyleSheet("color: gray; font-size: 11px;");
        vlay->addWidget(placeholder);
    } else {
        vlay->addLayout(jogGrid);
    }
}

// ── Public update methods ──────────────────────────────────────────────────────
void WidgetLaserControl::updateAxisPosition(const QString& axis, double pos)
{
    if (m_posLabels.contains(axis))
        m_posLabels[axis]->setText(QString::asprintf("%8.3f", pos));
}

void WidgetLaserControl::updateConnectionStatus(bool connected)
{
    m_connected = connected;
    refreshStatusBanner();
    refreshDeviceSummary();
}

void WidgetLaserControl::updateSimulationMode(bool enabled)
{
    m_simulationMode = enabled;
    refreshStatusBanner();
    refreshDeviceSummary();
}

void WidgetLaserControl::updateSystemStatus(const QString& status)
{
    m_statusText = status;
    refreshStatusBanner();
    refreshDeviceSummary();
}

void WidgetLaserControl::updateRunState(lcnc::ProcessRunState state)
{
    m_runState = state;
    if (!m_btnRun || !m_btnPause || !m_btnResume)
        return;

    const bool running = state == lcnc::ProcessRunState::Running;
    const bool paused = state == lcnc::ProcessRunState::Paused;
    m_btnRun->setVisible(!running && !paused);
    m_btnPause->setVisible(running);
    m_btnResume->setVisible(paused);
    if (m_btnStop)
        m_btnStop->setVisible(true);
    refreshStatusBanner();
    refreshDeviceSummary();
}

void WidgetLaserControl::updateAxisEnabled(const QString& axis, bool enabled)
{
    const QString key = axis.trimmed().toUpper();
    auto* button = m_axisButtons.value(key, nullptr);
    if (!button)
        return;
    const QSignalBlocker blocker(button);
    button->setChecked(enabled);
    updateAxisButtonStyle(key, enabled);
}

void WidgetLaserControl::updateDigitalOutput(const QString& outputName, const QString& channel, bool value)
{
    auto* button = m_ioButtons.value(outputName, nullptr);
    if (!button)
        return;
    const QSignalBlocker blocker(button);
    button->setChecked(value);
    if (!channel.trimmed().isEmpty())
        button->setToolTip(tr("%1: %2").arg(outputName, channel));
    updateIoButtonStyle(outputName, value);
}

void WidgetLaserControl::appendLogMessage(const QString& level, const QString& message)
{
    if (!m_logView || message.trimmed().isEmpty())
        return;
    const QString time = QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss.zzz"));
    const QString normalizedLevel = level.trimmed().isEmpty() ? QStringLiteral("info") : level.trimmed().toLower();
    const QString html = QStringLiteral("<div><span style='color:#94A3B8'>[%1]</span> "
                                        "<span style='color:%2'>[%3]</span> %4</div>")
        .arg(time,
             logColor(normalizedLevel),
             normalizedLevel.toUpper().toHtmlEscaped(),
             message.toHtmlEscaped());
    m_logView->append(html);
}

void WidgetLaserControl::startJogHold(const QString& axis, int direction)
{
    m_activeJogAxis = axis;
    m_activeJogDirection = direction;
    emitJogRequest(axis, direction);
    if (m_jogHoldTimer)
        m_jogHoldTimer->start();
}

void WidgetLaserControl::stopJogHold()
{
    if (m_jogHoldTimer)
        m_jogHoldTimer->stop();
    m_activeJogAxis.clear();
    m_activeJogDirection = 0;
}

void WidgetLaserControl::emitJogRequest(const QString& axis, int direction)
{
    const double distance = m_jogDistanceSpin ? m_jogDistanceSpin->value() : 1.0;
    emit jogRequested(axis, direction, m_jogSpeedLevel, distance);
}

void WidgetLaserControl::updateAxisButtonStyle(const QString& axis, bool enabled)
{
    auto* button = m_axisButtons.value(axis.trimmed().toUpper(), nullptr);
    if (!button)
        return;
    button->setStyleSheet(enabled
        ? QStringLiteral("background-color: #18864B; color: white; font-weight: 600;")
        : QStringLiteral("background-color: #6B7280; color: white;"));
}

void WidgetLaserControl::updateIoButtonStyle(const QString& outputName, bool value)
{
    auto* button = m_ioButtons.value(outputName, nullptr);
    if (!button)
        return;
    button->setStyleSheet(value
        ? QStringLiteral("background-color: #16A34A; color: white; font-weight: 600;")
        : QStringLiteral("background-color: #B91C1C; color: white;"));
}

void WidgetLaserControl::refreshStatusBanner()
{
    if (!m_statusLabel)
        return;
    const QString fallback = m_simulationMode
        ? (m_connected ? tr("仿真模式 — 控制器已连接") : tr("仿真模式 — 未连接"))
        : (m_connected ? tr("控制器模式 — 已连接") : tr("控制器模式 — 未连接"));
    const QString status = m_statusText.isEmpty() ? fallback : m_statusText;
    const QString text = tr("状态机: %1\n%2").arg(stateText(m_runState), status);

    QString style = QStringLiteral("background: #333; color: #AAFFAA; padding: 2px 4px; border-radius: 3px;");
    if (m_runState == lcnc::ProcessRunState::EmergencyStop) {
        style = QStringLiteral("background: #7F1D1D; color: white; padding: 2px 4px; border-radius: 3px;");
    } else if (m_runState == lcnc::ProcessRunState::Error) {
        style = QStringLiteral("background: #92400E; color: white; padding: 2px 4px; border-radius: 3px;");
    } else if (m_runState == lcnc::ProcessRunState::Paused) {
        style = QStringLiteral("background: #B45309; color: white; padding: 2px 4px; border-radius: 3px;");
    } else if (m_runState == lcnc::ProcessRunState::Running) {
        style = QStringLiteral("background: #14532D; color: #A7F3D0; padding: 2px 4px; border-radius: 3px;");
    } else if (m_connected && !m_simulationMode) {
        style = QStringLiteral("background: #1B5E20; color: #00FF88; padding: 2px 4px; border-radius: 3px;");
    }

    m_statusLabel->setText(text);
    m_statusLabel->setStyleSheet(style);
}

QString WidgetLaserControl::stateText(lcnc::ProcessRunState state) const
{
    switch (state) {
    case lcnc::ProcessRunState::Idle:
        return tr("空闲");
    case lcnc::ProcessRunState::Running:
        return tr("运行中");
    case lcnc::ProcessRunState::Paused:
        return tr("暂停");
    case lcnc::ProcessRunState::Error:
        return tr("错误");
    case lcnc::ProcessRunState::EmergencyStop:
        return tr("急停");
    }
    return tr("未知");
}

void WidgetLaserControl::refreshDeviceSummary()
{
    if (!m_deviceSummaryLabel)
        return;

    const QString modeText = m_simulationMode ? tr("纯仿真") : tr("控制器联机");
    QString summary = tr("模式: %1  流程: %2").arg(modeText, stateText(m_runState));
    summary += tr("  已连接: %1").arg(m_connected ? tr("是") : tr("否"));
    if (!m_statusText.trimmed().isEmpty())
        summary += tr("\n%1").arg(m_statusText.trimmed());

    QString style = QStringLiteral("background: #E2E8F0; color: #0F172A; padding: 4px 6px; border-radius: 4px;");
    if (m_runState == lcnc::ProcessRunState::EmergencyStop || m_runState == lcnc::ProcessRunState::Error) {
        style = QStringLiteral("background: #7F1D1D; color: white; padding: 4px 6px; border-radius: 4px;");
    } else if (m_runState == lcnc::ProcessRunState::Paused) {
        style = QStringLiteral("background: #B45309; color: white; padding: 4px 6px; border-radius: 4px;");
    } else if (m_connected && !m_simulationMode) {
        style = QStringLiteral("background: #14532D; color: #DCFCE7; padding: 4px 6px; border-radius: 4px;");
    }

    m_deviceSummaryLabel->setText(summary);
    m_deviceSummaryLabel->setStyleSheet(style);
}

QString WidgetLaserControl::deviceStateStyle(bool connected) const
{
    return connected
        ? QStringLiteral("color: #15803D; font-weight: 700;")
        : QStringLiteral("color: #64748B; font-weight: 600;");
}

QString WidgetLaserControl::logColor(const QString& level) const
{
    if (level == QStringLiteral("error"))
        return QStringLiteral("#F87171");
    if (level == QStringLiteral("warn") || level == QStringLiteral("warning"))
        return QStringLiteral("#FBBF24");
    if (level == QStringLiteral("state"))
        return QStringLiteral("#A78BFA");
    if (level == QStringLiteral("operation"))
        return QStringLiteral("#60A5FA");
    if (level == QStringLiteral("process"))
        return QStringLiteral("#34D399");
    return QStringLiteral("#CBD5E1");
}
