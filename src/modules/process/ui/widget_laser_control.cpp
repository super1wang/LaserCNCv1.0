#include "modules/process/ui/widget_laser_control.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QButtonGroup>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QProgressBar>
#include <QLabel>
#include <QFrame>
#include <QGridLayout>
#include <QLayoutItem>
#include <QSignalBlocker>
#include <QTabWidget>
#include <QTextEdit>
#include <QTimer>
#include <QStackedLayout>
#include <QScrollArea>

#include <algorithm>

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

QList<MachineAxisDef> orderedAxes(const QList<MachineAxisDef>& source)
{
    QList<MachineAxisDef> axes = source;
    const auto priority = [](const QString& axis) {
        const QString name = axis.trimmed().toUpper();
        if (name == QStringLiteral("X")) return 0;
        if (name == QStringLiteral("Y")) return 1;
        if (name == QStringLiteral("Z")) return 2;
        if (name == QStringLiteral("A")) return 3;
        if (name == QStringLiteral("B")) return 4;
        if (name == QStringLiteral("C")) return 5;
        return 100;
    };
    std::sort(axes.begin(), axes.end(), [&priority](const MachineAxisDef& left, const MachineAxisDef& right) {
        const int leftPriority = priority(left.name);
        const int rightPriority = priority(right.name);
        return leftPriority == rightPriority
            ? left.name.compare(right.name, Qt::CaseInsensitive) < 0
            : leftPriority < rightPriority;
    });
    return axes;
}

} // namespace

WidgetLaserControl::WidgetLaserControl(QWidget* parent)
    : QWidget(parent)
{
    buildUi();
    refreshStatusBanner();
}

void WidgetLaserControl::buildUi()
{
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(6, 6, 6, 6);
    mainLayout->setSpacing(8);

    m_tabs = new QTabWidget(this);
    auto* controlScroll = new QScrollArea(m_tabs);
    controlScroll->setWidgetResizable(true);
    controlScroll->setFrameShape(QFrame::NoFrame);
    controlScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    controlScroll->setStyleSheet("QScrollArea { background: #202B35; }");
    m_controlPage = new QWidget(controlScroll);
    m_controlPage->setStyleSheet("background: #202B35;");
    m_controlLayout = new QVBoxLayout(m_controlPage);
    m_controlLayout->setContentsMargins(6, 6, 6, 6);
    m_controlLayout->setSpacing(8);

    m_logPage = new QWidget(m_tabs);
    auto* logLayout = new QVBoxLayout(m_logPage);
    logLayout->setContentsMargins(4, 4, 4, 4);
    m_logView = new QTextEdit(m_logPage);
    m_logView->setReadOnly(true);
    m_logView->setAcceptRichText(true);
    m_logView->setStyleSheet("QTextEdit { background: #101820; color: #D6E4EA; font-family: Consolas, monospace; font-size: 11px; }");
    logLayout->addWidget(m_logView);

    controlScroll->setWidget(m_controlPage);
    // 中文翻译：控制
    m_tabs->addTab(controlScroll, tr("control"));
    // 中文翻译：系统日志
    m_tabs->addTab(m_logPage, tr("System log"));
    mainLayout->addWidget(m_tabs);

    buildProcessGroup();
    buildStatusGroup();
    buildIoGroup();
    buildAxisGroup();
    buildJogGroup();

    m_controlLayout->addStretch();
}

void WidgetLaserControl::buildAxisGroup()
{
    // 中文翻译：轴位置
    m_axisGroup = new QGroupBox(tr("axis position"), this);
    m_controlLayout->addWidget(m_axisGroup);
    rebuildAxisGroup();
}

void WidgetLaserControl::buildJogGroup()
{
    // 中文翻译：手动点动
    m_jogGroup = new QGroupBox(tr("Manual jog"), this);
    m_controlLayout->addWidget(m_jogGroup);
    rebuildJogGroup();
}

void WidgetLaserControl::buildProcessGroup()
{
    // 中文翻译：加工控制
    auto* group = new QGroupBox(tr("Process control"), this);
    auto* vlay  = new QVBoxLayout(group);
    auto* row = new QHBoxLayout();

    // 中文翻译：运行
    m_btnRun = new QPushButton(QIcon(":/icons/start.svg"), tr("run"), group);
    // 中文翻译：暂停
    m_btnPause = new QPushButton(QIcon(":/icons/pause.svg"), tr("pause"), group);
    // 中文翻译：继续
    m_btnResume = new QPushButton(QIcon(":/icons/start.svg"), tr("continue"), group);
    // 中文翻译：停止
    m_btnStop = new QPushButton(QIcon(":/icons/stop.svg"), tr("stop"), group);

    m_btnRun->setProperty("role", "run");
    m_btnPause->setProperty("role", "pause");
    m_btnResume->setProperty("role", "resume");
    m_btnStop->setProperty("role", "stop");
    for (auto* button : {m_btnRun, m_btnPause, m_btnResume, m_btnStop}) {
        button->setMinimumHeight(38);
        button->setIconSize(QSize(18, 18));
    }

    auto* actionSlot = new QWidget(group);
    m_runActionStack = new QStackedLayout(actionSlot);
    m_runActionStack->setContentsMargins(0, 0, 0, 0);
    m_runActionStack->addWidget(m_btnRun);
    m_runActionStack->addWidget(m_btnPause);
    m_runActionStack->addWidget(m_btnResume);
    row->addWidget(actionSlot, 1);
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
    // 中文翻译：IO 状态
    m_ioGroup = new QGroupBox(tr("IO status"), this);
    auto* grid = new QGridLayout(m_ioGroup);
    grid->setContentsMargins(4, 4, 4, 4);
    // 初始为空 — 由 ProcessModule::digitalOutputDescriptorsChanged 通过
    // setDigitalOutputDescriptors 注入按钮（依据 settings 中 showInMain 字段）。
    m_controlLayout->addWidget(m_ioGroup);
}

void WidgetLaserControl::setDigitalOutputDescriptors(const QList<DigitalOutputDescriptor>& descriptors)
{
    if (!m_ioGroup)
        return;
    auto* layout = m_ioGroup->layout();
    if (!layout) {
        layout = new QGridLayout(m_ioGroup);
        layout->setContentsMargins(4, 4, 4, 4);
    }
    auto* grid = qobject_cast<QGridLayout*>(layout);

    // 清空旧按钮 / 占位 label。
    while (QLayoutItem* item = layout->takeAt(0)) {
        if (QWidget* w = item->widget())
            delete w;
        delete item;
    }
    m_ioButtons.clear();

    if (descriptors.isEmpty()) {
        // 中文翻译：未配置主界面 IO
        auto* placeholder = new QLabel(tr("Main interface IO is not configured"), m_ioGroup);
        placeholder->setStyleSheet("color: gray; font-size: 11px;");
        if (grid)
            grid->addWidget(placeholder, 0, 0, 1, 2);
        else
            layout->addWidget(placeholder);
        return;
    }

    for (int index = 0; index < descriptors.size(); ++index) {
        const DigitalOutputDescriptor& desc = descriptors.at(index);
        const QString display = desc.name.isEmpty() ? desc.channel : desc.name;
        auto* button = new QPushButton(display, m_ioGroup);
        button->setCheckable(true);
        button->setMinimumHeight(30);
        button->setProperty("channel", desc.channel);
        // 中文翻译：点击切换 %1 输出 (channel=%2)
        button->setToolTip(tr("Click to toggle %1 output (channel=%2)").arg(display, desc.channel));
        m_ioButtons.insert(display, button);
        updateIoButtonStyle(display, false);
        connect(button, &QPushButton::clicked, this, [this, display](bool checked) {
            updateIoButtonStyle(display, checked);
            emit digitalOutputToggled(display, checked);
        });
        if (grid)
            grid->addWidget(button, index / 2, index % 2);
        else
            layout->addWidget(button);
    }
}

void WidgetLaserControl::buildStatusGroup()
{
    // 中文翻译：状态显示
    auto* group = new QGroupBox(tr("status display"), this);
    auto* layout = new QVBoxLayout(group);
    m_statusLabel = new QLabel(group);
    m_statusLabel->setAlignment(Qt::AlignCenter);
    m_statusLabel->setWordWrap(true);
    m_statusLabel->setMinimumHeight(36);
    layout->addWidget(m_statusLabel);

    m_processingProgressBar = new QProgressBar(group);
    m_processingProgressBar->setRange(0, 100);
    m_processingProgressBar->setValue(0);
    // 中文翻译：加工进度: %p%
    m_processingProgressBar->setFormat(tr("Processing progress: %p%"));
    layout->addWidget(m_processingProgressBar);

    auto* stats = new QGridLayout();
    // 中文翻译：加工时间:
    stats->addWidget(new QLabel(tr("Processing time:"), group), 0, 0);
    m_processingTimeLabel = new QLabel(group);
    stats->addWidget(m_processingTimeLabel, 0, 1);
    // 中文翻译：总轮廓数:
    stats->addWidget(new QLabel(tr("Total number of contours:"), group), 1, 0);
    m_totalContoursLabel = new QLabel(group);
    stats->addWidget(m_totalContoursLabel, 1, 1);
    // 中文翻译：已加工轮廓数:
    stats->addWidget(new QLabel(tr("Number of contours processed:"), group), 2, 0);
    m_completedContoursLabel = new QLabel(group);
    stats->addWidget(m_completedContoursLabel, 2, 1);
    layout->addLayout(stats);

    m_processingTimer = new QTimer(this);
    m_processingTimer->setInterval(250);
    connect(m_processingTimer, &QTimer::timeout,
            this, &WidgetLaserControl::refreshProcessingStats);
    refreshProcessingStats();
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
    m_axisButtons.clear();

    auto* grid = new QGridLayout(m_axisGroup);
    grid->setHorizontalSpacing(5);
    grid->setVerticalSpacing(3);
    int axisIndex = 0;
    for (const MachineAxisDef& axis : orderedAxes(m_axisDefinitions)) {
        if (axis.name == QStringLiteral("BASE"))
            continue;

        // 使能控件放在坐标前；运动行只保留静态轴名，避免点击 +/- 时
        // 误触紧邻的使能按钮。
        auto* btnAxis = new QPushButton(axis.name, m_axisGroup);
        btnAxis->setCheckable(true);
        btnAxis->setChecked(true);
        btnAxis->setMinimumWidth(46);
        auto* val  = new QLabel("  0.000", m_axisGroup);
        val->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        val->setMinimumWidth(70);
        val->setStyleSheet("font-family: Consolas, monospace; color: #49E6B5; font-weight: 600;");
        auto* unit = new QLabel(axis.motionType == MachineAxisDef::Linear ? "mm" : "°", m_axisGroup);
        const QString axisName = axis.name.trimmed().toUpper();
        m_axisButtons.insert(axisName, btnAxis);
        updateAxisButtonStyle(axisName, true);
        connect(btnAxis, &QPushButton::toggled, this, [this, axisName](bool checked) {
            updateAxisButtonStyle(axisName, checked);
            emit axisEnableToggled(axisName, checked);
        });
        const int row = axisIndex / 2;
        const int column = (axisIndex % 2) * 3;
        grid->addWidget(btnAxis, row, column);
        grid->addWidget(val, row, column + 1);
        grid->addWidget(unit, row, column + 2);
        m_posLabels[axis.name] = val;
        ++axisIndex;
    }

    if (axisIndex == 0) {
        // 中文翻译：加载机台并配置轴系后显示
        auto* placeholder = new QLabel(tr("Display after loading the machine and configuring the axis system"), m_axisGroup);
        placeholder->setStyleSheet("color: gray; font-size: 11px;");
        grid->addWidget(placeholder, 0, 0, 1, 6);
    }
}

void WidgetLaserControl::rebuildJogGroup()
{
    if (!m_jogGroup)
        return;

    clearLayout(m_jogGroup->layout());
    delete m_jogGroup->layout();

    auto* vlay = new QVBoxLayout(m_jogGroup);

    auto* modeSpeedRow = new QHBoxLayout();
    modeSpeedRow->setSpacing(3);
    // 中文翻译：模式:
    modeSpeedRow->addWidget(new QLabel(tr("Mode:"), m_jogGroup));
    // 中文翻译：相对
    auto* btnRelative = new QPushButton(tr("relatively"), m_jogGroup);
    // 中文翻译：绝对
    auto* btnAbsolute = new QPushButton(tr("Absolutely"), m_jogGroup);
    // 中文翻译：连续
    auto* btnContinuous = new QPushButton(tr("continuous"), m_jogGroup);
    for (auto* b : {btnRelative, btnAbsolute, btnContinuous}) {
        b->setCheckable(true);
        b->setFixedWidth(38);
    }
    btnRelative->setChecked(m_jogMode == JogMode::Relative);
    btnAbsolute->setChecked(m_jogMode == JogMode::Absolute);
    btnContinuous->setChecked(m_jogMode == JogMode::Continuous);
    auto* modeGroup = new QButtonGroup(m_jogGroup);
    modeGroup->setExclusive(true);
    modeGroup->addButton(btnRelative, 0);
    modeGroup->addButton(btnAbsolute, 1);
    modeGroup->addButton(btnContinuous, 2);
    connect(modeGroup, &QButtonGroup::idClicked, this, [this](int id) {
        switch (id) {
        case 1: m_jogMode = JogMode::Absolute; break;
        case 2: m_jogMode = JogMode::Continuous; break;
        case 0:
        default: m_jogMode = JogMode::Relative; break;
        }
        updateJogModeUi();
    });
    modeSpeedRow->addWidget(btnRelative);
    modeSpeedRow->addWidget(btnAbsolute);
    modeSpeedRow->addWidget(btnContinuous);
    modeSpeedRow->addSpacing(8);
    // 中文翻译：速度:
    modeSpeedRow->addWidget(new QLabel(tr("Speed:"), m_jogGroup));
    // 中文翻译：慢
    auto* btnSlow = new QPushButton(tr("slow"), m_jogGroup);
    // 中文翻译：中
    auto* btnMed  = new QPushButton(tr("in"), m_jogGroup);
    // 中文翻译：快
    auto* btnFast = new QPushButton(tr("Fast"), m_jogGroup);
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
        b->setFixedWidth(30);
    modeSpeedRow->addWidget(btnSlow);
    modeSpeedRow->addWidget(btnMed);
    modeSpeedRow->addWidget(btnFast);
    modeSpeedRow->addStretch();
    vlay->addLayout(modeSpeedRow);

    auto* distanceRow = new QHBoxLayout();
    // 中文翻译：距离:
    m_jogValueLabel = new QLabel(tr("Distance:"), m_jogGroup);
    distanceRow->addWidget(m_jogValueLabel);
    m_jogDistanceSpin = new QDoubleSpinBox(m_jogGroup);
    m_jogDistanceSpin->setRange(-1000000.0, 1000000.0);
    m_jogDistanceSpin->setDecimals(3);
    m_jogDistanceSpin->setSingleStep(0.1);
    m_jogDistanceSpin->setSuffix(QStringLiteral(" mm"));
    if (m_jogDistanceSpin->value() == 0.0)
        m_jogDistanceSpin->setValue(1.0);
    distanceRow->addWidget(m_jogDistanceSpin);
    vlay->addLayout(distanceRow);
    updateJogModeUi();

    auto* jogGrid = new QGridLayout();
    int row = 0;
    for (const MachineAxisDef& axis : orderedAxes(m_axisDefinitions)) {
        if (axis.name == QStringLiteral("BASE"))
            continue;

        auto* lblAxis = new QLabel(axis.name, m_jogGroup);
        lblAxis->setMinimumWidth(46);
        lblAxis->setAlignment(Qt::AlignCenter);
        lblAxis->setStyleSheet("background:#253542; border:1px solid #4A606D; border-radius:3px; padding:5px; color:#9EDBEC; font-weight:700;");
        auto* btnPlus  = new QPushButton(QIcon(":/icons/jog_positive.svg"), tr("+"), m_jogGroup);
        auto* btnMinus = new QPushButton(QIcon(":/icons/jog_negative.svg"), tr("−"), m_jogGroup);
        btnPlus->setProperty("jogDirection", "positive");
        btnMinus->setProperty("jogDirection", "negative");
        // 中文翻译：%1 正方向点动
        btnPlus->setToolTip(tr("%1 Jog in positive direction").arg(axis.name));
        // 中文翻译：%1 负方向点动
        btnMinus->setToolTip(tr("%1 Negative direction jog").arg(axis.name));
        btnPlus->setIconSize(QSize(16, 16));
        btnMinus->setIconSize(QSize(16, 16));
        btnPlus->setAutoRepeat(false);
        btnMinus->setAutoRepeat(false);

        const QString axisName = axis.name.trimmed().toUpper();
        connect(btnPlus, &QPushButton::pressed,
            this, [this, axisName]{ handleMotionPressed(axisName, +1); });
        connect(btnPlus, &QPushButton::released,
            this, &WidgetLaserControl::handleMotionReleased);
        connect(btnMinus, &QPushButton::pressed,
            this, [this, axisName]{ handleMotionPressed(axisName, -1); });
        connect(btnMinus, &QPushButton::released,
            this, &WidgetLaserControl::handleMotionReleased);

        jogGrid->addWidget(lblAxis,  row, 0);
        jogGrid->addWidget(btnPlus,  row, 1);
        jogGrid->addWidget(btnMinus, row, 2);
        ++row;
    }

    if (row == 0) {
        // 中文翻译：加载机台并配置轴系后显示
        auto* placeholder = new QLabel(tr("Display after loading the machine and configuring the axis system"), m_jogGroup);
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
}

void WidgetLaserControl::updateSimulationMode(bool enabled)
{
    m_simulationMode = enabled;
    refreshStatusBanner();
}

void WidgetLaserControl::updateSystemStatus(const QString& status)
{
    m_statusText = status;
    refreshStatusBanner();
}

void WidgetLaserControl::updateRunState(lcnc::ProcessRunState state)
{
    if (state == lcnc::ProcessRunState::Running) {
        startProcessingClock();
    } else if (state == lcnc::ProcessRunState::Paused) {
        pauseProcessingClock();
    } else if (state == lcnc::ProcessRunState::Idle
               || state == lcnc::ProcessRunState::Error
               || state == lcnc::ProcessRunState::EmergencyStop) {
        pauseProcessingClock();
    }

    m_runState = state;
    if (!m_btnRun || !m_btnPause || !m_btnResume)
        return;

    const bool running = state == lcnc::ProcessRunState::Running;
    const bool paused = state == lcnc::ProcessRunState::Paused;
    if (m_runActionStack) {
        m_runActionStack->setCurrentWidget(running ? m_btnPause : (paused ? m_btnResume : m_btnRun));
    }
    if (m_btnStop)
        m_btnStop->setVisible(true);
    refreshStatusBanner();
}

void WidgetLaserControl::beginProcessingRun()
{
    resetProcessingProgress();
    startProcessingClock();
}

void WidgetLaserControl::updateProcessingProgress(int completedContours, int totalContours)
{
    m_totalContours = qMax(0, totalContours);
    m_completedContours = qBound(0, completedContours, m_totalContours);
    refreshProcessingStats();
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

void WidgetLaserControl::handleMotionPressed(const QString& axis, int direction)
{
    if (m_jogMode == JogMode::Relative) {
        emitJogRequest(axis, direction);
        return;
    }

    if (m_jogMode == JogMode::Absolute) {
        const double target = m_jogDistanceSpin ? m_jogDistanceSpin->value() * (direction > 0 ? 1.0 : -1.0) : 0.0;
        emit absoluteMoveRequested(axis, target, m_jogSpeedLevel);
        return;
    }

    m_activeJogAxis = axis;
    m_activeJogDirection = direction;
    emit continuousJogStarted(axis, direction, m_jogSpeedLevel);
}

void WidgetLaserControl::handleMotionReleased()
{
    if (m_jogMode == JogMode::Continuous && !m_activeJogAxis.isEmpty())
        emit continuousJogStopped(m_activeJogAxis);
    m_activeJogAxis.clear();
    m_activeJogDirection = 0;
}

void WidgetLaserControl::emitJogRequest(const QString& axis, int direction)
{
    const double distance = m_jogDistanceSpin ? m_jogDistanceSpin->value() : 1.0;
    emit jogRequested(axis, direction, m_jogSpeedLevel, distance);
}

void WidgetLaserControl::updateJogModeUi()
{
    if (!m_jogDistanceSpin || !m_jogValueLabel)
        return;

    switch (m_jogMode) {
    case JogMode::Absolute:
        // 中文翻译：位置:
        m_jogValueLabel->setText(tr("Location:"));
        m_jogDistanceSpin->setEnabled(true);
        m_jogDistanceSpin->setMinimum(0.0);
        // 中文翻译：+ 按钮移动到正目标位置，- 按钮移动到负目标位置。
        m_jogDistanceSpin->setToolTip(tr("The + button moves to the positive target position and the - button moves to the negative target position."));
        break;
    case JogMode::Continuous:
        // 中文翻译：距离:
        m_jogValueLabel->setText(tr("Distance:"));
        m_jogDistanceSpin->setEnabled(false);
        // 中文翻译：连续模式按住 +/- 运动，松开停止。
        m_jogDistanceSpin->setToolTip(tr("Continuous mode press and hold +/- to move, release to stop."));
        break;
    case JogMode::Relative:
    default:
        // 中文翻译：距离:
        m_jogValueLabel->setText(tr("Distance:"));
        m_jogDistanceSpin->setEnabled(true);
        m_jogDistanceSpin->setMinimum(0.001);
        // 中文翻译：相对模式每次点击按该距离运动。
        m_jogDistanceSpin->setToolTip(tr("In relative mode, each click moves by this distance."));
        break;
    }
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
        // 中文翻译：仿真模式 — 控制器已连接；仿真模式 — 未连接
        ? (m_connected ? tr("Simulation mode - controller connected") : tr("Emulation mode - not connected"))
        // 中文翻译：控制器模式 — 已连接；控制器模式 — 未连接
        : (m_connected ? tr("Controller Mode - Connected") : tr("Controller mode - not connected"));
    const QString status = m_statusText.isEmpty() ? fallback : m_statusText;
    // 中文翻译：状态机: %1\n%2
    const QString text = tr("State machine: %1\n%2").arg(stateText(m_runState), status);

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
        // 中文翻译：空闲
        return tr("free");
    case lcnc::ProcessRunState::Running:
        // 中文翻译：运行中
        return tr("Running");
    case lcnc::ProcessRunState::Paused:
        // 中文翻译：暂停
        return tr("pause");
    case lcnc::ProcessRunState::Error:
        // 中文翻译：错误
        return tr("Error");
    case lcnc::ProcessRunState::EmergencyStop:
        // 中文翻译：急停
        return tr("emergency stop");
    }
    // 中文翻译：未知
    return tr("unknown");
}

void WidgetLaserControl::resetProcessingProgress()
{
    m_totalContours = 0;
    m_completedContours = 0;
    m_processingElapsedMs = 0;
    m_processingElapsedTimer.invalidate();
    if (m_processingTimer)
        m_processingTimer->stop();
    refreshProcessingStats();
}

void WidgetLaserControl::startProcessingClock()
{
    if (!m_processingElapsedTimer.isValid())
        m_processingElapsedTimer.start();
    if (m_processingTimer && !m_processingTimer->isActive())
        m_processingTimer->start();
    refreshProcessingStats();
}

void WidgetLaserControl::pauseProcessingClock()
{
    if (m_processingElapsedTimer.isValid()) {
        m_processingElapsedMs += m_processingElapsedTimer.elapsed();
        m_processingElapsedTimer.invalidate();
    }
    if (m_processingTimer)
        m_processingTimer->stop();
    refreshProcessingStats();
}

void WidgetLaserControl::refreshProcessingStats()
{
    const qint64 elapsedMs = m_processingElapsedMs
        + (m_processingElapsedTimer.isValid() ? m_processingElapsedTimer.elapsed() : 0);
    const qint64 totalSeconds = elapsedMs / 1000;
    const qint64 hours = totalSeconds / 3600;
    const qint64 minutes = (totalSeconds % 3600) / 60;
    const qint64 seconds = totalSeconds % 60;

    if (m_processingProgressBar) {
        const int percent = m_totalContours > 0
            ? (m_completedContours * 100) / m_totalContours
            : 0;
        m_processingProgressBar->setValue(qBound(0, percent, 100));
    }
    if (m_processingTimeLabel) {
        m_processingTimeLabel->setText(
            QStringLiteral("%1:%2:%3")
                .arg(hours, 2, 10, QLatin1Char('0'))
                .arg(minutes, 2, 10, QLatin1Char('0'))
                .arg(seconds, 2, 10, QLatin1Char('0')));
    }
    if (m_totalContoursLabel)
        m_totalContoursLabel->setText(QString::number(m_totalContours));
    if (m_completedContoursLabel)
        m_completedContoursLabel->setText(QString::number(m_completedContours));
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
