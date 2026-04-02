#include "app/widget_laser_control.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QButtonGroup>
#include <QPushButton>
#include <QLabel>
#include <QSlider>
#include <QGridLayout>
#include <QFrame>

WidgetLaserControl::WidgetLaserControl(QWidget* parent)
    : QWidget(parent)
{
    buildUi();
    refreshStatusBanner();
}

void WidgetLaserControl::buildUi()
{
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(4, 4, 4, 4);
    mainLayout->setSpacing(6);

    buildAxisGroup();
    buildJogGroup();
    buildProcessGroup();

    mainLayout->addStretch();

    // ── Status bar ────────────────────────────────────────────────────────
    m_statusLabel = new QLabel(this);
    m_statusLabel->setAlignment(Qt::AlignCenter);
    mainLayout->addWidget(m_statusLabel);
}

void WidgetLaserControl::buildAxisGroup()
{
    auto* group  = new QGroupBox(tr("轴位置"), this);
    auto* grid   = new QGridLayout(group);
    const QStringList axes = {"X", "Y", "Z", "A", "C"};
    int row = 0;
    for (const QString& ax : axes) {
        auto* lbl  = new QLabel(ax + ":", this);
        auto* val  = new QLabel("  0.000", this);
        val->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        val->setMinimumWidth(70);
        val->setStyleSheet("font-family: Consolas, monospace; color: #00FF88;");
        auto* unit = new QLabel("mm", this);
        grid->addWidget(lbl,  row, 0);
        grid->addWidget(val,  row, 1);
        grid->addWidget(unit, row, 2);
        m_posLabels[ax] = val;
        ++row;
    }

    this->layout()->addWidget(group);
}

void WidgetLaserControl::buildJogGroup()
{
    auto* group  = new QGroupBox(tr("手动点动"), this);
    auto* vlay   = new QVBoxLayout(group);

    // Speed selector
    auto* speedRow = new QHBoxLayout();
    speedRow->addWidget(new QLabel(tr("速度:"), this));
    auto* btnSlow = new QPushButton(tr("慢"), this);
    auto* btnMed  = new QPushButton(tr("中"), this);
    auto* btnFast = new QPushButton(tr("快"), this);
    btnSlow->setCheckable(true);
    btnMed->setCheckable(true);
    btnFast->setCheckable(true);
    btnMed->setChecked(true);
        auto* speedGroup = new QButtonGroup(this);
        speedGroup->setExclusive(true);
        speedGroup->addButton(btnSlow, 0);
        speedGroup->addButton(btnMed, 1);
        speedGroup->addButton(btnFast, 2);
        connect(speedGroup, &QButtonGroup::idClicked,
            this, [this](int id) { m_jogSpeedLevel = id; });
    for (auto* b : {btnSlow, btnMed, btnFast}) b->setMaximumWidth(40);
    speedRow->addWidget(btnSlow);
    speedRow->addWidget(btnMed);
    speedRow->addWidget(btnFast);
    speedRow->addStretch();
    vlay->addLayout(speedRow);

    // Jog buttons for each axis
    auto* jogGrid = new QGridLayout();
    const QStringList axes = {"X", "Y", "Z", "A", "C"};
    int row = 0;
    for (const QString& ax : axes) {
        auto* lblAx = new QLabel(ax, this);
        lblAx->setAlignment(Qt::AlignCenter);
        auto* btnPlus  = new QPushButton("▲ +", this);
        auto* btnMinus = new QPushButton("▼ -", this);
        btnPlus->setFixedWidth(55);
        btnMinus->setFixedWidth(55);

        connect(btnPlus,  &QPushButton::clicked,
            this, [this, ax]{ emit jogRequested(ax, +1, m_jogSpeedLevel); });
        connect(btnMinus, &QPushButton::clicked,
            this, [this, ax]{ emit jogRequested(ax, -1, m_jogSpeedLevel); });

        jogGrid->addWidget(lblAx,    row, 0);
        jogGrid->addWidget(btnPlus,  row, 1);
        jogGrid->addWidget(btnMinus, row, 2);
        ++row;
    }
    vlay->addLayout(jogGrid);

    // Home button
    auto* btnHome = new QPushButton(tr("归零 (Home)"), this);
    btnHome->setIcon(QIcon(":/icons/home.svg"));
    connect(btnHome, &QPushButton::clicked, this, &WidgetLaserControl::homeRequested);
    vlay->addWidget(btnHome);

    this->layout()->addWidget(group);
}

void WidgetLaserControl::buildProcessGroup()
{
    auto* group = new QGroupBox(tr("加工控制"), this);
    auto* vlay  = new QVBoxLayout(group);

    auto* row1 = new QHBoxLayout();
    auto* btnStart = new QPushButton(QIcon(":/icons/start.svg"), tr("运行"), this);
    auto* btnPause = new QPushButton(QIcon(":/icons/pause.svg"), tr("暂停"), this);
    btnStart->setStyleSheet("background-color: #2E7D32; color: white;");
    btnPause->setStyleSheet("background-color: #E65100; color: white;");
    row1->addWidget(btnStart);
    row1->addWidget(btnPause);
    vlay->addLayout(row1);

    auto* btnStop  = new QPushButton(QIcon(":/icons/stop.svg"), tr("停止"), this);
    vlay->addWidget(btnStop);

    auto* btnEStop = new QPushButton(tr("E-STOP"), this);
    btnEStop->setMinimumHeight(50);
    btnEStop->setStyleSheet(
        "background-color: #B71C1C; color: white; "
        "font-weight: bold; font-size: 16px; border-radius: 6px;");
    vlay->addWidget(btnEStop);

    // Feed override slider
    auto* feedRow = new QHBoxLayout();
    feedRow->addWidget(new QLabel(tr("进给倍率:"), this));
    auto* slider = new QSlider(Qt::Horizontal, this);
    slider->setRange(0, 200);
    slider->setValue(100);
    auto* labelPct = new QLabel("100%", this);
    labelPct->setMinimumWidth(35);
    connect(slider, &QSlider::valueChanged, this,
            [this, labelPct](int v) {
            labelPct->setText(QString::number(v) + "%");
            emit feedOverrideChanged(static_cast<double>(v) / 100.0);
            });
    feedRow->addWidget(slider);
    feedRow->addWidget(labelPct);
    vlay->addLayout(feedRow);

    connect(btnStart, &QPushButton::clicked, this, &WidgetLaserControl::startRequested);
    connect(btnPause, &QPushButton::clicked, this, &WidgetLaserControl::pauseRequested);
    connect(btnStop,  &QPushButton::clicked, this, &WidgetLaserControl::stopRequested);
    connect(btnEStop, &QPushButton::clicked, this, &WidgetLaserControl::eStopRequested);

    this->layout()->addWidget(group);
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

void WidgetLaserControl::refreshStatusBanner()
{
    const QString fallback = m_simulationMode
        ? (m_connected ? tr("仿真模式 — 控制器已连接") : tr("仿真模式 — 未连接"))
        : (m_connected ? tr("控制器模式 — 已连接") : tr("控制器模式 — 未连接"));
    const QString text = m_statusText.isEmpty() ? fallback : m_statusText;

    QString style = QStringLiteral("background: #333; color: #AAFFAA; padding: 2px 4px; border-radius: 3px;");
    if (text.contains(tr("急停"))) {
        style = QStringLiteral("background: #7F1D1D; color: white; padding: 2px 4px; border-radius: 3px;");
    } else if (text.contains(tr("错误")) || text.contains(tr("无法"))) {
        style = QStringLiteral("background: #92400E; color: white; padding: 2px 4px; border-radius: 3px;");
    } else if (text.contains(tr("暂停"))) {
        style = QStringLiteral("background: #B45309; color: white; padding: 2px 4px; border-radius: 3px;");
    } else if (m_connected && !m_simulationMode) {
        style = QStringLiteral("background: #1B5E20; color: #00FF88; padding: 2px 4px; border-radius: 3px;");
    }

    m_statusLabel->setText(text);
    m_statusLabel->setStyleSheet(style);
}
