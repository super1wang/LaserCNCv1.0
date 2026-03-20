#include "app/widget_laser_control.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QLabel>
#include <QSlider>
#include <QGridLayout>
#include <QTimer>
#include <QFrame>
#include <cmath>

WidgetLaserControl::WidgetLaserControl(QWidget* parent)
    : QWidget(parent)
{
    buildUi();

    // ── Simulation timer (Phase 1 placeholder) ────────────────────────────
    m_simTimer = new QTimer(this);
    m_simTimer->setInterval(200);       // 200 ms refresh
    connect(m_simTimer, &QTimer::timeout, this, &WidgetLaserControl::onSimTick);
    m_simTimer->start();
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
    m_statusLabel = new QLabel(tr("仿真模式 — 未连接"), this);
    m_statusLabel->setAlignment(Qt::AlignCenter);
    m_statusLabel->setStyleSheet(
        "background: #333; color: #AAFFAA; padding: 2px 4px; border-radius: 3px;");
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
                this, [this, ax]{ emit jogRequested(ax, +1, 1); });
        connect(btnMinus, &QPushButton::clicked,
                this, [this, ax]{ emit jogRequested(ax, -1, 1); });

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
            [labelPct](int v){ labelPct->setText(QString::number(v) + "%"); });
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
    m_statusLabel->setText(connected ? tr("已连接") : tr("仿真模式 — 未连接"));
    m_statusLabel->setStyleSheet(
        connected ? "background: #1B5E20; color: #00FF88; padding: 2px 4px; border-radius: 3px;"
                  : "background: #333; color: #AAFFAA; padding: 2px 4px; border-radius: 3px;");
}

void WidgetLaserControl::updateSystemStatus(const QString& status)
{
    m_statusLabel->setText(status);
}

// ── Simulation tick ────────────────────────────────────────────────────────────
void WidgetLaserControl::onSimTick()
{
    static double t = 0.0;
    t += 0.05;
    const QStringList axes = {"X", "Y", "Z", "A", "C"};
    for (int i = 0; i < axes.size(); ++i)
        updateAxisPosition(axes[i], std::sin(t + i) * 10.0);
}
