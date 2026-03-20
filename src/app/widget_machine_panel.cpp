#include "app/widget_machine_panel.h"

#include <QVBoxLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QLabel>
#include <QComboBox>
#include <QFormLayout>
#include <QLineEdit>

WidgetMachinePanel::WidgetMachinePanel(QWidget* parent)
    : QWidget(parent)
{
    buildUi();
}

void WidgetMachinePanel::buildUi()
{
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(4, 4, 4, 4);
    mainLayout->setSpacing(6);

    // ── 机台模型管理 ──────────────────────────────────────────────────────
    auto* machineGroup = new QGroupBox(tr("机台模型"), this);
    auto* machineLayout = new QVBoxLayout(machineGroup);

    auto* btnLoadMachine = new QPushButton(QIcon(":/icons/machine.svg"),
                                            tr("加载机台模型..."), this);
    auto* lblMachineName = new QLabel(tr("未加载"), this);
    lblMachineName->setAlignment(Qt::AlignCenter);
    lblMachineName->setStyleSheet("color: gray;");

    auto* comboKinematics = new QComboBox(this);
    comboKinematics->addItem(tr("AC 转台（垂直主轴）"), "VERTICAL_AC_TABLE");
    comboKinematics->addItem(tr("BC 转台（垂直主轴）"), "VERTICAL_BC_TABLE");
    comboKinematics->addItem(tr("AB 摆头"),             "AB_HEAD");
    comboKinematics->addItem(tr("AC 摆头"),             "AC_HEAD");

    auto* formLayout = new QFormLayout();
    formLayout->addRow(tr("机台构型:"), comboKinematics);

    machineLayout->addWidget(btnLoadMachine);
    machineLayout->addWidget(lblMachineName);
    machineLayout->addLayout(formLayout);

    // ── 工件模型管理 ──────────────────────────────────────────────────────
    auto* wpcGroup    = new QGroupBox(tr("工件模型"), this);
    auto* wpcLayout   = new QVBoxLayout(wpcGroup);
    auto* btnLoadWpc  = new QPushButton(QIcon(":/icons/workpiece.svg"),
                                         tr("加载工件模型..."), this);
    wpcLayout->addWidget(btnLoadWpc);

    // ── 原点设置 ──────────────────────────────────────────────────────────
    auto* originGroup  = new QGroupBox(tr("加工原点"), this);
    auto* originLayout = new QFormLayout(originGroup);
    for (const QString& ax : {tr("X"), tr("Y"), tr("Z"), tr("A"), tr("C")}) {
        auto* le = new QLineEdit("0.000", this);
        le->setReadOnly(true);
        originLayout->addRow(ax + ":", le);
    }
    auto* btnSetOrigin = new QPushButton(tr("设为加工原点"), this);
    originLayout->addRow(btnSetOrigin);

    mainLayout->addWidget(machineGroup);
    mainLayout->addWidget(wpcGroup);
    mainLayout->addWidget(originGroup);
    mainLayout->addStretch();

    // ── Connections ───────────────────────────────────────────────────────
    connect(btnLoadMachine, &QPushButton::clicked,
            this, &WidgetMachinePanel::loadMachineRequested);
    connect(btnLoadWpc, &QPushButton::clicked,
            this, &WidgetMachinePanel::loadWorkpieceRequested);
    connect(btnSetOrigin, &QPushButton::clicked,
            this, &WidgetMachinePanel::setMachineOriginRequested);
}
