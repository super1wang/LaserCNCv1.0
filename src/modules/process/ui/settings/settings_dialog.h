#pragma once
// ── Unified settings dialog using UIC-generated pages from Setting_*.ui ──
// Pure Qt6, no old code dependencies.  .ui files remain editable in Designer.

#include <QDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDialogButtonBox>
#include <QTreeWidget>
#include <QStackedWidget>
#include <QScrollArea>
#include <QPushButton>
#include <QHash>

// UIC-generated headers
#include "ui_Setting_MotionControl.h"
#include "ui_Setting_Axis.h"
#include "ui_Setting_IOIndex.h"
#include "ui_Setting_Digital.h"
#include "ui_Setting_Analog.h"
#include "ui_Setting_Laser.h"
#include "ui_Setting_Internet.h"
#include "ui_Setting_Tool.h"
#include "ui_Setting_Gas.h"
#include "ui_Setting_Water.h"
#include "ui_Setting_Monitor.h"
#include "ui_Setting_LoadingPos.h"
#include "ui_Setting_Camera.h"

// ── Thin page wrappers ──────────────────────────────────────────────────
#define MAKE_PAGE(ClassName, UiClass)                                \
class ClassName : public QDialog {                                   \
public:                                                              \
    explicit ClassName(QWidget* p = nullptr) : QDialog(p) { ui.setupUi(this); } \
    UiClass ui;                                                      \
};

MAKE_PAGE(PgMotionControl, Ui::Dialog_Setting_MotionControl)
MAKE_PAGE(PgAxis,         Ui::Dialog_Setting_Axis)
MAKE_PAGE(PgIOIndex,      Ui::Dialog_Setting_IOIndex)
MAKE_PAGE(PgDigital,      Ui::Dialog_Setting_Digital)
MAKE_PAGE(PgAnalog,       Ui::Dialog_Setting_Analog)
MAKE_PAGE(PgLaser,        Ui::Dialog_Setting_Laser)
MAKE_PAGE(PgInternet,     Ui::Dialog_Setting_Internet)
MAKE_PAGE(PgTool,         Ui::Dialog_Setting_Tool)
MAKE_PAGE(PgGas,          Ui::Dialog_Setting_Gas)
MAKE_PAGE(PgWater,        Ui::Dialog_Setting_Water)
MAKE_PAGE(PgMonitor,      Ui::Dialog_Setting_Monitor)
MAKE_PAGE(PgLoadingPos,   Ui::Dialog_Setting_LoadingPos)
MAKE_PAGE(PgCamera,       Ui::Dialog_Setting_Camera)
#undef MAKE_PAGE

// ── Dialog ──────────────────────────────────────────────────────────────
class SettingsDialog : public QDialog
{
    Q_OBJECT
public:
    explicit SettingsDialog(QWidget* parent = nullptr)
        : QDialog(parent)
    {
        setWindowTitle(tr("参数设置")); resize(980, 640);
        auto* root = new QVBoxLayout(this);
        auto* body = new QHBoxLayout();
        m_tree = new QTreeWidget(this);
        m_tree->setMinimumWidth(200); m_tree->setMaximumWidth(260); m_tree->setHeaderHidden(true);
        m_stack = new QStackedWidget(this);
        body->addWidget(m_tree); body->addWidget(m_stack, 1); root->addLayout(body, 1);
        auto* btnRow = new QHBoxLayout();
        auto *apply=new QPushButton(tr("Apply"),this), *ok=new QPushButton(tr("OK"),this), *cancel=new QPushButton(tr("Cancel"),this);
        btnRow->addStretch(); btnRow->addWidget(apply); btnRow->addWidget(ok); btnRow->addWidget(cancel);
        root->addLayout(btnRow);
        connect(apply,&QPushButton::clicked,this,&SettingsDialog::onApply);
        connect(ok,&QPushButton::clicked,this,[this]{ onApply(); accept(); });
        connect(cancel,&QPushButton::clicked,this,&QDialog::reject);
        buildPages();
        connect(m_tree,&QTreeWidget::currentItemChanged,this,&SettingsDialog::onPageSelected);
        if (m_tree->topLevelItemCount()>0 && m_tree->topLevelItem(0)->childCount()>0)
            m_tree->setCurrentItem(m_tree->topLevelItem(0)->child(0));
    }

private:
    void buildPages() {
        auto add=[this](QTreeWidgetItem* parent,const QString& title,QWidget* page){
            auto* item=new QTreeWidgetItem(parent,{title});
            auto* scroll=new QScrollArea(this); scroll->setWidgetResizable(true); scroll->setWidget(page);
            m_stack->addWidget(scroll); m_map[item]=scroll; return item;
        };
        auto* ext=new QTreeWidgetItem(m_tree,{tr("外设")});
        auto* proc=new QTreeWidgetItem(m_tree,{tr("加工")});
        add(ext,tr("运动控制器"),new PgMotionControl(this));
        add(ext,tr("轴参数"),    new PgAxis(this));
        add(ext,tr("IO索引"),    new PgIOIndex(this));
        add(ext,tr("数字IO"),    new PgDigital(this));
        add(ext,tr("模拟IO"),    new PgAnalog(this));
        add(ext,tr("激光器"),    new PgLaser(this));
        add(ext,tr("互联网"),    new PgInternet(this));
        add(proc,tr("工具"),     new PgTool(this));
        add(proc,tr("气体"),     new PgGas(this));
        add(proc,tr("水冷"),     new PgWater(this));
        add(proc,tr("监控"),     new PgMonitor(this));
        add(proc,tr("上料位"),   new PgLoadingPos(this));
        add(proc,tr("相机"),     new PgCamera(this));
        m_tree->expandAll();
    }
    void onPageSelected(QTreeWidgetItem* cur, QTreeWidgetItem*) {
        if (!cur) return;
        auto it=m_map.find(cur);
        if (it!=m_map.end()) m_stack->setCurrentWidget(it.value());
    }
    void onApply() { /* Future: iterate pages and write to SETTINGS/Service */ }

    QTreeWidget* m_tree{nullptr};
    QStackedWidget* m_stack{nullptr};
    QHash<QTreeWidgetItem*, QWidget*> m_map;
};

inline bool openSettingsDialog() { SettingsDialog d; return d.exec()==QDialog::Accepted; }
