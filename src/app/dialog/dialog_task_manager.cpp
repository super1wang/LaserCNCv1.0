#include "app/dialog/dialog_task_manager.h"
#include "core/kernel/kernel.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QProgressBar>
#include <QLabel>
#include <QPushButton>
#include <QFrame>

DialogTaskManager::DialogTaskManager(QWidget* parent)
    : QDialog(parent, Qt::Tool | Qt::WindowStaysOnTopHint)
{
    setWindowTitle(tr("任务进度"));
    setMinimumWidth(380);

    auto* outer = new QVBoxLayout(this);
    m_rowsLayout = new QVBoxLayout();
    outer->addLayout(m_rowsLayout);
    outer->addStretch();

    auto* mgr = lcnc::Kernel::current().taskManager();
    connect(mgr, &TaskManager::taskStarted,          this, &DialogTaskManager::onTaskStarted);
    connect(mgr, &TaskManager::taskProgressChanged,  this, &DialogTaskManager::onTaskProgressChanged);
    connect(mgr, &TaskManager::taskStepChanged,      this, &DialogTaskManager::onTaskStepChanged);
    connect(mgr, &TaskManager::taskFinished,         this, &DialogTaskManager::onTaskFinished);
}

void DialogTaskManager::onTaskStarted(TaskId id, const QString& label)
{
    addRow(id, label);
    updateVisibility();
}

void DialogTaskManager::onTaskProgressChanged(TaskId id, int percent)
{
    if (m_rows.contains(id))
        m_rows[id].bar->setValue(percent);
}

void DialogTaskManager::onTaskStepChanged(TaskId id, const QString& step)
{
    if (m_rows.contains(id))
        m_rows[id].stepWidget->setText(step);
}

void DialogTaskManager::onTaskFinished(TaskId id, bool /*success*/)
{
    removeRow(id);
    updateVisibility();
}

void DialogTaskManager::addRow(TaskId id, const QString& label)
{
    if (m_rows.contains(id)) return;

    auto* frame  = new QFrame(this);
    frame->setFrameStyle(QFrame::Box | QFrame::Sunken);
    auto* rowLay = new QVBoxLayout(frame);

    auto* topRow = new QHBoxLayout();
    auto* lbl    = new QLabel(label,  this);
    lbl->setStyleSheet("font-weight: bold;");
    auto* abortBtn = new QPushButton(tr("中止"), this);
    abortBtn->setMaximumWidth(55);
    topRow->addWidget(lbl);
    topRow->addStretch();
    topRow->addWidget(abortBtn);
    rowLay->addLayout(topRow);

    auto* bar  = new QProgressBar(this);
    bar->setRange(0, 100);
    bar->setValue(0);
    rowLay->addWidget(bar);

    auto* step = new QLabel(tr("初始化..."), this);
    step->setStyleSheet("color: gray; font-size: 10px;");
    rowLay->addWidget(step);

    m_rowsLayout->addWidget(frame);

    connect(abortBtn, &QPushButton::clicked, this, [id](){
        lcnc::Kernel::current().taskManager()->requestAbort(id);
    });

    TaskRow row;
    row.labelWidget = lbl;
    row.bar         = bar;
    row.stepWidget  = step;
    row.abortBtn    = abortBtn;
    m_rows.insert(id, row);
}

void DialogTaskManager::removeRow(TaskId id)
{
    if (!m_rows.contains(id)) return;
    // Find and remove the parent frame widget
    TaskRow& row = m_rows[id];
    if (row.bar) {
        QWidget* frame = row.bar->parentWidget();
        if (frame) {
            m_rowsLayout->removeWidget(frame);
            frame->deleteLater();
        }
    }
    m_rows.remove(id);
    adjustSize();
}

void DialogTaskManager::updateVisibility()
{
    if (m_rows.isEmpty())
        hide();
    else
        show();
}
