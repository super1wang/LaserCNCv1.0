#pragma once

#include <QDialog>
#include <QMap>
#include "core/task/task_manager.h"

class QLabel;
class QProgressBar;
class QPushButton;
class QVBoxLayout;

/**
 * @brief Floating dialog that shows running task progress bars.
 *
 * Automatically shows itself when a task starts and hides when all tasks
 * have finished.  Individual tasks can be cancelled via the Abort button.
 */
class DialogTaskManager : public QDialog
{
    Q_OBJECT
public:
    explicit DialogTaskManager(QWidget* parent = nullptr);

private slots:
    void onTaskStarted(TaskId id, const QString& label);
    void onTaskProgressChanged(TaskId id, int percent);
    void onTaskStepChanged(TaskId id, const QString& step);
    void onTaskFinished(TaskId id, bool success);

private:
    struct TaskRow {
        QLabel*      labelWidget{nullptr};
        QProgressBar* bar{nullptr};
        QLabel*      stepWidget{nullptr};
        QPushButton* abortBtn{nullptr};
    };

    QVBoxLayout*         m_rowsLayout{nullptr};
    QMap<TaskId, TaskRow> m_rows;

    void addRow(TaskId id, const QString& label);
    void removeRow(TaskId id);
    void updateVisibility();
};
