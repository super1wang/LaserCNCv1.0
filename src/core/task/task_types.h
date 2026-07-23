#pragma once

#include <QMetaType>
#include <QString>

using TaskId = int;
constexpr TaskId kInvalidTaskId = -1;

/// Scheduling intent shared by all task producers.  The general pool currently
/// treats these as metadata; the Process device executor consumes the complete
/// ordering (Stop > Workflow > Interactive > Normal > Polling).
enum class TaskPriority {
    Stop,
    Workflow,
    Interactive,
    Normal,
    Polling,
};

/// Observable task lifecycle.  A cancellation request is not completion: a
/// worker must still reach a cooperative boundary before it becomes Cancelled.
enum class TaskExecutionStatus {
    Queued,
    Running,
    CancelRequested,
    Succeeded,
    Cancelled,
    Failed,
    TimedOut,
    Stale,
};

/// Describes one user-visible or internal operation.  Scope is deliberately a
/// string at the core boundary so CAD/CAM/Process can use document/session ids
/// without pulling module types into core.
struct TaskSpec {
    QString label;
    QString scope;
    TaskPriority priority{TaskPriority::Normal};
    bool userVisible{true};
    bool cancellable{true};
};

struct TaskSnapshot {
    TaskId taskId{kInvalidTaskId};
    QString label;
    QString scope;
    QString step;
    QString error;
    TaskPriority priority{TaskPriority::Normal};
    TaskExecutionStatus status{TaskExecutionStatus::Queued};
    int percent{0};
    bool userVisible{true};
    bool cancellable{true};
};

Q_DECLARE_METATYPE(TaskPriority)
Q_DECLARE_METATYPE(TaskExecutionStatus)
