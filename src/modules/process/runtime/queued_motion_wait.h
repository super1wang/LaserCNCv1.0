#pragma once

#include "modules/process/runtime/device_command_queue.h"

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QThread>

namespace lcnc::process {

// Called by the workflow thread. Each poll is a separate device command;
// sleep never occupies the SDK executor or prevents Stop/status IO work.
inline DeviceCommandResult waitForQueuedMotion(
    DeviceCommandQueue& queue,
    std::function<DeviceCommandResult(bool&)> poll,
    const std::function<bool()>& cancelled,
    int timeoutMs = -1)
{
    if (queue.isWorkerThread()) {
        // 中文翻译：运动等待不能占用设备执行线程
        return {false, QCoreApplication::translate("DeviceCommandQueue",
            "Motion wait must not occupy the device executor")};
    }
    QElapsedTimer elapsed;
    elapsed.start();
    for (;;) {
        if (cancelled && cancelled()) {
            // 中文翻译：普通切割已被中断
            return {false, QCoreApplication::translate("lcnc::process::NormalCuttingManager",
                "Normal cutting has been interrupted"), DeviceCommandCompletion::Cancelled};
        }
        if (timeoutMs >= 0 && elapsed.elapsed() >= timeoutMs) {
            // 中文翻译：等待绝对运动完成超时
            return {false, QObject::tr("Timed out waiting for absolute motion to complete"),
                DeviceCommandCompletion::TimedOut};
        }
        const auto running = std::make_shared<bool>(false);
        const auto result = queue.executeAndWait(
            [poll, running] { return poll(*running); }, TaskPriority::Workflow, 1000);
        if (!result.success || !*running)
            return result;
        QThread::msleep(10);
    }
}

} // namespace lcnc::process
