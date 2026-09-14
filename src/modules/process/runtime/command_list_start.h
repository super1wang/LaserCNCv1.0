#pragma once

namespace lcnc::process {

enum class CommandListStartState {
    Started,
    Cancelled,
    FinalizeFailed,
    StartFailed,
};

struct CommandListStartResult {
    CommandListStartState state{CommandListStartState::Cancelled};
    short apiResult{0};
    int finalizeAttempts{0};
};

// Keep the cancellation checks beside the submission boundary, not only in
// the point-building loop. In particular, a successful finalization or a
// diagnostic read can finish after the operator has requested Stop.
// 中文翻译：封表成功或诊断读取结束时可能已收到停止请求，启动边界必须重新检查。
template<class Cancelled, class Finalize, class PrepareStart, class Start,
         class RetryAllowed, class Wait>
CommandListStartResult startCancellableCommandList(
    Cancelled cancelled, Finalize finalize, PrepareStart prepareStart,
    Start start, RetryAllowed retryAllowed, Wait wait, short pendingResult)
{
    CommandListStartResult result;
    for (;;) {
        if (cancelled())
            return result;
        ++result.finalizeAttempts;
        result.apiResult = finalize();
        if (result.apiResult != 0 && result.apiResult != pendingResult) {
            result.state = CommandListStartState::FinalizeFailed;
            return result;
        }
        if (cancelled())
            return result;
        if (result.apiResult == 0)
            break;
        if (!retryAllowed()) {
            result.state = CommandListStartState::FinalizeFailed;
            return result;
        }
        wait();
    }
    prepareStart();
    if (cancelled())
        return result;
    // A vendor call already entered is not preemptible. The caller must retain
    // the Stop request and execute the queued hardware stop after it returns.
    // 中文翻译：已进入的厂商调用不可抢占；停止请求须保留，并在返回后执行硬件停止。
    result.apiResult = start();
    result.state = result.apiResult == 0
        ? CommandListStartState::Started : CommandListStartState::StartFailed;
    return result;
}

} // namespace lcnc::process
