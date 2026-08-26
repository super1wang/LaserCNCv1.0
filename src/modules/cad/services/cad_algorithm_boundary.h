#pragma once

#include "core/logging/logger.h"

#include <QObject>
#include <QString>
#include <Standard_Failure.hxx>

#include <exception>
#include <functional>
#include <type_traits>
#include <utility>

namespace lcnc::cad {

template <typename Function>
auto invokeCadAlgorithm(Function&& function, QString* errorMessage) noexcept
    -> std::invoke_result_t<Function>
{
    using Result = std::invoke_result_t<Function>;
    try {
        return std::invoke(std::forward<Function>(function));
    } catch (const std::invalid_argument& exception) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "CAD algorithm rejected input: {}",
                 exception.what());
        if (errorMessage)
            // 中文翻译：CAD 操作参数无效: %1
            *errorMessage = QObject::tr("Invalid CAD operation parameters: %1")
                                .arg(QString::fromUtf8(exception.what()));
    } catch (const Standard_Failure& exception) {
        const char* const message = exception.what();
        LCNC_ERR(lcnc::LogCode::Generic,
                 "CAD OCC operation failed: {}",
                 message ? message : "unknown OCC failure");
        if (errorMessage)
            // 中文翻译：CAD 几何运算失败: %1
            *errorMessage = QObject::tr("CAD geometry operation failed: %1")
                                .arg(QString::fromUtf8(message ? message : "unknown OCC failure"));
    } catch (const std::exception& exception) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "CAD operation failed: {}",
                 exception.what());
        if (errorMessage)
            // 中文翻译：CAD 操作失败: %1
            *errorMessage = QObject::tr("CAD operation failed: %1")
                                .arg(QString::fromUtf8(exception.what()));
    } catch (...) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "CAD operation failed with an unknown exception");
        if (errorMessage)
            // 中文翻译：CAD 操作发生未知错误
            *errorMessage = QObject::tr("An unknown error occurred during the CAD operation");
    }
    return Result{};
}

} // namespace lcnc::cad
