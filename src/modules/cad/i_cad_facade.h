#pragma once

#include "core/kernel/i_service.h"
#include "core/project/project_types.h"

#include <QByteArray>
#include <QString>
#include <QStringList>

class QObject;
class TaskProgress;

namespace lcnc {

struct CadModelEnvelopeRequest
{
    QString sourceModelPath;
    QString generatorPath;
    QString assetRootDirectory;
    /// Optional user-facing AP242 tessellated STEP output. The CAD boundary
    /// publishes it atomically only after validating the complete generation.
    QString simplifiedStepOutputPath;
    double deflectionMm{2.0};
    double angleRad{0.35};
    double alphaMm{10.0};
    double offsetMm{2.5};
    /// Optional per-axis visual-fidelity profile. It still produces a closed,
    /// conservative wrap, but uses a smaller feature threshold and clearance.
    QStringList detailAxes;
    double detailAlphaMm{3.0};
    double detailOffsetMm{0.5};
    int maximumValidationSamples{20'000};
    int progressMinimum{0};
    int progressMaximum{100};
};

struct CadModelEnvelopeResult
{
    QString manifestPath;
    QString simplifiedStepOutputPath;
    QByteArray manifestSha256;
    int bodyCount{0};
};

/**
 * @brief CAD 模块对外门面接口（Phase 7）。
 *
 * 目的：将 UI / 命令对 CAD 子系统的依赖收窄为一组语义接口，避免直接持有
 * @c CadModule* 并 #include 模块内部头。
 *
 * 本接口继承 @ref IService。为避免多重继承中 IService 不明确，CadModule
 * 不再直接继承 IService，而是仅通过本接口带入 IService 身份。
 *
 * 当前接口仅覆盖了 UI 频繁触发的少量动作，未来按需扩充；CadModule 在
 * init() 时通过 ServiceRegistry 同时注册 `ICadFacade` 与具体类型。
 */
class ICadFacade : public IService
{
public:
    ~ICadFacade() override = default;

    /// 用于让调用方挂接信号槽（CadModule 是 QObject）。
    virtual QObject* asQObject() = 0;

    /// 切换 3D 视图至工件工作区。@p id 缺省时使用当前项目文档。
    virtual void requestWorkpieceView(DocumentId id = kInvalidDocumentId) = 0;

    /// 当前项目的工件文档 ID（无项目时返回 @c kInvalidDocumentId）。
    virtual DocumentId workpieceDocumentId() const = 0;

    /// Worker-safe external shrink-wrap boundary. The selected backend remains
    /// an independent process; CAD validates it and atomically publishes a
    /// content-addressed generation before returning.
    virtual bool runModelEnvelopeGenerator(
        const CadModelEnvelopeRequest& request,
        TaskProgress* progress,
        CadModelEnvelopeResult* result,
        QString* errorMessage = nullptr) = 0;
};

} // namespace lcnc
