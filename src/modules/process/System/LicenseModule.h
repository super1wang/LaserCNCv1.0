#pragma once

#include "LockValidator.h"

#include <QObject>
#include <QCoreApplication>
#include <QDate>
#include <QDateTime>
#include <QString>
#include <memory>

class QTimer;
class VirboxTool;

// 统一许可证守护类：优先使用 info.enc + license.enc，本地文件不存在时回退 Virbox。
class LicenseModule : public QObject
{
    Q_OBJECT

public:
    enum LicenseMode
    {
        LICENSE_MODE_NONE = 0,
        LICENSE_MODE_ENC = 1,
        LICENSE_MODE_VIRBOX = 2
    };

    explicit LicenseModule(QObject *parent = nullptr);
    ~LicenseModule();

    // 启动许可证校验流程，并开启定时检测。
    bool Start();
    void Stop();

    // 立即执行一次校验，返回当前模式下是否通过。
    bool CheckNow();

    // 检查程序目录下是否同时存在 info.enc 与 license.enc。
    bool HasRequiredEncFiles() const;

    LicenseMode CurrentMode() const { return m_mode; }
    bool IsUsingEncLicense() const { return m_mode == LICENSE_MODE_ENC; }

    // 当前许可证等级：Virbox 模式由 VirboxTool 返回。
    int CurrentLicenseLevel() const { return m_licenseLevel; }

    // 扩展字段 Simulat:0/1 的统一读取接口。
    bool IsSimulatModeEnabled() const { return m_simulatModeEnabled; }

    int LastValidateResult() const { return m_lastResult; }
    QDateTime LastDueDateTime() const { return m_lastDueDateTime; }
    QString LastErrorMessage() const { return m_lastErrorMessage; }

signals:
    void sigLicenseCheckFailed(int result, const QString &message);

private:
    QString LicenseDirPath() const { return QCoreApplication::applicationDirPath(); }

    bool CheckEncNow();
    bool CheckVirboxNow();
    bool EnsureVirboxReady();

    void ReportError(const QString &message);

    static std::wstring ToWString(const QString &text);

private:
    QTimer *m_timer;

    LicenseMode m_mode;
    int m_licenseLevel;
    bool m_simulatModeEnabled;

    int m_lastResult;
    QDateTime m_lastDueDateTime;
    QString m_lastErrorMessage;

    QDate m_lastVirboxTimeCheckDate;
    std::unique_ptr<VirboxTool> m_virboxTool;
    bool m_virboxInitialized;
};
