#include "LicenseModule.h"

#include "VirboxTool.h"

#include <QCoreApplication>
#include <QDate>
#include <QRandomGenerator>
#include <QTimer>
#include <QFileInfo>
#include <QDir>

namespace
{
static bool ParseSimulatFlag(const std::wstring &wstrExtendedFunction, bool &bEnabled, QString &qstrWarn)
{
    bEnabled = false;
    qstrWarn.clear();
    if (wstrExtendedFunction.empty())
    {
        return false;
    }

    bool bFound = false;
    size_t iStart = 0;
    while (iStart <= wstrExtendedFunction.size())
    {
        size_t iEnd = wstrExtendedFunction.find(L'|', iStart);
        std::wstring wstrToken;
        if (iEnd == std::wstring::npos)
        {
            wstrToken = wstrExtendedFunction.substr(iStart);
            iStart = wstrExtendedFunction.size() + 1;
        }
        else
        {
            wstrToken = wstrExtendedFunction.substr(iStart, iEnd - iStart);
            iStart = iEnd + 1;
        }

        if (wstrToken.empty())
        {
            continue;
        }

        size_t iSign = wstrToken.find(L':');
        if (iSign == std::wstring::npos)
        {
            continue;
        }

        const std::wstring wstrKey = wstrToken.substr(0, iSign);
        const std::wstring wstrValue = wstrToken.substr(iSign + 1);
        if (wstrKey != L"Simulat")
        {
            continue;
        }

        bFound = true;
        if (wstrValue == L"1")
        {
            bEnabled = true;
        }
        else if (wstrValue == L"0")
        {
            bEnabled = false;
        }
        else
        {
            bEnabled = false;
            qstrWarn = QString::fromUtf8("Simulat 字段取值非法，已按 false 处理。");
        }
    }

    return bFound;
}
}

LicenseModule::LicenseModule(QObject *parent)
    : QObject(parent)
    , m_timer(new QTimer(this))
    , m_mode(LICENSE_MODE_NONE)
    , m_licenseLevel(0)
    , m_simulatModeEnabled(false)
    , m_lastResult(PERMISSION_DENIED)
    , m_virboxInitialized(false)
{
    QObject::connect(m_timer, &QTimer::timeout, [this]() {
        this->CheckNow();
    });
}

LicenseModule::~LicenseModule()
{
    Stop();
}

bool LicenseModule::Start()
{
    m_lastErrorMessage.clear();
    m_simulatModeEnabled = false;
    m_licenseLevel = 0;
	int iCheckIntervalMs = 5 * 1000;

	// 硬锁检测一次许可证等级，如果为调试锁，则继续使用硬锁模式
    bool bOk = CheckVirboxNow();
    if (m_virboxTool)
    	m_licenseLevel = m_virboxTool->GetLicenseLevel();
    if (m_licenseLevel > 6 && bOk)
    {
		m_mode = LICENSE_MODE_VIRBOX;
		m_timer->start(iCheckIntervalMs);
        return true;
    }

    if (HasRequiredEncFiles())
    {
        // 软锁每五分钟检查一次
        m_mode = LICENSE_MODE_ENC;
        iCheckIntervalMs = 5 * 60 * 1000;
    }
    else
    {
        m_mode = LICENSE_MODE_VIRBOX;
    }

    if (!CheckNow())
        return false;

    m_timer->start(iCheckIntervalMs);
    return true;
}

void LicenseModule::Stop()
{
    if (m_timer != NULL)
    {
        m_timer->stop();
    }
}

bool LicenseModule::CheckNow()
{
    bool bOk = false;
    if (m_mode == LICENSE_MODE_ENC)
    {
        bOk = CheckEncNow();
    }
    else if (m_mode == LICENSE_MODE_VIRBOX)
    {
        bOk = CheckVirboxNow();
    }
    else
    {
        ReportError(QString::fromUtf8("无授权方式"));
        m_lastResult = PERMISSION_DENIED;
        bOk = false;
    }

    if (!bOk)
    {
        emit sigLicenseCheckFailed(m_lastResult, m_lastErrorMessage);
    }

    return bOk;
}

bool LicenseModule::HasRequiredEncFiles() const
{
    const QDir dir(LicenseDirPath());
    const QString infoPath = dir.filePath("info.enc");
    const QString licensePath = dir.filePath("license.enc");
    return QFileInfo::exists(infoPath) && QFileInfo::exists(licensePath);
}

bool LicenseModule::CheckEncNow()
{
    const QString path = LicenseDirPath();

    const long rnd = static_cast<long>(QRandomGenerator::global()->bounded(100000, 999999));
    long check = 0;
    time_t dueTime = 0;
    std::wstring functionText;
    HardwareIdSource source = HARDWARE_ID_UNKNOWN;

    m_lastResult = stValidate(ToWString(path), rnd, check, dueTime, functionText, source);
    m_lastDueDateTime = QDateTime::fromSecsSinceEpoch(static_cast<qint64>(dueTime));
    m_simulatModeEnabled = false;

    if (m_lastResult == SUCCESS)
    {
        QString qstrWarn;
        ParseSimulatFlag(functionText, m_simulatModeEnabled, qstrWarn);
        m_lastErrorMessage.clear();
        return true;
    }

    if (m_lastResult == OVERDUE)
    {
        ReportError(QString::fromUtf8("授权已到期。"));
    }
    else
    {
        ReportError(QString::fromUtf8("授权校验失败。"));
    }
    return false;
}

bool LicenseModule::EnsureVirboxReady()
{
    if (!m_virboxTool)
    {
        m_virboxTool.reset(new VirboxTool(nullptr));
    }

    if (!m_virboxInitialized)
    {
        if (!m_virboxTool->VirboxToolManager())
        {
            m_virboxInitialized = false;
            ReportError(QString::fromUtf8("Virbox 授权初始化失败。"));
            return false;
        }
        m_virboxInitialized = true;
    }

    return true;
}

bool LicenseModule::CheckVirboxNow()
{
    m_simulatModeEnabled = false;

    if (!EnsureVirboxReady())
    {
        m_lastResult = LICENSE_INVALID;
        return false;
    }

    if (m_virboxTool->GetVirboxException())
    {
        ReportError(QString::fromUtf8("授权校验失败。"));

        m_virboxInitialized = false;
        if (!m_virboxTool->VirboxToolManager())
        {
            m_lastResult = LICENSE_INVALID;
            return false;
        }
        m_virboxInitialized = true;
    }

    const QDate today = QDate::currentDate();
    if (m_lastVirboxTimeCheckDate != today)
    {
        if (!m_virboxTool->VirboxAuthorizationTimeCheck())
        {
            ReportError(QString::fromUtf8("授权校验失败。"));
            m_lastResult = TIME_FALSIFIED;
            return false;
        }
        m_lastVirboxTimeCheckDate = today;
    }

    int daysLeft = -1;
    QDateTime dueDateTime;
    if (m_virboxTool->QueryRemainingValidity(daysLeft, dueDateTime) && dueDateTime.isValid())
    {
        if (daysLeft < 0)
        {
            ReportError(QString::fromUtf8("授权已到期。"));
            m_lastResult = OVERDUE;
            return false;
        }
    }

    m_lastErrorMessage.clear();
    m_lastResult = SUCCESS;
    return true;
}

void LicenseModule::ReportError(const QString &message)
{
    m_lastErrorMessage = message;
}

std::wstring LicenseModule::ToWString(const QString &text)
{
    return text.toStdWString();
}
