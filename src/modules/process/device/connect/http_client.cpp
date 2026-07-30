#include "http_client.h"
#include <QCoreApplication>
#include <QEvent>
#include <QEventLoop>
#include <QMetaObject>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QThread>
#include <QTimer>
#include <QUrl>
#include "core/logging/logger.h"

namespace
{
	constexpr int kDefaultFastResponseWindowMs = 50;
	const char* kBackgroundHandledProperty = "HTTPClientBackgroundHandled";

	QString CompletionToText(HTTPClient::RequestCompletion completion)
	{
		switch (completion)
		{
		case HTTPClient::RequestCompletion::VerifiedResponse:
			return QStringLiteral("VerifiedResponse");
		case HTTPClient::RequestCompletion::SentWithoutResponse:
			return QStringLiteral("SentWithoutResponse");
		case HTTPClient::RequestCompletion::SentIgnoringResponse:
			return QStringLiteral("SentIgnoringResponse");
		case HTTPClient::RequestCompletion::Failed:
			return QStringLiteral("Failed");
		case HTTPClient::RequestCompletion::None:
		default:
			return QStringLiteral("None");
		}
	}
}

HTTPClient::HTTPClient(QObject* parent)
	: QObject(parent)
	, m_pManager(new QNetworkAccessManager(this))
	, m_iTimeout(1000)
	, m_iLastStatusCode(0)
	, m_eLastCompletion(RequestCompletion::None)
{}

HTTPClient::~HTTPClient()
{}

bool HTTPClient::Get(const QString& url, QByteArray& response)
{
	return Request("GET", url, QByteArray(), response, QString(), ResponsePolicy::RequireResponse,
		0, true);
}

bool HTTPClient::GetSilent(const QString& url, QByteArray& response)
{
	return Request("GET", url, QByteArray(), response, QString(), ResponsePolicy::RequireResponse,
		0, false);
}

bool HTTPClient::Post(const QString& url, const QByteArray& data, QByteArray& response, const QString& contentType)
{
	return Request("POST", url, data, response, contentType, ResponsePolicy::RequireResponse,
		0, true);
}

bool HTTPClient::Put(const QString& url, const QByteArray& data, QByteArray& response,
	const QString& contentType, bool bEnableLog)
{
	return Request("PUT", url, data, response, contentType, ResponsePolicy::RequireResponse,
		0, bEnableLog);
}

bool HTTPClient::PostFast(const QString& url, const QByteArray& data,
	const QString& contentType, int responseWindowMs, bool bEnableLog)
{
	QByteArray response;
	return Request("POST", url, data, response, contentType,
		ResponsePolicy::AcceptNoResponseAfterTimeout, responseWindowMs, bEnableLog);
}

bool HTTPClient::PutFast(const QString& url, const QByteArray& data,
	const QString& contentType, int responseWindowMs, bool bEnableLog)
{
	QByteArray response;
	return Request("PUT", url, data, response, contentType,
		ResponsePolicy::AcceptNoResponseAfterTimeout, responseWindowMs, bEnableLog);
}

bool HTTPClient::PostSendOnly(const QString& url, const QByteArray& data,
	const QString& contentType, bool bEnableLog)
{
	QByteArray response;
	return Request("POST", url, data, response, contentType,
		ResponsePolicy::IgnoreResponseImmediately, 0, bEnableLog);
}

bool HTTPClient::PutSendOnly(const QString& url, const QByteArray& data,
	const QString& contentType, bool bEnableLog)
{
	QByteArray response;
	return Request("PUT", url, data, response, contentType,
		ResponsePolicy::IgnoreResponseImmediately, 0, bEnableLog);
}

void HTTPClient::SetTimeout(int timeoutMs)
{
	m_iTimeout = timeoutMs > 0 ? timeoutMs : 1000;
}

int HTTPClient::GetTimeout() const
{
	return m_iTimeout;
}

int HTTPClient::GetLastStatusCode() const
{
	return m_iLastStatusCode;
}

QByteArray HTTPClient::GetLastResponse() const
{
	return m_baLastResponse;
}

QString HTTPClient::GetLastError() const
{
	return m_qstrLastError;
}

HTTPClient::RequestCompletion HTTPClient::GetLastCompletion() const
{
	return m_eLastCompletion;
}

QString HTTPClient::GetLastCompletionText() const
{
	return CompletionToText(m_eLastCompletion);
}

bool HTTPClient::Request(const QString& method, const QString& url, const QByteArray& data,
	QByteArray& response, const QString& contentType, ResponsePolicy responsePolicy,
	int responseWindowMs, bool bEnableLog)
{
	if (QThread::currentThread() == thread())
		return DoRequest(method, url, data, response, contentType, responsePolicy, responseWindowMs,
			bEnableLog);

	if (responsePolicy == ResponsePolicy::IgnoreResponseImmediately)
	{
		response.clear();
		ResetLastResult();
		SetLastResult(RequestCompletion::SentIgnoringResponse);

		const bool invoked = QMetaObject::invokeMethod(
			this,
			[this, method, url, data, contentType, responsePolicy, responseWindowMs, bEnableLog]() {
				QByteArray ignoredResponse;
				DoRequest(method, url, data, ignoredResponse, contentType, responsePolicy,
					responseWindowMs, bEnableLog);
			},
			Qt::QueuedConnection);

		if (!invoked)
		{
			response.clear();
			ResetLastResult();
			m_qstrLastError = tr("Failed to queue HTTP request on the target thread.");
			m_eLastCompletion = RequestCompletion::Failed;
			if (bEnableLog)
			{
				LCNC_ERR(lcnc::LogCode::Generic, "{}", "HTTP queue failed, method: " + method.toStdString()
					+ ", url: " + url.toStdString());
			}
			return false;
		}

		return true;
	}

	struct RequestResult
	{
		bool success = false;
		QByteArray response;
	};

	RequestResult result;
	bool invoked = QMetaObject::invokeMethod(
		this,
		[this, &result, method, url, data, contentType, responsePolicy, responseWindowMs, bEnableLog]() {
			result.success = DoRequest(method, url, data, result.response, contentType,
				responsePolicy, responseWindowMs, bEnableLog);
		},
		Qt::BlockingQueuedConnection);

	if (!invoked)
	{
		response.clear();
		ResetLastResult();
		m_qstrLastError = tr("Failed to invoke HTTP request on the target thread.");
		m_eLastCompletion = RequestCompletion::Failed;
		if (bEnableLog)
			LCNC_ERR(lcnc::LogCode::Generic, "{}", "HTTP invoke failed, method: " + method.toStdString() + ", url: " + url.toStdString());
		return false;
	}

	response = result.response;
	return result.success;
}

bool HTTPClient::DoRequest(const QString& method, const QString& url, const QByteArray& data,
	QByteArray& response, const QString& contentType, ResponsePolicy responsePolicy,
	int responseWindowMs, bool bEnableLog)
{
	ResetLastResult();
	response.clear();

	QUrl qurl = QUrl::fromUserInput(url.trimmed());
	const QString qstrScheme = qurl.scheme().toLower();
	if (!qurl.isValid() || (qstrScheme != "http" && qstrScheme != "https") || qurl.host().isEmpty())
	{
		SetLastResult(RequestCompletion::Failed, 0, QByteArray(), tr("Invalid HTTP url: %1").arg(url));
		if (bEnableLog)
			LCNC_ERR(lcnc::LogCode::Generic, "{}", "HTTP invalid url: " + url.toStdString());
		return false;
	}

	QNetworkRequest request(qurl);
	if ((method == "POST" || method == "PUT") && !contentType.isEmpty())
		request.setHeader(QNetworkRequest::ContentTypeHeader, contentType);

	if (bEnableLog)
		LCNC_INFO(lcnc::LogCode::Generic, "{}", "HTTP " + method.toStdString() + " request: " + qurl.toString().toStdString());

	QNetworkReply* reply = DispatchRequest(method, request, data);
	if (!reply)
	{
		SetLastResult(RequestCompletion::Failed, 0, QByteArray(), tr("Unsupported HTTP method: %1").arg(method));
		if (bEnableLog)
			LCNC_ERR(lcnc::LogCode::Generic, "{}", "HTTP unsupported method: " + method.toStdString());
		return false;
	}

	if (responsePolicy == ResponsePolicy::IgnoreResponseImmediately)
	{
		SetLastResult(RequestCompletion::SentIgnoringResponse);
		AttachBackgroundHandler(reply, method, qurl, RequestCompletion::SentIgnoringResponse, bEnableLog);
		if (bEnableLog)
		{
			LCNC_INFO(lcnc::LogCode::Generic, "{}", "HTTP " + method.toStdString() + " dispatched without waiting, url: "
				+ qurl.toString().toStdString() + ", completion: "
				+ GetLastCompletionText().toStdString());
		}
		return true;
	}

	QEventLoop loop;
	QTimer timer;
	bool bTimedOut = false;
	timer.setSingleShot(true);

	connect(reply, &QNetworkReply::finished, &loop, &QEventLoop::quit);
	if (responsePolicy == ResponsePolicy::RequireResponse)
	{
		connect(&timer, &QTimer::timeout, this, [&bTimedOut, reply]() {
			if (!reply->isFinished())
			{
				bTimedOut = true;
				reply->abort();
			}
		});
	}
	else
	{
		connect(&timer, &QTimer::timeout, &loop, [&bTimedOut, reply, &loop]() {
			if (!reply->isFinished())
			{
				bTimedOut = true;
				loop.quit();
			}
		});
	}

	const int iWaitMs = responsePolicy == ResponsePolicy::RequireResponse
		? m_iTimeout
		: (responseWindowMs > 0 ? responseWindowMs : kDefaultFastResponseWindowMs);
	timer.start(iWaitMs);
	if (!reply->isFinished())
		loop.exec();
	timer.stop();

	if (responsePolicy == ResponsePolicy::AcceptNoResponseAfterTimeout && bTimedOut)
	{
		SetLastResult(RequestCompletion::SentWithoutResponse);
		AttachBackgroundHandler(reply, method, qurl, RequestCompletion::SentWithoutResponse, bEnableLog);
		if (bEnableLog)
		{
			LCNC_INFO(lcnc::LogCode::Generic, "{}", "HTTP " + method.toStdString() + " timed out waiting for response window, url: "
				+ qurl.toString().toStdString() + ", completion: "
				+ GetLastCompletionText().toStdString());
		}
		return true;
	}

	const int iStatusCode = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
	const QByteArray baReply = reply->readAll();
	const QNetworkReply::NetworkError eError = reply->error();
	const bool bSuccess = !bTimedOut && eError == QNetworkReply::NoError
		&& iStatusCode >= 200 && iStatusCode < 300;

	if (!bSuccess)
	{
		QString qstrError;
		if (bTimedOut)
			qstrError = tr("HTTP request timed out after %1 ms.").arg(m_iTimeout);
		else if (eError != QNetworkReply::NoError)
			qstrError = reply->errorString();
		else
			qstrError = tr("HTTP request failed with status code %1.").arg(iStatusCode);

		SetLastResult(RequestCompletion::Failed, 0, QByteArray(), qstrError);
		if (bEnableLog)
		{
			LCNC_ERR(lcnc::LogCode::Generic, "{}", "HTTP " + method.toStdString() + " failed, url: "
				+ qurl.toString().toStdString() + ", status: " + std::to_string(iStatusCode)
				+ ", error: " + qstrError.toStdString());
		}
	}
	else
	{
		response = baReply;
		SetLastResult(RequestCompletion::VerifiedResponse, iStatusCode, baReply);
		if (bEnableLog)
		{
			LCNC_INFO(lcnc::LogCode::Generic, "{}", "HTTP " + method.toStdString() + " succeeded, url: "
				+ qurl.toString().toStdString() + ", status: " + std::to_string(iStatusCode)
				+ ", completion: " + GetLastCompletionText().toStdString());
		}
	}

	reply->deleteLater();
	QCoreApplication::sendPostedEvents(nullptr, QEvent::DeferredDelete);
	return bSuccess;
}

QNetworkReply* HTTPClient::DispatchRequest(const QString& method, const QNetworkRequest& request,
	const QByteArray& data)
{
	if (method == "GET")
		return m_pManager->get(request);
	if (method == "POST")
		return m_pManager->post(request, data);
	if (method == "PUT")
		return m_pManager->put(request, data);
	return nullptr;
}

void HTTPClient::AttachBackgroundHandler(QNetworkReply* reply, const QString& method,
	const QUrl& qurl, RequestCompletion completion, bool bEnableLog)
{
	const QString qstrCompletion = CompletionToText(completion);
	auto backgroundHandler = [reply, method, qurl, qstrCompletion, bEnableLog]() {
		if (reply->property(kBackgroundHandledProperty).toBool())
			return;

		reply->setProperty(kBackgroundHandledProperty, true);

		const int iStatusCode = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
		const QNetworkReply::NetworkError eError = reply->error();
		if (eError == QNetworkReply::NoError && iStatusCode >= 200 && iStatusCode < 300)
		{
			if (bEnableLog)
			{
				LCNC_INFO(lcnc::LogCode::Generic, "{}", "HTTP " + method.toStdString() + " background completion succeeded, url: "
					+ qurl.toString().toStdString() + ", status: " + std::to_string(iStatusCode)
					+ ", completion: " + qstrCompletion.toStdString());
			}
		}
		else
		{
			const QString qstrError = eError != QNetworkReply::NoError
				? reply->errorString()
				: QObject::tr("HTTP request failed with status code %1.").arg(iStatusCode);
			if (bEnableLog)
			{
				LCNC_ERR(lcnc::LogCode::Generic, "{}", "HTTP " + method.toStdString() + " background completion failed, url: "
					+ qurl.toString().toStdString() + ", status: " + std::to_string(iStatusCode)
					+ ", completion: " + qstrCompletion.toStdString()
					+ ", error: " + qstrError.toStdString());
			}
		}

		reply->deleteLater();
	};

	QObject::connect(reply, &QNetworkReply::finished, reply, backgroundHandler);
	if (reply->isFinished())
		backgroundHandler();
}

void HTTPClient::SetLastResult(RequestCompletion completion, int statusCode,
	const QByteArray& response, const QString& error)
{
	m_eLastCompletion = completion;
	m_iLastStatusCode = statusCode;
	m_baLastResponse = response;
	m_qstrLastError = error;
}

void HTTPClient::ResetLastResult()
{
	SetLastResult(RequestCompletion::None);
}
