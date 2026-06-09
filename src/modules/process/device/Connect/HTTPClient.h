#pragma once

#include <QObject>
#include <QByteArray>
#include <QString>

class QNetworkAccessManager;
class QNetworkReply;
class QNetworkRequest;
class QUrl;

class HTTPClient : public QObject
{
	Q_OBJECT

public:
	enum class RequestCompletion
	{
		None,
		VerifiedResponse,
		SentWithoutResponse,
		SentIgnoringResponse,
		Failed
	};

public:
	explicit HTTPClient(QObject* parent = nullptr);
	~HTTPClient();

	bool Get(const QString& url, QByteArray& response);
	bool GetSilent(const QString& url, QByteArray& response);
	bool Post(const QString& url, const QByteArray& data, QByteArray& response,
		const QString& contentType = "application/json;charset=UTF-8");
	bool Put(const QString& url, const QByteArray& data, QByteArray& response,
		const QString& contentType = "application/json;charset=UTF-8",
		bool bEnableLog = true);
	bool PostFast(const QString& url, const QByteArray& data,
		const QString& contentType = "application/json;charset=UTF-8",
		int responseWindowMs = 50, bool bEnableLog = true);
	bool PutFast(const QString& url, const QByteArray& data,
		const QString& contentType = "application/json;charset=UTF-8",
		int responseWindowMs = 50, bool bEnableLog = true);
	bool PostSendOnly(const QString& url, const QByteArray& data,
		const QString& contentType = "application/json;charset=UTF-8",
		bool bEnableLog = true);
	bool PutSendOnly(const QString& url, const QByteArray& data,
		const QString& contentType = "application/json;charset=UTF-8",
		bool bEnableLog = true);

	void SetTimeout(int timeoutMs);
	int GetTimeout() const;

	int GetLastStatusCode() const;
	QByteArray GetLastResponse() const;
	QString GetLastError() const;
	RequestCompletion GetLastCompletion() const;
	QString GetLastCompletionText() const;

private:
	enum class ResponsePolicy
	{
		RequireResponse,
		AcceptNoResponseAfterTimeout,
		IgnoreResponseImmediately
	};

	bool Request(const QString& method, const QString& url, const QByteArray& data,
		QByteArray& response, const QString& contentType = QString(),
		ResponsePolicy responsePolicy = ResponsePolicy::RequireResponse,
		int responseWindowMs = 0, bool bEnableLog = true);
	bool DoRequest(const QString& method, const QString& url, const QByteArray& data,
		QByteArray& response, const QString& contentType,
		ResponsePolicy responsePolicy, int responseWindowMs, bool bEnableLog);
	QNetworkReply* DispatchRequest(const QString& method, const QNetworkRequest& request,
		const QByteArray& data);
	void AttachBackgroundHandler(QNetworkReply* reply, const QString& method,
		const QUrl& qurl, RequestCompletion completion, bool bEnableLog);
	void SetLastResult(RequestCompletion completion, int statusCode = 0,
		const QByteArray& response = QByteArray(), const QString& error = QString());
	void ResetLastResult();

private:
	QNetworkAccessManager* m_pManager;
	int m_iTimeout;
	int m_iLastStatusCode;
	QByteArray m_baLastResponse;
	QString m_qstrLastError;
	RequestCompletion m_eLastCompletion;
};
