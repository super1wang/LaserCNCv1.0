#pragma once

#include <QObject>

class HTTPServer  : public QObject
{
	Q_OBJECT

public:
	HTTPServer(QObject *parent = nullptr);
	~HTTPServer();
};
