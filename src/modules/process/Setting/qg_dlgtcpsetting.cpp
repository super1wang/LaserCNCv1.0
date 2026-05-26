#include "qg_dlgtcpsetting.h"

QG_dlgTCPSetting::QG_dlgTCPSetting(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::FramelessWindowHint);
}

QG_dlgTCPSetting::~QG_dlgTCPSetting()
{}
