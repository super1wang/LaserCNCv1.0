#include "qg_dlgjsonsetting.h"

QG_dlgJsonSetting::QG_dlgJsonSetting(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::FramelessWindowHint);
}

QG_dlgJsonSetting::~QG_dlgJsonSetting()
{}
