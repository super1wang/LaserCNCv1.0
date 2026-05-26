#include "qg_dlgsmcsetting.h"

QG_dlgSMCSetting::QG_dlgSMCSetting(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::FramelessWindowHint);
}

QG_dlgSMCSetting::~QG_dlgSMCSetting()
{}
