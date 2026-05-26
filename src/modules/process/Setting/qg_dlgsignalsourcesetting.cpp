#include "qg_dlgsignalsourcesetting.h"

QG_dlgSignalSourceSetting::QG_dlgSignalSourceSetting(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::FramelessWindowHint);
}

QG_dlgSignalSourceSetting::~QG_dlgSignalSourceSetting()
{}
