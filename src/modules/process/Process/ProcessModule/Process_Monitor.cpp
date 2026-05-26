#include "Process_Monitor.h"
#include <qDebug>
#include <QRegExpValidator>


ProcessMonitor::ProcessMonitor(TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Monitor;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"InterLock",""}, {"Pressure",""}, {"WaterLeakage",""}, {"WaterPressure",""}, {"WaterPressureLimit",""}, {"WaterTankError",""}, {"WaterTankLevel",""}, {"WaterTankLevelLimit",""} };
	m_itemData.resize(2);
}

ProcessMonitor::ProcessMonitor(const QString& text, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Monitor;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"InterLock",""}, {"Pressure",""}, {"WaterLeakage",""}, {"WaterPressure",""}, {"WaterPressureLimit",""}, {"WaterTankError",""}, {"WaterTankLevel",""}, {"WaterTankLevelLimit",""} };
	m_itemData.resize(2);
	UpdateInfo();
}

ProcessMonitor::ProcessMonitor(const QVector<QVariant>& data, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Monitor;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"InterLock",""}, {"Pressure",""}, {"WaterLeakage",""}, {"WaterPressure",""}, {"WaterPressureLimit",""}, {"WaterTankError",""}, {"WaterTankLevel",""}, {"WaterTankLevelLimit",""} };
	m_itemData.resize(2);
	m_itemData		= data;
}

ProcessMonitor::~ProcessMonitor(void)
{
	qDeleteAll(m_childItems);
	m_childItems.clear();
	qDeleteAll(m_childItems);
}

TreeItem* ProcessMonitor::clone() const
{
	TreeItem* newItem = nullptr;

	if (!m_itemData.isEmpty()) {
		newItem = new ProcessMonitor(QStringLiteral("Monitor"));
	}
	else {
		newItem = new TreeItem(static_cast<TreeItem*>(nullptr));
	}

	newItem->m_type = m_type;
	newItem->m_state = m_state;
	newItem->m_stateSave = m_stateSave;
	newItem->m_maps = m_maps;

	for (TreeItem* child : m_childItems)
	{
		TreeItem* clonedChild = child->clone();
		newItem->appendChild(clonedChild);
	}

	return newItem;
}

void ProcessMonitor::Edit()
{
	if (m_state == ItemState::Unavailable || m_state == ItemState::Enable || m_state == ItemState::Disable)
	{
		Dialog_ProcessSetting_Monitor* Dialog = new Dialog_ProcessSetting_Monitor();
		Dialog->setAttribute(Qt::WA_DeleteOnClose);    //关闭对话框的同时，回收内存
		Dialog->m_pProcessMonitor = this;
		Dialog->ViewSetting();
		Dialog->show();
	}
}

void ProcessMonitor::UpdateInfo()
{
	if (TreeItem::IsChinese())
	{
		setData(0, "\xE7\x9B\x91\xE6\x8E\xA7"); // 监控
		setData(1, "\xE5\x88\x87\xE5\x89\xB2\xE7\x9B\x91\xE6\x8E\xA7"); // 切割监控
	}
	else
	{
		setData(0, "Monitor");
		setData(1, "Cutting Monitor");
	}
}

void ProcessMonitor::SwitchState(ItemState state)
{
	if (state == ItemState::StateSave)
	{
		if (m_state == ItemState::Unrun || m_state == ItemState::Run ||
			m_state == ItemState::Pause || m_state == ItemState::Stop)
			SetState(ItemState::Enable);
		else if (m_state == ItemState::Unuse)
			SetState(ItemState::Disable);
		else
			SetState(m_stateSave);
	}
	else
	{
		m_stateSave = m_state;
		SetState(state);
	}
}


Dialog_ProcessSetting_Monitor::Dialog_ProcessSetting_Monitor(QWidget* parent) :
	QDialog(parent),
	m_pProcessMonitor(nullptr),
	Dialog_Monitor(new Ui::Dialog_ProcessSetting_Monitor)
{
	Dialog_Monitor->setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowTitleHint);

	connect(Dialog_Monitor->Button_Monitor_OK,		SIGNAL(clicked()), this, SLOT(ButtonOK()));
	connect(Dialog_Monitor->Button_Monitor_Cancel,	SIGNAL(clicked()), this, SLOT(ButtonCancel()));
}

Dialog_ProcessSetting_Monitor::~Dialog_ProcessSetting_Monitor()
{
	delete Dialog_Monitor;
}

void Dialog_ProcessSetting_Monitor::ViewSetting()
{
	//修改标签状态，避免重复打开标签设置
	m_pProcessMonitor->SwitchState(ItemState::Editing);

	//限制输入值
	Dialog_Monitor->lineEdit_Monitor_WaterPressureLimit->setValidator(new QRegExpValidator(Regex_Nonnegative_Double));
	Dialog_Monitor->lineEdit_Monitor_WaterTankLevelLimit->setValidator(new QRegExpValidator(Regex_Nonnegative_Double));

	//读取参数
	bool	bInterLock				= m_pProcessMonitor->GetMonitorInterLock();
	bool	bPressure				= m_pProcessMonitor->GetMonitorPressure();
	bool	bWaterLeakage			= m_pProcessMonitor->GetMonitorWaterLeakage();
	bool	bWaterPressure			= m_pProcessMonitor->GetMonitorWaterPressure();
	double	dWaterPressureLimit		= m_pProcessMonitor->GetMonitorWaterPressureLimit();
	bool	bWaterTankError			= m_pProcessMonitor->GetMonitorWaterTankError();
	bool	bWaterTankLevel			= m_pProcessMonitor->GetMonitorWaterTankLevel();
	double	dWaterTankLevelLimit	= m_pProcessMonitor->GetMonitorWaterTankLevelLimit();
	
	//显示参数
	Dialog_Monitor->checkBox_Monitor_InterLock->setChecked(bInterLock);
	Dialog_Monitor->checkBox_Monitor_Pressure->setChecked(bPressure);
	Dialog_Monitor->checkBox_Monitor_WaterLeakage->setChecked(bWaterLeakage);
	Dialog_Monitor->checkBox_Monitor_WaterPressure->setChecked(bWaterPressure);
	Dialog_Monitor->lineEdit_Monitor_WaterPressureLimit->setText(QString::number(dWaterPressureLimit));
	Dialog_Monitor->checkBox_Monitor_WaterTankError->setChecked(bWaterTankError);
	Dialog_Monitor->checkBox_Monitor_WaterTankLevel->setChecked(bWaterTankLevel);
	Dialog_Monitor->lineEdit_Monitor_WaterTankLevelLimit->setText(QString::number(dWaterTankLevelLimit));
}

void Dialog_ProcessSetting_Monitor::ButtonOK()
{
	//获取参数
	bool	bInterLock				= Dialog_Monitor->checkBox_Monitor_InterLock->checkState();
	bool	bPressure				= Dialog_Monitor->checkBox_Monitor_Pressure->checkState();
	bool	bWaterLeakage			= Dialog_Monitor->checkBox_Monitor_WaterLeakage->checkState();
	bool	bWaterPressure			= Dialog_Monitor->checkBox_Monitor_WaterPressure->checkState();
	double	dWaterPressureLimit		= Dialog_Monitor->lineEdit_Monitor_WaterPressureLimit->text().toDouble();
	bool	bWaterTankError			= Dialog_Monitor->checkBox_Monitor_WaterTankError->checkState();
	bool	bWaterTankLevel			= Dialog_Monitor->checkBox_Monitor_WaterTankLevel->checkState();
	double	dWaterTankLevelLimit	= Dialog_Monitor->lineEdit_Monitor_WaterTankLevelLimit->text().toDouble();

	//设置参数
	m_pProcessMonitor->SetMonitorInterLock(bInterLock);
	m_pProcessMonitor->SetMonitorPressure(bPressure);
	m_pProcessMonitor->SetMonitorWaterLeakage(bWaterLeakage);
	m_pProcessMonitor->SetMonitorWaterPressure(bWaterPressure);
	m_pProcessMonitor->SetMonitorWaterPressureLimit(dWaterPressureLimit);
	m_pProcessMonitor->SetMonitorWaterTankError(bWaterTankError);
	m_pProcessMonitor->SetMonitorWaterTankLevel(bWaterTankLevel);
	m_pProcessMonitor->SetMonitorWaterTankLevelLimit(dWaterTankLevelLimit);
	
	m_pProcessMonitor->SetState(ItemState::Enable);
	m_pProcessMonitor->SwitchState(ItemState::Enable);
	m_pProcessMonitor->UpdateInfo();
	QDialog::accept();
}

void Dialog_ProcessSetting_Monitor::ButtonCancel()
{
	//点击取消/ESC退出均不保存参数，还原节点状态
	reject();
}

void Dialog_ProcessSetting_Monitor::reject()
{
	m_pProcessMonitor->SwitchState();
	m_pProcessMonitor->UpdateInfo();
	QDialog::reject();
}