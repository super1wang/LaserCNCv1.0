#include "Process_RunGroup.h"
#include <qDebug>


ProcessRunGroup::ProcessRunGroup(TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::RunGroup;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Name",""}, {"Thread", "0"}};
	m_itemData.resize(2);
}

ProcessRunGroup::ProcessRunGroup(const QString& text, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::RunGroup;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Name",""}, {"Thread", "0"}};
	m_itemData.resize(2);
	UpdateInfo();
}

ProcessRunGroup::ProcessRunGroup(const QVector<QVariant>& data, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::RunGroup;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Name",""}, {"Thread", "0"}};
	m_itemData.resize(2);
	m_itemData		= data;
}

ProcessRunGroup::~ProcessRunGroup(void)
{
	qDeleteAll(m_childItems);
	m_childItems.clear();
	qDeleteAll(m_childItems);
}

TreeItem* ProcessRunGroup::clone() const
{
	TreeItem* newItem = nullptr;

	if (!m_itemData.isEmpty()) {
		newItem = new ProcessRunGroup(QStringLiteral("RunGroup"));
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

void ProcessRunGroup::Edit()
{
	if (m_state == ItemState::Unavailable || m_state == ItemState::Enable || m_state == ItemState::Disable)
	{
		Dialog_ProcessSetting_RunGroup* Dialog = new Dialog_ProcessSetting_RunGroup();
		Dialog->setAttribute(Qt::WA_DeleteOnClose);    //关闭对话框的同时，回收内存
		Dialog->m_pProcessRunGroup = this;
		Dialog->ViewSetting();
		Dialog->show();
	}
}

void ProcessRunGroup::UpdateInfo()
{
	QString qstrB;
	if (TreeItem::IsChinese())
	{
		setData(0, "\xE8\xBF\x90\xE8\xA1\x8C\xE7\xBB\x84"); // 运行组
		qstrB = m_maps["Thread"].toInt() ? "\xE5\x90\x8E\xE5\x8F\xB0\xE6\x89\xA7\xE8\xA1\x8C " : ""; // 后台执行
	}
	else
	{
		setData(0, "RunGroup");
		qstrB = m_maps["Thread"].toInt() ? "Background run " : "";
	}
	
	setData(1, qstrB + m_maps["Name"]);
}

void ProcessRunGroup::SwitchState(ItemState state)
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


Dialog_ProcessSetting_RunGroup::Dialog_ProcessSetting_RunGroup(QWidget* parent) :
	QDialog(parent),
	m_pProcessRunGroup(nullptr),
	Dialog_RunGroup(new Ui::Dialog_ProcessSetting_RunGroup)
{
	Dialog_RunGroup->setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowTitleHint);

	connect(Dialog_RunGroup->Button_RunGroup_OK,		SIGNAL(clicked()), this, SLOT(ButtonOK()));
	connect(Dialog_RunGroup->Button_RunGroup_Cancel,	SIGNAL(clicked()), this, SLOT(ButtonCancel()));
}

Dialog_ProcessSetting_RunGroup::~Dialog_ProcessSetting_RunGroup()
{
	delete Dialog_RunGroup;
}

void Dialog_ProcessSetting_RunGroup::ViewSetting()
{
	//修改标签状态，避免重复打开标签设置
	m_pProcessRunGroup->SetState(ItemState::Editing);

	//测试用临时行
	//Dialog_RunGroup->comboBox_RunGroup_Name->addItems(QStringList() << "Group 1" << "Group 2");

	//获取参数
	QString	QstrName	= m_pProcessRunGroup->GetRunGroupName();
	QString	QstrThread	= m_pProcessRunGroup->GetRunGroupThread();

	//填入参数
	//Dialog_RunGroup->comboBox_RunGroup_Name->setCurrentText(QstrValue);
	Dialog_RunGroup->lineEdit_RunGroup_Name->setText(QstrName);
	Dialog_RunGroup->checkBox_RunGroup_Thread->setChecked(QstrThread.toInt());

}

void Dialog_ProcessSetting_RunGroup::ButtonOK()
{
	//获取参数
	//QString		QstrValue = Dialog_RunGroup->comboBox_RunGroup_Name->currentText();
	QString	QstrName	= Dialog_RunGroup->lineEdit_RunGroup_Name->text();
	QString	QstrThread  = QString::number(Dialog_RunGroup->checkBox_RunGroup_Thread->isChecked());
	
	//保存参数
	m_pProcessRunGroup->SetRunGroupName(QstrName);
	m_pProcessRunGroup->SetRunGroupThread(QstrThread);

	m_pProcessRunGroup->SetState(ItemState::Enable);
	m_pProcessRunGroup->SwitchState(ItemState::Enable);
	m_pProcessRunGroup->UpdateInfo();
	QDialog::accept();
}

void Dialog_ProcessSetting_RunGroup::ButtonCancel()
{
	//点击取消/ESC退出均不保存参数，还原节点状态
	reject();
}

void Dialog_ProcessSetting_RunGroup::reject()
{
	m_pProcessRunGroup->SwitchState();
	m_pProcessRunGroup->UpdateInfo();
	QDialog::reject();
}