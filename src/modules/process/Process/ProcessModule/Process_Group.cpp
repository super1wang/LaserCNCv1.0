#include "Process_Group.h"
#include <qDebug>


ProcessGroup::ProcessGroup(TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Group;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Name",""} };
	m_itemData.resize(2);
}

ProcessGroup::ProcessGroup(const QString& text, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Group;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Name",""} };
	m_itemData.resize(2);
	UpdateInfo();
}

ProcessGroup::ProcessGroup(const QVector<QVariant>& data, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Group;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Name",""} };
	m_itemData.resize(2);
	m_itemData		= data;
}

ProcessGroup::~ProcessGroup(void)
{
	qDeleteAll(m_childItems);
	m_childItems.clear();
	qDeleteAll(m_childItems);
}

TreeItem* ProcessGroup::clone() const
{
	TreeItem* newItem = nullptr;

	if (!m_itemData.isEmpty()) {
		newItem = new ProcessGroup(QStringLiteral("Group"));
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

void ProcessGroup::Edit()
{
	if (m_state == ItemState::Unavailable || m_state == ItemState::Enable || m_state == ItemState::Disable)
	{
		Dialog_ProcessSetting_Group* Dialog = new Dialog_ProcessSetting_Group();
		Dialog->setAttribute(Qt::WA_DeleteOnClose);    //关闭对话框的同时，回收内存
		Dialog->m_pProcessGroup = this;
		Dialog->ViewSetting();
		Dialog->show();
	}
}

void ProcessGroup::UpdateInfo()
{
	if (TreeItem::IsChinese())
		setData(0, "\xE7\xBB\x84");// 组
	else
		setData(0, "Group");
	
	setData(1, m_maps["Name"]);
}

void ProcessGroup::SwitchState(ItemState state)
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


Dialog_ProcessSetting_Group::Dialog_ProcessSetting_Group(QWidget* parent) :
	QDialog(parent),
	m_pProcessGroup(nullptr),
	Dialog_Group(new Ui::Dialog_ProcessSetting_Group)
{
	Dialog_Group->setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowTitleHint);

	connect(Dialog_Group->Button_Group_OK,		SIGNAL(clicked()), this, SLOT(ButtonOK()));
	connect(Dialog_Group->Button_Group_Cancel,	SIGNAL(clicked()), this, SLOT(ButtonCancel()));
}

Dialog_ProcessSetting_Group::~Dialog_ProcessSetting_Group()
{
	delete Dialog_Group;
}

void Dialog_ProcessSetting_Group::ViewSetting()
{
	//修改标签状态，避免重复打开标签设置
	m_pProcessGroup->SwitchState(ItemState::Editing);

	//读取参数
	QString		QstrGroupName = m_pProcessGroup->GetGroupName();

	//显示参数
	Dialog_Group->lineEdit_Group_Name->setText(QstrGroupName);
}

void Dialog_ProcessSetting_Group::ButtonOK()
{
	//获取参数
	QString		QstrGroupName = Dialog_Group->lineEdit_Group_Name->text();

	//设置参数
	m_pProcessGroup->SetGroupName(QstrGroupName);

	m_pProcessGroup->SetState(ItemState::Enable);
	m_pProcessGroup->SwitchState(ItemState::Enable);
	m_pProcessGroup->UpdateInfo();
	QDialog::accept();
}

void Dialog_ProcessSetting_Group::ButtonCancel()
{
	//点击取消/ESC退出均不保存参数，还原节点状态
	reject();
}

void Dialog_ProcessSetting_Group::reject()
{
	m_pProcessGroup->SwitchState();
	m_pProcessGroup->UpdateInfo();
	QDialog::reject();
}
