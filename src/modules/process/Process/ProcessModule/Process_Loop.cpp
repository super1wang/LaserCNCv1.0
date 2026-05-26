#include "Process_Loop.h"
#include <qDebug>
#include <QRegExpValidator>

ProcessLoop::ProcessLoop(TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Loop;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Number","1"} };
	m_itemData.resize(2);
}

ProcessLoop::ProcessLoop(const QString& text, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Loop;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Number","1"} };
	m_itemData.resize(2);
	UpdateInfo();
}

ProcessLoop::ProcessLoop(const QVector<QVariant>& data, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Loop;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Number","1"} };
	m_itemData.resize(2);
	m_itemData		= data;
}

ProcessLoop::~ProcessLoop(void)
{
	qDeleteAll(m_childItems);
	m_childItems.clear();
	qDeleteAll(m_childItems);
}

TreeItem* ProcessLoop::clone() const
{
	TreeItem* newItem = nullptr;

	if (!m_itemData.isEmpty()) {
		newItem = new ProcessLoop(QStringLiteral("Loop"));
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

void ProcessLoop::Edit()
{
	if (m_state == ItemState::Unavailable || m_state == ItemState::Enable || m_state == ItemState::Disable)
	{
		Dialog_ProcessSetting_Loop* Dialog = new Dialog_ProcessSetting_Loop();
		Dialog->setAttribute(Qt::WA_DeleteOnClose);    //关闭对话框的同时，回收内存
		Dialog->m_pProcessLoop = this;
		Dialog->ViewSetting();
		Dialog->show();
	}
}

void ProcessLoop::UpdateInfo()
{
	if (TreeItem::IsChinese())
	{
		setData(0, "\xE5\xBE\xAA\xE7\x8E\xAF"); // 循环
		setData(1, "\xE9\x87\x8D\xE5\xA4\x8D " + m_maps["Number"] + " \xE6\xAC\xA1"); //重复 次
	}
	else
	{
		setData(0, "Loop");
		setData(1, m_maps["Number"] + " times");
	}
}

void ProcessLoop::SwitchState(ItemState state)
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


Dialog_ProcessSetting_Loop::Dialog_ProcessSetting_Loop(QWidget* parent) :
	QDialog(parent),
	m_pProcessLoop(nullptr),
	Dialog_Loop(new Ui::Dialog_ProcessSetting_Loop)
{
	Dialog_Loop->setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowTitleHint);

	connect(Dialog_Loop->Button_Loop_OK,		SIGNAL(clicked()),		this,		SLOT(ButtonOK()));
	connect(Dialog_Loop->Button_Loop_Cancel,	SIGNAL(clicked()),		this,		SLOT(ButtonCancel()));
}

Dialog_ProcessSetting_Loop::~Dialog_ProcessSetting_Loop()
{
	delete Dialog_Loop;
}

void Dialog_ProcessSetting_Loop::ViewSetting()
{
	//修改标签状态，避免重复打开标签设置
	m_pProcessLoop->SwitchState(ItemState::Editing);

	//限制输入值
	Dialog_Loop->lineEdit_Loop_Number->setValidator(new QRegExpValidator(Regex_All_Int));
	
	//读取参数
	int		iNumber = m_pProcessLoop->GetLoopNumber();

	//显示参数
	Dialog_Loop->lineEdit_Loop_Number->setText(QString::number(iNumber));
}

void Dialog_ProcessSetting_Loop::ButtonOK()
{
	//获取参数
	int		iNumber = Dialog_Loop->lineEdit_Loop_Number->text().toInt();
	
	//设置参数
	m_pProcessLoop->SetLoopNumber(iNumber);

	m_pProcessLoop->SetState(ItemState::Enable);
	m_pProcessLoop->SwitchState(ItemState::Enable);
	m_pProcessLoop->UpdateInfo();
	QDialog::accept();
}

void Dialog_ProcessSetting_Loop::ButtonCancel()
{
	//点击取消/ESC退出均不保存参数，还原节点状态
	reject();
}

void Dialog_ProcessSetting_Loop::reject()
{
	m_pProcessLoop->SwitchState();
	m_pProcessLoop->UpdateInfo();
	QDialog::reject();
}
