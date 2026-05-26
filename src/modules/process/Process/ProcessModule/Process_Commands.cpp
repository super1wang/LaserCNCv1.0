#include "Process_Commands.h"
#include <qDebug>
#include <QRegExpValidator>

ProcessCommands::ProcessCommands(TreeItem *parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Commands;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Note",""}, {"File",""} };
	m_itemData.resize(2);
}

ProcessCommands::ProcessCommands(const QString &text, TreeItem *parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Commands;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Note",""}, {"File",""} };
	m_itemData.resize(2);
	UpdateInfo();
}

ProcessCommands::ProcessCommands(const QVector<QVariant> &data, TreeItem *parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Commands;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Note",""}, {"File",""} };
	m_itemData.resize(2);
	m_itemData		= data;
}

ProcessCommands::~ProcessCommands(void)
{
	qDeleteAll(m_childItems);
	m_childItems.clear();
	qDeleteAll(m_childItems);
}

TreeItem* ProcessCommands::clone() const
{
	TreeItem* newItem = nullptr;

	if (!m_itemData.isEmpty()) {
		newItem = new ProcessCommands(QStringLiteral("Commands"));
	}
	else {
		newItem = new TreeItem(static_cast<TreeItem*>(nullptr));
	}

	newItem->m_type		 = m_type;
	newItem->m_state	 = m_state;
	newItem->m_stateSave = m_stateSave;
	newItem->m_maps		 = m_maps;

	for (TreeItem* child : m_childItems)
	{
		TreeItem* clonedChild = child->clone();
		newItem->appendChild(clonedChild);
	}

	return newItem;
}

void ProcessCommands::Edit() 
{
	if (m_state == ItemState::Unavailable || m_state == ItemState::Enable || m_state == ItemState::Disable)
	{
		Dialog_ProcessSetting_Commands* Dialog = new Dialog_ProcessSetting_Commands();
		Dialog->setAttribute(Qt::WA_DeleteOnClose);    //关闭对话框的同时，回收内存
		Dialog->m_pProcessCommands = this;
		Dialog->ViewSetting();
		Dialog->show();
	}
}

void ProcessCommands::UpdateInfo()
{
	if (TreeItem::IsChinese())
		setData(0, "\xE6\x8C\x87\xE4\xBB\xA4\xE9\x9B\x86");// 指令集
	else
		setData(0, "Commands");
	
	setData(1, m_maps["Note"]);
}

void ProcessCommands::SwitchState(ItemState state)
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


Dialog_ProcessSetting_Commands::Dialog_ProcessSetting_Commands(QWidget* parent) :
	QDialog(parent),
	m_pProcessCommands(nullptr),
	Dialog_Commands(new Ui::Dialog_ProcessSetting_Commands)
{
	Dialog_Commands->setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowTitleHint);

	connect(Dialog_Commands->Button_Commands_OK,		SIGNAL(clicked()), this, SLOT(ButtonOK()));
	connect(Dialog_Commands->Button_Commands_Cancel,	SIGNAL(clicked()), this, SLOT(ButtonCancel()));
	connect(Dialog_Commands->Button_Commands_File,		SIGNAL(clicked()), this, SLOT(ButtonFile()));
}

Dialog_ProcessSetting_Commands::~Dialog_ProcessSetting_Commands()
{
	delete Dialog_Commands;
}

void Dialog_ProcessSetting_Commands::ViewSetting()
{
	//修改标签状态，避免重复打开标签设置
	m_pProcessCommands->SwitchState(ItemState::Editing);

	//显示参数
	Dialog_Commands->lineEdit_Commands_Note->setText(m_pProcessCommands->GetCommandsNote());
	Dialog_Commands->lineEdit_Commands_File->setText(m_pProcessCommands->GetCommandsFile());
}

void Dialog_ProcessSetting_Commands::ButtonFile()
{
	QString qstrFile = QFileDialog::getOpenFileName(this, tr("Open Commands File"), "", tr("文本文件 (*.txt)"));
	if (!qstrFile.isEmpty())
		Dialog_Commands->lineEdit_Commands_File->setText(qstrFile);
}

void Dialog_ProcessSetting_Commands::ButtonOK()
{
	//获取参数
	QString qstrNote = Dialog_Commands->lineEdit_Commands_Note->text();
	QString qstrFile = Dialog_Commands->lineEdit_Commands_File->text();

	//设置参数
	m_pProcessCommands->SetCommandsNote(qstrNote);
	m_pProcessCommands->SetCommandsFile(qstrFile);

	if (!qstrFile.isEmpty())
	{
		m_pProcessCommands->SetState(ItemState::Enable);
		m_pProcessCommands->SwitchState(ItemState::Enable);
	}
	m_pProcessCommands->UpdateInfo();
	QDialog::accept();
}

void Dialog_ProcessSetting_Commands::ButtonCancel()
{
	//点击取消/ESC退出均不保存参数，还原节点状态
	reject();
}

void Dialog_ProcessSetting_Commands::reject()
{
	m_pProcessCommands->SwitchState();
	m_pProcessCommands->UpdateInfo();
	QDialog::reject();
}