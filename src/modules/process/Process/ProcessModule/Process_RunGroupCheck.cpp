#include "Process_RunGroupCheck.h"
#include <qDebug>
#include <QRegExpValidator>

ProcessRunGroupCheck::ProcessRunGroupCheck(TreeItem *parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::RunGroupCheck;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Value","30"}, {"Unit","1"}};
	m_itemData.resize(2);
}

ProcessRunGroupCheck::ProcessRunGroupCheck(const QString& text, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::RunGroupCheck;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Value","30"}, {"Unit","1"} };
	m_itemData.resize(2);
	UpdateInfo();
}

ProcessRunGroupCheck::ProcessRunGroupCheck(const QVector<QVariant>& data, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::RunGroupCheck;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Value","30"}, {"Unit","1"} };
	m_itemData.resize(2);
	m_itemData		= data;
}

ProcessRunGroupCheck::~ProcessRunGroupCheck(void)
{
	qDeleteAll(m_childItems);
	m_childItems.clear();
	qDeleteAll(m_childItems);
}

TreeItem* ProcessRunGroupCheck::clone() const
{
	TreeItem* newItem = nullptr;

	if (!m_itemData.isEmpty()) {
		newItem = new ProcessRunGroupCheck(QStringLiteral("RunGroupCheck"));
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

void ProcessRunGroupCheck::Edit()
{
	if (m_state == ItemState::Unavailable || m_state == ItemState::Enable || m_state == ItemState::Disable)
	{
		Dialog_ProcessSetting_RunGroupCheck* Dialog = new Dialog_ProcessSetting_RunGroupCheck();
		Dialog->setAttribute(Qt::WA_DeleteOnClose);    //关闭对话框的同时，回收内存
		Dialog->m_pProcessRunGroupCheck = this;
		Dialog->ViewSetting();
		Dialog->show();
	}
}

void ProcessRunGroupCheck::UpdateInfo()
{
	QString info = "";
	bool isChinese = TreeItem::IsChinese();
	
	switch (m_maps["Unit"].toInt())
	{
	case 0:
		if (isChinese)
			info = "\xE8\xB6\x85\xE6\x97\xB6\xE7\xAD\x89\xE5\xBE\x85 " + m_maps["Value"] + " \xE6\xAF\xAB\xE7\xA7\x92";	// 超时等待 毫秒
		else
			info = "Timeout waiting " + m_maps["Value"] + " ms";
		break;
	case 1:
		if (isChinese)
			info = "\xE8\xB6\x85\xE6\x97\xB6\xE7\xAD\x89\xE5\xBE\x85 " + m_maps["Value"] + " \xE7\xA7\x92";	// 超时等待 秒
		else
			info = "Timeout waiting " + m_maps["Value"] + " s";
		break;
	case 2:
		if (isChinese)
			info = "\xE8\xB6\x85\xE6\x97\xB6\xE7\xAD\x89\xE5\xBE\x85 " + m_maps["Value"] + " \xE5\x88\x86\xE9\x92\x9F";	// 超时等待 分钟
		else
			info = "Timeout waiting " + m_maps["Value"] + " min";
		break;
	}
	
	if (isChinese)
		setData(0, "\xE5\x90\x8E\xE5\x8F\xB0\xE8\xBF\x90\xE8\xA1\x8C\xE7\xBB\x84\xE6\xA0\xA1\xE9\xAA\x8C");	// 后台运行组校验
	else
		setData(0, "RunGroupCheck");
	
	setData(1, info);
}

void ProcessRunGroupCheck::SwitchState(ItemState state)
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


Dialog_ProcessSetting_RunGroupCheck::Dialog_ProcessSetting_RunGroupCheck(QWidget* parent) :
	QDialog(parent),
	m_pProcessRunGroupCheck(nullptr),
	Dialog_RunGroupCheck(new Ui::Dialog_ProcessSetting_RunGroupCheck)
{
	Dialog_RunGroupCheck->setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowTitleHint);

	connect(Dialog_RunGroupCheck->Button_RunGroupCheck_OK,		SIGNAL(clicked()), this, SLOT(ButtonOK()));
	connect(Dialog_RunGroupCheck->Button_RunGroupCheck_Cancel,	SIGNAL(clicked()), this, SLOT(ButtonCancel()));
}

Dialog_ProcessSetting_RunGroupCheck::~Dialog_ProcessSetting_RunGroupCheck()
{
	delete Dialog_RunGroupCheck;
}

void Dialog_ProcessSetting_RunGroupCheck::ViewSetting()
{
	//修改标签状态，避免重复打开标签设置
	m_pProcessRunGroupCheck->SwitchState(ItemState::Editing);

	//限制输入值
	Dialog_RunGroupCheck->lineEdit_RunGroupCheck_Value->setValidator(new QRegExpValidator(Regex_Nonnegative_Double));

	//读取参数
	double	dValue	= m_pProcessRunGroupCheck->GetRunGroupCheckValue();
	int		iUnit	= m_pProcessRunGroupCheck->GetRunGroupCheckUnit();

	//显示参数
	Dialog_RunGroupCheck->lineEdit_RunGroupCheck_Value->setText(QString::number(dValue));
	Dialog_RunGroupCheck->comboBox_RunGroupCheck_Unit->setCurrentIndex(iUnit);
}

void Dialog_ProcessSetting_RunGroupCheck::ButtonOK()
{
	//获取参数
	double	dValue	= Dialog_RunGroupCheck->lineEdit_RunGroupCheck_Value->text().toDouble();
	int		iUnit	= Dialog_RunGroupCheck->comboBox_RunGroupCheck_Unit->currentIndex();

	//设置参数
	m_pProcessRunGroupCheck->SetRunGroupCheckValue(dValue);
	m_pProcessRunGroupCheck->SetRunGroupCheckUnit(iUnit);

	m_pProcessRunGroupCheck->SetState(ItemState::Enable);
	m_pProcessRunGroupCheck->SwitchState(ItemState::Enable);
	m_pProcessRunGroupCheck->UpdateInfo();
	QDialog::accept();
}

void Dialog_ProcessSetting_RunGroupCheck::ButtonCancel()
{
	//点击取消/ESC退出均不保存参数，还原节点状态
	reject();
}

void Dialog_ProcessSetting_RunGroupCheck::reject()
{
	m_pProcessRunGroupCheck->SwitchState();
	m_pProcessRunGroupCheck->UpdateInfo();
	QDialog::reject();
}