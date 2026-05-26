#include "Process_Wait.h"
#include <qDebug>
#include <QRegExpValidator>

ProcessWait::ProcessWait(TreeItem *parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Wait;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Value",""}, {"Unit",""}};
	m_itemData.resize(2);
}

ProcessWait::ProcessWait(const QString& text, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Wait;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Value",""}, {"Unit",""} };
	m_itemData.resize(2);
	UpdateInfo();
}

ProcessWait::ProcessWait(const QVector<QVariant>& data, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Wait;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Value",""}, {"Unit",""} };
	m_itemData.resize(2);
	m_itemData		= data;
}

ProcessWait::~ProcessWait(void)
{
	qDeleteAll(m_childItems);
	m_childItems.clear();
	qDeleteAll(m_childItems);
}

TreeItem* ProcessWait::clone() const
{
	TreeItem* newItem = nullptr;

	if (!m_itemData.isEmpty()) {
		newItem = new ProcessWait(QStringLiteral("Wait"));
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

void ProcessWait::Edit()
{
	if (m_state == ItemState::Unavailable || m_state == ItemState::Enable || m_state == ItemState::Disable)
	{
		Dialog_ProcessSetting_Wait* Dialog = new Dialog_ProcessSetting_Wait();
		Dialog->setAttribute(Qt::WA_DeleteOnClose);    //关闭对话框的同时，回收内存
		Dialog->m_pProcessWait = this;
		Dialog->ViewSetting();
		Dialog->show();
	}
}

void ProcessWait::UpdateInfo()
{
	QString info = "";
	bool isChinese = TreeItem::IsChinese();
	
	switch (m_maps["Unit"].toInt())
	{
	case 0:
		if (isChinese)
			info = "\xE7\xAD\x89\xE5\xBE\x85 " + m_maps["Value"] + " \xE6\xAF\xAB\xE7\xA7\x92";	// 等待 毫秒
		else
			info = "Wait " + m_maps["Value"] + " ms";
		break;
	case 1:
		if (isChinese)
			info = "\xE7\xAD\x89\xE5\xBE\x85 " + m_maps["Value"] + " \xE7\xA7\x92";				// 等待 秒
		else
			info = "Wait " + m_maps["Value"] + " s";
		break;
	case 2:
		if (isChinese)
			info = "\xE7\xAD\x89\xE5\xBE\x85 " + m_maps["Value"] + " \xE5\x88\x86\xE9\x92\x9F";	// 等待 分钟
		else
			info = "Wait " + m_maps["Value"] + " min";
		break;
	}
	
	if (isChinese)
		setData(0, "\xE5\xBB\xB6\xE6\x97\xB6"); // 延时
	else
		setData(0, "Wait");
	
	setData(1, info);
}

void ProcessWait::SwitchState(ItemState state)
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


Dialog_ProcessSetting_Wait::Dialog_ProcessSetting_Wait(QWidget* parent) :
	QDialog(parent),
	m_pProcessWait(nullptr),
	Dialog_Wait(new Ui::Dialog_ProcessSetting_Wait)
{
	Dialog_Wait->setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowTitleHint);

	connect(Dialog_Wait->Button_Wait_OK,		SIGNAL(clicked()), this, SLOT(ButtonOK()));
	connect(Dialog_Wait->Button_Wait_Cancel,	SIGNAL(clicked()), this, SLOT(ButtonCancel()));
}

Dialog_ProcessSetting_Wait::~Dialog_ProcessSetting_Wait()
{
	delete Dialog_Wait;
}

void Dialog_ProcessSetting_Wait::ViewSetting()
{
	//修改标签状态，避免重复打开标签设置
	m_pProcessWait->SwitchState(ItemState::Editing);

	//限制输入值
	Dialog_Wait->lineEdit_Wait_Value->setValidator(new QRegExpValidator(Regex_Nonnegative_Double));

	//读取参数
	double	dValue	= m_pProcessWait->GetWaitValue();
	int		iUnit	= m_pProcessWait->GetWaitUnit();

	//显示参数
	Dialog_Wait->lineEdit_Wait_Value->setText(QString::number(dValue));
	Dialog_Wait->comboBox_Wait_Unit->setCurrentIndex(iUnit);
}

void Dialog_ProcessSetting_Wait::ButtonOK()
{
	//获取参数
	double	dValue	= Dialog_Wait->lineEdit_Wait_Value->text().toDouble();
	int		iUnit	= Dialog_Wait->comboBox_Wait_Unit->currentIndex();

	//设置参数
	m_pProcessWait->SetWaitValue(dValue);
	m_pProcessWait->SetWaitUnit(iUnit);

	m_pProcessWait->SetState(ItemState::Enable);
	m_pProcessWait->SwitchState(ItemState::Enable);
	m_pProcessWait->UpdateInfo();
	QDialog::accept();
}

void Dialog_ProcessSetting_Wait::ButtonCancel()
{
	//点击取消/ESC退出均不保存参数，还原节点状态
	reject();
}

void Dialog_ProcessSetting_Wait::reject()
{
	m_pProcessWait->SwitchState();
	m_pProcessWait->UpdateInfo();
	QDialog::reject();
}