#include "Process_Cutting.h"
#include <qDebug>


ProcessCutting::ProcessCutting(TreeItem *parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Cutting;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Note",""}, {"Count", "1"}, {"StartNumber",""}, {"EndNumber",""}, {"CompensationIndex", ""}};
	m_itemData.resize(2);
}

ProcessCutting::ProcessCutting(const QString& text, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Cutting;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Note",""}, {"Count", "1"}, {"StartNumber",""}, {"EndNumber",""}, {"CompensationIndex", ""} };
	m_itemData.resize(2);
	UpdateInfo();
}

ProcessCutting::ProcessCutting(const QVector<QVariant>& data, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Cutting;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Note",""}, {"Count", "1"}, {"StartNumber",""}, {"EndNumber",""}, {"CompensationIndex", ""} };
	m_itemData.resize(2);
	m_itemData		= data;
}

ProcessCutting::~ProcessCutting(void)
{
	qDeleteAll(m_childItems);
	m_childItems.clear();
	qDeleteAll(m_childItems);
}

TreeItem* ProcessCutting::clone() const
{
	TreeItem* newItem = nullptr;

	if (!m_itemData.isEmpty()) {
		newItem = new ProcessCutting(QStringLiteral("Cutting"));
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

void ProcessCutting::Edit()
{
	if (m_state == ItemState::Unavailable || m_state == ItemState::Enable || m_state == ItemState::Disable)
	{
		Dialog_ProcessSetting_Cutting* Dialog = new Dialog_ProcessSetting_Cutting();
		Dialog->setAttribute(Qt::WA_DeleteOnClose);    //关闭对话框的同时，回收内存
		Dialog->m_pProcessCutting = this;
		Dialog->ViewSetting();
		Dialog->show();
	}
}

void ProcessCutting::UpdateInfo()
{
	if (TreeItem::IsChinese())
		setData(0, "\xE5\x88\x87\xE5\x89\xB2"); // 切割
	else
		setData(0, "Cutting");
	
	setData(1, m_maps["Note"]);
}

void ProcessCutting::SwitchState(ItemState state)
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


Dialog_ProcessSetting_Cutting::Dialog_ProcessSetting_Cutting(QWidget* parent) :
	QDialog(parent),
	m_pProcessCutting(nullptr),
	Dialog_Cutting(new Ui::Dialog_ProcessSetting_Cutting)
{
	Dialog_Cutting->setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowTitleHint);

	connect(Dialog_Cutting->Button_Cutting_OK,		SIGNAL(clicked()), this, SLOT(ButtonOK()));
	connect(Dialog_Cutting->Button_Cutting_Cancel,	SIGNAL(clicked()), this, SLOT(ButtonCancel()));
}

Dialog_ProcessSetting_Cutting::~Dialog_ProcessSetting_Cutting()
{
	delete Dialog_Cutting;
}

void Dialog_ProcessSetting_Cutting::ViewSetting()
{
	//修改标签状态，避免重复打开标签设置
	m_pProcessCutting->SwitchState(ItemState::Editing);

	//限制输入值
	Dialog_Cutting->lineEdit_Cutting_StartNumber->setValidator(new QRegExpValidator(Regex_Positive_Int));
	Dialog_Cutting->lineEdit_Cutting_StartNumber->setValidator(new QRegExpValidator(Regex_Positive_Int));

	//读取参数
	QString QstrNote				= m_pProcessCutting->GetCuttingNote();
	bool	bCount					= m_pProcessCutting->GetCuttingCount();
	QString QstrStartNumber			= m_pProcessCutting->GetCuttingStartNumber();
	QString QstrEndNumber			= m_pProcessCutting->GetCuttingEndNumber();
	QString QstrCompensationIndex	= m_pProcessCutting->GetCuttingCompensationIndex();

	//显示参数
	Dialog_Cutting->lineEdit_Cutting_Note				->setText(QstrNote);
	Dialog_Cutting->checkBox_Cutting_Count				->setChecked(bCount);
	Dialog_Cutting->lineEdit_Cutting_StartNumber		->setText(QstrStartNumber);
	Dialog_Cutting->lineEdit_Cutting_EndNumber			->setText(QstrEndNumber);
	Dialog_Cutting->lineEdit_Cutting_CompensationIndex	->setText(QstrCompensationIndex);
}

void Dialog_ProcessSetting_Cutting::ButtonOK()
{
	//获取参数
	QString QstrNote				= Dialog_Cutting->lineEdit_Cutting_Note				->text();
	bool	bCount					= Dialog_Cutting->checkBox_Cutting_Count			->isChecked();
	QString QstrStartNumber			= Dialog_Cutting->lineEdit_Cutting_StartNumber		->text();
	QString QstrEndNumber			= Dialog_Cutting->lineEdit_Cutting_EndNumber		->text();
	QString QstrCompensationIndex	= Dialog_Cutting->lineEdit_Cutting_CompensationIndex->text();

	//设置参数
	m_pProcessCutting->SetCuttingNote(QstrNote);
	m_pProcessCutting->SetCuttingCount(bCount);
	m_pProcessCutting->SetCuttingStartNumber(QstrStartNumber);
	m_pProcessCutting->SetCuttingEndNumber(QstrEndNumber);
	m_pProcessCutting->SetCuttingCompensationIndex(QstrCompensationIndex);

	m_pProcessCutting->SetState(ItemState::Enable);
	m_pProcessCutting->SwitchState(ItemState::Enable);
	m_pProcessCutting->UpdateInfo();
	QDialog::accept();
}

void Dialog_ProcessSetting_Cutting::ButtonCancel()
{
	//点击取消/ESC退出均不保存参数，还原节点状态
	reject();
}

void Dialog_ProcessSetting_Cutting::reject()
{
	m_pProcessCutting->SwitchState();
	m_pProcessCutting->UpdateInfo();
	QDialog::reject();
}