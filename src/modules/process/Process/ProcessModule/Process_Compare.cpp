#include "Process_Compare.h"
#include <qDebug>


ProcessCompare::ProcessCompare(TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Compare;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Note",""}, {"Formula",""}, {"True",""}, {"False",""} };
	m_itemData.resize(2);
}

ProcessCompare::ProcessCompare(const QString& text, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Compare;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Note",""}, {"Formula",""}, {"True",""}, {"False",""} };
	m_itemData.resize(2);
	UpdateInfo();
}

ProcessCompare::ProcessCompare(const QVector<QVariant>& data, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Compare;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Note",""}, {"Formula",""}, {"True",""}, {"False",""} };
	m_itemData.resize(2);
	m_itemData		= data;
}

ProcessCompare::~ProcessCompare(void)
{
	qDeleteAll(m_childItems);
	m_childItems.clear();
	qDeleteAll(m_childItems);
}

TreeItem* ProcessCompare::clone() const
{
	TreeItem* newItem = nullptr;

	if (!m_itemData.isEmpty()) {
		newItem = new ProcessCompare(QStringLiteral("Compare"));
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

void ProcessCompare::Edit()
{
	if (m_state == ItemState::Unavailable || m_state == ItemState::Enable || m_state == ItemState::Disable)
	{
		Dialog_ProcessSetting_Compare* Dialog = new Dialog_ProcessSetting_Compare();
		Dialog->setAttribute(Qt::WA_DeleteOnClose);    //关闭对话框的同时，回收内存
		Dialog->m_pProcessCompare = this;
		Dialog->ViewSetting();
		Dialog->show();
	}
}

void ProcessCompare::UpdateInfo()
{
	if (TreeItem::IsChinese())
		setData(0, "\xE5\xAF\xB9\xE6\xAF\x94");	// 对比
	else
		setData(0, "Compare");
	
	setData(1, m_maps["Note"]);
}

void ProcessCompare::SwitchState(ItemState state)
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


Dialog_ProcessSetting_Compare::Dialog_ProcessSetting_Compare(QWidget* parent) :
	QDialog(parent),
	m_pProcessCompare(nullptr),
	Dialog_Compare(new Ui::Dialog_ProcessSetting_Compare)
{
	Dialog_Compare->setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowTitleHint);

	connect(Dialog_Compare->Button_Compare_OK,		SIGNAL(clicked()), this, SLOT(ButtonOK()));
	connect(Dialog_Compare->Button_Compare_Cancel,	SIGNAL(clicked()), this, SLOT(ButtonCancel()));
}

Dialog_ProcessSetting_Compare::~Dialog_ProcessSetting_Compare()
{
	delete Dialog_Compare;
}

void Dialog_ProcessSetting_Compare::ViewSetting()
{
	//修改标签状态，避免重复打开标签设置
	m_pProcessCompare->SwitchState(ItemState::Editing);

	Dialog_Compare->lineEdit_Compare_Note	->setText(m_pProcessCompare->GetCompareNote());
	Dialog_Compare->lineEdit_Compare_Formula->setText(m_pProcessCompare->GetCompareFormula());
	Dialog_Compare->lineEdit_Compare_True	->setText(m_pProcessCompare->GetCompareTrue());
	Dialog_Compare->lineEdit_Compare_False	->setText(m_pProcessCompare->GetCompareFalse());
}

void Dialog_ProcessSetting_Compare::ButtonOK()
{
	//获取参数
	QString QstrNote	= Dialog_Compare->lineEdit_Compare_Note		->text();
	QString	QstrFormula	= Dialog_Compare->lineEdit_Compare_Formula	->text();
	QString QstrTrue	= Dialog_Compare->lineEdit_Compare_True		->text();
	QString QstrFalse	= Dialog_Compare->lineEdit_Compare_False	->text();

	//设置参数
	m_pProcessCompare->SetCompareNote	(QstrNote);
	m_pProcessCompare->SetCompareFormula(QstrFormula);
	m_pProcessCompare->SetCompareTrue	(QstrTrue);
	m_pProcessCompare->SetCompareFalse	(QstrFalse);

	m_pProcessCompare->SetState(ItemState::Enable);
	m_pProcessCompare->SwitchState(ItemState::Enable);
	m_pProcessCompare->UpdateInfo();
	QDialog::accept();
}

void Dialog_ProcessSetting_Compare::ButtonCancel()
{
	//点击取消/ESC退出均不保存参数，还原节点状态
	reject();
}

void Dialog_ProcessSetting_Compare::reject()
{
	m_pProcessCompare->SwitchState();
	m_pProcessCompare->UpdateInfo();
	QDialog::reject();
}