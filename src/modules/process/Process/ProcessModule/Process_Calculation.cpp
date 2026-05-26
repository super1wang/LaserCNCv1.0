#include "Process_Calculation.h"
#include <qDebug>


ProcessCalculation::ProcessCalculation(TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Calculation;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Index",""}, {"X","0"}, {"Y","0"} };
	m_itemData.resize(2);
}

ProcessCalculation::ProcessCalculation(const QString& text, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Calculation;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Index",""}, {"X","0"}, {"Y","0"} };
	m_itemData.resize(2);
	UpdateInfo();
}

ProcessCalculation::ProcessCalculation(const QVector<QVariant>& data, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Calculation;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Index",""}, {"X","0"}, {"Y","0"} };
	m_itemData.resize(2);
	m_itemData		= data;
}

ProcessCalculation::~ProcessCalculation(void)
{
	qDeleteAll(m_childItems);
	m_childItems.clear();
	qDeleteAll(m_childItems);
}

TreeItem* ProcessCalculation::clone() const
{
	TreeItem* newItem = nullptr;

	if (!m_itemData.isEmpty()) {
		newItem = new ProcessCalculation(QStringLiteral("Calculation"));
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

void ProcessCalculation::Edit()
{
	if (m_state == ItemState::Unavailable || m_state == ItemState::Enable || m_state == ItemState::Disable)
	{
		Dialog_ProcessSetting_Calculation* Dialog = new Dialog_ProcessSetting_Calculation();
		Dialog->setAttribute(Qt::WA_DeleteOnClose);    //关闭对话框的同时，回收内存
		Dialog->m_pProcessCalculation = this;
		Dialog->ViewSetting();
		Dialog->show();
	}
}

void ProcessCalculation::UpdateInfo()
{
	if (TreeItem::IsChinese())
	{
		setData(0, "\xE8\xAE\xA1\xE7\xAE\x97");	// 计算
		setData(1, "\xE7\xBB\x93\xE6\x9E\x9C\xE7\xB4\xA2\xE5\xBC\x95 " + m_maps["Index"]);// 结果索引
	}
	else
	{
		setData(0, "Calculation");
		setData(1, "Result index " + m_maps["Index"]);
	}
}

void ProcessCalculation::SwitchState(ItemState state)
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


Dialog_ProcessSetting_Calculation::Dialog_ProcessSetting_Calculation(QWidget* parent) :
	QDialog(parent),
	m_pProcessCalculation(nullptr),
	Dialog_Calculation(new Ui::Dialog_ProcessSetting_Calculation)
{
	Dialog_Calculation->setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowTitleHint);

	connect(Dialog_Calculation->Button_Calculation_OK,		SIGNAL(clicked()), this, SLOT(ButtonOK()));
	connect(Dialog_Calculation->Button_Calculation_Cancel,	SIGNAL(clicked()), this, SLOT(ButtonCancel()));
}

Dialog_ProcessSetting_Calculation::~Dialog_ProcessSetting_Calculation()
{
	delete Dialog_Calculation;
}

void Dialog_ProcessSetting_Calculation::ViewSetting()
{
	//修改标签状态，避免重复打开标签设置
	m_pProcessCalculation->SetState(ItemState::Editing);

	//获取参数
	QString	QstrIndex 	= m_pProcessCalculation->GetCalculationIndex();
	QString	QstrX 		= m_pProcessCalculation->GetCalculationX();
	QString	QstrY 		= m_pProcessCalculation->GetCalculationY();

	//填入参数
	Dialog_Calculation->lineEdit_Calculation_Index	->setText(QstrIndex);
	Dialog_Calculation->lineEdit_Calculation_X		->setText(QstrX);
	Dialog_Calculation->lineEdit_Calculation_Y		->setText(QstrY);
}

void Dialog_ProcessSetting_Calculation::ButtonOK()
{
	//获取参数
	QString	QstrIndex 	= Dialog_Calculation->lineEdit_Calculation_Index->text();
	QString	QstrX 		= Dialog_Calculation->lineEdit_Calculation_X->text();
	QString	QstrY 		= Dialog_Calculation->lineEdit_Calculation_Y->text();
	
	//保存参数
	m_pProcessCalculation->SetCalculationIndex(QstrIndex);
	m_pProcessCalculation->SetCalculationX(QstrX);
	m_pProcessCalculation->SetCalculationY(QstrY);

	m_pProcessCalculation->SetState(ItemState::Enable);
	m_pProcessCalculation->SwitchState(ItemState::Enable);
	m_pProcessCalculation->UpdateInfo();
	QDialog::accept();
}

void Dialog_ProcessSetting_Calculation::ButtonCancel()
{
	//点击取消/ESC退出均不保存参数，还原节点状态
	reject();
}

void Dialog_ProcessSetting_Calculation::reject()
{
	m_pProcessCalculation->SwitchState();
	m_pProcessCalculation->UpdateInfo();
	QDialog::reject();
}