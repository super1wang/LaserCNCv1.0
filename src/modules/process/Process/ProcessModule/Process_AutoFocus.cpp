#include "Process_AutoFocus.h"
#include <qDebug>
#include <QRegExpValidator>

ProcessAutoFocus::ProcessAutoFocus(TreeItem *parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::AutoFocus;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Note", ""}, {"StepInterval", "1"}, {"EnergyStep", "0"}, {"FrequencyStep", "0"}, {"PulseWidthStep", "0"}, {"CuttingHighStep", "0"} };
	m_itemData.resize(2);
}

ProcessAutoFocus::ProcessAutoFocus(const QString &text, TreeItem *parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::AutoFocus;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Note", ""}, {"StepInterval", "1"}, {"EnergyStep", "0"}, {"FrequencyStep", "0"}, {"PulseWidthStep", "0"}, {"CuttingHighStep", "0"} };
	m_itemData.resize(2);
	UpdateInfo();
}

ProcessAutoFocus::ProcessAutoFocus(const QVector<QVariant> &data, TreeItem *parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::AutoFocus;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Note", ""}, {"StepInterval", "1"}, {"EnergyStep", "0"}, {"FrequencyStep", "0"}, {"PulseWidthStep", "0"}, {"CuttingHighStep", "0"} };
	m_itemData.resize(2);
	m_itemData		= data;
}

ProcessAutoFocus::~ProcessAutoFocus(void)
{
	qDeleteAll(m_childItems);
	m_childItems.clear();
	qDeleteAll(m_childItems);
}

TreeItem* ProcessAutoFocus::clone() const
{
	TreeItem* newItem = nullptr;

	if (!m_itemData.isEmpty()) {
		newItem = new ProcessAutoFocus(QStringLiteral("AutoFocus"));
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

void ProcessAutoFocus::Edit() 
{
	if (m_state == ItemState::Unavailable || m_state == ItemState::Enable || m_state == ItemState::Disable)
	{
		Dialog_ProcessSetting_AutoFocus* Dialog = new Dialog_ProcessSetting_AutoFocus();
		Dialog->setAttribute(Qt::WA_DeleteOnClose);    //关闭对话框的同时，回收内存
		Dialog->m_pProcessAutoFocus = this;
		Dialog->ViewSetting();
		Dialog->show();
	}
}

void ProcessAutoFocus::UpdateInfo()
{
	if (TreeItem::IsChinese())
		setData(0, "\xE8\x87\xAA\xE5\x8A\xA8\xE5\xAF\xBB\xE7\x84\xA6");// 自动寻焦
	else
		setData(0, "AutoFocus");
	
	setData(1, m_maps["Note"]);
}

void ProcessAutoFocus::SwitchState(ItemState state)
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


Dialog_ProcessSetting_AutoFocus::Dialog_ProcessSetting_AutoFocus(QWidget* parent) :
	QDialog(parent),
	m_pProcessAutoFocus(nullptr),
	Dialog_AutoFocus(new Ui::Dialog_ProcessSetting_AutoFocus)
{
	Dialog_AutoFocus->setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowTitleHint);

	connect(Dialog_AutoFocus->Button_AutoFocus_OK,		SIGNAL(clicked()), this, SLOT(ButtonOK()));
	connect(Dialog_AutoFocus->Button_AutoFocus_Cancel,	SIGNAL(clicked()), this, SLOT(ButtonCancel()));
}

Dialog_ProcessSetting_AutoFocus::~Dialog_ProcessSetting_AutoFocus()
{
	delete Dialog_AutoFocus;
}

void Dialog_ProcessSetting_AutoFocus::ViewSetting()
{
	//修改标签状态，避免重复打开标签设置
	m_pProcessAutoFocus->SwitchState(ItemState::Editing);

	//限制输入值
	Dialog_AutoFocus->lineEdit_AutoFocus_StepInterval	->setValidator(new QRegExpValidator(Regex_Nonnegative_Int));
	Dialog_AutoFocus->lineEdit_AutoFocus_EnergyStep		->setValidator(new QRegExpValidator(Regex_All_Double));
	Dialog_AutoFocus->lineEdit_AutoFocus_FrequencyStep	->setValidator(new QRegExpValidator(Regex_All_Double));
	Dialog_AutoFocus->lineEdit_AutoFocus_PulseWidthStep	->setValidator(new QRegExpValidator(Regex_All_Double));
	Dialog_AutoFocus->lineEdit_AutoFocus_CuttingHighStep->setValidator(new QRegExpValidator(Regex_All_Double));

	//显示参数
	Dialog_AutoFocus->lineEdit_AutoFocus_Note			->setText(m_pProcessAutoFocus->GetAutoFocusNote());
	Dialog_AutoFocus->lineEdit_AutoFocus_StepInterval	->setText(m_pProcessAutoFocus->GetAutoFocusStepInterval());
	Dialog_AutoFocus->lineEdit_AutoFocus_EnergyStep		->setText(m_pProcessAutoFocus->GetAutoFocusEnergyStep());
	Dialog_AutoFocus->lineEdit_AutoFocus_FrequencyStep	->setText(m_pProcessAutoFocus->GetAutoFocusFrequencyStep());
	Dialog_AutoFocus->lineEdit_AutoFocus_PulseWidthStep	->setText(m_pProcessAutoFocus->GetAutoFocusPulseWidthStep());
	Dialog_AutoFocus->lineEdit_AutoFocus_CuttingHighStep->setText(m_pProcessAutoFocus->GetAutoFocusCuttingHighStep());
}

void Dialog_ProcessSetting_AutoFocus::ButtonOK()
{
	//获取参数
	QString qstrNote			= Dialog_AutoFocus->lineEdit_AutoFocus_Note				->text();
	QString qstrStepInterval	= Dialog_AutoFocus->lineEdit_AutoFocus_StepInterval		->text();
	QString qstrEnergyStep		= Dialog_AutoFocus->lineEdit_AutoFocus_EnergyStep		->text();
	QString qstrFrequencyStep	= Dialog_AutoFocus->lineEdit_AutoFocus_FrequencyStep	->text();
	QString qstrPulseWidthStep	= Dialog_AutoFocus->lineEdit_AutoFocus_PulseWidthStep	->text();
	QString qstrCuttingHighStep = Dialog_AutoFocus->lineEdit_AutoFocus_CuttingHighStep	->text();

	//设置参数
	m_pProcessAutoFocus->SetAutoFocusNote			(qstrNote);
	m_pProcessAutoFocus->SetAutoFocusStepInterval	(qstrStepInterval);
	m_pProcessAutoFocus->SetAutoFocusEnergyStep		(qstrEnergyStep);
	m_pProcessAutoFocus->SetAutoFocusFrequencyStep	(qstrFrequencyStep);
	m_pProcessAutoFocus->SetAutoFocusPulseWidthStep	(qstrPulseWidthStep);
	m_pProcessAutoFocus->SetAutoFocusCuttingHighStep(qstrCuttingHighStep);

	m_pProcessAutoFocus->SetState(ItemState::Enable);
	m_pProcessAutoFocus->SwitchState(ItemState::Enable);
	m_pProcessAutoFocus->UpdateInfo();
	QDialog::accept();
}

void Dialog_ProcessSetting_AutoFocus::ButtonCancel()
{
	//点击取消/ESC退出均不保存参数，还原节点状态
	reject();
}

void Dialog_ProcessSetting_AutoFocus::reject()
{
	m_pProcessAutoFocus->SwitchState();
	m_pProcessAutoFocus->UpdateInfo();
	QDialog::reject();
}