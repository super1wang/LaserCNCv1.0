#include "Process_Feeding.h"
#include <qDebug>


ProcessFeeding::ProcessFeeding(TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Feeding;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"OpenChuck","500"}, {"OpenPliers","500"}, {"CloseChuck","500"}, {"ClosePliers","500"}, {"Compensation","0"}, {"AxisSpeed","1"} };
	m_itemData.resize(2);
}

ProcessFeeding::ProcessFeeding(const QString& text, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Feeding;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"OpenChuck","500"}, {"OpenPliers","500"}, {"CloseChuck","500"}, {"ClosePliers","500"}, {"Compensation","0"}, {"AxisSpeed","1"} };
	m_itemData.resize(2);
	UpdateInfo();
}

ProcessFeeding::ProcessFeeding(const QVector<QVariant>& data, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Feeding;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"OpenChuck","500"}, {"OpenPliers","500"}, {"CloseChuck","500"}, {"ClosePliers","500"}, {"Compensation","0"}, {"AxisSpeed","1"} };
	m_itemData.resize(2);
	m_itemData		= data;
}

ProcessFeeding::~ProcessFeeding(void)
{
	qDeleteAll(m_childItems);
	m_childItems.clear();
	qDeleteAll(m_childItems);
}

TreeItem* ProcessFeeding::clone() const
{
	TreeItem* newItem = nullptr;

	if (!m_itemData.isEmpty()) {
		newItem = new ProcessFeeding(QStringLiteral("Feeding"));
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

void ProcessFeeding::Edit()
{
	if (m_state == ItemState::Unavailable || m_state == ItemState::Enable || m_state == ItemState::Disable)
	{
		Dialog_ProcessSetting_Feeding* Dialog = new Dialog_ProcessSetting_Feeding();
		Dialog->setAttribute(Qt::WA_DeleteOnClose);    //关闭对话框的同时，回收内存
		Dialog->m_pProcessFeeding = this;
		Dialog->ViewSetting();
		Dialog->show();
	}
}

void ProcessFeeding::UpdateInfo()
{
	if (TreeItem::IsChinese())
	{
		setData(0, "\xE8\xBF\x9B\xE7\xBB\x99"); // 进给
		setData(1, "\xE8\xBF\x9B\xE7\xBB\x99\xE8\xA1\xA5\xE5\x81\xBF " + m_maps["Compensation"] + " \xE6\xAF\xAB\xE7\xB1\xB3"); // 进给补偿 毫米
	}
	else
	{
		setData(0, "Feeding");
		setData(1, "Compensation " + m_maps["Compensation"] + " mm");
	}
}

void ProcessFeeding::SwitchState(ItemState state)
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


Dialog_ProcessSetting_Feeding::Dialog_ProcessSetting_Feeding(QWidget* parent) :
	QDialog(parent),
	m_pProcessFeeding(nullptr),
	Dialog_Feeding(new Ui::Dialog_ProcessSetting_Feeding)
{
	Dialog_Feeding->setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowTitleHint);

	connect(Dialog_Feeding->Button_Feeding_OK,		SIGNAL(clicked()), this, SLOT(ButtonOK()));
	connect(Dialog_Feeding->Button_Feeding_Cancel,	SIGNAL(clicked()), this, SLOT(ButtonCancel()));
}

Dialog_ProcessSetting_Feeding::~Dialog_ProcessSetting_Feeding()
{
	delete Dialog_Feeding;
}

void Dialog_ProcessSetting_Feeding::ViewSetting()
{
	//修改标签状态，避免重复打开标签设置
	m_pProcessFeeding->SwitchState(ItemState::Editing);

	//限制输入值
	Dialog_Feeding->lineEdit_Feeding_OpenChuckDelay		->setValidator(new QRegExpValidator(Regex_Nonnegative_Int));
	Dialog_Feeding->lineEdit_Feeding_OpenPliersDelay	->setValidator(new QRegExpValidator(Regex_Nonnegative_Int));
	Dialog_Feeding->lineEdit_Feeding_CloseChuckDelay	->setValidator(new QRegExpValidator(Regex_Nonnegative_Int));
	Dialog_Feeding->lineEdit_Feeding_ClosePliersDelay	->setValidator(new QRegExpValidator(Regex_Nonnegative_Int));
	Dialog_Feeding->lineEdit_Feeding_Compensation		->setValidator(new QRegExpValidator(Regex_Pos_Double));

	//读取参数
	QString		QstrFeedingOpenChuck	= m_pProcessFeeding->GetFeedingOpenChuck();
	QString		QstrFeedingOpenPliers	= m_pProcessFeeding->GetFeedingOpenPliers();
	QString		QstrFeedingCloseChuck	= m_pProcessFeeding->GetFeedingCloseChuck();
	QString		QstrFeedingClosePliers	= m_pProcessFeeding->GetFeedingClosePliers();
	QString		QstrFeedingCompensation = m_pProcessFeeding->GetFeedingCompensation();
	int			iFeedingAxisSpeed		= m_pProcessFeeding->GetFeedingAxisSpeed();

	//显示参数
	Dialog_Feeding->lineEdit_Feeding_OpenChuckDelay		->setText(QstrFeedingOpenChuck);
	Dialog_Feeding->lineEdit_Feeding_OpenPliersDelay	->setText(QstrFeedingOpenPliers);
	Dialog_Feeding->lineEdit_Feeding_CloseChuckDelay	->setText(QstrFeedingCloseChuck);
	Dialog_Feeding->lineEdit_Feeding_ClosePliersDelay	->setText(QstrFeedingClosePliers);
	Dialog_Feeding->lineEdit_Feeding_Compensation		->setText(QstrFeedingCompensation);
	Dialog_Feeding->comboBox_Feeding_Speed		->setCurrentIndex(iFeedingAxisSpeed);
}

void Dialog_ProcessSetting_Feeding::ButtonOK()
{
	//获取参数
	QString		QstrFeedingOpenChuck	= Dialog_Feeding->lineEdit_Feeding_OpenChuckDelay	->text();
	QString		QstrFeedingOpenPliers	= Dialog_Feeding->lineEdit_Feeding_OpenPliersDelay	->text();
	QString		QstrFeedingCloseChuck	= Dialog_Feeding->lineEdit_Feeding_CloseChuckDelay	->text();
	QString		QstrFeedingClosePliers	= Dialog_Feeding->lineEdit_Feeding_ClosePliersDelay	->text();
	QString		QstrFeedingCompensation = Dialog_Feeding->lineEdit_Feeding_Compensation		->text();
	int			iFeedingAxisSpeed		= Dialog_Feeding->comboBox_Feeding_Speed	->currentIndex();

	//设置参数
	m_pProcessFeeding->SetFeedingOpenChuck(QstrFeedingOpenChuck);
	m_pProcessFeeding->SetFeedingOpenPliers(QstrFeedingOpenPliers);
	m_pProcessFeeding->SetFeedingCloseChuck(QstrFeedingCloseChuck);
	m_pProcessFeeding->SetFeedingClosePliers(QstrFeedingClosePliers);
	m_pProcessFeeding->SetFeedingCompensation(QstrFeedingCompensation);
	m_pProcessFeeding->SetFeedingAxisSpeed(iFeedingAxisSpeed);

	m_pProcessFeeding->SetState(ItemState::Enable);
	m_pProcessFeeding->SwitchState(ItemState::Enable);
	m_pProcessFeeding->UpdateInfo();
	QDialog::accept();
}

void Dialog_ProcessSetting_Feeding::ButtonCancel()
{
	//点击取消/ESC退出均不保存参数，还原节点状态
	reject();
}

void Dialog_ProcessSetting_Feeding::reject()
{
	m_pProcessFeeding->SwitchState();
	m_pProcessFeeding->UpdateInfo();
	QDialog::reject();
}
