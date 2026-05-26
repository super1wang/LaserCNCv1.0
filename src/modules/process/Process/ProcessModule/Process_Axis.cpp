#include "Process_Axis.h"
#include <qDebug>
#include <QRegExpValidator>

ProcessAxis::ProcessAxis(TreeItem *parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Axis;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Note",""}, {"Axis","X"}, {"Speed","1"}, {"Mode","0"}, {"Pos",""} };
	m_itemData.resize(2);
}

ProcessAxis::ProcessAxis(const QString &text, TreeItem *parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Axis;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Note",""}, {"Axis","X"}, {"Speed","1"}, {"Mode","0"}, {"Pos",""} };
	m_itemData.resize(2);
	UpdateInfo();
}

ProcessAxis::ProcessAxis(const QVector<QVariant> &data, TreeItem *parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Axis;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Note",""}, {"Axis","X"}, {"Speed","1"}, {"Mode","0"}, {"Pos",""} };
	m_itemData.resize(2);
	m_itemData		= data;
}

ProcessAxis::~ProcessAxis(void)
{
	qDeleteAll(m_childItems);
	m_childItems.clear();
	qDeleteAll(m_childItems);
}

TreeItem* ProcessAxis::clone() const
{
	TreeItem* newItem = nullptr;

	if (!m_itemData.isEmpty()) {
		newItem = new ProcessAxis(QStringLiteral("Axis"));
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

void ProcessAxis::Edit() 
{
	if (m_state == ItemState::Unavailable || m_state == ItemState::Enable || m_state == ItemState::Disable)
	{
		Dialog_ProcessSetting_Axis* Dialog = new Dialog_ProcessSetting_Axis();
		Dialog->setAttribute(Qt::WA_DeleteOnClose);    //关闭对话框的同时，回收内存
		Dialog->m_pProcessAxis = this;
		Dialog->ViewSetting();
		Dialog->show();
	}
}

void ProcessAxis::UpdateInfo()
{
	if (TreeItem::IsChinese())
		setData(0, "\xE8\xBD\xB4\xE8\xBF\x90\xE5\x8A\xA8");	// 轴运动
	else
		setData(0, "Axis");

	setData(1, m_maps["Note"]);
}

void ProcessAxis::SwitchState(ItemState state)
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


Dialog_ProcessSetting_Axis::Dialog_ProcessSetting_Axis(QWidget* parent) :
	QDialog(parent),
	m_pProcessAxis(nullptr),
	Dialog_Axis(new Ui::Dialog_ProcessSetting_Axis)
{
	Dialog_Axis->setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowTitleHint);

	connect(Dialog_Axis->Button_Axis_OK,		SIGNAL(clicked()), this, SLOT(ButtonOK()));
	connect(Dialog_Axis->Button_Axis_Cancel,	SIGNAL(clicked()), this, SLOT(ButtonCancel()));
}

Dialog_ProcessSetting_Axis::~Dialog_ProcessSetting_Axis()
{
	delete Dialog_Axis;
}

void Dialog_ProcessSetting_Axis::ViewSetting()
{
	//修改标签状态，避免重复打开标签设置
	m_pProcessAxis->SwitchState(ItemState::Editing);

	//限制输入值
	//Dialog_Axis->lineEdit_Axis_Pos->setValidator(new QRegExpValidator(Regex_Pos_Double));
	
	//设置可选轴
	for (int i = 0; i < 8; i++)
	{
		if (DT::IsAxisUse((Axis)i))
		{
			Dialog_Axis->comboBox_Axis_Axis->addItem(QString::fromStdString(enum_name((Axis)i).data()));
		}
	}

	//显示参数
	Dialog_Axis->lineEdit_Axis_Note	->setText			(m_pProcessAxis->GetAxisNote());
	Dialog_Axis->comboBox_Axis_Axis	->setCurrentText	(m_pProcessAxis->GetAxisAxis());
	Dialog_Axis->comboBox_Axis_Speed->setCurrentIndex	(m_pProcessAxis->GetAxisSpeed());
	Dialog_Axis->comboBox_Axis_Model->setCurrentIndex	(m_pProcessAxis->GetAxisModel());
	Dialog_Axis->lineEdit_Axis_Pos	->setText			(m_pProcessAxis->GetAxisPos());
}

void Dialog_ProcessSetting_Axis::ButtonOK()
{
	//获取参数
	QString qstrNote	= Dialog_Axis->lineEdit_Axis_Note->text();
	QString qstrAxis	= Dialog_Axis->comboBox_Axis_Axis->currentText();
	int		iSpeed		= Dialog_Axis->comboBox_Axis_Speed->currentIndex();
	int		iModel		= Dialog_Axis->comboBox_Axis_Model->currentIndex();
	QString qstrPos		= Dialog_Axis->lineEdit_Axis_Pos->text();

	//设置参数
	m_pProcessAxis->SetAxisNote	(qstrNote);
	m_pProcessAxis->SetAxisAxis	(qstrAxis);
	m_pProcessAxis->SetAxisSpeed(iSpeed);
	m_pProcessAxis->SetAxisModel(iModel);
	m_pProcessAxis->SetAxisPos	(qstrPos);

	m_pProcessAxis->SetState(ItemState::Enable);
	m_pProcessAxis->SwitchState(ItemState::Enable);
	m_pProcessAxis->UpdateInfo();
	QDialog::accept();
}

void Dialog_ProcessSetting_Axis::ButtonCancel()
{
	//点击取消/ESC退出均不保存参数，还原节点状态
	reject();
}

void Dialog_ProcessSetting_Axis::reject()
{
	m_pProcessAxis->SwitchState();
	m_pProcessAxis->UpdateInfo();
	QDialog::reject();
}