#include "Process_If.h"
#include <qDebug>


ProcessIf::ProcessIf(TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::If;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Note",""}, {"Type",""}, {"IO",""}, {"On",""}, {"Off",""} };
	m_itemData.resize(2);
}

ProcessIf::ProcessIf(const QString& text, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::If;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Note",""}, {"Type",""}, {"IO",""}, {"On",""}, {"Off",""} };
	m_itemData.resize(2);
	UpdateInfo();
}

ProcessIf::ProcessIf(const QVector<QVariant>& data, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::If;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Note",""}, {"Type",""}, {"IO",""}, {"On",""}, {"Off",""} };
	m_itemData.resize(2);
	m_itemData		= data;
}

ProcessIf::~ProcessIf(void)
{
	qDeleteAll(m_childItems);
	m_childItems.clear();
	qDeleteAll(m_childItems);
}

TreeItem* ProcessIf::clone() const
{
	TreeItem* newItem = nullptr;

	if (!m_itemData.isEmpty()) {
		newItem = new ProcessIf(QStringLiteral("If"));
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

void ProcessIf::Edit()
{
	if (m_state == ItemState::Unavailable || m_state == ItemState::Enable || m_state == ItemState::Disable)
	{
		Dialog_ProcessSetting_If* Dialog = new Dialog_ProcessSetting_If();
		Dialog->setAttribute(Qt::WA_DeleteOnClose);    //关闭对话框的同时，回收内存
		Dialog->m_pProcessIf = this;
		Dialog->ViewSetting();
		Dialog->show();
	}
}

void ProcessIf::UpdateInfo()
{
	QString qstrIf = "\xE4\xBF\xA1\xE5\x8F\xB7\xE8\xB7\xB3\xE8\xBD\xAC";// 信号跳转
	if (m_maps.at("On").isEmpty() || m_maps.at("Off").isEmpty())
		qstrIf = "\xE7\xAD\x89\xE5\xBE\x85\xE4\xBF\xA1\xE5\x8F\xB7"; // 等待信号

	if (TreeItem::IsChinese())
		setData(0, qstrIf);
	else
		setData(0, "If");
	
	setData(1, m_maps["Note"]);
}

void ProcessIf::SwitchState(ItemState state)
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


Dialog_ProcessSetting_If::Dialog_ProcessSetting_If(QWidget* parent) :
	QDialog(parent),
	m_pProcessIf(nullptr),
	Dialog_If(new Ui::Dialog_ProcessSetting_If)
{
	Dialog_If->setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowTitleHint);

	connect(Dialog_If->Button_If_OK,		SIGNAL(clicked()), this, SLOT(ButtonOK()));
	connect(Dialog_If->Button_If_Cancel,	SIGNAL(clicked()), this, SLOT(ButtonCancel()));
	connect(Dialog_If->comboBox_If_Type,	SIGNAL(currentIndexChanged(int)), this, SLOT(UpdatePage()));
}

Dialog_ProcessSetting_If::~Dialog_ProcessSetting_If()
{
	delete Dialog_If;
}

void Dialog_ProcessSetting_If::ViewSetting()
{
	//修改标签状态，避免重复打开标签设置
	m_pProcessIf->SwitchState(ItemState::Editing);

	//获取IO列表
	m_DigitalOUTList = DT::getDigitalOUTList();
	m_DigitalINList = DT::getDigitalINList();

	//显示参数
	Dialog_If->comboBox_If_Type->blockSignals(true);
	if (m_pProcessIf->GetIfType()) {
		Dialog_If->comboBox_If_Type->setCurrentIndex(1);
		Dialog_If->comboBox_If_IO->addItems(m_DigitalOUTList);
		if (!m_DigitalOUTList.count(m_pProcessIf->GetIfIO()))
			Dialog_If->comboBox_If_IO->addItem(m_pProcessIf->GetIfIO());
	}
	else {
		Dialog_If->comboBox_If_Type->setCurrentIndex(0);
		Dialog_If->comboBox_If_IO->addItems(m_DigitalINList);
		if (!m_DigitalINList.count(m_pProcessIf->GetIfIO()))
			Dialog_If->comboBox_If_IO->addItem(m_pProcessIf->GetIfIO());
	}

	Dialog_If->lineEdit_If_Note	->setText(m_pProcessIf->GetIfNote());
	Dialog_If->comboBox_If_Type	->setCurrentIndex(m_pProcessIf->GetIfType());
	Dialog_If->comboBox_If_IO	->setCurrentText(m_pProcessIf->GetIfIO());
	Dialog_If->lineEdit_If_On	->setText(m_pProcessIf->GetIfOn());
	Dialog_If->lineEdit_If_Off	->setText(m_pProcessIf->GetIfOff());

	Dialog_If->comboBox_If_Type ->blockSignals(false);
}

void Dialog_ProcessSetting_If::ButtonOK()
{
	//获取参数
	QString QstrNote	= Dialog_If->lineEdit_If_Note	->text();
	int		iType		= Dialog_If->comboBox_If_Type	->currentIndex();
	QString	QstrIO		= Dialog_If->comboBox_If_IO		->currentText();
	QString QstrOn		= Dialog_If->lineEdit_If_On		->text();
	QString QstrOff		= Dialog_If->lineEdit_If_Off	->text();

	//设置参数
	m_pProcessIf->SetIfNote	(QstrNote);
	m_pProcessIf->SetIfType	(iType);
	m_pProcessIf->SetIfIO	(QstrIO);
	m_pProcessIf->SetIfOn	(QstrOn);
	m_pProcessIf->SetIfOff	(QstrOff);

	m_pProcessIf->SetState(ItemState::Enable);
	m_pProcessIf->SwitchState(ItemState::Enable);
	m_pProcessIf->UpdateInfo();
	QDialog::accept();
}

void Dialog_ProcessSetting_If::ButtonCancel()
{
	//点击取消/ESC退出均不保存参数，还原节点状态
	reject();
}

void Dialog_ProcessSetting_If::UpdatePage()
{
	m_iSaveIndex = Dialog_If->comboBox_If_IO->currentIndex();
	if (Dialog_If->comboBox_If_Type->currentIndex())
	{
		Dialog_If->comboBox_If_IO->clear();
		Dialog_If->comboBox_If_IO->addItems(m_DigitalOUTList);
	}
	else
	{
		Dialog_If->comboBox_If_IO->clear();
		Dialog_If->comboBox_If_IO->addItems(m_DigitalINList);
	}
	Dialog_If->comboBox_If_IO->setCurrentIndex(m_iSaveIndex);
}

void Dialog_ProcessSetting_If::reject()
{
	m_pProcessIf->SwitchState();
	m_pProcessIf->UpdateInfo();
	QDialog::reject();
}