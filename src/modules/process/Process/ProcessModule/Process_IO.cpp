#include "Process_IO.h"
#include <qDebug>
#include <QRegExpValidator>

ProcessIO::ProcessIO(TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::IO;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Note",""}, {"Type","1"}, {"IO",""}, {"Value",""} };
	m_itemData.resize(2);
}

ProcessIO::ProcessIO(const QString& text, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::IO;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Note",""}, {"Type","1"}, {"IO",""}, {"Value",""} };
	m_itemData.resize(2);
	UpdateInfo();
}

ProcessIO::ProcessIO(const QVector<QVariant>& data, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::IO;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Note",""}, {"Type","1"}, {"IO",""}, {"Value",""} };
	m_itemData.resize(2);
	m_itemData		= data;
}

ProcessIO::~ProcessIO(void)
{
	qDeleteAll(m_childItems);
	m_childItems.clear();
	qDeleteAll(m_childItems);
}

TreeItem* ProcessIO::clone() const
{
	TreeItem* newItem = nullptr;

	if (!m_itemData.isEmpty()) {
		newItem = new ProcessIO(QStringLiteral("IO"));
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

void ProcessIO::Edit()
{
	if (m_state == ItemState::Unavailable || m_state == ItemState::Enable || m_state == ItemState::Disable)
	{
		Dialog_ProcessSetting_IO* Dialog = new Dialog_ProcessSetting_IO();
		Dialog->setAttribute(Qt::WA_DeleteOnClose);    //关闭对话框的同时，回收内存
		Dialog->m_pProcessIO = this;
		Dialog->ViewSetting();
		Dialog->show();
	}
}

void ProcessIO::UpdateInfo()
{
	if (TreeItem::IsChinese())
		setData(0, "\xE8\xBE\x93\xE5\x87\xBA\xE4\xBF\xA1\xE5\x8F\xB7"); // 输出信号
	else
		setData(0, "IO");
	
	setData(1, m_maps["Note"]);
}

void ProcessIO::SwitchState(ItemState state)
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


Dialog_ProcessSetting_IO::Dialog_ProcessSetting_IO(QWidget* parent) :
	QDialog(parent),
	m_pProcessIO(nullptr), m_iSaveIndex(0), m_bIONone(false),
	Dialog_IO(new Ui::Dialog_ProcessSetting_IO)
{
	Dialog_IO->setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowTitleHint);

	connect(Dialog_IO->Button_IO_OK,		SIGNAL(clicked()),					this, SLOT(ButtonOK()));
	connect(Dialog_IO->Button_IO_Cancel,	SIGNAL(clicked()),					this, SLOT(ButtonCancel()));
	connect(Dialog_IO->comboBox_IO_Type,	SIGNAL(currentIndexChanged(int)),	this, SLOT(UpdatePage()));
	connect(Dialog_IO->comboBox_IO_IO,		SIGNAL(currentIndexChanged(int)),	this, SLOT(UpdateLineEdit()));
}

Dialog_ProcessSetting_IO::~Dialog_ProcessSetting_IO()
{
	delete Dialog_IO;
}

void Dialog_ProcessSetting_IO::ViewSetting()
{
	//修改标签状态，避免重复打开标签设置
	m_pProcessIO->SwitchState(ItemState::Editing);

	//构建正则表达式限制输入值
	//Str为仅可输入 大小写字母、数字、点
	QRegExp Str("^[A-Za-z0-9.]+$");
	Dialog_IO->lineEdit_IO_Value->setValidator(new QRegExpValidator(Str));

	//获取IO列表
	m_DigitalList = DT::getDigitalOUTList();
	m_AnalogList = DT::getAnalogOUTList();

	//显示参数
	Dialog_IO->comboBox_IO_Type->blockSignals(true);
	if (m_pProcessIO->GetIOType())
	{
		Dialog_IO->lineEdit_IO_Value->setValidator(new QRegExpValidator(Regex_Digital_Out));
		Dialog_IO->comboBox_IO_IO->addItems(m_DigitalList);
		if (!m_DigitalList.count(m_pProcessIO->GetIOIO()))
			Dialog_IO->comboBox_IO_IO->addItem(m_pProcessIO->GetIOIO());
	}
	else
	{
		Dialog_IO->lineEdit_IO_Value->setValidator(new QRegExpValidator(Regex_Analog_Out));
		Dialog_IO->comboBox_IO_IO->addItems(m_AnalogList);
		if (!m_AnalogList.count(m_pProcessIO->GetIOIO()))
			Dialog_IO->comboBox_IO_IO->addItem(m_pProcessIO->GetIOIO());
	}

	Dialog_IO->lineEdit_IO_Note	->setText(m_pProcessIO->GetIONote());
	Dialog_IO->comboBox_IO_Type	->setCurrentIndex(m_pProcessIO->GetIOType());
	Dialog_IO->comboBox_IO_IO	->setCurrentText(m_pProcessIO->GetIOIO());
	Dialog_IO->lineEdit_IO_Value->setText(m_pProcessIO->GetIOValue());

	Dialog_IO->comboBox_IO_Type->blockSignals(false);
}

void Dialog_ProcessSetting_IO::ButtonOK()
{
	//获取参数
	QString QstrNote	= Dialog_IO->lineEdit_IO_Note	->text();
	int		iType		= Dialog_IO->comboBox_IO_Type	->currentIndex();
	QString QstrIO		= Dialog_IO->comboBox_IO_IO		->currentText();
	QString QstrValue	= Dialog_IO->lineEdit_IO_Value	->text();

	//设置参数
	m_pProcessIO->SetIONote	(QstrNote);
	m_pProcessIO->SetIOType	(iType);
	m_pProcessIO->SetIOIO	(QstrIO);
	m_pProcessIO->SetIOValue(QstrValue);

	m_pProcessIO->SetState(ItemState::Enable);
	m_pProcessIO->SwitchState(ItemState::Enable);
	m_pProcessIO->UpdateInfo();
	QDialog::accept();
}

void Dialog_ProcessSetting_IO::ButtonCancel()
{
	//点击取消/ESC退出均不保存参数，还原节点状态
	reject();
}

void Dialog_ProcessSetting_IO::UpdatePage()
{
	m_iSaveIndex = Dialog_IO->comboBox_IO_IO->currentIndex();
	if (Dialog_IO->comboBox_IO_Type->currentIndex())
	{
		Dialog_IO->comboBox_IO_IO->clear();
		Dialog_IO->comboBox_IO_IO->addItems(m_DigitalList);
		Dialog_IO->lineEdit_IO_Value->clear();
		Dialog_IO->lineEdit_IO_Value->setValidator(new QRegExpValidator(Regex_Digital_Out));
	}
	else
	{
		Dialog_IO->comboBox_IO_IO->clear();
		Dialog_IO->comboBox_IO_IO->addItems(m_AnalogList);
		Dialog_IO->lineEdit_IO_Value->clear();
		Dialog_IO->lineEdit_IO_Value->setValidator(new QRegExpValidator(Regex_Analog_Out));
	}
	Dialog_IO->comboBox_IO_IO->setCurrentIndex(m_iSaveIndex);
}

void Dialog_ProcessSetting_IO::UpdateLineEdit()
{
	if (Dialog_IO->comboBox_IO_IO->currentText() == tr("NONE"))
	{
		m_bIONone = true;
		Dialog_IO->lineEdit_IO_Value->clear();
		if (Dialog_IO->comboBox_IO_Type->currentIndex() == 1)
			Dialog_IO->lineEdit_IO_Value->setValidator(new QRegExpValidator(Regex_Digital_IndexOut));
		else
			Dialog_IO->lineEdit_IO_Value->setValidator(new QRegExpValidator(Regex_Analog_IndexOut));		
	}
	else if (m_bIONone)
	{
		m_bIONone = false;
		Dialog_IO->lineEdit_IO_Value->clear();
		if (Dialog_IO->comboBox_IO_Type->currentIndex() == 1)
			Dialog_IO->lineEdit_IO_Value->setValidator(new QRegExpValidator(Regex_Digital_Out));
		else
			Dialog_IO->lineEdit_IO_Value->setValidator(new QRegExpValidator(Regex_Analog_Out));
	}
}

void Dialog_ProcessSetting_IO::reject()
{
	m_pProcessIO->SwitchState();
	m_pProcessIO->UpdateInfo();
	QDialog::reject();
}