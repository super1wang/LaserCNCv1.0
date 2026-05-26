#include "Process_Measurement.h"
#include <qDebug>


ProcessMeasurement::ProcessMeasurement(TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Measurement;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Source","0"}, {"Index",""}, {"X","X"}, {"Y","Y"} };
	m_itemData.resize(2);
}

ProcessMeasurement::ProcessMeasurement(const QString& text, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Measurement;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Source","0"}, {"Index",""}, {"X","X"}, {"Y","Y"} };
	m_itemData.resize(2);
	UpdateInfo();
}

ProcessMeasurement::ProcessMeasurement(const QVector<QVariant>& data, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Measurement;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Source","0"}, {"Index",""}, {"X","X"}, {"Y","Y"} };
	m_itemData.resize(2);
	m_itemData		= data;
}

ProcessMeasurement::~ProcessMeasurement(void)
{
	qDeleteAll(m_childItems);
	m_childItems.clear();
	qDeleteAll(m_childItems);
}

TreeItem* ProcessMeasurement::clone() const
{
	TreeItem* newItem = nullptr;

	if (!m_itemData.isEmpty()) {
		newItem = new ProcessMeasurement(QStringLiteral("Measurement"));
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

void ProcessMeasurement::Edit()
{
	if (m_state == ItemState::Unavailable || m_state == ItemState::Enable || m_state == ItemState::Disable)
	{
		Dialog_ProcessSetting_Measurement* Dialog = new Dialog_ProcessSetting_Measurement();
		Dialog->setAttribute(Qt::WA_DeleteOnClose);    //关闭对话框的同时，回收内存
		Dialog->m_pProcessMeasurement = this;
		Dialog->ViewSetting();
		Dialog->show();
	}
}

void ProcessMeasurement::UpdateInfo()
{
	if (TreeItem::IsChinese())
	{
		setData(0, "\xE6\xB5\x8B\xE9\x87\x8F");	// 测量
		setData(1, "\xE7\xBB\x93\xE6\x9E\x9C\xE7\xB4\xA2\xE5\xBC\x95 " + m_maps["Index"]); // 结果索引
	}
	else
	{
		setData(0, "Measurement");
		setData(1, "Result index " + m_maps["Index"]);
	}
}

void ProcessMeasurement::SwitchState(ItemState state)
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


Dialog_ProcessSetting_Measurement::Dialog_ProcessSetting_Measurement(QWidget* parent) :
	QDialog(parent),
	m_pProcessMeasurement(nullptr),
	Dialog_Measurement(new Ui::Dialog_ProcessSetting_Measurement)
{
	Dialog_Measurement->setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowTitleHint);

	connect(Dialog_Measurement->Button_Measurement_OK,		SIGNAL(clicked()), this, SLOT(ButtonOK()));
	connect(Dialog_Measurement->Button_Measurement_Cancel,	SIGNAL(clicked()), this, SLOT(ButtonCancel()));
}

Dialog_ProcessSetting_Measurement::~Dialog_ProcessSetting_Measurement()
{
	delete Dialog_Measurement;
}

void Dialog_ProcessSetting_Measurement::ViewSetting()
{
	//修改标签状态，避免重复打开标签设置
	m_pProcessMeasurement->SetState(ItemState::Editing);

	//添加索引
	if (DT::getCustomerID() == "JAPHL")
	{
		Dialog_Measurement->comboBox_Measurement_Source->addItem("LP-L");
		Dialog_Measurement->comboBox_Measurement_Source->addItem("LP-R");
	}
	else
	{
		//相机部分索引
		Dialog_Measurement->comboBox_Measurement_Source->addItems(DT::getCameraCommands());
	}

	//获取参数
	QString	QstrSource 	= m_pProcessMeasurement->GetMeasurementSource();
	QString	QstrIndex 	= m_pProcessMeasurement->GetMeasurementIndex();
	QString	QstrX 		= m_pProcessMeasurement->GetMeasurementX();
	QString	QstrY 		= m_pProcessMeasurement->GetMeasurementY();

	//填入参数
	Dialog_Measurement->comboBox_Measurement_Source	->setCurrentIndex(QstrSource.toInt());
	Dialog_Measurement->lineEdit_Measurement_Index	->setText(QstrIndex);
	Dialog_Measurement->lineEdit_Measurement_X		->setText(QstrX);
	Dialog_Measurement->lineEdit_Measurement_Y		->setText(QstrY);
}

void Dialog_ProcessSetting_Measurement::ButtonOK()
{
	//获取参数
	QString	QstrSource 	= QString::number(Dialog_Measurement->comboBox_Measurement_Source->currentIndex());
	QString	QstrIndex 	= Dialog_Measurement->lineEdit_Measurement_Index->text();
	QString	QstrX 		= Dialog_Measurement->lineEdit_Measurement_X->text();
	QString	QstrY 		= Dialog_Measurement->lineEdit_Measurement_Y->text();
	
	//保存参数
	m_pProcessMeasurement->SetMeasurementSource(QstrSource);
	m_pProcessMeasurement->SetMeasurementIndex(QstrIndex);
	m_pProcessMeasurement->SetMeasurementX(QstrX);
	m_pProcessMeasurement->SetMeasurementY(QstrY);

	m_pProcessMeasurement->SetState(ItemState::Enable);
	m_pProcessMeasurement->SwitchState(ItemState::Enable);
	m_pProcessMeasurement->UpdateInfo();
	QDialog::accept();
}

void Dialog_ProcessSetting_Measurement::ButtonCancel()
{
	//点击取消/ESC退出均不保存参数，还原节点状态
	reject();
}

void Dialog_ProcessSetting_Measurement::reject()
{
	m_pProcessMeasurement->SwitchState();
	m_pProcessMeasurement->UpdateInfo();
	QDialog::reject();
}