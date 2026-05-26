#include "Process_Camera.h"
#include "VisionModule.h"
#include <qDebug>


ProcessCamera::ProcessCamera(TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Camera;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"WorkflowID",""} };
	m_itemData.resize(2);
}

ProcessCamera::ProcessCamera(const QString& text, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Camera;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"WorkflowID",""} };
	m_itemData.resize(2);
	UpdateInfo();
}

ProcessCamera::ProcessCamera(const QVector<QVariant>& data, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::Camera;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"WorkflowID",""} };
	m_itemData.resize(2);
	m_itemData		= data;
}

ProcessCamera::~ProcessCamera(void)
{
	qDeleteAll(m_childItems);
	m_childItems.clear();
	qDeleteAll(m_childItems);
}

TreeItem* ProcessCamera::clone() const
{
	TreeItem* newItem = nullptr;

	if (!m_itemData.isEmpty()) {
		newItem = new ProcessCamera(QStringLiteral("Camera"));
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

void ProcessCamera::Edit()
{
	if (m_state == ItemState::Unavailable || m_state == ItemState::Enable || m_state == ItemState::Disable)
	{
		Dialog_ProcessSetting_Camera* Dialog = new Dialog_ProcessSetting_Camera();
		Dialog->setAttribute(Qt::WA_DeleteOnClose);    //关闭对话框的同时，回收内存
		Dialog->m_pProcessCamera = this;
		Dialog->ViewSetting();
		Dialog->show();
	}
}

void ProcessCamera::UpdateInfo()
{
	if (TreeItem::IsChinese())
	{
		setData(0, "\xE7\x9B\xB8\xE6\x9C\xBA"); // 相机
		setData(1, "\xE5\xAE\x9A\xE4\xBD\x8D"); // 定位
	}
	else
	{
		setData(0, "Camera");
		setData(1, "Locate");
	}
}

void ProcessCamera::SwitchState(ItemState state)
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


Dialog_ProcessSetting_Camera::Dialog_ProcessSetting_Camera(QWidget* parent) :
	QDialog(parent),
	m_pProcessCamera(nullptr),
	Dialog_Camera(new Ui::Dialog_ProcessSetting_Camera)
{
	Dialog_Camera->setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowTitleHint);

	connect(Dialog_Camera->Button_Camera_OK,		SIGNAL(clicked()), this, SLOT(ButtonOK()));
	connect(Dialog_Camera->Button_Camera_Cancel,	SIGNAL(clicked()), this, SLOT(ButtonCancel()));
}

Dialog_ProcessSetting_Camera::~Dialog_ProcessSetting_Camera()
{
	delete Dialog_Camera;
}

void Dialog_ProcessSetting_Camera::ViewSetting()
{
	//修改标签状态，避免重复打开标签设置
	m_pProcessCamera->SwitchState(ItemState::Editing);

	Dialog_Camera->comboBox_Camera_Tool->clear();
	const QString workflowId = m_pProcessCamera->GetWorkflowID();
	const QStringList workflowIds = VisionModule::instance()->getWorkflowIds();
	for (const QString& id : workflowIds) {
		Dialog_Camera->comboBox_Camera_Tool->addItem(id, id);
	}

	int currentIndex = Dialog_Camera->comboBox_Camera_Tool->findData(workflowId);
	if (currentIndex < 0 && !workflowId.isEmpty()) {
		Dialog_Camera->comboBox_Camera_Tool->addItem(workflowId, workflowId);
		currentIndex = Dialog_Camera->comboBox_Camera_Tool->count() - 1;
	}
	if (Dialog_Camera->comboBox_Camera_Tool->count() == 0) {
		Dialog_Camera->comboBox_Camera_Tool->addItem(QString(), QString());
		currentIndex = 0;
	}
	Dialog_Camera->comboBox_Camera_Tool->setCurrentIndex(qMax(0, currentIndex));
}

void Dialog_ProcessSetting_Camera::ButtonOK()
{
	QString workflowId = Dialog_Camera->comboBox_Camera_Tool->currentData().toString().trimmed();
	if (workflowId.isEmpty())
		workflowId = Dialog_Camera->comboBox_Camera_Tool->currentText().trimmed();

	m_pProcessCamera->SetWorkflowID(workflowId);

	m_pProcessCamera->SetState(ItemState::Enable);
	m_pProcessCamera->SwitchState(ItemState::Enable);
	m_pProcessCamera->UpdateInfo();
	QDialog::accept();
}

void Dialog_ProcessSetting_Camera::ButtonCancel()
{
	//点击取消/ESC退出均不保存参数，还原节点状态
	reject();
}

void Dialog_ProcessSetting_Camera::reject()
{
	m_pProcessCamera->SwitchState();
	m_pProcessCamera->UpdateInfo();
	QDialog::reject();
}