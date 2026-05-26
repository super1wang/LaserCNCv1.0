#include "Process_EnergySwitch.h"
#include <QList>
#include <QLineEdit>
#include <QRegExpValidator>

namespace {

QList<QLineEdit*> PpDividerEdits(Ui::Dialog_ProcessSetting_EnergySwitch* ui)
{
	return {
		ui->lineEdit_EnergySwitch_PpDivider1,
		ui->lineEdit_EnergySwitch_PpDivider2,
		ui->lineEdit_EnergySwitch_PpDivider3,
		ui->lineEdit_EnergySwitch_PpDivider4,
		ui->lineEdit_EnergySwitch_PpDivider5,
		ui->lineEdit_EnergySwitch_PpDivider6,
		ui->lineEdit_EnergySwitch_PpDivider7,
		ui->lineEdit_EnergySwitch_PpDivider8,
		ui->lineEdit_EnergySwitch_PpDivider9
	};
}

QList<QLineEdit*> DurationEdits(Ui::Dialog_ProcessSetting_EnergySwitch* ui)
{
	return {
		ui->lineEdit_EnergySwitch_Duration1,
		ui->lineEdit_EnergySwitch_Duration2,
		ui->lineEdit_EnergySwitch_Duration3,
		ui->lineEdit_EnergySwitch_Duration4,
		ui->lineEdit_EnergySwitch_Duration5,
		ui->lineEdit_EnergySwitch_Duration6,
		ui->lineEdit_EnergySwitch_Duration7,
		ui->lineEdit_EnergySwitch_Duration8,
		ui->lineEdit_EnergySwitch_Duration9
	};
}

}

ProcessEnergySwitch::ProcessEnergySwitch(TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::EnergySwitch;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= {
		{"PpDivider1", "0"}, {"PpDivider2", "0"}, {"PpDivider3", "0"},
		{"PpDivider4", "0"}, {"PpDivider5", "0"}, {"PpDivider6", "0"},
		{"PpDivider7", "0"}, {"PpDivider8", "0"}, {"PpDivider9", "0"},
		{"Duration1", "100"}, {"Duration2", "100"}, {"Duration3", "100"},
		{"Duration4", "100"}, {"Duration5", "100"}, {"Duration6", "100"},
		{"Duration7", "100"}, {"Duration8", "100"}, {"Duration9", "100"}
	};
	m_itemData.resize(2);
}

ProcessEnergySwitch::ProcessEnergySwitch(const QString& text, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::EnergySwitch;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= {
		{"PpDivider1", "0"}, {"PpDivider2", "0"}, {"PpDivider3", "0"},
		{"PpDivider4", "0"}, {"PpDivider5", "0"}, {"PpDivider6", "0"},
		{"PpDivider7", "0"}, {"PpDivider8", "0"}, {"PpDivider9", "0"},
		{"Duration1", "100"}, {"Duration2", "100"}, {"Duration3", "100"},
		{"Duration4", "100"}, {"Duration5", "100"}, {"Duration6", "100"},
		{"Duration7", "100"}, {"Duration8", "100"}, {"Duration9", "100"}
	};
	m_itemData.resize(2);
	UpdateInfo();
}

ProcessEnergySwitch::ProcessEnergySwitch(const QVector<QVariant>& data, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::EnergySwitch;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= {
		{"PpDivider1", "0"}, {"PpDivider2", "0"}, {"PpDivider3", "0"},
		{"PpDivider4", "0"}, {"PpDivider5", "0"}, {"PpDivider6", "0"},
		{"PpDivider7", "0"}, {"PpDivider8", "0"}, {"PpDivider9", "0"},
		{"Duration1", "100"}, {"Duration2", "100"}, {"Duration3", "100"},
		{"Duration4", "100"}, {"Duration5", "100"}, {"Duration6", "100"},
		{"Duration7", "100"}, {"Duration8", "100"}, {"Duration9", "100"}
	};
	m_itemData.resize(2);
	m_itemData		= data;
}

ProcessEnergySwitch::~ProcessEnergySwitch(void)
{
	qDeleteAll(m_childItems);
	m_childItems.clear();
	qDeleteAll(m_childItems);
}

TreeItem* ProcessEnergySwitch::clone() const
{
	TreeItem* newItem = nullptr;

	if (!m_itemData.isEmpty()) {
		newItem = new ProcessEnergySwitch(QStringLiteral("EnergySwitch"));
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

void ProcessEnergySwitch::Edit()
{
	if (m_state == ItemState::Unavailable || m_state == ItemState::Enable || m_state == ItemState::Disable)
	{
		Dialog_ProcessSetting_EnergySwitch* Dialog = new Dialog_ProcessSetting_EnergySwitch();
		Dialog->setAttribute(Qt::WA_DeleteOnClose);
		Dialog->m_pProcessEnergySwitch = this;
		Dialog->ViewSetting();
		Dialog->show();
	}
}

void ProcessEnergySwitch::UpdateInfo()
{
	if (TreeItem::IsChinese())
	{
		setData(0, "\xE8\x83\xBD\xE9\x87\x8F\xE5\x88\x87\xE6\x8D\xA2");
	}
	else
	{
		setData(0, "EnergySwitch");
	}
	setData(1, "");
}

void ProcessEnergySwitch::SwitchState(ItemState state)
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


Dialog_ProcessSetting_EnergySwitch::Dialog_ProcessSetting_EnergySwitch(QWidget* parent) :
	QDialog(parent),
	m_pProcessEnergySwitch(nullptr),
	Dialog_EnergySwitch(new Ui::Dialog_ProcessSetting_EnergySwitch)
{
	Dialog_EnergySwitch->setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowTitleHint);

	connect(Dialog_EnergySwitch->Button_EnergySwitch_OK,		SIGNAL(clicked()), this, SLOT(ButtonOK()));
	connect(Dialog_EnergySwitch->Button_EnergySwitch_Cancel,	SIGNAL(clicked()), this, SLOT(ButtonCancel()));
}

Dialog_ProcessSetting_EnergySwitch::~Dialog_ProcessSetting_EnergySwitch()
{
	delete Dialog_EnergySwitch;
}

void Dialog_ProcessSetting_EnergySwitch::ViewSetting()
{
	m_pProcessEnergySwitch->SwitchState(ItemState::Editing);

	const QList<QLineEdit*> ppDividerEdits = PpDividerEdits(Dialog_EnergySwitch);
	const QList<QLineEdit*> durationEdits = DurationEdits(Dialog_EnergySwitch);
	const map<QString, QString> maps = m_pProcessEnergySwitch->GetMaps();

	for (QLineEdit* edit : ppDividerEdits)
	{
		edit->setValidator(new QRegExpValidator(Regex_Nonnegative_Int));
	}

	for (QLineEdit* edit : durationEdits)
	{
		edit->setValidator(new QRegExpValidator(Regex_Nonnegative_Int));
	}

	for (int i = 0; i < ppDividerEdits.size(); ++i)
	{
		ppDividerEdits[i]->setText(maps.at(QString("PpDivider%1").arg(i + 1)));
		durationEdits[i]->setText(maps.at(QString("Duration%1").arg(i + 1)));
	}
}

void Dialog_ProcessSetting_EnergySwitch::ButtonOK()
{
	const QList<QLineEdit*> ppDividerEdits = PpDividerEdits(Dialog_EnergySwitch);
	const QList<QLineEdit*> durationEdits = DurationEdits(Dialog_EnergySwitch);
	map<QString, QString> maps = m_pProcessEnergySwitch->GetMaps();

	for (int i = 0; i < ppDividerEdits.size(); ++i)
	{
		maps[QString("PpDivider%1").arg(i + 1)] = ppDividerEdits[i]->text();
		maps[QString("Duration%1").arg(i + 1)] = durationEdits[i]->text();
	}

	m_pProcessEnergySwitch->SetMaps(maps);
	m_pProcessEnergySwitch->SetState(ItemState::Enable);
	m_pProcessEnergySwitch->SwitchState(ItemState::Enable);
	m_pProcessEnergySwitch->UpdateInfo();
	QDialog::accept();
}

void Dialog_ProcessSetting_EnergySwitch::ButtonCancel()
{
	reject();
}

void Dialog_ProcessSetting_EnergySwitch::reject()
{
	m_pProcessEnergySwitch->SwitchState();
	m_pProcessEnergySwitch->UpdateInfo();
	QDialog::reject();
}
