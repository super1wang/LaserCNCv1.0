#include "Process_Alignment.h"

#include "../../Vision/VisionModule.h"
#include <algorithm>
#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <QVBoxLayout>

namespace {
QString alignmentRowKey(int row, const QString& suffix)
{
	return QString("Row%1_%2").arg(row).arg(suffix);
}
}

ProcessAlignment::ProcessAlignment(TreeItem* parent)
{
	m_parentItem = parent;
	m_type = ItemType::Alignment;
	m_state = ItemState::Unavailable;
	m_stateSave = m_state;
	m_maps = {
		{"Mode", "0"},
		{"GroupCount", "1"},
		{"Row0_MarkId1", ""},
		{"Row0_MarkId2", ""},
		{"Row0_AllowRotate", "1"},
		{"Row0_AllowScale", "0"}
	};
	m_itemData.resize(2);
}

ProcessAlignment::ProcessAlignment(const QString& text, TreeItem* parent)
	: ProcessAlignment(parent)
{
	Q_UNUSED(text);
	UpdateInfo();
}

ProcessAlignment::ProcessAlignment(const QVector<QVariant>& data, TreeItem* parent)
	: ProcessAlignment(parent)
{
	m_itemData = data;
}

ProcessAlignment::~ProcessAlignment(void)
{
	qDeleteAll(m_childItems);
	m_childItems.clear();
}

TreeItem* ProcessAlignment::clone() const
{
	TreeItem* newItem = nullptr;

	if (!m_itemData.isEmpty()) {
		newItem = new ProcessAlignment(QStringLiteral("Alignment"));
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

void ProcessAlignment::Edit()
{
	if (m_state == ItemState::Unavailable || m_state == ItemState::Enable || m_state == ItemState::Disable)
	{
		Dialog_ProcessSetting_Alignment* dialog = new Dialog_ProcessSetting_Alignment();
		dialog->setAttribute(Qt::WA_DeleteOnClose);
		dialog->m_pProcessAlignment = this;
		dialog->ViewSetting();
		dialog->show();
	}
}

void ProcessAlignment::UpdateInfo()
{
	if (TreeItem::IsChinese())
	{
		setData(0, QString::fromUtf8("对位计算"));
		setData(1, QString::fromUtf8("组数 ") + QString::number(GetGroupCount()));
	}
	else
	{
		setData(0, "Alignment");
		setData(1, "Groups " + QString::number(GetGroupCount()));
	}
}

void ProcessAlignment::SwitchState(ItemState state)
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

int ProcessAlignment::GetMode() const
{
	bool ok = false;
	int mode = m_maps.count("Mode") ? m_maps.at("Mode").toInt(&ok) : 0;
	if (!ok || mode < 0 || mode > 1) mode = 0;
	return mode;
}

void ProcessAlignment::SetMode(int mode)
{
	m_maps["Mode"] = QString::number(mode == 1 ? 1 : 0);
}

int ProcessAlignment::GetGroupCount() const
{
	bool ok = false;
	int count = m_maps.count("GroupCount") ? m_maps.at("GroupCount").toInt(&ok) : 1;
	if (!ok || count < 1) count = 1;
	return count;
}

void ProcessAlignment::SetGroupCount(int count)
{
	m_maps["GroupCount"] = QString::number(std::max(1, count));
}

QString ProcessAlignment::rowValue(int row, const QString& suffix) const
{
	QString key = alignmentRowKey(row, suffix);
	return m_maps.count(key) ? m_maps.at(key) : QString();
}

void ProcessAlignment::setRowValue(int row, const QString& suffix, const QString& value)
{
	m_maps[alignmentRowKey(row, suffix)] = value;
}

Dialog_ProcessSetting_Alignment::Dialog_ProcessSetting_Alignment(QWidget* parent)
	: QDialog(parent)
{
	setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowTitleHint);
	setWindowTitle(QObject::tr("Alignment Setting"));
	resize(760, 360);

	auto* rootLayout = new QVBoxLayout(this);
	auto* formLayout = new QFormLayout();
	m_modeCombo = new QComboBox(this);
	m_modeCombo->addItem(QObject::tr("SinglePoint"));
	m_modeCombo->addItem(QObject::tr("TwoPoint"));
	m_groupCountSpin = new QSpinBox(this);
	m_groupCountSpin->setRange(1, 32);
	formLayout->addRow(QObject::tr("Mode"), m_modeCombo);
	formLayout->addRow(QObject::tr("GroupCount"), m_groupCountSpin);
	rootLayout->addLayout(formLayout);

	m_table = new QTableWidget(this);
	m_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	rootLayout->addWidget(m_table);

	auto* buttonLayout = new QHBoxLayout();
	buttonLayout->addStretch();
	m_btnOk = new QPushButton(QObject::tr("OK"), this);
	m_btnCancel = new QPushButton(QObject::tr("Cancel"), this);
	buttonLayout->addWidget(m_btnOk);
	buttonLayout->addWidget(m_btnCancel);
	rootLayout->addLayout(buttonLayout);

	connect(m_modeCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int) {
		rebuildTable();
		populateCombos();
	});
	connect(m_groupCountSpin, qOverload<int>(&QSpinBox::valueChanged), this, [this](int) {
		rebuildTable();
		populateCombos();
	});
	connect(m_btnOk, &QPushButton::clicked, this, [this]() { ButtonOK(); });
	connect(m_btnCancel, &QPushButton::clicked, this, [this]() { ButtonCancel(); });
}

Dialog_ProcessSetting_Alignment::~Dialog_ProcessSetting_Alignment() = default;

void Dialog_ProcessSetting_Alignment::ViewSetting()
{
	if (!m_pProcessAlignment) return;
	m_stateBeforeEdit = m_pProcessAlignment->GetState();
	m_pProcessAlignment->SwitchState(ItemState::Editing);
	m_modeCombo->setCurrentIndex(m_pProcessAlignment->GetMode());
	m_groupCountSpin->setValue(m_pProcessAlignment->GetGroupCount());
	rebuildTable();
	populateCombos();

	for (int row = 0; row < m_table->rowCount(); ++row)
	{
		auto* mark1Combo = qobject_cast<QComboBox*>(m_table->cellWidget(row, 0));
		auto* rotateCheck = qobject_cast<QCheckBox*>(m_table->cellWidget(row, m_modeCombo->currentIndex() == 0 ? 1 : 2));
		auto* scaleCheck = qobject_cast<QCheckBox*>(m_table->cellWidget(row, m_modeCombo->currentIndex() == 0 ? 2 : 3));
		if (mark1Combo) mark1Combo->setCurrentText(m_pProcessAlignment->rowValue(row, "MarkId1"));
		if (m_modeCombo->currentIndex() == 1)
		{
			auto* mark2Combo = qobject_cast<QComboBox*>(m_table->cellWidget(row, 1));
			if (mark2Combo) mark2Combo->setCurrentText(m_pProcessAlignment->rowValue(row, "MarkId2"));
		}
		if (rotateCheck) rotateCheck->setChecked(m_pProcessAlignment->rowValue(row, "AllowRotate") != "0");
		if (scaleCheck) scaleCheck->setChecked(m_pProcessAlignment->rowValue(row, "AllowScale") != "0");
	}
}

void Dialog_ProcessSetting_Alignment::ButtonOK()
{
	if (!m_pProcessAlignment) {
		QDialog::accept();
		return;
	}

	const int mode = m_modeCombo->currentIndex();
	m_pProcessAlignment->SetMode(mode);
	m_pProcessAlignment->SetGroupCount(m_groupCountSpin->value());

	for (int row = 0; row < m_table->rowCount(); ++row)
	{
		auto* mark1Combo = qobject_cast<QComboBox*>(m_table->cellWidget(row, 0));
		m_pProcessAlignment->setRowValue(row, "MarkId1", mark1Combo ? mark1Combo->currentText().trimmed() : QString());
		if (mode == 1)
		{
			auto* mark2Combo = qobject_cast<QComboBox*>(m_table->cellWidget(row, 1));
			m_pProcessAlignment->setRowValue(row, "MarkId2", mark2Combo ? mark2Combo->currentText().trimmed() : QString());
		}
		else
		{
			m_pProcessAlignment->setRowValue(row, "MarkId2", QString());
		}

		auto* rotateCheck = qobject_cast<QCheckBox*>(m_table->cellWidget(row, mode == 0 ? 1 : 2));
		auto* scaleCheck = qobject_cast<QCheckBox*>(m_table->cellWidget(row, mode == 0 ? 2 : 3));
		m_pProcessAlignment->setRowValue(row, "AllowRotate", rotateCheck && rotateCheck->isChecked() ? "1" : "0");
		m_pProcessAlignment->setRowValue(row, "AllowScale", scaleCheck && scaleCheck->isChecked() ? "1" : "0");
	}

	m_pProcessAlignment->SetState(ItemState::Enable);
	m_pProcessAlignment->SwitchState(ItemState::Enable);
	m_pProcessAlignment->UpdateInfo();
	QDialog::accept();
}

void Dialog_ProcessSetting_Alignment::ButtonCancel()
{
	reject();
}

void Dialog_ProcessSetting_Alignment::reject()
{
	if (m_pProcessAlignment)
	{
		m_pProcessAlignment->SwitchState(m_stateBeforeEdit == ItemState::Editing ? ItemState::StateSave : m_stateBeforeEdit);
		m_pProcessAlignment->UpdateInfo();
	}
	QDialog::reject();
}

void Dialog_ProcessSetting_Alignment::rebuildTable()
{
	const bool twoPoint = m_modeCombo->currentIndex() == 1;
	m_table->clear();
	if (twoPoint)
	{
		m_table->setColumnCount(4);
		m_table->setHorizontalHeaderLabels({
			QObject::tr("MarkID1"),
			QObject::tr("MarkID2"),
			QObject::tr("AllowRotate"),
			QObject::tr("AllowScale")
		});
	}
	else
	{
		m_table->setColumnCount(3);
		m_table->setHorizontalHeaderLabels({
			QObject::tr("MarkID"),
			QObject::tr("AllowRotate"),
			QObject::tr("AllowScale")
		});
	}
	m_table->setRowCount(m_groupCountSpin->value());

	for (int row = 0; row < m_table->rowCount(); ++row)
	{
		m_table->setCellWidget(row, 0, new QComboBox(m_table));
		if (twoPoint)
			m_table->setCellWidget(row, 1, new QComboBox(m_table));

		auto* rotateCheck = new QCheckBox(m_table);
		rotateCheck->setChecked(true);
		m_table->setCellWidget(row, twoPoint ? 2 : 1, rotateCheck);

		auto* scaleCheck = new QCheckBox(m_table);
		scaleCheck->setChecked(false);
		scaleCheck->setEnabled(twoPoint);
		m_table->setCellWidget(row, twoPoint ? 3 : 2, scaleCheck);
	}
}

void Dialog_ProcessSetting_Alignment::populateCombos()
{
	const QStringList markIds = availableMarkIds();
	const bool twoPoint = m_modeCombo->currentIndex() == 1;
	for (int row = 0; row < m_table->rowCount(); ++row)
	{
		auto* mark1Combo = qobject_cast<QComboBox*>(m_table->cellWidget(row, 0));
		if (mark1Combo)
		{
			QString current = mark1Combo->currentText();
			mark1Combo->clear();
			mark1Combo->addItems(markIds);
			int idx = mark1Combo->findText(current);
			if (idx >= 0) mark1Combo->setCurrentIndex(idx);
		}
		if (twoPoint)
		{
			auto* mark2Combo = qobject_cast<QComboBox*>(m_table->cellWidget(row, 1));
			if (mark2Combo)
			{
				QString current = mark2Combo->currentText();
				mark2Combo->clear();
				mark2Combo->addItems(markIds);
				int idx = mark2Combo->findText(current);
				if (idx >= 0) mark2Combo->setCurrentIndex(idx);
			}
		}
	}
}

QStringList Dialog_ProcessSetting_Alignment::availableMarkIds() const
{
	QStringList ids = VisionModule::instance()->getAllMarkIds();
	if (ids.isEmpty()) ids << QString();
	return ids;
}
