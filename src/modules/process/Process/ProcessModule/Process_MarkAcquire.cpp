#include "Process_MarkAcquire.h"

#include "../../Vision/VisionModule.h"
#include <algorithm>
#include <QComboBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <QVBoxLayout>

namespace {
QString rowKey(int row, const QString& suffix)
{
	return QString("Row%1_%2").arg(row).arg(suffix);
}
}

ProcessMarkAcquire::ProcessMarkAcquire(TreeItem* parent)
{
	m_parentItem = parent;
	m_type = ItemType::MarkAcquire;
	m_state = ItemState::Unavailable;
	m_stateSave = m_state;
	m_maps = {
		{"Count", "1"},
		{"Row0_MarkId", ""},
		{"Row0_WorkflowID", ""},
		{"Row0_SafeZ", "0"},
		{"Row0_CaptureZ", "0"},
		{"Row0_StabilizeMs", "500"}
	};
	m_itemData.resize(2);
}

ProcessMarkAcquire::ProcessMarkAcquire(const QString& text, TreeItem* parent)
	: ProcessMarkAcquire(parent)
{
	Q_UNUSED(text);
	UpdateInfo();
}

ProcessMarkAcquire::ProcessMarkAcquire(const QVector<QVariant>& data, TreeItem* parent)
	: ProcessMarkAcquire(parent)
{
	m_itemData = data;
}

ProcessMarkAcquire::~ProcessMarkAcquire(void)
{
	qDeleteAll(m_childItems);
	m_childItems.clear();
}

TreeItem* ProcessMarkAcquire::clone() const
{
	TreeItem* newItem = nullptr;

	if (!m_itemData.isEmpty()) {
		newItem = new ProcessMarkAcquire(QStringLiteral("MarkAcquire"));
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

void ProcessMarkAcquire::Edit()
{
	if (m_state == ItemState::Unavailable || m_state == ItemState::Enable || m_state == ItemState::Disable)
	{
		Dialog_ProcessSetting_MarkAcquire* dialog = new Dialog_ProcessSetting_MarkAcquire();
		dialog->setAttribute(Qt::WA_DeleteOnClose);
		dialog->m_pProcessMarkAcquire = this;
		dialog->ViewSetting();
		dialog->show();
	}
}

void ProcessMarkAcquire::UpdateInfo()
{
	if (TreeItem::IsChinese())
	{
		setData(0, QString::fromUtf8("Mark采集"));
		setData(1, QString::fromUtf8("采集数 ") + QString::number(GetCount()));
	}
	else
	{
		setData(0, "MarkAcquire");
		setData(1, "Count " + QString::number(GetCount()));
	}
}

void ProcessMarkAcquire::SwitchState(ItemState state)
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

int ProcessMarkAcquire::GetCount() const
{
	bool ok = false;
	int count = m_maps.count("Count") ? m_maps.at("Count").toInt(&ok) : 1;
	if (!ok || count < 1) count = 1;
	return count;
}

void ProcessMarkAcquire::SetCount(int count)
{
	m_maps["Count"] = QString::number(std::max(1, count));
}

QString ProcessMarkAcquire::rowValue(int row, const QString& suffix) const
{
	QString key = rowKey(row, suffix);
	return m_maps.count(key) ? m_maps.at(key) : QString();
}

void ProcessMarkAcquire::setRowValue(int row, const QString& suffix, const QString& value)
{
	m_maps[rowKey(row, suffix)] = value;
}

Dialog_ProcessSetting_MarkAcquire::Dialog_ProcessSetting_MarkAcquire(QWidget* parent)
	: QDialog(parent)
{
	setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowTitleHint);
	setWindowTitle(QObject::tr("Mark Acquire Setting"));
	resize(760, 360);

	auto* rootLayout = new QVBoxLayout(this);
	auto* formLayout = new QFormLayout();
	m_countSpin = new QSpinBox(this);
	m_countSpin->setRange(1, 32);
	formLayout->addRow(QObject::tr("Count"), m_countSpin);
	rootLayout->addLayout(formLayout);

	m_table = new QTableWidget(this);
	m_table->setColumnCount(5);
	m_table->setHorizontalHeaderLabels({
		QObject::tr("MarkID"),
		QObject::tr("Workflow"),
		QObject::tr("CameraSafeZ"),
		QObject::tr("CameraCaptureZ"),
		QObject::tr("StabilizeMs")
	});
	m_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	rootLayout->addWidget(m_table);

	auto* buttonLayout = new QHBoxLayout();
	buttonLayout->addStretch();
	m_btnOk = new QPushButton(QObject::tr("OK"), this);
	m_btnCancel = new QPushButton(QObject::tr("Cancel"), this);
	buttonLayout->addWidget(m_btnOk);
	buttonLayout->addWidget(m_btnCancel);
	rootLayout->addLayout(buttonLayout);

	connect(m_countSpin, qOverload<int>(&QSpinBox::valueChanged), this, [this](int) {
		rebuildTable();
		populateCombos();
	});
	connect(m_btnOk, &QPushButton::clicked, this, [this]() { ButtonOK(); });
	connect(m_btnCancel, &QPushButton::clicked, this, [this]() { ButtonCancel(); });
}

Dialog_ProcessSetting_MarkAcquire::~Dialog_ProcessSetting_MarkAcquire() = default;

void Dialog_ProcessSetting_MarkAcquire::ViewSetting()
{
	if (!m_pProcessMarkAcquire) return;
	m_stateBeforeEdit = m_pProcessMarkAcquire->GetState();
	m_pProcessMarkAcquire->SwitchState(ItemState::Editing);
	m_countSpin->setValue(m_pProcessMarkAcquire->GetCount());
	rebuildTable();
	populateCombos();

	for (int row = 0; row < m_table->rowCount(); ++row)
	{
		auto* markCombo = qobject_cast<QComboBox*>(m_table->cellWidget(row, 0));
		auto* workflowCombo = qobject_cast<QComboBox*>(m_table->cellWidget(row, 1));
		auto* safeEdit = qobject_cast<QLineEdit*>(m_table->cellWidget(row, 2));
		auto* captureEdit = qobject_cast<QLineEdit*>(m_table->cellWidget(row, 3));
		auto* stabilizeEdit = qobject_cast<QLineEdit*>(m_table->cellWidget(row, 4));

		if (markCombo) markCombo->setCurrentText(m_pProcessMarkAcquire->rowValue(row, "MarkId"));
		if (workflowCombo) workflowCombo->setCurrentText(m_pProcessMarkAcquire->rowValue(row, "WorkflowID"));
		if (safeEdit) safeEdit->setText(m_pProcessMarkAcquire->rowValue(row, "SafeZ"));
		if (captureEdit) captureEdit->setText(m_pProcessMarkAcquire->rowValue(row, "CaptureZ"));
		if (stabilizeEdit)
		{
			const QString value = m_pProcessMarkAcquire->rowValue(row, "StabilizeMs");
			stabilizeEdit->setText(value.isEmpty() ? QStringLiteral("500") : value);
		}
	}
}

void Dialog_ProcessSetting_MarkAcquire::ButtonOK()
{
	if (!m_pProcessMarkAcquire) {
		QDialog::accept();
		return;
	}

	m_pProcessMarkAcquire->SetCount(m_countSpin->value());
	for (int row = 0; row < m_table->rowCount(); ++row)
	{
		auto* markCombo = qobject_cast<QComboBox*>(m_table->cellWidget(row, 0));
		auto* workflowCombo = qobject_cast<QComboBox*>(m_table->cellWidget(row, 1));
		auto* safeEdit = qobject_cast<QLineEdit*>(m_table->cellWidget(row, 2));
		auto* captureEdit = qobject_cast<QLineEdit*>(m_table->cellWidget(row, 3));
		auto* stabilizeEdit = qobject_cast<QLineEdit*>(m_table->cellWidget(row, 4));

		m_pProcessMarkAcquire->setRowValue(row, "MarkId", markCombo ? markCombo->currentText().trimmed() : QString());
		m_pProcessMarkAcquire->setRowValue(row, "WorkflowID", workflowCombo ? workflowCombo->currentText().trimmed() : QString());
		m_pProcessMarkAcquire->setRowValue(row, "SafeZ", safeEdit ? safeEdit->text().trimmed() : QString());
		m_pProcessMarkAcquire->setRowValue(row, "CaptureZ", captureEdit ? captureEdit->text().trimmed() : QString());
		m_pProcessMarkAcquire->setRowValue(row, "StabilizeMs", stabilizeEdit ? stabilizeEdit->text().trimmed() : QStringLiteral("500"));
	}

	m_pProcessMarkAcquire->SetState(ItemState::Enable);
	m_pProcessMarkAcquire->SwitchState(ItemState::Enable);
	m_pProcessMarkAcquire->UpdateInfo();
	QDialog::accept();
}

void Dialog_ProcessSetting_MarkAcquire::ButtonCancel()
{
	reject();
}

void Dialog_ProcessSetting_MarkAcquire::reject()
{
	if (m_pProcessMarkAcquire)
	{
		m_pProcessMarkAcquire->SwitchState(m_stateBeforeEdit == ItemState::Editing ? ItemState::StateSave : m_stateBeforeEdit);
		m_pProcessMarkAcquire->UpdateInfo();
	}
	QDialog::reject();
}

void Dialog_ProcessSetting_MarkAcquire::rebuildTable()
{
	m_table->setRowCount(m_countSpin->value());
	for (int row = 0; row < m_table->rowCount(); ++row)
	{
		if (!m_table->cellWidget(row, 0)) m_table->setCellWidget(row, 0, new QComboBox(m_table));
		if (!m_table->cellWidget(row, 1)) m_table->setCellWidget(row, 1, new QComboBox(m_table));
		if (!m_table->cellWidget(row, 2)) m_table->setCellWidget(row, 2, new QLineEdit(m_table));
		if (!m_table->cellWidget(row, 3)) m_table->setCellWidget(row, 3, new QLineEdit(m_table));
		if (!m_table->cellWidget(row, 4)) m_table->setCellWidget(row, 4, new QLineEdit(m_table));
	}
}

void Dialog_ProcessSetting_MarkAcquire::populateCombos()
{
	const QStringList markIds = availableMarkIds();
	const QStringList workflowIds = availableWorkflowIds();
	for (int row = 0; row < m_table->rowCount(); ++row)
	{
		auto* markCombo = qobject_cast<QComboBox*>(m_table->cellWidget(row, 0));
		auto* workflowCombo = qobject_cast<QComboBox*>(m_table->cellWidget(row, 1));
		if (markCombo)
		{
			QString current = markCombo->currentText();
			markCombo->clear();
			markCombo->addItems(markIds);
			int idx = markCombo->findText(current);
			if (idx >= 0) markCombo->setCurrentIndex(idx);
		}
		if (workflowCombo)
		{
			QString current = workflowCombo->currentText();
			workflowCombo->clear();
			workflowCombo->addItems(workflowIds);
			int idx = workflowCombo->findText(current);
			if (idx >= 0) workflowCombo->setCurrentIndex(idx);
		}
	}
}

QStringList Dialog_ProcessSetting_MarkAcquire::availableMarkIds() const
{
	QStringList ids = VisionModule::instance()->getAllMarkIds();
	if (ids.isEmpty()) ids << QString();
	return ids;
}

QStringList Dialog_ProcessSetting_MarkAcquire::availableWorkflowIds() const
{
	QStringList ids = VisionModule::instance()->getWorkflowIds();
	if (ids.isEmpty()) ids << QString();
	return ids;
}
