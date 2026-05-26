#include "Process_OverCutting.h"
#include "CuttingCommand/OverCuttingDivisionPlanner.h"
#include "LogModule.h"
#include "MessageModule.h"
#include "MotionControl.h"
#include "qc_applicationwindow.h"
#include "qc_mdiwindow.h"
#include "rs_document.h"
#include "rs_entitycontainer.h"
#include "rs_layer.h"
#include "rs_layerlist.h"
#include "rs_line.h"
#include "rs_selection.h"
#include "rs_vector.h"
#include <qDebug>
#include <QMessageBox>
#include <QStringList>
#include <algorithm>

namespace
{
	enum class OverCuttingEntitySource
	{
		None,
		Selection,
		Sequence,
		Document
	};

	struct OverCuttingEntityResolveResult
	{
		QList<RS_Entity*> entities;
		RS_Vector minPoint{ 0, 0 };
		RS_Vector maxPoint{ 0, 0 };
		OverCuttingEntitySource source{ OverCuttingEntitySource::None };
		ErrorCode errorCode{ ErrorCode::ERROR_NONE };
	};

	bool IsOnLayer(RS_Entity* entity, const QString& layerName)
	{
		return entity && entity->getLayer() && entity->getLayer()->getName() == layerName;
	}

	bool IsCuttableEntity(RS_Entity* entity)
	{
		if (!entity)
			return false;

		switch (entity->rtti())
		{
		case RS2::EntityLine:
		case RS2::EntityArc:
		case RS2::EntityCircle:
		case RS2::EntityPolyline:
			return true;
		default:
			return false;
		}
	}

	bool IsUsableCutEntity(RS_Entity* entity)
	{
		return entity && !entity->isUndone() && entity->isVisible() && !entity->isLocked() && IsCuttableEntity(entity);
	}

	void UpdateBoundByEntity(RS_Vector& minPoint, RS_Vector& maxPoint, RS_Entity* entity, bool& initialized)
	{
		if (!entity || entity->isUndone())
			return;

		if (!initialized)
		{
			minPoint = entity->getMin();
			maxPoint = entity->getMax();
			initialized = true;
			return;
		}

		minPoint.x = std::min(minPoint.x, entity->getMin().x);
		minPoint.y = std::min(minPoint.y, entity->getMin().y);
		maxPoint.x = std::max(maxPoint.x, entity->getMax().x);
		maxPoint.y = std::max(maxPoint.y, entity->getMax().y);
	}

	bool BuildOverCuttingEntityList(const QList<RS_Entity*>& rawList,
		QList<RS_Entity*>& entityList,
		RS_Vector& minPoint,
		RS_Vector& maxPoint,
		const QString& ignoredLayerName,
		bool requireSelected)
	{
		entityList.clear();
		bool initialized = false;
		for (RS_Entity* entity : rawList)
		{
			if (!IsUsableCutEntity(entity))
				continue;
			if (requireSelected && !entity->isSelected())
				continue;
			if (!ignoredLayerName.isEmpty() && IsOnLayer(entity, ignoredLayerName))
				continue;

			entityList.push_back(entity);
			UpdateBoundByEntity(minPoint, maxPoint, entity, initialized);
		}

		if (initialized)
			return true;

		minPoint = RS_Vector(0, 0);
		maxPoint = RS_Vector(0, 0);
		entityList.clear();
		return false;
	}

	OverCuttingEntityResolveResult ResolveOverCuttingCutEntities(const QString& ignoredLayerName)
	{
		OverCuttingEntityResolveResult result;
		RS_GraphicView* g = QC_ApplicationWindow::getAppWindow()->getGraphicView();
		QC_MDIWindow* m = QC_ApplicationWindow::getAppWindow()->getMDIWindow();
		if (!g || !m || !m->getDocument())
		{
			result.errorCode = ErrorCode::ERROR_OVERCUTTING_NOCUTENTITY;
			return result;
		}

		RS_Selection selection((RS_EntityContainer&)*m->getDocument(), g);

		// 依次尝试：当前框选 → TOML 切割顺序 → 全图。谁先得到非空可用列表就用谁，
		// 不要在选择列表上做"有就只能用它"的短路，避免选择残留中的分割图层图元把入口卡死。
		if (BuildOverCuttingEntityList(selection.GetSelectSequenceList(),
			result.entities, result.minPoint, result.maxPoint, ignoredLayerName, true))
		{
			result.source = OverCuttingEntitySource::Selection;
			return result;
		}

		if (BuildOverCuttingEntityList(selection.GetSequenceList(),
			result.entities, result.minPoint, result.maxPoint, ignoredLayerName, false))
		{
			result.source = OverCuttingEntitySource::Sequence;
			return result;
		}

		if (BuildOverCuttingEntityList(m->getDocument()->getEntityList(),
			result.entities, result.minPoint, result.maxPoint, ignoredLayerName, false))
		{
			result.source = OverCuttingEntitySource::Document;
			return result;
		}

		// 三条来源都没有可用图元，把每条来源里被各种过滤器淘汰的数量统计一下，方便定位
		auto countByPredicate = [&](const QList<RS_Entity*>& list, auto&& pred) {
			int n = 0;
			for (RS_Entity* e : list) if (pred(e)) ++n;
			return n;
		};
		const auto& docList = m->getDocument()->getEntityList();
		const int docUsable = countByPredicate(docList, [](RS_Entity* e) { return IsUsableCutEntity(e); });
		const int docOnIgnored = countByPredicate(docList, [&](RS_Entity* e) { return IsUsableCutEntity(e) && IsOnLayer(e, ignoredLayerName); });
		LOG_PROCESS_ERROR(QObject::tr("[OverCutting][Resolve] all three branches empty. doc total=%1 usable=%2 onIgnored=%3 ignoredLayer='%4'")
			.arg(docList.size()).arg(docUsable).arg(docOnIgnored).arg(ignoredLayerName).toUtf8().data());

		result.errorCode = ErrorCode::ERROR_OVERCUTTING_NOCUTENTITY;
		return result;
	}

	QString FormatAutoDivisionSegments(double startPoint, double endPoint, const vector<double>& divisions)
	{
		QStringList segments;
		double lastPoint = startPoint;
		for (double point : divisions)
		{
			segments << QString("[%1,%2]").arg(lastPoint, 0, 'f', 3).arg(point, 0, 'f', 3);
			lastPoint = point;
		}
		segments << QString("[%1,%2]").arg(lastPoint, 0, 'f', 3).arg(endPoint, 0, 'f', 3);
		return segments.join(" ");
	}
}


ProcessOverCutting::ProcessOverCutting(TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::OverCutting;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Note",""}, {"Count", "1"}, {"Layer", "0"}, {"XCompensation","0"}, {"XSpeed", "1"},  {"YCompensation","0"}, {"YSpeed", "1"},
						{"OpenChuck","500"}, {"OpenPliers","500"}, {"CloseChuck","500"}, {"ClosePliers","500"}, {"EqualLength","1"}, {"SuggestedLen","0"} };
	m_itemData.resize(2);
}

ProcessOverCutting::ProcessOverCutting(const QString& text, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::OverCutting;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Note",""}, {"Count", "1"}, {"Layer", "0"}, {"XCompensation","0"}, {"XSpeed", "1"},  {"YCompensation","0"}, {"YSpeed", "1"},
						{"OpenChuck","500"}, {"OpenPliers","500"}, {"CloseChuck","500"}, {"ClosePliers","500"}, {"EqualLength","1"}, {"SuggestedLen","0"} };
	m_itemData.resize(2);
	UpdateInfo();
}

ProcessOverCutting::ProcessOverCutting(const QVector<QVariant>& data, TreeItem* parent)
{
	m_parentItem	= parent;
	m_type			= ItemType::OverCutting;
	m_state			= ItemState::Unavailable;
	m_stateSave		= m_state;
	m_maps			= { {"Note",""}, {"Count", "1"}, {"Layer", "0"}, {"XCompensation","0"}, {"XSpeed", "1"},  {"YCompensation","0"}, {"YSpeed", "1"},
						{"OpenChuck","500"}, {"OpenPliers","500"}, {"CloseChuck","500"}, {"ClosePliers","500"}, {"EqualLength","1"}, {"SuggestedLen","0"} };
	m_itemData.resize(2);
	m_itemData		= data;
}

ProcessOverCutting::~ProcessOverCutting(void)
{
	qDeleteAll(m_childItems);
	m_childItems.clear();
	qDeleteAll(m_childItems);
}

TreeItem* ProcessOverCutting::clone() const
{
	TreeItem* newItem = nullptr;

	if (!m_itemData.isEmpty()) {
		newItem = new ProcessOverCutting(QStringLiteral("OverCutting"));
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

void ProcessOverCutting::Edit()
{
	if (m_state == ItemState::Unavailable || m_state == ItemState::Enable || m_state == ItemState::Disable)
	{
		Dialog_ProcessSetting_OverCutting* Dialog = new Dialog_ProcessSetting_OverCutting();
		Dialog->setAttribute(Qt::WA_DeleteOnClose);    //关闭对话框的同时，回收内存
		Dialog->m_pProcessOverCutting = this;
		Dialog->ViewSetting();
		Dialog->show();
	}
}

void ProcessOverCutting::UpdateInfo()
{
	if (TreeItem::IsChinese())
		setData(0, "\xE8\xB6\x85\xE8\xA1\x8C\xE7\xA8\x8B\xE5\x88\x87\xE5\x89\xB2"); // 超行程切割
	else
		setData(0, "OverCutting");
	
	setData(1, m_maps["Note"]);
}

void ProcessOverCutting::SwitchState(ItemState state)
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

#include "qc_applicationwindow.h"
#include "rs_graphic.h"
Dialog_ProcessSetting_OverCutting::Dialog_ProcessSetting_OverCutting(QWidget* parent) :
	QDialog(parent),
	m_pProcessOverCutting(nullptr),
	Dialog_OverCutting(new Ui::Dialog_ProcessSetting_OverCutting)
{
	Dialog_OverCutting->setupUi(this);
	setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowTitleHint);

	connect(Dialog_OverCutting->Button_OverCutting_OK,		SIGNAL(clicked()), this, SLOT(ButtonOK()));
	connect(Dialog_OverCutting->Button_OverCutting_Cancel,	SIGNAL(clicked()), this, SLOT(ButtonCancel()));
	connect(Dialog_OverCutting->Button_OverCutting_AutoDivision, SIGNAL(clicked()), this, SLOT(ButtonAutoDivision()));
}

Dialog_ProcessSetting_OverCutting::~Dialog_ProcessSetting_OverCutting()
{
	delete Dialog_OverCutting;
}

void Dialog_ProcessSetting_OverCutting::ViewSetting()
{
	

	//修改标签状态，避免重复打开标签设置
	m_pProcessOverCutting->SwitchState(ItemState::Editing);

	//限制输入值
	Dialog_OverCutting->lineEdit_OverCutting_XCompensation		->setValidator(new QRegExpValidator(Regex_Pos_Double));
    Dialog_OverCutting->lineEdit_OverCutting_YCompensation		->setValidator(new QRegExpValidator(Regex_Pos_Double));
	Dialog_OverCutting->lineEdit_OverCutting_OpenChuckDelay		->setValidator(new QRegExpValidator(Regex_Nonnegative_Int));
	Dialog_OverCutting->lineEdit_OverCutting_OpenPliersDelay	->setValidator(new QRegExpValidator(Regex_Nonnegative_Int));
	Dialog_OverCutting->lineEdit_OverCutting_CloseChuckDelay	->setValidator(new QRegExpValidator(Regex_Nonnegative_Int));
	Dialog_OverCutting->lineEdit_OverCutting_ClosePliersDelay	->setValidator(new QRegExpValidator(Regex_Nonnegative_Int));
	Dialog_OverCutting->lineEdit_OverCutting_SuggestedLen		->setValidator(new QRegExpValidator(Regex_Pos_Double));
	
	//读取参数
	QString		qstrNote			= m_pProcessOverCutting->GetOverCuttingNote();
	bool		bCount				= m_pProcessOverCutting->GetOverCuttingCount();
	QString		qstrLayer			= m_pProcessOverCutting->GetOverCuttingLayer();
	QString		qstrXCompensation	= m_pProcessOverCutting->GetOverCuttingXCompensation();
	int			iXSpeed				= m_pProcessOverCutting->GetOverCuttingXSpeed();
	QString		qstrYCompensation	= m_pProcessOverCutting->GetOverCuttingYCompensation();
	int			iYSpeed				= m_pProcessOverCutting->GetOverCuttingYSpeed();
	QString		qstrOpenChuck		= m_pProcessOverCutting->GetOverCuttingOpenChuck();
	QString		qstrOpenPliers		= m_pProcessOverCutting->GetOverCuttingOpenPliers();
	QString		qstrCloseChuck		= m_pProcessOverCutting->GetOverCuttingCloseChuck();
	QString		qstrClosePliers		= m_pProcessOverCutting->GetOverCuttingClosePliers();
	bool		bEqualLength		= m_pProcessOverCutting->GetOverCuttingEqualLength();
	QString		qstrSuggestedLen	= m_pProcessOverCutting->GetOverCuttingSuggestedLen();

	//显示参数
	Dialog_OverCutting->lineEdit_OverCutting_Note				->setText(qstrNote);
	Dialog_OverCutting->checkBox_OverCutting_Count				->setChecked(bCount);
	
	Dialog_OverCutting->lineEdit_OverCutting_XCompensation		->setText(qstrXCompensation);
	Dialog_OverCutting->comboBox_OverCutting_XSpeed				->setCurrentIndex(iXSpeed);
	Dialog_OverCutting->lineEdit_OverCutting_YCompensation		->setText(qstrYCompensation);
	Dialog_OverCutting->comboBox_OverCutting_YSpeed				->setCurrentIndex(iYSpeed);
	Dialog_OverCutting->lineEdit_OverCutting_OpenChuckDelay		->setText(qstrOpenChuck);
	Dialog_OverCutting->lineEdit_OverCutting_OpenPliersDelay	->setText(qstrOpenPliers);
	Dialog_OverCutting->lineEdit_OverCutting_CloseChuckDelay	->setText(qstrCloseChuck);
	Dialog_OverCutting->lineEdit_OverCutting_ClosePliersDelay	->setText(qstrClosePliers);
	Dialog_OverCutting->checkBox_OverCutting_EqualLength		->setChecked(bEqualLength);
	Dialog_OverCutting->lineEdit_OverCutting_SuggestedLen		->setText(qstrSuggestedLen);

	RS_EntityContainer* d = QC_ApplicationWindow::getAppWindow()->getDocument();
	RS_Graphic* grap = (RS_Graphic*)d;
	RS_LayerList* layerList = grap->getLayerList();
	bool bsuccess = false;
	for (auto e : *layerList)
	{
		Dialog_OverCutting->comboBox_OverCutting_Layer->addItem(e->getName());
		if (e->getName() == qstrLayer)
		{
			bsuccess = true;
		}
	}
	if (bsuccess)
		Dialog_OverCutting->comboBox_OverCutting_Layer->setCurrentText(qstrLayer);
	else
		Dialog_OverCutting->comboBox_OverCutting_Layer->setCurrentIndex(0);

}

void Dialog_ProcessSetting_OverCutting::ButtonOK()
{
	//获取参数
	QString		qstrNote			= Dialog_OverCutting->lineEdit_OverCutting_Note				->text();
	bool		bCount				= Dialog_OverCutting->checkBox_OverCutting_Count			->isChecked();
	QString		qstrLayer			= Dialog_OverCutting->comboBox_OverCutting_Layer			->currentText();
	QString		qstrXCompensation	= Dialog_OverCutting->lineEdit_OverCutting_XCompensation	->text();
	int			iXSpeed				= Dialog_OverCutting->comboBox_OverCutting_XSpeed			->currentIndex();
	QString		qstrYCompensation	= Dialog_OverCutting->lineEdit_OverCutting_YCompensation	->text();
	int			iYSpeed				= Dialog_OverCutting->comboBox_OverCutting_YSpeed			->currentIndex();
	QString		qstrOpenChuck		= Dialog_OverCutting->lineEdit_OverCutting_OpenChuckDelay	->text();
	QString		qstrOpenPliers		= Dialog_OverCutting->lineEdit_OverCutting_OpenPliersDelay	->text();
	QString		qstrCloseChuck		= Dialog_OverCutting->lineEdit_OverCutting_CloseChuckDelay	->text();
	QString		qstrClosePliers		= Dialog_OverCutting->lineEdit_OverCutting_ClosePliersDelay	->text();
	bool		bEqualLength		= Dialog_OverCutting->checkBox_OverCutting_EqualLength		->isChecked();
	QString		qstrSuggestedLen	= Dialog_OverCutting->lineEdit_OverCutting_SuggestedLen		->text();

	//设置参数
	m_pProcessOverCutting->SetOverCuttingNote(qstrNote);
	m_pProcessOverCutting->SetOverCuttingCount(bCount);
	m_pProcessOverCutting->SetOverCuttingLayer(qstrLayer);
	m_pProcessOverCutting->SetOverCuttingXCompensation(qstrXCompensation);
	m_pProcessOverCutting->SetOverCuttingXSpeed(iXSpeed);
	m_pProcessOverCutting->SetOverCuttingYCompensation(qstrYCompensation);
	m_pProcessOverCutting->SetOverCuttingYSpeed(iYSpeed);
	m_pProcessOverCutting->SetOverCuttingOpenChuck(qstrOpenChuck);
	m_pProcessOverCutting->SetOverCuttingOpenPliers(qstrOpenPliers);
	m_pProcessOverCutting->SetOverCuttingCloseChuck(qstrCloseChuck);
	m_pProcessOverCutting->SetOverCuttingClosePliers(qstrClosePliers);
	m_pProcessOverCutting->SetOverCuttingEqualLength(bEqualLength);
	m_pProcessOverCutting->SetOverCuttingSuggestedLen(qstrSuggestedLen);

	m_pProcessOverCutting->SetState(ItemState::Enable);
	m_pProcessOverCutting->SwitchState(ItemState::Enable);
	m_pProcessOverCutting->UpdateInfo();
	QDialog::accept();
}

void Dialog_ProcessSetting_OverCutting::ButtonAutoDivision()
{
	QString qstrLayer = Dialog_OverCutting->comboBox_OverCutting_Layer->currentText();
	if (qstrLayer.isEmpty())
	{
		SHOW_PROCESS_ERROR(ErrorCode::ERROR_OVERCUTTING_NODIVISIONLAYER);
		return;
	}

	RS_Document* document = QC_ApplicationWindow::getAppWindow()->getDocument();
	RS_GraphicView* view = QC_ApplicationWindow::getAppWindow()->getGraphicView();
	MotionControl* pMC = QC_ApplicationWindow::getAppWindow()->getService()->GetMotionControl();
	if (!document || !view || !pMC)
	{
		SHOW_PROCESS_ERROR(ErrorCode::ERROR_OVERCUTTING_NOCUTENTITY);
		return;
	}

	OverCuttingEntityResolveResult resolveResult = ResolveOverCuttingCutEntities(qstrLayer);
	if (resolveResult.entities.empty())
	{
		SHOW_PROCESS_ERROR(resolveResult.errorCode == ErrorCode::ERROR_NONE ?
			ErrorCode::ERROR_OVERCUTTING_NOCUTENTITY : resolveResult.errorCode);
		return;
	}

	double lx = 0.0, rx = 0.0;
	pMC->ReadAxisSoftLimit(Axis::X, lx, rx);
	// 软限位未就绪（设备未初始化完成）：lx 与 rx 都为 0 或区间退化
	if (rx - lx <= 1e-6)
	{
		LOG_PROCESS_ERROR(QObject::tr("[OverCutting][Auto] X soft limit not ready: lx=%1 rx=%2")
			.arg(lx, 0, 'f', 3).arg(rx, 0, 'f', 3).toUtf8().data());
		SHOW_PROCESS_ERROR(ErrorCode::ERROR_OVERCUTTING_LIMITNOTREADY);
		return;
	}

	// 图元起点已超出右限位 → 真正的越限
	double dLimited = rx - resolveResult.minPoint.x;
	if (dLimited <= 0)
	{
		SHOW_PROCESS_ERROR(ErrorCode::ERROR_OVERCUTTING_OUTOFLIMIT);
		return;
	}

	OverCuttingDivisionParams params;
	params.startX = resolveResult.minPoint.x;
	params.endX = resolveResult.maxPoint.x;
	params.maxLength = dLimited;
	params.suggestedLength = Dialog_OverCutting->lineEdit_OverCutting_SuggestedLen->text().toDouble();
	params.equalLength = Dialog_OverCutting->checkBox_OverCutting_EqualLength->isChecked();

	OverCuttingDivisionResult result;
	ErrorCode errorCode = OverCuttingDivisionPlanner::Plan(resolveResult.entities, params, result);
	if (errorCode != ErrorCode::ERROR_NONE)
	{
		SHOW_PROCESS_ERROR(errorCode);
		return;
	}
	if (result.divisions.empty())
	{
		QMessageBox::information(this,
			tr("Auto division"),
			tr("The drawing fits within the available X travel. No division line is needed."));
		LOG_PROCESS_INFO(QObject::tr("[OverCutting][Auto] No division line is needed. Range: [%1,%2], capacity: %3")
			.arg(resolveResult.minPoint.x, 0, 'f', 3)
			.arg(resolveResult.maxPoint.x, 0, 'f', 3)
			.arg(result.capacity, 0, 'f', 3).toUtf8().data());
		return;
	}

	if (QMessageBox::question(this,
		tr("Auto division"),
		tr("Existing entities on the target division layer will be replaced. Continue?"),
		QMessageBox::Yes | QMessageBox::No,
		QMessageBox::No) != QMessageBox::Yes)
	{
		return;
	}

	QList<RS_Entity*> removeList;
	for (auto entity : *document)
	{
		if (entity && !entity->isUndone() && entity->getLayer() && entity->getLayer()->getName() == qstrLayer)
			removeList.push_back(entity);
	}

	RS_Graphic* graphic = (RS_Graphic*)document;
	RS_Layer* layer = document->getLayerList()->find(qstrLayer);
	if (!layer)
	{
		layer = new RS_Layer(qstrLayer);
		graphic->addLayer(layer);
	}

	double yMargin = std::max((resolveResult.maxPoint.y - resolveResult.minPoint.y) * 2.0, 100.0);
	double yStart = resolveResult.minPoint.y - yMargin;
	double yEnd = resolveResult.maxPoint.y + yMargin;

	document->startUndoCycle();
	for (RS_Entity* entity : removeList)
	{
		entity->setSelected(false);
		entity->changeUndoState();
		document->addUndoable(entity);
	}
	for (double divisionX : result.divisions)
	{
		RS_Line* line = new RS_Line(document, RS_LineData(RS_Vector(divisionX, yStart), RS_Vector(divisionX, yEnd)));
		line->setLayer(layer);
		line->setPenToActive();
		document->addEntity(line);
		document->addUndoable(line);
	}
	document->endUndoCycle();
	document->calculateBorders();
	view->redraw(RS2::RedrawDrawing);

	LOG_PROCESS_INFO(QObject::tr("[OverCutting][Auto] Generated %1 division lines on layer %2. Segments: %3")
		.arg(static_cast<int>(result.divisions.size())).arg(qstrLayer)
		.arg(FormatAutoDivisionSegments(resolveResult.minPoint.x, resolveResult.maxPoint.x, result.divisions)).toUtf8().data());
}

void Dialog_ProcessSetting_OverCutting::ButtonCancel()
{
	//点击取消/ESC退出均不保存参数，还原节点状态
	reject();
}

void Dialog_ProcessSetting_OverCutting::reject()
{
	m_pProcessOverCutting->SwitchState();
	m_pProcessOverCutting->UpdateInfo();
	QDialog::reject();
}
