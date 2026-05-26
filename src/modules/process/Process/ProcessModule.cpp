#include <QFile>
#include <QMetaObject>
#include <QPointF>
#include <QThread>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <set>
#include "ProcessModule.h"
#include "qg_IOWidget.h"
#include "qg_AxisWidget.h"
#include "qg_CuttingInfoWidget.h"
#include "qc_applicationwindow.h"
#include "qc_mdiwindow.h"
#include "qg_graphicview.h"
#include "rs_document.h"
#include "rs_entity.h"
#include "rs_selection.h"
#include "DataType.h"
#include "VisionModule.h"
#include "Vision/Manager/VisionManager.h"

namespace {

constexpr double kPi = 3.14159265358979323846;

struct AlignmentTransform
{
	RS_Vector offset;
	RS_Vector basePoint;
	double angleRad = 0.0;
	double scale = 1.0;
};

struct AlignmentSourceContext
{
	QList<RS_Entity*> sourceEntities;
	QList<RS_Entity*> savedSequence;
	QList<RS_Entity*> savedSelectedSequence;
};

QString rowMapValue(const map<QString, QString>& maps, int row, const QString& suffix)
{
	const QString key = QString("Row%1_%2").arg(row).arg(suffix);
	auto it = maps.find(key);
	return it == maps.end() ? QString() : it->second;
}

int mapIntValue(const map<QString, QString>& maps, const QString& key, int defaultValue)
{
	auto it = maps.find(key);
	if (it == maps.end()) return defaultValue;
	bool ok = false;
	int value = it->second.toInt(&ok);
	return ok ? value : defaultValue;
}

void runOnObjectThread(QObject* context, const std::function<void()>& task)
{
	if (!task) {
		return;
	}

	if (!context || context->thread() == QThread::currentThread()) {
		task();
		return;
	}

	QMetaObject::invokeMethod(context, [task]() {
		task();
	}, Qt::BlockingQueuedConnection);
}

QString resolveMarkAcquireCameraId(const VisionWorkflow& workflow)
{
	const QString workflowCameraId = workflow.cameraId.trimmed();
	if (!workflowCameraId.isEmpty()) {
		return workflowCameraId;
	}

	auto cameraIdFromSteps = [&workflow](const QString& preferredStepType) {
		for (const VisionStep& step : workflow.steps) {
			if (!preferredStepType.isEmpty() &&
				step.type.compare(preferredStepType, Qt::CaseInsensitive) != 0) {
				continue;
			}
			if (!step.params.is_table() ||
				!step.params.contains("camera_id") ||
				!step.params.at("camera_id").is_string()) {
				continue;
			}

			const QString stepCameraId = QString::fromStdString(step.params.at("camera_id").as_string()).trimmed();
			if (!stepCameraId.isEmpty()) {
				return stepCameraId;
			}
		}
		return QString();
	};

	const QString captureCameraId = cameraIdFromSteps(QStringLiteral("Capture"));
	if (!captureCameraId.isEmpty()) {
		return captureCameraId;
	}

	return cameraIdFromSteps(QString());
}

QPointF resolveMarkAcquireCaptureOffset(const QString& workflowId)
{
	QPointF offset(0.0, 0.0);
	VisionManager* manager = VisionModule::instance()->getManager();
	if (!manager) {
		return offset;
	}

	runOnObjectThread(manager, [&]() {
		VisionWorkflow* workflow = manager->getWorkflow(workflowId);
		if (!workflow) {
			return;
		}

		const QString cameraId = resolveMarkAcquireCameraId(*workflow);
		CameraManager* cameraManager = manager->getCameraManager();
		if (cameraId.isEmpty() || !cameraManager) {
			return;
		}

		CameraCoordinateTransform transform;
		if (!cameraManager->getCoordinateTransform(cameraId, transform) ||
			transform.method != CameraTransformMethod::Pixel) {
			return;
		}

		offset.setX(transform.pixel.cameraOffsetX);
		offset.setY(transform.pixel.cameraOffsetY);
	});

	return offset;
}

bool resolveExpressionValue(Service* service, const QString& text, double& value, ErrorCode& errorCode)
{
	bool ok = false;
	value = text.toDouble(&ok);
	if (ok) {
		errorCode = ErrorCode::ERROR_NONE;
		return true;
	}

	QString resolved;
	errorCode = Expression::GetResult(text, service->GetCompDevice()->GetComps(), resolved);
	if (errorCode != ErrorCode::ERROR_NONE) {
		return false;
	}

	value = resolved.toDouble(&ok);
	if (!ok) {
		errorCode = ErrorCode::ERROR_EXPRES_EXPRESSIONINVALID;
		return false;
	}
	return true;
}

bool getCurrentDocumentContext(QC_ApplicationWindow* window, RS_Document*& document, QG_GraphicView*& view)
{
	document = nullptr;
	view = nullptr;
	if (!window) return false;
	QC_MDIWindow* mdi = window->getMDIWindow();
	if (!mdi || !mdi->getDocument()) return false;
	document = mdi->getDocument();
	view = mdi->getGraphicView();
	return document != nullptr;
}

AlignmentSourceContext collectAlignmentSourceContext(RS_Document* document, QG_GraphicView* view)
{
	AlignmentSourceContext context;
	if (!document) {
		return context;
	}

	RS_Selection selection((RS_EntityContainer&)*document, view);
	context.savedSelectedSequence = selection.GetSelectSequenceList();
	context.savedSequence = selection.GetSequenceList();
	if (!context.savedSelectedSequence.empty()) {
		context.sourceEntities = context.savedSelectedSequence;
		return context;
	}

	if (!context.savedSequence.empty()) {
		context.sourceEntities = context.savedSequence;
		return context;
	}

	context.sourceEntities = document->getEntityList();
	return context;
}

RS_Vector alignmentWorldMidpoint(const MarkPoint& mark1, const MarkPoint* mark2)
{
	if (!mark2) {
		return RS_Vector(mark1.worldX, mark1.worldY);
	}

	return RS_Vector((mark1.worldX + mark2->worldX) * 0.5,
		(mark1.worldY + mark2->worldY) * 0.5);
}

RS_Entity* createAlignedClone(RS_Entity* sourceEntity, const AlignmentTransform& transform, RS_Document* document)
{
	if (!sourceEntity) {
		return nullptr;
	}

	RS_Entity* cloneEntity = sourceEntity->clone();
	if (!cloneEntity) {
		return nullptr;
	}

	cloneEntity->setParent(document);
	cloneEntity->move(transform.offset);
	if (std::abs(transform.angleRad) > 1e-9) {
		cloneEntity->rotate(transform.basePoint, transform.angleRad);
	}
	if (std::abs(transform.scale - 1.0) > 1e-9) {
		cloneEntity->scale(transform.basePoint, RS_Vector(transform.scale, transform.scale));
	}

	return cloneEntity;
}

void applyAlignmentPreviewStyle(RS_Entity* entity)
{
	if (!entity) {
		return;
	}

	entity->setSelected(false);
	entity->setHighlighted(false);

	RS_Pen previewPen = entity->getPen(true);
	previewPen.setLineType(RS2::DashLine);
	entity->setPen(previewPen);

	if (entity->isContainer() && entity->rtti() != RS2::EntityHatch) {
		auto* container = static_cast<RS_EntityContainer*>(entity);
		for (RS_Entity* child : *container) {
			applyAlignmentPreviewStyle(child);
		}
	}
}

void rebuildAlignmentOverlay(QG_GraphicView* view, const QList<RS_Entity*>& previewEntities)
{
	if (!view) {
		return;
	}

	RS_EntityContainer* overlay = view->getOverlayContainer(RS2::AlignmentOverlay);
	if (!overlay) {
		return;
	}

	overlay->clear();
	for (RS_Entity* entity : previewEntities) {
		if (!entity || entity->isUndone()) {
			continue;
		}

		RS_Entity* previewClone = entity->clone();
		if (!previewClone) {
			continue;
		}

		applyAlignmentPreviewStyle(previewClone);
		overlay->addEntity(previewClone);
	}

	view->redraw(RS2::RedrawOverlay);
}

void clearAlignmentOverlay(QG_GraphicView* view)
{
	if (!view) {
		return;
	}

	RS_EntityContainer* overlay = view->getOverlayContainer(RS2::AlignmentOverlay);
	if (!overlay) {
		return;
	}

	overlay->clear();
	view->redraw(RS2::RedrawOverlay);
}

void clearAlignmentPresentation(QObject* context,
	RS_Document* document,
	QG_GraphicView* view,
	const QList<RS_Entity*>& savedSequence,
	const QList<RS_Entity*>& savedSelectedSequence,
	bool restoreCuttingScope)
{
	runOnObjectThread(context, [document, view, savedSequence, savedSelectedSequence, restoreCuttingScope]() {
		if (document && restoreCuttingScope) {
			auto* container = static_cast<RS_EntityContainer*>(document);
			const QList<RS_Entity*> currentSelected = container->GetSelectSequenceList();
			for (RS_Entity* entity : currentSelected) {
				if (!entity || entity->isUndone()) {
					continue;
				}
				entity->setSelected(false);
			}

			container->ClearSelectSequenceList();
			container->ClearSequenceList();
			if (!savedSequence.isEmpty()) {
				container->SetSequenceList(savedSequence);
			}

			for (RS_Entity* entity : savedSelectedSequence) {
				if (!entity || entity->isUndone()) {
					continue;
				}
				entity->setSelected(true);
				container->AddEntityToList(entity);
			}

			if (view) {
				view->redraw();
			}
		}

		clearAlignmentOverlay(view);
	});
}

void updateAlignmentPresentation(QObject* context,
	RS_Document* document,
	QG_GraphicView* view,
	const QList<RS_Entity*>& alignedSequence)
{
	runOnObjectThread(context, [document, view, alignedSequence]() {
		if (!document) {
			return;
		}

		auto* container = static_cast<RS_EntityContainer*>(document);
		const QList<RS_Entity*> currentSelected = container->GetSelectSequenceList();
		for (RS_Entity* entity : currentSelected) {
			if (!entity || entity->isUndone()) {
				continue;
			}
			entity->setSelected(false);
		}

		container->ClearSelectSequenceList();
		container->SetSequenceList(alignedSequence);

		if (!view) {
			return;
		}

		rebuildAlignmentOverlay(view, alignedSequence);
		view->redraw();
	});
}

}

// 初始化系统状态
boost::atomic<enum::SystemStatus>	g_eState			{ SystemStatus::UnInit };	// 系统状态

// 初始化切割计数
boost::atomic<int>					g_iCuttedOpening	{ 0 };		// 已切开口数
boost::atomic<int>					g_iTotalOpening		{ 0 };		// 总开口数
boost::atomic<int>					g_iCurrentFeeding	{ 0 };		// 当前进给数
boost::atomic<int>					g_iTotalFeeding		{ 0 };		// 总进给数
boost::atomic<int>					g_iCurrentNumber	{ 0 };		// 本次加工数
boost::atomic<int>					g_iTotalNumber		{ 0 };		// 累计加工数

// 初始化标志位
boost::atomic<bool>					g_bInit				{ false };		// 初始化标志
boost::atomic<bool>					g_bRunning			{ false };		// 流程运行标志
boost::atomic<bool>					g_bFinish			{ false };		// 节点线程标志
boost::atomic<bool>					g_bPause			{ false };		// 暂停标志
boost::atomic<bool>					g_bPausing			{ false };		// 暂停中标志
boost::atomic<bool>					g_bStop				{ false };		// 停止标志
boost::atomic<bool>					g_bGroupRuning		{ false };		// 后台运行标志
boost::atomic<bool>					g_bGroupError		{ false };		// 后台运行错误标志

namespace
{
	constexpr int kEnergySwitchTriggerPollMs = 1;
	constexpr int kEnergySwitchStepDelayMs = 5;
	const char* kEnergySwitchTriggerName = "EnergySwitch";

	QString EnergySwitchMapKey(const char* strPrefix, int iIndex)
	{
		return QString("%1%2").arg(strPrefix).arg(iIndex);
	}

	bool TryGetEnergySwitchIntValue(const map<QString, QString>& maps, const char* strPrefix,
		int iIndex, int& iValue)
	{
		auto it = maps.find(EnergySwitchMapKey(strPrefix, iIndex));
		if (it == maps.end())
			return false;

		bool bOk = false;
		iValue = it->second.toInt(&bOk);
		return bOk;
	}

	bool WaitEnergySwitchInterval(int iWaitTime, int iSliceMs)
	{
		if (iWaitTime <= 0)
			return true;

		const int iSleepSliceMs = iSliceMs > 0 ? iSliceMs : iWaitTime;
		auto tpDeadline = std::chrono::steady_clock::now()
			+ std::chrono::milliseconds(iWaitTime);

		while (true)
		{
			if (g_bPause.load())
			{
				auto tpPauseStart = std::chrono::steady_clock::now();
				while (g_bPause.load() && !g_bStop.load())
				{
					boost::this_thread::sleep_for(boost::chrono::milliseconds(iSleepSliceMs));
				}

				if (g_bStop.load())
					return false;

				tpDeadline += std::chrono::steady_clock::now() - tpPauseStart;
			}

			if (g_bStop.load())
				return false;

			const auto tpNow = std::chrono::steady_clock::now();
			if (tpNow >= tpDeadline)
				return true;

			const auto iRemain = std::chrono::duration_cast<std::chrono::milliseconds>(
				tpDeadline - tpNow).count();
			const auto iWait = (std::min)(static_cast<long long>(iSleepSliceMs), iRemain);
			boost::this_thread::sleep_for(boost::chrono::milliseconds(iWait));
		}
	}
}


class MotionControl;
class LaserDevice;

ProcessModule* ProcessModule::uniqueInstance = nullptr;

ProcessModule::ProcessModule(QC_ApplicationWindow * parent) :
	m_iNestingNumber(0)
	, main_window(parent)
	, m_bProcessTest(false)
	, m_pTreeView(NULL)
	, m_pService(NULL)
	, m_pInitThread(NULL)
	, m_pLoadingPosThread(NULL)
{
}

ProcessModule* ProcessModule::instance(ProcessModule* pProcessModule)
{
	if (!uniqueInstance)
		uniqueInstance = pProcessModule;
	return uniqueInstance;
}

void ProcessModule::SetTreeView(ProcessTreeView* model)
{
	m_pTreeView = model;
	connect(this, &ProcessModule::SignalUpdateTreeView, m_pTreeView, &ProcessTreeView::ViewportUpdate, Qt::QueuedConnection);
}

void ProcessModule::SetService(Service* pService)
{
	m_pService = pService;
}

bool ProcessModule::SaveProcessValue(toml::value& valueProcess)
{
	return m_pTreeView->SaveValue(valueProcess);
}

bool ProcessModule::LoadProcessValue(const toml::value& valueProcess)
{
	if (!valueProcess.size())
	{
		SHOW_OPER_INFO(InfoCode::INFO_FILE_NONEPROCESS);
		return false;
	}

	return m_pTreeView->LoadValue(valueProcess);
}

void ProcessModule::Init()
{
	LOG_OPER_INFO(QObject::tr("Click Init").toUtf8().data());// 上锁
	if (m_pInitThread != NULL)
	{
		delete m_pInitThread;
		m_pInitThread = NULL;
	}
	m_pInitThread = new boost::thread(boost::bind(&ProcessModule::InitProcess, this));
}

void ProcessModule::Start(RunMode eModel)
{
	LOG_OPER_INFO(QObject::tr("Click Start").toUtf8().data());
	if (!g_bRunning && g_bInit && g_eState.load() == SystemStatus::Idle)
	{
		SetRunMode(eModel);	

		if (!m_pService->GetMotionControl()->IsEnabled())
		{
			g_eState.store(SystemStatus::Error);
			SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_AXISDISABLED);
			return;
		}

		LOG_PROCESS_START();
		LOG_PROCESS_INFO(QObject::tr("[PUSH START]").toUtf8().data());

		m_iNestingNumber = 0;
		g_bRunning = true;
		g_bFinish = false;
		g_bPause = false;
		g_bPausing = false;
		g_bStop = false;
		g_bGroupRuning = false;
		g_bGroupError = false;

		g_eState.store(SystemStatus::Processing);
		main_window->update1();
		// 运行前数据整理及检测
		GetRunItemList();
		LOG_PROCESS_INFO(QObject::tr("Get run item list done.").toUtf8().data());

		m_pService->GetCompDevice()->CleanComps();
		if (!CheckRunItemList())
		{
			RestoreState();
			g_eState.store(SystemStatus::Error);
			g_bRunning = false;
			LOG_PROCESS_STOP();
			return;
		}
		LOG_PROCESS_INFO(QObject::tr("Check run item list done.").toUtf8().data());

		std::set<QString> visitedGroups;
		std::function<bool(const vector<Item>&)> containsFeeding = [&](const vector<Item>& items) -> bool
		{
			for (const Item& item : items)
			{
				if (item.Type == ItemType::Feeding)
					return true;

				if (item.Type != ItemType::RunGroup)
					continue;

				auto itName = item.maps.find("Name");
				if (itName == item.maps.end())
					continue;

				const QString groupName = itName->second;
				if (!visitedGroups.insert(groupName).second)
					continue;

				auto itGroup = map_Group.find(groupName);
				if (itGroup != map_Group.end() && containsFeeding(itGroup->second))
					return true;
			}
			return false;
		};
		std::set<QString> countingGroups;
		std::function<int(const vector<Item>&)> countTotalFeedings = [&](const vector<Item>& items) -> int
		{
			int totalFeedings = 0;
			for (int index = 0; index < static_cast<int>(items.size()); ++index)
			{
				const Item& item = items[index];
				if (item.Type == ItemType::Feeding)
				{
					totalFeedings++;
					continue;
				}

				if (item.Type == ItemType::RunGroup)
				{
					auto itName = item.maps.find("Name");
					if (itName == item.maps.end())
						continue;

					const QString groupName = itName->second;
					if (!countingGroups.insert(groupName).second)
						continue;

					auto itGroup = map_Group.find(groupName);
					if (itGroup != map_Group.end())
						totalFeedings += countTotalFeedings(itGroup->second);

					countingGroups.erase(groupName);
					continue;
				}

				if (item.Type != ItemType::Loop)
					continue;

				bool ok = false;
				const int loopTotal = item.maps.at("Number").toInt(&ok);
				const int effectiveLoopTotal = ok ? loopTotal : 0;
				const int loopParent = item.iParentIndex;
				vector<Item> loopItems;
				for (++index; index < static_cast<int>(items.size()) && items[index].iParentIndex == loopParent; ++index)
				{
					loopItems.push_back(items[index]);
				}
				--index;

				if (!loopItems.empty() && effectiveLoopTotal > 0)
					totalFeedings += countTotalFeedings(loopItems) * effectiveLoopTotal;
			}
			return totalFeedings;
		};
		QG_CuttingInfoWidget::SetUseFeedingCycleTimer(containsFeeding(vec_RunItemList));

		QG_CuttingInfoWidget::ResetCount();
		QG_CuttingInfoWidget::SetTotalCutCount(0);
		QG_CuttingInfoWidget::SetTotalFeedCount(countTotalFeedings(vec_RunItemList));
		QG_CuttingInfoWidget::StartTimer();
		m_ProcessThread.detach();
		m_ProcessThread = boost::thread(boost::bind(&ProcessModule::ProcessThread, this, vec_RunItemList));
	}
}

void ProcessModule::Pause()
{
	LOG_OPER_INFO(QObject::tr("Click Pause").toUtf8().data());
	if (g_bRunning)
	{
		LOG_PROCESS_WARN(QObject::tr("[PUSH PAUSE]").toUtf8().data());
		g_bPause.store(true);
	}
}

void ProcessModule::Continue()
{
	LOG_OPER_INFO(QObject::tr("Click Continue").toUtf8().data());
	if (g_bPause.load() || g_bPausing.load())
	{
		QG_CuttingInfoWidget::ContinueTimer();
		LOG_PROCESS_WARN(QObject::tr("[PUSH CONTINUE]").toUtf8().data());
		g_bPause.store(false);
		ContinueProcess();
	}
}

void ProcessModule::Stop()
{
	LOG_OPER_INFO(QObject::tr("Click Stop").toUtf8().data());
	if (g_bRunning.load() || g_bPause.load())
	{
		g_bStop.store(true);
		LOG_PROCESS_WARN(QObject::tr("[PUSH STOP]").toUtf8().data());
	}
	else
	{
		StopProcess();
	}
}

void ProcessModule::BackToIdle()
{
	if (!g_bRunning.load() && (g_eState.load() == SystemStatus::Error || g_eState.load() == SystemStatus::Idle))
	{
		g_bRunning = false;
		g_bFinish = false;
		g_bPause = false;
		g_bStop = false;

		if (g_eState.load() != SystemStatus::UnInit)
		{
			QG_CuttingInfoWidget::ResetTimer();
			QG_CuttingInfoWidget::ResetCount();
			//RestoreState();
			g_eState.store(SystemStatus::Idle);
		}

		if (m_pService->GetMotionControl())
		{
			m_pService->GetMotionControl()->DigitalOutputSet(DigitalOUT::Buzzer, 0);
			m_pService->GetMotionControl()->ClearAxisState();
		}

		AXISWIDGET->ClearTimer();
		IOWIDGET->ClearTimer();

		main_window->update1();
	}
}

void ProcessModule::SetRunMode(RunMode eModel)
{
	m_eRunMode = eModel;
	switch (m_eRunMode)
	{
	case RunMode::SignMode:
		m_bProcessTest = false;
		LOG_PROCESS_INFO(QObject::tr("[RunMode] SignMode").toUtf8().data());
		break;
	case RunMode::CuttingTest:
		m_bProcessTest = false;
		LOG_PROCESS_INFO(QObject::tr("[RunMode] CuttingTest").toUtf8().data());
		break;
	case RunMode::ProcessTest:
		m_bProcessTest = true;
		LOG_PROCESS_INFO(QObject::tr("[RunMode] ProcessTest").toUtf8().data());
		break;
	default:
		break;
	}
}


void ProcessModule::LoadingPos()
{
	LOG_OPER_INFO(QObject::tr("Click LoadingPos").toUtf8().data());
	if (!g_bRunning && g_bInit)
	{
		g_bRunning = true;
		g_bStop = false;

		if (m_pLoadingPosThread != NULL)
		{
			delete m_pLoadingPosThread;
			m_pLoadingPosThread = NULL;
		}
		m_pLoadingPosThread = new boost::thread(boost::bind(&ProcessModule::LoadingPosProcess, this));
	}
}

void ProcessModule::GetRunItemList()
{
	Item item;

	//出现问题，指针需要初始化
	if (!vec_RunItemList.empty())
		vec_RunItemList.clear();
	if (!map_Group.empty())
		map_Group.clear();

	bool bStart = false;

	// 该部分后续验证成功后根据获取的容器重写该处理函数
	//vector<Item> vecTreeView;
	//vecTreeView = m_pTreeView->GetTreeItem();

	//遍历主节点
	int parentCount = m_pTreeView->Count();

	for (int i = 0; i < parentCount; i++)
	{
		ItemType typeParent = m_pTreeView->GetItem(i)->GetType();
		ItemState stateParent = m_pTreeView->GetItem(i)->GetState();
		//判断主节点是否为Start，是则从此处开始向后添加
		if (!bStart)
		{
			if (typeParent == ItemType::Start && stateParent == ItemState::Enable)
				bStart = true;
		}

		//判断主节点是否为Group,且是否启用状态
		vector<Item> vGroup;
		bool bGroup = false;
		QString qstrGroupName;
		if (typeParent == ItemType::Group)
		{
			if (stateParent == ItemState::Enable)
			{
				bGroup = true;
				vGroup.clear();
				map<QString, QString> maps = m_pTreeView->GetItem(i)->GetMaps();
				qstrGroupName = maps["Name"];
			}
			else
			{
				for (int j = -1; j < m_pTreeView->GetItem(i)->childCount(); j++)
					m_pTreeView->GetItem(i, j)->SwitchState(ItemState::Unuse);
				continue;
			}
		}

		//从主节点（-1）开始遍历该主节点下的子节点
		int childCount = m_pTreeView->Count(i);
		for (int j = -1; j < childCount; j++)
		{
			//获取iParentIndex、iChildrenIndex
			item.iParentIndex = i;
			item.iChildrenIndex = j;
			ItemState state = m_pTreeView->GetItem(i, j)->GetState();
			if (state == ItemState::Enable && stateParent == ItemState::Enable)
			{
				//获取Tpye和maps
				item.Type = m_pTreeView->GetItem(i, j)->GetType();
				item.maps = m_pTreeView->GetItem(i, j)->GetMaps();

				//向容器压入数据
				if (!bGroup)
				{
					if (bStart)
						vec_RunItemList.push_back(item);
				}
				else
					vGroup.push_back(item);

				m_pTreeView->GetItem(i, j)->SwitchState(ItemState::Unrun);
			}
			else
			{
				m_pTreeView->GetItem(i, j)->SwitchState(ItemState::Unuse);
			}
		}

		//向map添加运行的组索引
		if (bGroup)
		{
			bGroup = false;
			map_Group[qstrGroupName] = vGroup;
			vGroup.swap(vGroup);
		}
	}
}

bool ProcessModule::CheckRunItemList()
{
	bool bFound = false;
	for (int i = 0; i < vec_RunItemList.size(); i++)
	{
		switch (vec_RunItemList[i].Type)	{
		case ItemType::AxesMove:	{
			struct AxisMoveInfo { Axis axis = Axis::X; QString sAxis = ""; bool b = false; double dPos = 0.0; };
			AxisMoveInfo moves[10];
			switch (vec_RunItemList[i].maps.at("Mode").toInt())	{
			case 0: {	// 上料位
				table tL = SETTINGS->GetTable(SettingSection::LoadingPos, "LoadingPos");
				moves[0] = { Axis::Z,  "ZIdle",	 tL["bLoadingPosZIdle"].as_boolean(),	tL["fLoadingPosZIdle"].as_floating() };
				moves[1] = { Axis::Z1, "Z1Idle", tL["bLoadingPosZ1Idle"].as_boolean(),	tL["fLoadingPosZ1Idle"].as_floating()};
				moves[2] = { Axis::X,  "X",		 tL["bLoadingPosX"].as_boolean(),		tL["fLoadingPosX"].as_floating()	 };
				moves[3] = { Axis::Y,  "Y",		 tL["bLoadingPosY"].as_boolean(),		tL["fLoadingPosY"].as_floating()	 };
				moves[4] = { Axis::A,  "A",		 tL["bLoadingPosA"].as_boolean(),		tL["fLoadingPosA"].as_floating()	 };
				moves[5] = { Axis::A1, "A1",	 tL["bLoadingPosA1"].as_boolean(),		tL["fLoadingPosA1"].as_floating()	 };
				moves[6] = { Axis::X1, "X1",	 tL["bLoadingPosX1"].as_boolean(),		tL["fLoadingPosX1"].as_floating()	 };
				moves[7] = { Axis::Y1, "Y1",	 tL["bLoadingPosY1"].as_boolean(),		tL["fLoadingPosY1"].as_floating()	 };
				moves[8] = { Axis::Z,  "Z",		 tL["bLoadingPosZ"].as_boolean(),		tL["fLoadingPosZ"].as_floating()	 };
				moves[9] = { Axis::Z1, "Z1",	 tL["bLoadingPosZ1"].as_boolean(),		tL["fLoadingPosZ1"].as_floating()	 };
				break;	}
	
			case 1:	{	// 下料位
				table tB = SETTINGS->GetTable(SettingSection::LoadingPos, "BlankingPos");
				moves[0] = { Axis::Z,  "ZIdle",	 tB["bBlankingPosZIdle"].as_boolean(),	tB["fBlankingPosZIdle"].as_floating()	};
				moves[1] = { Axis::Z1, "Z1Idle", tB["bBlankingPosZ1Idle"].as_boolean(),	tB["fBlankingPosZ1Idle"].as_floating()	};
				moves[2] = { Axis::X,  "X",		 tB["bBlankingPosX"].as_boolean(),		tB["fBlankingPosX"].as_floating()		};
				moves[3] = { Axis::Y,  "Y",		 tB["bBlankingPosY"].as_boolean(),		tB["fBlankingPosY"].as_floating()		};
				moves[4] = { Axis::A,  "A",		 tB["bBlankingPosA"].as_boolean(),		tB["fBlankingPosA"].as_floating()		};
				moves[5] = { Axis::A1, "A1",	 tB["bBlankingPosA1"].as_boolean(),		tB["fBlankingPosA1"].as_floating()		};
				moves[6] = { Axis::X1, "X1",	 tB["bBlankingPosX1"].as_boolean(),		tB["fBlankingPosX1"].as_floating()		};
				moves[7] = { Axis::Y1, "Y1",	 tB["bBlankingPosY1"].as_boolean(),		tB["fBlankingPosY1"].as_floating()		};
				moves[8] = { Axis::Z,  "Z",		 tB["bBlankingPosZ"].as_boolean(),		tB["fBlankingPosZ"].as_floating()		};
				moves[9] = { Axis::Z1, "Z1",	 tB["bBlankingPosZ1"].as_boolean(),		tB["fBlankingPosZ1"].as_floating()		};
				break;	}

			case 2: {	// 手动模式
				map<QString, QString> maps = vec_RunItemList[i].maps;
				moves[0] = { Axis::Z,  "ZIdle",	 (bool)maps.at("ZIdle").toInt(),	maps.at("ZIdlePos").toDouble() 	};
				moves[1] = { Axis::Z1, "Z1Idle", (bool)maps.at("Z1Idle").toInt(),	maps.at("Z1IdlePos").toDouble() };
				moves[2] = { Axis::X,  "X",		 (bool)maps.at("X").toInt(),		maps.at("XPos").toDouble() 		};
				moves[3] = { Axis::Y,  "Y",		 (bool)maps.at("Y").toInt(),		maps.at("YPos").toDouble() 		};
				moves[4] = { Axis::A,  "A",		 (bool)maps.at("A").toInt(),		maps.at("APos").toDouble() 		};
				moves[5] = { Axis::A1, "A1",	 (bool)maps.at("A1").toInt(),		maps.at("A1Pos").toDouble() 	};
				moves[6] = { Axis::X1, "X1",	 (bool)maps.at("X1").toInt(),		maps.at("X1Pos").toDouble() 	};
				moves[7] = { Axis::Y1, "Y1",	 (bool)maps.at("Y1").toInt(),		maps.at("Y1Pos").toDouble() 	};
				moves[8] = { Axis::Z,  "Z",		 (bool)maps.at("Z").toInt(),		maps.at("ZPos").toDouble() 		};
				moves[9] = { Axis::Z1, "Z1",	 (bool)maps.at("Z1").toInt(),		maps.at("Z1Pos").toDouble() 	};
				break;	}

			default:	break;	}

			for (const AxisMoveInfo& m : moves)
			{
				if (!DT::IsAxisUse(m.axis))	continue;
				if (!m.b)					continue;
				if (!m_pService->GetMotionControl()->IsReachPos(m.axis, false, m.dPos))
				{
					SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_AXISOUTOFLIMIT, QObject::tr("%1\Axis %2 move to %3 out of limit !")
						.arg("[ERROR_PROCESS_AXISOUTOFLIMIT]").arg(m.sAxis).arg(QString::number(m.dPos)).toUtf8().data());
					return false;
				}
			}
			break;
		}

		case ItemType::IO:
		{
			if (vec_RunItemList[i].maps.at("Type").toInt())
			{
				if (!DT::IsDigitalOUTList(vec_RunItemList[i].maps.at("IO")))
				{
					SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_IOINVALID, QObject::tr("%1\IO index \"%2\" is invalid !")
						.arg("[ERROR_PROCESS_IOINVALID]").arg(vec_RunItemList[i].maps.at("IO")).toUtf8().data());
					return false;
				}
				if (vec_RunItemList[i].maps.at("IO") != "NONE")
				{
					if (!Regex_Digital_Out.exactMatch(vec_RunItemList[i].maps.at("Value")))
					{
						SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_IOINVALID, QObject::tr("%1\IO value \"%2\" is invalid !")
							.arg("[ERROR_PROCESS_IOINVALID]").arg(vec_RunItemList[i].maps.at("Value")).toUtf8().data());
						return false;
					}
				}
				else
				{
					if (!Regex_Digital_IndexOut.exactMatch(vec_RunItemList[i].maps.at("Value")))
					{
						SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_IOINVALID, QObject::tr("%1\IO index and value \"%2\" is invalid !")
							.arg("[ERROR_PROCESS_IOINVALID]").arg(vec_RunItemList[i].maps.at("Value")).toUtf8().data());
						return false;
					}
				}
			}
			else
			{
				if (!DT::IsAnalogOUTList(vec_RunItemList[i].maps.at("IO")))
				{
					SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_IOINVALID, QObject::tr("%1\IO index \"%2\" is invalid !")
						.arg("[ERROR_PROCESS_IOINVALID]").arg(vec_RunItemList[i].maps.at("IO")).toUtf8().data());
					return false;
				}
				if (vec_RunItemList[i].maps.at("IO") != "NONE")
				{
					if (!Regex_Analog_Out.exactMatch(vec_RunItemList[i].maps.at("Value")))
					{
						SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_IOINVALID, QObject::tr("%1\IO value \"%2\" is invalid !")
							.arg("[ERROR_PROCESS_IOINVALID]").arg(vec_RunItemList[i].maps.at("Value")).toUtf8().data());
						return false;
					}
				}
				else
				{
					if (!Regex_Analog_IndexOut.exactMatch(vec_RunItemList[i].maps.at("Value")))
					{
						SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_IOINVALID, QObject::tr("%1\IO index and value \"%2\" is invalid !")
							.arg("[ERROR_PROCESS_IOINVALID]").arg(vec_RunItemList[i].maps.at("Value")).toUtf8().data());
						return false;
					}
				}
			}
			break;
		}

		case ItemType::Measurement:
		{
			ErrorCode eCode;
			QString qstrIndex = vec_RunItemList[i].maps.at("Index");
			CompValue cValue{ "1","1" }, cResult;
			m_pService->GetCompDevice()->SetCompValue(qstrIndex, cValue);

			bool bNumber;
			vec_RunItemList[i].maps.at("X").toDouble(&bNumber);
			if (bNumber)
				cResult.X = vec_RunItemList[i].maps.at("X");
			else
			{
				eCode = Expression::GetResult(vec_RunItemList[i].maps.at("X"), cValue, cResult.X);
				if (eCode != ErrorCode::ERROR_NONE)
				{
					SHOW_PROCESS_ERROR(ErrorCode::ERROR_EXPRES_EXPRESSIONINVALID, QObject::tr("%1 %2")
						.arg("[ERROR_EXPRES_EXPRESSIONINVALID]").arg(cResult.X).toUtf8().data());
					return false;
				}
			}

			vec_RunItemList[i].maps.at("Y").toDouble(&bNumber);
			if (bNumber)
				cResult.Y = vec_RunItemList[i].maps.at("Y");
			else
			{
				eCode = Expression::GetResult(vec_RunItemList[i].maps.at("Y"), cValue, cResult.Y);
				if (eCode != ErrorCode::ERROR_NONE)
				{
					SHOW_PROCESS_ERROR(ErrorCode::ERROR_EXPRES_EXPRESSIONINVALID, QObject::tr("%1 %2")
						.arg("[ERROR_EXPRES_EXPRESSIONINVALID]").arg(cResult.Y).toUtf8().data());
					return false;
				}
			}

			m_pService->GetCompDevice()->SetCompValue(qstrIndex, cResult);
			break;
		}

		case ItemType::Calculation:
		{
			ErrorCode eCode;
			QString qstrIndex = vec_RunItemList[i].maps.at("Index");
			CompValue cResult;
			map<QString, CompValue>	mapComps = m_pService->GetCompDevice()->GetComps();
			bool bNumber;
			QString qstrX = vec_RunItemList[i].maps.at("X");
			if (qstrX[0] == '<' || qstrX[0] == '=')
			{	// 误差判定
				qstrX.mid(1).toDouble(&bNumber);
				if (!bNumber)
				{
					SHOW_PROCESS_ERROR(ErrorCode::ERROR_EXPRES_EXPRESSIONINVALID, QObject::tr("%1 %2")
						.arg("[ERROR_EXPRES_EXPRESSIONINVALID]").arg(cResult.X).toUtf8().data());
					return false;
				}
			}
			else
			{	// 公式/数字
				qstrX.toDouble(&bNumber);
				if (bNumber)
					cResult.X = qstrX;
				else
				{
					eCode = Expression::GetResult(qstrX, mapComps, cResult.X);
					if (eCode != ErrorCode::ERROR_NONE)
					{
						SHOW_PROCESS_ERROR(ErrorCode::ERROR_EXPRES_EXPRESSIONINVALID, QObject::tr("%1 %2")
							.arg("[ERROR_EXPRES_EXPRESSIONINVALID]").arg(cResult.X).toUtf8().data());
						return false;
					}
				}
			}

			QString qstrY = vec_RunItemList[i].maps.at("Y");
			if (qstrY[0] == '<' || qstrY[0] == '=')
			{	// 误差判定
				qstrY.mid(1).toDouble(&bNumber);
				if (!bNumber)
				{
					SHOW_PROCESS_ERROR(ErrorCode::ERROR_EXPRES_EXPRESSIONINVALID, QObject::tr("%1 %2")
						.arg("[ERROR_EXPRES_EXPRESSIONINVALID]").arg(cResult.Y).toUtf8().data());
					return false;
				}
			}
			else
			{	// 公式/数字
				qstrY.toDouble(&bNumber);
				if (bNumber)
					cResult.Y = qstrY;
				else
				{
					eCode = Expression::GetResult(qstrY, mapComps, cResult.Y);
					if (eCode != ErrorCode::ERROR_NONE)
					{
						SHOW_PROCESS_ERROR(ErrorCode::ERROR_EXPRES_EXPRESSIONINVALID, QObject::tr("%1 %2")
							.arg("[ERROR_EXPRES_EXPRESSIONINVALID]").arg(cResult.Y).toUtf8().data());
						return false;
					}
				}
			}
			m_pService->GetCompDevice()->SetCompValue(qstrIndex, cResult);
			break;
		}

		case ItemType::RunGroup:
		{
			// 检测组是否存在
			bFound = false;
			for (const auto& pair : map_Group)
			{
				if (pair.first == vec_RunItemList[i].maps.at("Name"))
				{
					bFound = true;
					break;
				}
			}
			if (!bFound)
			{
				SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_GROUPLOSS, QObject::tr("%1\The group with the name \"%2\" was not found !")
					.arg("[ERROR_PROCESS_GROUPLOSS]").arg(vec_RunItemList[i].maps.at("Name")).toUtf8().data());
				return false;
			}
			break;
		}

		case ItemType::If:
		{
			QString OnGroupName = vec_RunItemList[i].maps.at("On");
			QString OffGroupName = vec_RunItemList[i].maps.at("Off");

			if (OnGroupName.isEmpty() && OffGroupName.isEmpty())
			{
				SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_GROUPLOSS);
				return false;
			}

			// 分别检测两个组名是否存在
			bFound = false;
			for (const auto& pair : map_Group)
			{
				if (pair.first == OnGroupName)
				{
					bFound = true;
					break;
				}
			}
			if (!bFound && !OnGroupName.isEmpty())
			{
				SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_GROUPLOSS, QObject::tr("%1\The group with the name \"%2\" was not found !")
					.arg("[ERROR_PROCESS_GROUPLOSS]").arg(OnGroupName).toUtf8().data());
				return false;
			}
			bFound = false;
			for (const auto& pair : map_Group)
			{
				if (pair.first == OffGroupName)
				{
					bFound = true;
					break;
				}
			}
			if (!bFound && !OffGroupName.isEmpty())
			{
				SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_GROUPLOSS, QObject::tr("%1\The group with the name \"%2\" was not found !")
					.arg("[ERROR_PROCESS_GROUPLOSS]").arg(OffGroupName).toUtf8().data());
				return false;
			}
			break;
		}

		case ItemType::Feeding:
		{
			m_pService->GetCuttingDevice()->ResetLeftLimitPos();
			break;
		}

		case ItemType::Commands:
		{
			QString qstrFile = vec_RunItemList[i].maps.at("File");
			QFile File(qstrFile);
			if (!File.open(QIODevice::ReadOnly))
			{
				SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_NONECOMMANDSFILE, QObject::tr("%1\Commands file path \"%2\" is invalid !")
					.arg("[ERROR_PROCESS_NONECOMMANDSFILE]").arg(qstrFile).toUtf8().data());
				return false;
			}
			QByteArray data = File.readAll();
			File.close();
			string strCommands = std::string(data.constData(), data.length());
			if (!m_pService->GetMotionControl()->CheckBuffer(9, strCommands))
			{
				SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_COMMANDSINVALID, QObject::tr("%1 Path \"%2\" commands is invalid !")
					.arg("[ERROR_PROCESS_COMMANDSINVALID]").arg(qstrFile).toUtf8().data());
				return false;
			}
			break;
		}

		case ItemType::EnergySwitch:
		{
			bool bEnergySwitchUse = DT::getCustomerID() == "MaiTong"
				|| int(DT::getPermission()) > (int)PermissionLevel::Factory;
			if (!bEnergySwitchUse)
				break;

			for (int j = 1; j <= 9; j++)
			{
				int iDivider = 0;
				int iDuration = 0;
				const QString qstrDividerKey = EnergySwitchMapKey("PpDivider", j);
				const QString qstrDurationKey = EnergySwitchMapKey("Duration", j);
				if (!TryGetEnergySwitchIntValue(vec_RunItemList[i].maps, "PpDivider", j, iDivider))
				{
					SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_COMMANDSINVALID,
						QObject::tr("[ERROR_PROCESS_COMMANDSINVALID]EnergySwitch parameter %1 is invalid.")
						.arg(qstrDividerKey).toUtf8().data());
					return false;
				}
				if (!TryGetEnergySwitchIntValue(vec_RunItemList[i].maps, "Duration", j, iDuration))
				{
					SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_COMMANDSINVALID,
						QObject::tr("[ERROR_PROCESS_COMMANDSINVALID]EnergySwitch parameter %1 is invalid.")
						.arg(qstrDurationKey).toUtf8().data());
					return false;
				}
				if (iDivider < 0 || iDuration < 0)
				{
					SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_COMMANDSINVALID,
						QObject::tr("[ERROR_PROCESS_COMMANDSINVALID]EnergySwitch parameters must be nonnegative integers." )
						.toUtf8().data());
					return false;
				}
			}

			bool bHasDivider = false;
			for (int j = 1; j <= 9; j++)
			{
				int iDivider = 0;
				TryGetEnergySwitchIntValue(vec_RunItemList[i].maps, "PpDivider", j, iDivider);
				if (iDivider > 0)
				{
					bHasDivider = true;
					break;
				}
			}
			if (!bHasDivider)
			{
				SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_COMMANDSINVALID,
					QObject::tr("[ERROR_PROCESS_COMMANDSINVALID]EnergySwitch requires at least one positive PpDivider.")
					.toUtf8().data());
				return false;
			}

			MotionControl* pMotionControl = m_pService->GetMotionControl();
			if (!pMotionControl || (pMotionControl->GetName() != "ACS"
				&& pMotionControl->GetName() != "SimulatorCMHP"))
			{
				SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_COMMANDSINVALID,
					QObject::tr("[ERROR_PROCESS_COMMANDSINVALID]EnergySwitch only supports ACSCMHP or SimulatorCMHP motion control.")
					.toUtf8().data());
				return false;
			}

			LaserDevice* pLaserDevice = m_pService->GetLaserDevice();
			if (!pLaserDevice || pLaserDevice->GetName() != "Pharos")
			{
				SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_COMMANDSINVALID,
					QObject::tr("[ERROR_PROCESS_COMMANDSINVALID]EnergySwitch only supports Pharos laser.")
					.toUtf8().data());
				return false;
			}

			int iTriggerValue = 0;
			if (!pMotionControl->AcscReadInt(kEnergySwitchTriggerName, iTriggerValue))
			{
				SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_COMMANDSINVALID,
					QObject::tr("[ERROR_PROCESS_COMMANDSINVALID]EnergySwitch trigger variable is unavailable.")
					.toUtf8().data());
				return false;
			}
			break;
		}

		case ItemType::Cutting:
		{
			if (m_eRunMode == RunMode::ProcessTest)		break;

			m_pService->SetCuttingDevice("NormalCutting");

			// 索引检测
			QString qstrIndex = vec_RunItemList[i].maps.at("CompensationIndex");
			if (!qstrIndex.isEmpty() && !m_pService->GetCompDevice()->IsCompIndex(qstrIndex))
			{
				SHOW_PROCESS_ERROR(ErrorCode::ERROR_CUTTING_GETCOMPENSATIONFAILED, QObject::tr("%1 Invalid compensation index \"%2\"")
					.arg("[ERROR_CUTTING_GETCOMPENSATIONFAILED]").arg(qstrIndex).toUtf8().data());
				return false;
			}

			// 图纸为空检测
			if (!m_pService->GetCuttingDevice()->IsEntityList())
			{
				SHOW_PROCESS_ERROR(ErrorCode::ERROR_CUTTING_LISTEMPTY);
				return false;
			}

			// 框选切割忽略切割顺序
			if (m_pService->GetCuttingDevice()->IsSelectionCutting())
				break;

			int iStart = -1, iEnd = -1;
			int iPolyNum = m_pService->GetCuttingDevice()->SequenceListSize();
			if (!iPolyNum)
			{
				SHOW_PROCESS_INFO(InfoCode::INFO_PROCESS_SEQUENCEZERO);
				break;
			}

			if (vec_RunItemList[i].maps.at("StartNumber").size())
			{
				QString qstrStartNumber = vec_RunItemList[i].maps.at("StartNumber");
				iStart = qstrStartNumber.toInt();
				if (iStart <= 0)
				{
					SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_CUTOUTOFLIMIT, QObject::tr("%1\ StartNumber \"%2\" less than or equal to \"%3\" !")
						.arg("[ERROR_PROCESS_CUTOUTOFLIMIT]").arg(qstrStartNumber).arg(0).toUtf8().data());
					return false;
				}
				if (iStart > iPolyNum)
				{
					SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_CUTOUTOFLIMIT, QObject::tr("%1\ StartNumber \"%2\" out of polyline size \"%3\" !")
						.arg("[ERROR_PROCESS_CUTOUTOFLIMIT]").arg(qstrStartNumber).arg(iPolyNum).toUtf8().data());
					return false;
				}
			}
			if (vec_RunItemList[i].maps.at("EndNumber").size())
			{
				QString qstrStartNumber = vec_RunItemList[i].maps.at("StartNumber");
				QString qstrEndNumber = vec_RunItemList[i].maps.at("EndNumber");
				iEnd = qstrEndNumber.toInt();
				if (iEnd <= 0)
				{
					SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_CUTOUTOFLIMIT, QObject::tr("%1\ EndNumber \"%2\" less than or equal to \"%3\" !")
						.arg("[ERROR_PROCESS_CUTOUTOFLIMIT]").arg(qstrEndNumber).arg(0).toUtf8().data());
					return false;
				}
				if (iEnd > iPolyNum)
				{
					SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_CUTOUTOFLIMIT, QObject::tr("%1\ EndNumber \"%2\" out of polyline size \"%3\" !")
						.arg("[ERROR_PROCESS_CUTOUTOFLIMIT]").arg(qstrEndNumber).arg(iPolyNum).toUtf8().data());
					return false;
				}
				if (iStart > iEnd && iStart != -1 && iEnd != -1)
				{
					SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_CUTOUTOFLIMIT, QObject::tr("%1\ The StartNumber \"%2\"  is greater than the EndNumber \"%3\" !")
						.arg("[ERROR_PROCESS_CUTOUTOFLIMIT]").arg(qstrStartNumber).arg(qstrEndNumber).toUtf8().data());
					return false;
				}
			}
			break;
		}

		case ItemType::OverCutting:
		{
			if (m_eRunMode == RunMode::ProcessTest)		break;

			m_pService->SetCuttingDevice("OverCutting");

			// 层名检查
			if (vec_RunItemList[i].maps.at("Layer").size())
			{
				QString qstrDivision = vec_RunItemList[i].maps.at("Layer"); //层名拷贝
				RS_LayerList* layerList = QC_ApplicationWindow::getAppWindow()->getDocument()->getLayerList();
				if (!layerList->find(qstrDivision))
				{
					SHOW_PROCESS_ERROR(ErrorCode::ERROR_OVERCUTTING_DIVISIONLAYERERR);
					return false;
				}
			}
			else
			{
				SHOW_PROCESS_ERROR(ErrorCode::ERROR_OVERCUTTING_NODIVISIONLAYER);
				return false;
			}
			break;
		}

		case ItemType::AutoFocus:
		{
			if (m_eRunMode == RunMode::ProcessTest)		break;

			m_pService->SetCuttingDevice("FocusCutting");

			// 层名检查
			RS_LayerList* layerList = QC_ApplicationWindow::getAppWindow()->getDocument()->getLayerList();
			if ((!layerList->find("Focus|")) && (!layerList->find("Focus-")))
			{
				SHOW_PROCESS_ERROR(ErrorCode::ERROR_AUTOFOCUS_NOFOCUSLAYER);
				return false;
			}

			m_pService->GetCuttingDevice()->SetEntitysLeftLimitPos();
		}

		default:
			break;
		}
	}
	return true;
}

void ProcessModule::ProcessThread(const vector<Item>& ItemList)
{
	// 监控线程启用
	m_MonitorThread.detach();
	m_MonitorThread = boost::thread(boost::bind(&ProcessModule::MonitorThread, this));

	// 湿切
	bool bWater = false;
	SETTINGS->GetKeyValue("bWater", bWater, SettingSection::Water, "Water");
	double dWaterDelay = 0;
	SETTINGS->GetKeyValue("fWaterDelay", dWaterDelay, SettingSection::Water, "Water");
	if (bWater && m_eRunMode == RunMode::SignMode)
	{
		boost::this_thread::sleep_for(boost::chrono::milliseconds((int)dWaterDelay));
		m_pService->GetMotionControl()->DigitalOutputSet(DigitalOUT::Water, 1);
	}

	// 回水泵线程启用
	bool bPump = false;
	SETTINGS->GetKeyValue("bPump", bPump, SettingSection::Water, "Pump");
	m_PumpThread.detach();
	if (bPump && m_eRunMode == RunMode::SignMode)
		m_PumpThread = boost::thread(boost::bind(&ProcessModule::PumpThread, this));

	// 非Stop节点结束、无Stop节点主线程出来
	if (!RunItemList(ItemList, true))
	{
		main_window->update1();
		StopProcess();
		ClearAlignmentSequenceCache();
		RestoreState();
		g_bRunning.store(false);
		g_bPause.store(false);
		return;
	}

	ClearAlignmentSequenceCache();
}

bool ProcessModule::RunItemList(const vector<Item>& ItemList, bool bMain)
{
	// 嵌套检测
	m_iNestingNumber++;
	if (m_iNestingNumber > MaxNestingNumber)
	{
		g_bStop.store(true);
		QG_CuttingInfoWidget::StopTimer();
		g_eState.store(SystemStatus::Error);
		SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_OUTOFNESTING);
		LOG_PROCESS_STOP();
		return false;
	}

	for (int i = 0; i < ItemList.size(); i++)
	{
		m_pTreeView->GetItem(ItemList[i].iParentIndex, ItemList[i].iChildrenIndex)->SetState(ItemState::Run);
		emit SignalUpdateTreeView();
		g_bFinish.store(false);
		if (StateListen(ItemList[i]))	break;

		switch (ItemList[i].Type)
		{
		case ItemType::Start:
		{
			if (m_bProcessTest) { TestProcess(); break; }

			LOG_PROCESS_INFO(QObject::tr("[Start]").toUtf8().data());
			break;
		}

		case ItemType::Stop:
		{
			if (m_bProcessTest) { TestProcess(); }

			g_bStop.store(true);
			LOG_PROCESS_INFO(QObject::tr("[Stop]").toUtf8().data());
			main_window->update1();
			StopProcess();
			RestoreState();
			g_bRunning.store(false);
			return true;
		}

		case ItemType::If:
		{
			if (m_bProcessTest) { TestProcess(); break; }

			int iIOState = 0, iWant = 2, iType = ItemList[i].maps.at("Type").toInt();
			bool bWait = false;

			// 单group确定期望参数
			if (ItemList[i].maps.at("On").size() && !ItemList[i].maps.at("Off").size())
			{
				LOG_PROCESS_INFO(QObject::tr("[If]Wait %1 = 1, Run %2").arg(QObject::tr(ItemList[i].maps.at("IO").toUtf8().data()))
					.arg(ItemList[i].maps.at("On")).toUtf8().data());
				iWant = 1;
				bWait = true;
			}
			else if (!ItemList[i].maps.at("On").size() && ItemList[i].maps.at("Off").size())
			{
				LOG_PROCESS_INFO(QObject::tr("[If]Wait %1 = 0, Run %2").arg(QObject::tr(ItemList[i].maps.at("IO").toUtf8().data()))
					.arg(ItemList[i].maps.at("Off")).toUtf8().data());
				iWant = 0;
				bWait = true;
			}

			int  iTimeOut = 1800; // 默认三分钟超时
			bool bTimeOut = true; // 启用超时
			if (DT::getCustomerID() == "JAPHL")
				bTimeOut = false;
			do
			{
				if (StateListen(ItemList[i]))
					return false;
				boost::this_thread::sleep_for(boost::chrono::milliseconds(100));

				if (iType)
					m_pService->GetMotionControl()->DigitalOutputGet(ItemList[i].maps.at("IO"), iIOState);
				else
					m_pService->GetMotionControl()->DigitalInputGet(ItemList[i].maps.at("IO"), iIOState);

				if (bWait)
				{
					if (iIOState == iWant)
						break;
				}
				
				// 内置超时处理
				iTimeOut--;
				if (bTimeOut && !iTimeOut)
				{
					SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_IFTIMEOUT);
					g_bStop.store(true);
					return false;
				}
			} while (bWait);

			if (iIOState)
			{
				LOG_PROCESS_INFO(QObject::tr("[If]%1 = 1, Run %2").arg(QObject::tr(ItemList[i].maps.at("IO").toUtf8().data()))
					.arg(ItemList[i].maps.at("On")).toUtf8().data());
				//RunItemList(map_Group[ItemList[i].maps.at("On")]);
				if (!RunItemList(map_Group[ItemList[i].maps.at("On")]))
					return false;
			}
			else
			{
				LOG_PROCESS_INFO(QObject::tr("[If]%1 = 0, Run %2").arg(QObject::tr(ItemList[i].maps.at("IO").toUtf8().data()))
					.arg(ItemList[i].maps.at("Off")).toUtf8().data());
				//RunItemList(map_Group[ItemList[i].maps.at("Off")]);
				if (!RunItemList(map_Group[ItemList[i].maps.at("Off")]))
					return false;
			}
			break;
		}

		case ItemType::Loop:
		{
			int iLoopTotal = ItemList[i].maps.at("Number").toInt();
			int iLoopParent = ItemList[i].iParentIndex;
			vector<Item> vec_item;
			// 构建 Loop 内节点的容器
			for (i++; i < ItemList.size() && ItemList[i].iParentIndex == iLoopParent; i++)
			{
				vec_item.push_back(ItemList[i]);
			}
			i--;	// 抵消第一次++操作

			if (!vec_item.size())
			{
				LOG_PROCESS_INFO(QObject::tr("[Loop]Total: %1\t No child Item").arg(iLoopTotal).toUtf8().data());
				break;
			}

			if (!iLoopTotal)
			{
				LOG_PROCESS_INFO(QObject::tr("[Loop]Total: 0").toUtf8().data());
				break;
			}

			// Loop 次数的循环
			int iLoopNow = 1;
			int iLoopNumber = iLoopTotal;
			while (iLoopNumber)
			{
				LOG_PROCESS_INFO(QObject::tr("[Loop]Total: %1\t Now: %2").arg(iLoopTotal).arg(iLoopNow).toUtf8().data());
				if (!RunItemList(vec_item))
					return false;
				
				iLoopNow++;
				iLoopNumber--;
			}
			vec_item.clear();

			m_pTreeView->GetItem(ItemList[iLoopParent].iParentIndex, ItemList[iLoopParent].iChildrenIndex)->SetState(ItemState::Unrun);
			emit SignalUpdateTreeView();
			break;
		}

		case ItemType::Wait:
		{
			if (m_bProcessTest)
				m_ItemThread = boost::thread(boost::bind(&ProcessModule::TestProcess, this));
			else
				m_ItemThread = boost::thread(boost::bind(&ProcessModule::ItemWait, this, ItemList[i].maps));

			while (!g_bFinish.load())
			{
				if (StateListen(ItemList[i]))
					return false;
				boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
			}
			break;
		}

		case ItemType::RunGroup:
		{
			string strName = ItemList[i].maps.at("Name").toUtf8().data();
			if (ItemList[i].maps.at("Thread").toInt())
			{
				LOG_PROCESS_INFO(QObject::tr("[RunGroup]Run %1 in background").arg(strName.c_str()).toUtf8().data());
				vector<Item> Items = map_Group[ItemList[i].maps.at("Name")];
				pair<QString, QString> p = { QString{"BBBBB"}, QString{"1"} };
				for (auto& pair : Items)
				{
					pair.maps.insert(p);
				}
				if (g_bGroupRuning.load())
				{
					SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_GROUPRUNNING);
					return false;
				}
				g_bGroupRuning.store(true);
				m_GroupThread = boost::thread(boost::bind(&ProcessModule::RunGroupBackground, this, Items));
			}
			else
			{
				LOG_PROCESS_INFO(QObject::tr("[RunGroup]Run %1").arg(strName.c_str()).toUtf8().data());
				//RunItemList(map_Group[ItemList[i].maps.at("Name")]);
				if (!RunItemList(map_Group[ItemList[i].maps.at("Name")]))
					return false;
			}
			break;
		}

		case ItemType::RunGroupCheck:
		{
			if (m_bProcessTest)
				m_ItemThread = boost::thread(boost::bind(&ProcessModule::TestProcess, this));
			else
				m_ItemThread = boost::thread(boost::bind(&ProcessModule::ItemRunGroupCheck, this, ItemList[i].maps));

			while (!g_bFinish.load())
			{
				if (StateListen(ItemList[i]))
					return false;
				boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
			}
			break;
		}

		case ItemType::Calculation:
		{
			if (m_bProcessTest) { TestProcess(); break; }

			ItemCalculation(ItemList[i].maps);

			while (!g_bFinish.load())
			{
				if (StateListen(ItemList[i]))
					return false;
				boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
			}
			break;
		}

		case ItemType::Compare:
		{
			if (m_bProcessTest) { TestProcess(); break; }

			CompValue cResult;
			map<QString, CompValue>	mapComps = m_pService->GetCompDevice()->GetComps();
			QString qstrExpr = ItemList[i].maps.at("Formula");
			LOG_PROCESS_INFO(QObject::tr("[Compare] %1").arg(qstrExpr).toUtf8().data());
			QString qstrResult;
			ErrorCode eCode = Expression::Compare(qstrExpr, mapComps, qstrResult);

			if (eCode != ErrorCode::ERROR_NONE)
			{
				SHOW_PROCESS_ERROR(eCode);
				g_bStop.store(true);
				return false;
			}

			if (qstrResult == "T")
			{
				LOG_PROCESS_INFO(QObject::tr("[Compare]True, Run %1").arg(ItemList[i].maps.at("True")).toUtf8().data());
				if (!RunItemList(map_Group[ItemList[i].maps.at("True")]))
					return false;
			}
			else
			{
				LOG_PROCESS_INFO(QObject::tr("[Compare]False, Run %1").arg(ItemList[i].maps.at("False")).toUtf8().data());
				if (!RunItemList(map_Group[ItemList[i].maps.at("False")]))
					return false;
			}
			break;
		}

		case ItemType::Axis:
		{
			if (m_bProcessTest)
				m_ItemThread = boost::thread(boost::bind(&ProcessModule::TestProcess, this));
			else
				m_ItemThread = boost::thread(boost::bind(&ProcessModule::ItemAxis, this, ItemList[i].maps));

			while (!g_bFinish.load())
			{
				if (StateListen(ItemList[i]))
					return false;
				boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
			}
			break;
		}

		case ItemType::AxesMove:
		{
			if (m_bProcessTest)
				m_ItemThread = boost::thread(boost::bind(&ProcessModule::TestProcess, this));
			else
				m_ItemThread = boost::thread(boost::bind(&ProcessModule::ItemAxesMove, this, ItemList[i].maps));

			while (!g_bFinish.load())
			{
				if (StateListen(ItemList[i]))
					return false;
				boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
			}
			break;
		}

		case ItemType::IO:
		{
			if (m_bProcessTest) { TestProcess(); break; }

			ItemIO(ItemList[i].maps);

			while (!g_bFinish.load())
			{
				if (StateListen(ItemList[i]))
					return false;
				boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
			}
			break;
		}

		case ItemType::Camera:
		{
			if (m_bProcessTest) { TestProcess(); break; }

			m_ItemThread = boost::thread(boost::bind(&ProcessModule::ItemCamera, this, ItemList[i].maps));

			while (!g_bFinish.load())
			{
				if (StateListen(ItemList[i]))
					return false;
				boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
			}

			break;
		}

		case ItemType::MarkAcquire:
		{
			if (m_bProcessTest)
				m_ItemThread = boost::thread(boost::bind(&ProcessModule::TestProcess, this));
			else
				m_ItemThread = boost::thread(boost::bind(&ProcessModule::ItemMarkAcquire, this, ItemList[i].maps));

			while (!g_bFinish.load())
			{
				if (StateListen(ItemList[i]))
					return false;
				boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
			}
			break;
		}

		case ItemType::Alignment:
		{
			if (m_bProcessTest)
				m_ItemThread = boost::thread(boost::bind(&ProcessModule::TestProcess, this));
			else
				m_ItemThread = boost::thread(boost::bind(&ProcessModule::ItemAlignment, this, ItemList[i].maps));

			while (!g_bFinish.load())
			{
				if (StateListen(ItemList[i]))
					return false;
				boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
			}
			break;
		}

		case ItemType::Measurement:
		{
			if (m_bProcessTest)
				m_ItemThread = boost::thread(boost::bind(&ProcessModule::TestProcess, this));
			else
				m_ItemThread = boost::thread(boost::bind(&ProcessModule::ItemMeasurement, this, ItemList[i].maps));

			while (!g_bFinish.load())
			{
				if (StateListen(ItemList[i]))
					return false;
				boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
			}
			break;
		}

		case ItemType::Commands:
		{
			if (m_bProcessTest)
				m_ItemThread = boost::thread(boost::bind(&ProcessModule::TestProcess, this));
			else
				m_ItemThread = boost::thread(boost::bind(&ProcessModule::ItemCommands, this, ItemList[i].maps));

			while (!g_bFinish.load())
			{
				if (StateListen(ItemList[i]))
					return false;
				boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
			}
			break;
		}

		case ItemType::Feeding:
		{
			if (m_bProcessTest)
				m_ItemThread = boost::thread(boost::bind(&ProcessModule::TestProcess, this));
			else
				m_ItemThread = boost::thread(boost::bind(&ProcessModule::ItemFeeding, this, ItemList[i].maps));

			while (!g_bFinish.load())
			{
				if (StateListen(ItemList[i]))
					return false;
				boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
			}
			break;
		}

		case ItemType::Cutting:
		{
			if (m_bProcessTest)
				m_ItemThread = boost::thread(boost::bind(&ProcessModule::TestProcess, this));
			else
				m_ItemThread = boost::thread(boost::bind(&ProcessModule::ItemCutting, this, ItemList[i].maps));

			while (!g_bFinish.load())
			{
				if (StateListen(ItemList[i]))
					return false;
				boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
			}
			break;
		}

		case ItemType::OverCutting:
		{
			if (m_bProcessTest)
				m_ItemThread = boost::thread(boost::bind(&ProcessModule::TestProcess, this));
			else
				m_ItemThread = boost::thread(boost::bind(&ProcessModule::ItemOverCutting, this, ItemList[i].maps));

			while (!g_bFinish.load())
			{
				if (StateListen(ItemList[i]))
					return false;
				boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
			}
			break;
		}

		case ItemType::EnergySwitch:
		{
			bool bEnergySwitchUse = DT::getCustomerID() == "MaiTong"
				|| int(DT::getPermission()) > (int)PermissionLevel::Factory;
			if (!bEnergySwitchUse)
			{
				g_bFinish.store(true);
				break;
			}

			if (m_bProcessTest)
				m_ItemThread = boost::thread(boost::bind(&ProcessModule::TestProcess, this));
			else
				m_ItemThread = boost::thread(boost::bind(&ProcessModule::ItemEnergySwitch, this, ItemList[i].maps));

			while (!g_bFinish.load())
			{
				if (StateListen(ItemList[i]))
					return false;
				boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
			}
			if (g_bGroupError.load() || g_bStop.load())	return false;
			break;
		}

		case ItemType::AutoFocus:
		{
			if (m_bProcessTest)
				m_ItemThread = boost::thread(boost::bind(&ProcessModule::TestProcess, this));
			else
				m_ItemThread = boost::thread(boost::bind(&ProcessModule::ItemAutoFocus, this, ItemList[i].maps));

			while (!g_bFinish.load())
			{
				if (StateListen(ItemList[i]))
					return false;
				boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
			}
			return false;	// 切割完直接结束
		}

		default:
		{
			if (m_bProcessTest) { TestProcess(); break; }
			break;
		}
		}
		if (StateListen(ItemList[i]))	break;

		m_pTreeView->GetItem(ItemList[i].iParentIndex, ItemList[i].iChildrenIndex)->SetState(ItemState::Unrun);
		emit SignalUpdateTreeView();
	}

	if (bMain)
		return false;

	m_iNestingNumber--;
	return true;
}

bool ProcessModule::RunGroupBackground(const vector<Item>& ItemList)
{
	for (int i = 0; i < ItemList.size(); i++)
	{
		m_pTreeView->GetItem(ItemList[i].iParentIndex, ItemList[i].iChildrenIndex)->SetState(ItemState::Run);
		emit SignalUpdateTreeView();
		if (StateListen(ItemList[i]))	break;

		switch (ItemList[i].Type)
		{
		case ItemType::Stop:
		{
			if (m_bProcessTest) { TestProcess(); }

			LOG_PROCESS_INFO(QObject::tr("[B][Stop]").toUtf8().data());
			m_pTreeView->GetItem(ItemList[i].iParentIndex, ItemList[i].iChildrenIndex)->SetState(ItemState::Unrun);
			emit SignalUpdateTreeView();
			g_bGroupRuning.store(false);
			return true;
		}

		case ItemType::If:
		{
			if (m_bProcessTest) { TestProcess(); break; }

			int iIOState = 0, iWant = 2, iType = ItemList[i].maps.at("Type").toInt();
			bool bWait = false;

			// 单group确定期望参数
			if (ItemList[i].maps.at("On").size() && !ItemList[i].maps.at("Off").size())
			{
				LOG_PROCESS_INFO(QObject::tr("[B][If]Wait %1 = 1, Run %2").arg(QObject::tr(ItemList[i].maps.at("IO").toUtf8().data()))
					.arg(ItemList[i].maps.at("On")).toUtf8().data());
				iWant = 1;
				bWait = true;
			}
			else if (!ItemList[i].maps.at("On").size() && ItemList[i].maps.at("Off").size())
			{
				LOG_PROCESS_INFO(QObject::tr("[B][If]Wait %1 = 0, Run %2").arg(QObject::tr(ItemList[i].maps.at("IO").toUtf8().data()))
					.arg(ItemList[i].maps.at("Off")).toUtf8().data());
				iWant = 0;
				bWait = true;
			}

			int iTimeOut = 1800; // 默认三分钟超时
			bool bTimeOut = true; // 启用超时
			if (DT::getCustomerID() == "JAPHL")
				bTimeOut = false;
			do
			{
				if (StateListen(ItemList[i]))
					return false;
				boost::this_thread::sleep_for(boost::chrono::milliseconds(100));

				if (iType)
					m_pService->GetMotionControl()->DigitalOutputGet(ItemList[i].maps.at("IO"), iIOState);
				else
					m_pService->GetMotionControl()->DigitalInputGet(ItemList[i].maps.at("IO"), iIOState);

				if (bWait)
				{
					if (iIOState == iWant)
						break;
				}

				// 内置超时处理
				iTimeOut--;
				if (bTimeOut && !iTimeOut)
				{
					SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_IFTIMEOUT);
					g_bGroupError.store(true);
					return false;
				}
			} while (bWait);

			vector<Item> Items;
			if (iIOState)
			{
				LOG_PROCESS_INFO(QObject::tr("[B][If]%1 = 1, Run %2").arg(QObject::tr(ItemList[i].maps.at("IO").toUtf8().data()))
					.arg(ItemList[i].maps.at("On")).toUtf8().data());
				Items = map_Group[ItemList[i].maps.at("On")];
			}
			else
			{
				LOG_PROCESS_INFO(QObject::tr("[B][If]%1 = 0, Run %2").arg(QObject::tr(ItemList[i].maps.at("IO").toUtf8().data()))
					.arg(ItemList[i].maps.at("Off")).toUtf8().data());
				Items = map_Group[ItemList[i].maps.at("Off")];
			}
			pair<QString, QString> p = { QString{"BBBBB"}, QString{"1"} };
			for (auto& pair : Items)
			{
				pair.maps.insert(p);
			}
			if (!RunGroupBackground(Items))
				return false;
			break;
		}

		case ItemType::Wait:
		{
			if (m_bProcessTest) { TestProcess(); break; }

			ItemWait(ItemList[i].maps);
			
			if (g_bGroupError.load())	return false;
			break;
		}

		case ItemType::RunGroup:
		{
			string strName = ItemList[i].maps.at("Name").toUtf8().data();
			LOG_PROCESS_INFO(QObject::tr("[B][RunGroup]Run %1").arg(strName.c_str()).toUtf8().data());
			vector<Item> Items = map_Group[ItemList[i].maps.at("Name")];
			pair<QString, QString> p = { QString{"BBBBB"}, QString{"1"} };
			for (auto& pair : Items)
			{
				pair.maps.insert(p);
			}
			if (!RunGroupBackground(Items))
				return false;
			break;
		}

		case ItemType::IO:
		{
			if (m_bProcessTest) { TestProcess(); break; }

			ItemIO(ItemList[i].maps);

			if (g_bGroupError.load())	return false;
			break;
		}

		case ItemType::EnergySwitch:
		{
			bool bEnergySwitchUse = DT::getCustomerID() == "MaiTong"
				|| int(DT::getPermission()) > (int)PermissionLevel::Factory;
			if (!bEnergySwitchUse)
				break;

			if (m_bProcessTest) { TestProcess(); break; }

			ItemEnergySwitch(ItemList[i].maps);

			if (g_bGroupError.load() || g_bStop.load())	return false;
			break;
		}

		// 支线程动轴不符合设计安全，限定客户使用
		case ItemType::Axis:
		{
			bool bUse = DT::getCustomerID() == "JAPHL";
			if (!bUse)	break;

			if (m_bProcessTest) { TestProcess(); break; }
			
			ItemAxis(ItemList[i].maps);

			if (g_bGroupError.load())	return false;
			break;
		}

		// 支线程使用测量和计算的话，则最好将调用其他结果的测量和计算都放在支线程
		case ItemType::Measurement:
		{
			if (m_bProcessTest) { TestProcess(); break; }

			ItemMeasurement(ItemList[i].maps);

			if (g_bGroupError.load())	return false;
			break;
		}

		case ItemType::MarkAcquire:
		{
			if (m_bProcessTest) { TestProcess(); break; }

			ItemMarkAcquire(ItemList[i].maps);

			if (g_bGroupError.load()) return false;
			break;
		}

		case ItemType::Alignment:
		{
			if (m_bProcessTest) { TestProcess(); break; }

			ItemAlignment(ItemList[i].maps);

			if (g_bGroupError.load()) return false;
			break;
		}

		case ItemType::Calculation:
		{
			if (m_bProcessTest) { TestProcess(); break; }

			ItemCalculation(ItemList[i].maps);

			if (g_bGroupError.load())	return false;
			break;
		}

		case ItemType::Compare:
		{
			if (m_bProcessTest) { TestProcess(); break; }

			CompValue cResult;
			map<QString, CompValue>	mapComps = m_pService->GetCompDevice()->GetComps();
			QString qstrExpr = ItemList[i].maps.at("Formula");
			LOG_PROCESS_INFO(QObject::tr("[B][Compare] %1").arg(qstrExpr).toUtf8().data());
			QString qstrResult;
			ErrorCode eCode = Expression::Compare(qstrExpr, mapComps, qstrResult);

			if (eCode != ErrorCode::ERROR_NONE)
			{
				SHOW_PROCESS_ERROR(eCode);
				g_bGroupError.store(true);
				return false;
			}

			vector<Item> Items;
			if (qstrResult == "T")
			{
				LOG_PROCESS_INFO(QObject::tr("[B][Compare]True, Run %1").arg(ItemList[i].maps.at("True")).toUtf8().data());
				Items = map_Group[ItemList[i].maps.at("True")];
			}
			else
			{
				LOG_PROCESS_INFO(QObject::tr("[B][Compare]False, Run %1").arg(ItemList[i].maps.at("False")).toUtf8().data());
				Items = map_Group[ItemList[i].maps.at("False")];
			}
			pair<QString, QString> p = { QString{"BBBBB"}, QString{"1"} };
			for (auto& pair : Items)
			{
				pair.maps.insert(p);
			}
			if (!RunGroupBackground(Items))
				return false;

			break;
		}

		default:
		{
			if (m_bProcessTest) { TestProcess(); break; }
			break;
		}
		}
		if (StateListen(ItemList[i]))	break;

		m_pTreeView->GetItem(ItemList[i].iParentIndex, ItemList[i].iChildrenIndex)->SetState(ItemState::Unrun);
		emit SignalUpdateTreeView();
	}

	g_bGroupRuning.store(false);
	return true;
}

bool ProcessModule::StateListen(const Item& item)
{
	// 停止
	if (g_bStop.load())
	{
		if (item.maps.count("BBBBB")) return true;

		m_pTreeView->GetItem(item.iParentIndex, item.iChildrenIndex)->SetState(ItemState::Stop);
		emit SignalUpdateTreeView();

		// 等待节点完成，不能删除，确保子线程退出！并作支线程区分
		m_GroupThread.join();
		m_ItemThread.join();

		StopProcess();
		return true;
	}
	// 暂停
	else if (g_bPause.load())
	{
		g_eState.store(SystemStatus::Pausing);
		main_window->update1();
		m_pTreeView->GetItem(item.iParentIndex, item.iChildrenIndex)->SetState(ItemState::Pause);
		emit SignalUpdateTreeView();

		// 等待节点暂停完成
		while (g_bPausing.load())
		{
			if (!g_bPause.load())	// 暂停取消
				return false;

			if (g_bStop.load())		// 暂停中进停止
			{
				if (item.maps.count("BBBBB")) return true;

				m_pTreeView->GetItem(item.iParentIndex, item.iChildrenIndex)->SetState(ItemState::Stop);
				emit SignalUpdateTreeView();
				m_GroupThread.join();
				m_ItemThread.join();
				StopProcess();
				return true;
			}

			boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
			continue;
		}

		PauseProcess();

		// 继续/停止监听
		while (g_bPause.load())
		{
			if (g_bStop.load())
			{
				m_pTreeView->GetItem(item.iParentIndex, item.iChildrenIndex)->SetState(ItemState::Stop);
				emit SignalUpdateTreeView();

				StopProcess();
				return true;
			}
			else
				boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
			emit SignalUpdateTreeView();
		}

		// 重启节点运行
		if (item.Type == ItemType::Cutting || item.Type == ItemType::AutoFocus || item.Type == ItemType::OverCutting)
			g_eState.store(SystemStatus::LaserProcessing);
		else
			g_eState.store(SystemStatus::Processing);
		main_window->update1();
		m_pTreeView->GetItem(item.iParentIndex, item.iChildrenIndex)->SetState(ItemState::Run);
		emit SignalUpdateTreeView();
		return false;
	}
	else
		boost::this_thread::sleep_for(boost::chrono::milliseconds(10));

	emit SignalUpdateTreeView();
	return false;
}

void ProcessModule::MonitorThread()
{
	table tabMonitor = SETTINGS->GetTable(SettingSection::Monitor);
	bool bInterLock				= tabMonitor["Cutting"]["bInterLock"].as_boolean();				// 互锁：读io
	bool bSafetyLightCurtain	= tabMonitor["Cutting"]["bSafetyLightCurtain"].as_boolean();	// 安全光栅：读io
	bool bPressureMonitor		= tabMonitor["Gas"]["bPressureMonitor"].as_boolean();			// 气压：读io
	bool bWaterLeakageMonitor	= tabMonitor["Water"]["bWaterLeakageMonitor"].as_boolean();		// 漏水：读io
	bool bWaterTankMonitor		= tabMonitor["Water"]["bWaterTankMonitor"].as_boolean();		// 水箱：读io
	bool bWaterPressureMonitor	= tabMonitor["Water"]["bWaterPressureMonitor"].as_boolean();	// 水压：读模拟量
	bool bWaterLevelMonitor		= tabMonitor["Water"]["bWaterLevelMonitor"].as_boolean();		// 液位：读模拟量
	float fWaterPressureLimit		= tabMonitor["Water"]["fWaterPressureLimit"].as_floating();
	float fWaterLevelLimit			= tabMonitor["Water"]["fWaterLevelLimit"].as_floating();
	int iWaterPressureConversions	= tabMonitor["WaterSetting"]["iWaterPressureConversions"].as_integer();
	int iWaterLevelConversions		= tabMonitor["WaterSetting"]["iWaterLevelConversions"].as_integer();

	LOG_PROCESS_INFO(QObject::tr("Monitor ON :%1%2%3%4%5%6%7")
		.arg(bInterLock				? QObject::tr("Inter Lock\t") : 0)
		.arg(bSafetyLightCurtain	? QObject::tr("Safety Light Curtain\t") : 0)
		.arg(bPressureMonitor		? QObject::tr("Pressure Monitor\t") : 0)
		.arg(bWaterLeakageMonitor	? QObject::tr("Water Leakage Monitor\t") : 0)
		.arg(bWaterTankMonitor		? QObject::tr("Water Tank Monitor\t") : 0)
		.arg(bWaterPressureMonitor	? QObject::tr("Water Pressure Monitor\t") : 0)
		.arg(bWaterLevelMonitor		? QObject::tr("Water Level Monitor\t") : 0)
		.toUtf8().data());

	int		iInterLock, iSafetyLightCurtain, iPressureMonitor, iWaterLeakageMonitor, iWaterTankMonitor, iBlow;
	double	dWaterPressureMonitor, dWaterLevelMonitor;


	int iTimes = 0, iOUT = 0, iPressureTimes = 0;
	bool bJAPHL = DT::getCustomerID() == "JAPHL" ? true : false;
	bool bTOYO = DT::getCustomerID() == "TOYO" ? true : false;
	int iIN1;

	while (true)
	{
		if (!g_bPause.load())
		{
			if (!m_pService->GetMotionControl()->IsEnabled())
			{
				m_pService->GetMotionControl()->Enable();
				if (!m_pService->GetMotionControl()->IsEnabled())
				{
					g_bStop.store(true);
					g_eState.store(SystemStatus::Error);
					SHOW_PROCESS_ERROR(ErrorCode::ERROR_MONITOR_AXISDISABLED);
				}
			}

			if (bInterLock)
			{
				m_pService->GetMotionControl()->DigitalInputGet(DigitalIN::InterLock, iInterLock);
				if (iInterLock)
				{
					g_bPause.store(true);
					SHOW_PROCESS_WARN(WarnCode::WARN_MONITOR_INTERLOCK);
				}
			}

			if (bSafetyLightCurtain)
			{
				m_pService->GetMotionControl()->DigitalInputGet(DigitalIN::SafetyLightCurtain, iSafetyLightCurtain);
				if (iSafetyLightCurtain)
				{
					g_bPause.store(true);
					SHOW_PROCESS_WARN(WarnCode::WARN_MONITOR_SAFETYLIGHTCURTAIN);
				}
			}

			if (bPressureMonitor)
			{
				m_pService->GetMotionControl()->DigitalOutputGet(DigitalOUT::Blow, iBlow);
				m_pService->GetMotionControl()->DigitalInputGet(DigitalIN::PressureMonitor, iPressureMonitor);
				if (iBlow && iPressureMonitor)
				{
					// 实际有个气压升高的过程，连续500ms后报警则视为异常。
					iPressureTimes++;
					if (iPressureTimes > 5) {
						if (bTOYO) {
							g_bStop.store(true);
							SHOW_PROCESS_ERROR(ErrorCode::ERROR_MONITOR_PRESSURE);
						}
						else {
							g_bPause.store(true);
							SHOW_PROCESS_WARN(WarnCode::WARN_MONITOR_PRESSURE);
						}
					}
				}
				else
					iPressureTimes = 0;
			}

			if (bWaterLeakageMonitor)
			{
				m_pService->GetMotionControl()->DigitalInputGet(DigitalIN::WaterLeakageMonitor, iWaterLeakageMonitor);
				if (iWaterLeakageMonitor)
				{
					g_bStop.store(true);
					g_eState.store(SystemStatus::Error);
					SHOW_PROCESS_ERROR(ErrorCode::ERROR_MONITOR_WATERLEAKAGE);
				}
			}

			if (bWaterTankMonitor)
			{
				m_pService->GetMotionControl()->DigitalInputGet(DigitalIN::WaterTankMonitor, iWaterTankMonitor);
				if (iWaterTankMonitor)
				{
					g_bStop.store(true);
					g_eState.store(SystemStatus::Error);
					SHOW_PROCESS_ERROR(ErrorCode::ERROR_MONITOR_WATERTANK);
				}
			}

			if (bWaterPressureMonitor)
			{
				m_pService->GetMotionControl()->AnalogInputGet(AnalogIN::WaterPressure, dWaterPressureMonitor);
				if (dWaterPressureMonitor * iWaterPressureConversions < fWaterPressureLimit)
				{
					g_bPause.store(true);
					SHOW_PROCESS_WARN(WarnCode::WARN_MONITOR_WATERPRESSURE);
				}
			}

			if (bWaterLevelMonitor)
			{
				m_pService->GetMotionControl()->AnalogInputGet(AnalogIN::WaterLevel, dWaterLevelMonitor);
				if (dWaterLevelMonitor * iWaterLevelConversions < fWaterLevelLimit)
				{
					SHOW_PROCESS_INFO(InfoCode::INFO_MONITOR_WATERLEVEL);
					fWaterLevelLimit = 0;		// 让本次切割提示一次
				}
			}

			if (bJAPHL)
			{
				// 切割头报警
				m_pService->GetMotionControl()->DigitalInputGet(DigitalIN::IN1, iIN1);
				if (iIN1)
				{
					g_bStop.store(true);
					g_eState.store(SystemStatus::Error);
					SHOW_PROCESS_ERROR(ErrorCode::ERROR_MONITOR, QObject::tr("Laser cutting head malfunction !").toUtf8().data());
				}

				// 间隔0.5s心跳输出
				if (++iTimes % 5 == 0)
				{
					iOUT = iOUT ? 0 : 1;
					iTimes = 0;
					m_pService->GetMotionControl()->DigitalOutputSet(DigitalOUT::OUT2, iOUT);
				}
			}
		}

		if (g_bPause.load())
		{
			while (g_bPause.load() && !g_bStop.load())
			{
				boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
				continue;
			}

			if (g_bStop.load())
				return;
		}

		if (g_bStop.load())			return;
		if (!g_bRunning.load())		return;

		boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
	}
}

void ProcessModule::PumpThread()
{
	double dPumpOpenTime, dPumpCloseTime;
	SETTINGS->GetKeyValue("fPumpOpenTime",  dPumpOpenTime,  SettingSection::Water, "Pump");
	SETTINGS->GetKeyValue("fPumpCloseTime", dPumpCloseTime, SettingSection::Water, "Pump");
	int iPumpOpenTimes  = dPumpOpenTime  * 1000 / 50;
	int iPumpCloseTimes = dPumpCloseTime * 1000 / 50;

	LOG_PROCESS_INFO(QObject::tr("Use return pump with open %1 s, close %2 s.")
		.arg(dPumpOpenTime).arg(dPumpCloseTime).toUtf8().data());

	int	 iTimes	= 0;
	bool bOpen	= false;
	while (true)
	{
		if (!bOpen && iTimes == iPumpOpenTimes)
		{
			m_pService->GetMotionControl()->DigitalOutputSet(DigitalOUT::Pump, 1);
			iTimes	= 0;
			bOpen	= true;
		}

		if (bOpen && iTimes == iPumpCloseTimes)
		{
			m_pService->GetMotionControl()->DigitalOutputSet(DigitalOUT::Pump, 0);
			iTimes	= 0;
			bOpen	= false;
		}

		iTimes++;

		if (g_bPause.load())
		{
			while (g_bPause.load() && !g_bStop.load())
			{
				boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
				continue;
			}

			if (g_bStop.load())
				return;
		}

		if (g_bStop.load())			return;
		
		if (!g_bRunning.load())		return;

		boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
	}
}

void ProcessModule::RestoreState()
{
	int parentCount = m_pTreeView->Count();
	for (int i = 0; i < parentCount; i++)
	{
		int childCount = m_pTreeView->Count(i);
		for (int j = -1; j < childCount; j++)
		{
			m_pTreeView->GetItem(i, j)->SwitchState();
		}
	}
	emit SignalUpdateTreeView();
}

void ProcessModule::LoadingPosProcess()
{
	g_bRunning = true;
	table tabLoadingPos = SETTINGS->GetTable(SettingSection::LoadingPos, "LoadingPos");
	struct AxisMoveInfo { Axis axis = Axis::X; const char* b = NULL; const char* fPos = NULL; };
	AxisMoveInfo moves[] = {
		{ Axis::Z,	"bLoadingPosZIdle",		"fLoadingPosZIdle"	},
		{ Axis::Z1, "bLoadingPosZ1Idle",	"fLoadingPosZ1Idle"	},
		{ Axis::X,  "bLoadingPosX",			"fLoadingPosX"		},
		{ Axis::Y,  "bLoadingPosY",			"fLoadingPosY"		},
		{ Axis::A,	"bLoadingPosA",			"fLoadingPosA"		},
		{ Axis::A1, "bLoadingPosA1",		"fLoadingPosA1"		},
		{ Axis::X1, "bLoadingPosX1",		"fLoadingPosX1"		},
		{ Axis::Y1, "bLoadingPosY1",		"fLoadingPosY1"		},
		{ Axis::Z,  "bLoadingPosZ",			"fLoadingPosZ"		},
		{ Axis::Z1, "bLoadingPosZ1",		"fLoadingPosZ1"		}
	};

	for (const auto& m : moves)
	{
		if (!DT::IsAxisUse(m.axis))				continue;
		if (!tabLoadingPos[m.b].as_boolean())	continue;
		if (!AxisMoveToPos(m.axis, tabLoadingPos[m.fPos].as_floating(), m_strSpeedMode))
		{
			g_bRunning = false;
			return;
		}
	}

	g_bRunning = false;
	return;
}

void ProcessModule::InitProcess()
{
	LOG_SYS_INFO(QObject::tr("Start init.").toUtf8().data());
	ErrorCode eCode = ErrorCode::ERROR_NONE;

	// 运动控制器连接
	
	m_pService->SetMotionControl();
	m_pService->SetDigitalTable();
	m_pService->SetAnalogTable();
	MotionControl* m_pMotionControl = m_pService->GetMotionControl();
	
	m_pMotionControl->rebuildAxes();
	
	if (!m_pMotionControl->IsConnected())
		m_pMotionControl->Connect();

	if (!m_pMotionControl->IsConnected())
	{
		if (!m_pMotionControl->Connect())
		{
			SHOW_SYS_ERROR(ErrorCode::ERROR_MC_CONNECTIONFAILED);
			g_eState.store(SystemStatus::UnInit);
			return;
		}
	}
	LOG_SYS_INFO(QObject::tr("Motion control connected.").toUtf8().data());

	bool bBlow;
	SETTINGS->GetKeyValue("bBlow", bBlow, SettingSection::Gas, "Gas");
	if (bBlow)
		m_pService->SetGasTable();

	LOG_SYS_INFO(QObject::tr("Motion control parameter set completed.").toUtf8().data());

	g_eState.store(SystemStatus::Initializing);

	bool bHome = false;
	if (!m_pMotionControl->IsHomed())
	{
		LOG_SYS_INFO(QObject::tr("Axis not all homed.").toUtf8().data());
		bHome = true;
	}
	if (!m_pMotionControl->IsEnabled())
	{
		LOG_SYS_INFO(QObject::tr("Axis not all enabled.").toUtf8().data());
		bHome = true;
	}

	if (bHome)
	{
		if (!m_pMotionControl->Home())
		{
			SHOW_SYS_ERROR(ErrorCode::ERROR_MC_HOMEFAILED);
			g_eState.store(SystemStatus::UnInit);
			return;
		}
		LOG_SYS_INFO(QObject::tr("Motion control homed.").toUtf8().data());
	}

	m_pMotionControl->SetMotionControlTable();
	m_pMotionControl->SetPipeDiameterTable();

	m_pMotionControl->Enable();

	// 激光器连接
	m_pService->SetLaserDevice();
	LaserDevice* m_pLaserDevice = m_pService->GetLaserDevice();
	eCode = m_pLaserDevice->SetLaserTable();
	if (eCode != ErrorCode::ERROR_NONE)
	{
		SHOW_SYS_ERROR(eCode);
		g_eState.store(SystemStatus::UnInit);
		return;
	}
	m_pLaserDevice->StartLaser();
	double dEnergy = 0;
	SETTINGS->GetKeyValue("fEnergy", dEnergy, SettingSection::Laser, "Laser");
	m_pService->GetCuttingDevice()->LaserEnergySet(dEnergy);
	LOG_SYS_INFO(QObject::tr("Laser connected.").toUtf8().data());
		
	// 信号源连接
	bool bSerialPortFlag = false;
	SETTINGS->GetKeyValue("bSignal", bSerialPortFlag, SettingSection::Laser, "SignalSource");
	if (bSerialPortFlag)
	{
		if (m_pMotionControl->GetName() == "GTN")
			eCode = m_pService->GetMotionControl()->SetLaserParameterTable();
		else
			eCode = m_pService->GetSignalSource()->SetSignalSourceTable();
		if (eCode != ErrorCode::ERROR_NONE)
		{
			SHOW_SYS_ERROR(eCode);
			g_eState.store(SystemStatus::UnInit);
			return;
		}
		LOG_SYS_INFO(QObject::tr("Signal Source connected.").toUtf8().data());
	}

	// 补偿类外设连接
	m_pService->SetCompDevice(DT::getCustomerID());
	CompDevice* m_pCompDevice = m_pService->GetCompDevice();
	if (!m_pCompDevice->IsInited())
		eCode = m_pCompDevice->SetSpecialTable();

	if (eCode != ErrorCode::ERROR_NONE)
	{
		SHOW_SYS_ERROR(eCode);
		g_eState.store(SystemStatus::UnInit);
		return;
	}

	LOG_SYS_INFO(QObject::tr("System initialization completed.").toUtf8().data());
	LOG_REFRESH();

	g_bInit.store(true);
	g_eState.store(SystemStatus::Idle);
}

void ProcessModule::TestProcess()
{
	for (int i = 0; i < 10; i++)
	{
		if (g_bStop.load())
			return;

		else if (g_bPause.load())
		{
			while (g_bPause.load() && !g_bStop.load())
			{
				boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
				continue;
			}
			return;
		}

		else
			boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
	}
	g_bFinish.store(true);
}

void ProcessModule::ContinueProcess()
{
	if (DT::getCustomerID() == "JAPHL")
	{
		// OUT19 - 异常回位
		m_pService->GetMotionControl()->DigitalOutputSet(DigitalOUT::Buzzer, 0);
		m_pService->GetMotionControl()->DigitalOutputSet(DigitalOUT::OUT19, 0);
	}
}

void ProcessModule::PauseProcess()
{
	//关气、关水、关泵
	m_pService->GetMotionControl()->DigitalOutputSet(DigitalOUT::Blow, 0);

	bool bWater = false;
	SETTINGS->GetKeyValue("bWater", bWater, SettingSection::Water, "Water");
	if (bWater)
		m_pService->GetMotionControl()->DigitalOutputSet(DigitalOUT::Water, 0);

	bool bPump = false;
	SETTINGS->GetKeyValue("bPump", bPump, SettingSection::Water, "Pump");
	if (bPump)
		m_pService->GetMotionControl()->DigitalOutputSet(DigitalOUT::Pump, 0);

	QG_CuttingInfoWidget::PauseTimer();
	LOG_PROCESS_INFO(QObject::tr("PAUSED").toUtf8().data());
	g_eState.store(SystemStatus::Paused);
	main_window->update1();
}

void ProcessModule::StopProcess()
{
	//停止buffer、轴系
	//关气、关水、关泵
	if (m_pService->GetMotionControl())
	{
		m_pService->GetMotionControl()->StopHome();
		m_pService->GetMotionControl()->StopAllBuffer();
		m_pService->GetMotionControl()->StopMotion();
		m_pService->GetMotionControl()->DigitalOutputSet(DigitalOUT::Laser, 0);
		m_pService->GetMotionControl()->DigitalOutputSet(DigitalOUT::Blow,	0);
		m_pService->GetMotionControl()->GSN_SetLaserEnablePro(0);
		m_pService->GetMotionControl()->PrfTrapAxis();
		bool bWater = false;
		SETTINGS->GetKeyValue("bWater", bWater, SettingSection::Water, "Water");
		if (bWater)
			m_pService->GetMotionControl()->DigitalOutputSet(DigitalOUT::Water, 0);

		bool bPump = false;
		SETTINGS->GetKeyValue("bPump", bPump, SettingSection::Water, "Pump");
		if (bPump)
			m_pService->GetMotionControl()->DigitalOutputSet(DigitalOUT::Pump, 0);
	}

	if (DT::getCustomerID() == "HuaJin")
	{
		// OUT1~3回位
		for (int i = (int)DigitalOUT::OUT1; i < (int)DigitalOUT::OUT4; i++) {
			m_pService->GetMotionControl()->DigitalOutputSet(DigitalOUT(i), 0);
		}
	}
	
	if (DT::getCustomerID() == "JAPHL")
	{
		// OUT2~19回位
		for (int i = (int)DigitalOUT::OUT2; i < (int)DigitalOUT::OUT20; i++){
			m_pService->GetMotionControl()->DigitalOutputSet(DigitalOUT(i), 0);
		}
	}

	QG_CuttingInfoWidget::StopTimer();
	LOG_PROCESS_STOP();

	if (g_eState.load() == SystemStatus::Initializing || g_eState.load() == SystemStatus::UnInit)
		g_eState.store(SystemStatus::UnInit);
	else if (g_eState.load() != SystemStatus::Error)
		g_eState.store(SystemStatus::Idle);
	main_window->update1();
}

bool ProcessModule::WaitTime(int iWaitTime)
{
	return WaitEnergySwitchInterval(iWaitTime, 30);
}

void ProcessModule::ItemWait(const map<QString, QString>& maps)
{
	bool bB = maps.count("BBBBB");	// 支线程运行标志
	int iWait = 0;
	string strLOG = "";
	switch (maps.at("Unit").toInt())
	{
	case 0:	// ms
		iWait = maps.at("Value").toInt();
		strLOG = "Wait " + maps.at("Value").toStdString() + " ms";
		break;
	case 1: // s
		iWait = maps.at("Value").toInt() * 1000;
		strLOG = "Wait " + maps.at("Value").toStdString() + " s";
		break;
	case 2: // min
		iWait = maps.at("Value").toInt() * 60000;
		strLOG = "Wait " + maps.at("Value").toStdString() + " min";
		break;
	}
	
	LOG_PROCESS_INFO(QObject::tr("%1[Wait]%2").arg(bB ? "[B]":"").arg(strLOG.c_str()).toUtf8().data());

	if (!WaitTime(iWait))
		return;

	if (bB)		return;	// 支线程直接返回

	g_bFinish.store(true);
	return;
}

void ProcessModule::ItemEnergySwitch(const map<QString, QString>& maps)
{
	bool bB = maps.count("BBBBB");
	bool bEnergySwitchUse = DT::getCustomerID() == "MaiTong"
		|| int(DT::getPermission()) > (int)PermissionLevel::Factory;
	if (!bEnergySwitchUse)
	{
		if (!bB)
			g_bFinish.store(true);
		return;
	}

	MotionControl* pMotionControl = m_pService->GetMotionControl();
	LaserDevice* pLaserDevice = m_pService->GetLaserDevice();
	auto HandleError = [bB](ErrorCode eCode, const QString& qstrMessage) {
		SHOW_PROCESS_ERROR(eCode, qstrMessage.toUtf8().data());
		if (bB)
			g_bGroupError.store(true);
		else
			g_bStop.store(true);
	};

	if (!pMotionControl || (pMotionControl->GetName() != "ACS"
		&& pMotionControl->GetName() != "SimulatorCMHP"))
	{
		HandleError(ErrorCode::ERROR_PROCESS_COMMANDSINVALID,
			QObject::tr("[ERROR_PROCESS_COMMANDSINVALID]EnergySwitch only supports ACSCMHP or SimulatorCMHP motion control."));
		return;
	}

	if (!pLaserDevice || pLaserDevice->GetName() != "Pharos")
	{
		HandleError(ErrorCode::ERROR_PROCESS_COMMANDSINVALID,
			QObject::tr("[ERROR_PROCESS_COMMANDSINVALID]EnergySwitch only supports Pharos laser."));
		return;
	}

	LOG_PROCESS_INFO(QObject::tr("%1[EnergySwitch] Wait trigger %2")
		.arg(bB ? "[B]" : "").arg(kEnergySwitchTriggerName).toUtf8().data());

	int iTriggerValue = 0;
	while (true)
	{
		if (g_bStop.load())
			return;

		if (!pMotionControl->AcscReadInt(kEnergySwitchTriggerName, iTriggerValue))
		{
			HandleError(ErrorCode::ERROR_PROCESS_COMMANDSINVALID,
				QObject::tr("[ERROR_PROCESS_COMMANDSINVALID]Failed to read EnergySwitch trigger variable."));
			return;
		}

		if (iTriggerValue != 0)
			break;

		if (!WaitEnergySwitchInterval(kEnergySwitchTriggerPollMs, kEnergySwitchTriggerPollMs))
			return;
	}

	LOG_PROCESS_INFO(QObject::tr("%1[EnergySwitch] Triggered")
		.arg(bB ? "[B]" : "").toUtf8().data());

	for (int i = 1; i <= 9; i++)
	{
		if (!pMotionControl->AcscReadInt(kEnergySwitchTriggerName, iTriggerValue))
		{
			HandleError(ErrorCode::ERROR_PROCESS_COMMANDSINVALID,
				QObject::tr("[ERROR_PROCESS_COMMANDSINVALID]Failed to read EnergySwitch trigger variable."));
			return;
		}

		if (iTriggerValue == 0)
		{
			LOG_PROCESS_INFO(QObject::tr("%1[EnergySwitch] Trigger reset to 0, finish remaining gears")
				.arg(bB ? "[B]" : "").toUtf8().data());
			break;
		}

		int iDivider = 0;
		int iDuration = 0;
		if (!TryGetEnergySwitchIntValue(maps, "PpDivider", i, iDivider)
			|| !TryGetEnergySwitchIntValue(maps, "Duration", i, iDuration))
		{
			HandleError(ErrorCode::ERROR_PROCESS_COMMANDSINVALID,
				QObject::tr("[ERROR_PROCESS_COMMANDSINVALID]EnergySwitch parameters are incomplete."));
			return;
		}

		if (iDivider <= 0)
			continue;

		LOG_PROCESS_INFO(QObject::tr("%1[EnergySwitch] Gear %2, PpDivider %3, Duration %4 ms")
			.arg(bB ? "[B]" : "").arg(i).arg(iDivider).arg(iDuration).toUtf8().data());

		if (!pLaserDevice->SetPpDividerOnly(static_cast<double>(iDivider), LaserResponseMode::SendOnly))
		{
			HandleError(ErrorCode::ERROR_PROCESS_LASERSETFAILED,
				QObject::tr("[ERROR_PROCESS_LASERSETFAILED]EnergySwitch failed to send PpDivider%1 = %2.")
				.arg(i).arg(iDivider));
			return;
		}

		if (!WaitEnergySwitchInterval(kEnergySwitchStepDelayMs, kEnergySwitchStepDelayMs))
			return;

		if (!WaitEnergySwitchInterval(iDuration, kEnergySwitchStepDelayMs))
			return;
	}

	if (bB)
		return;

	g_bFinish.store(true);
}

void ProcessModule::ItemAxis(const map<QString, QString>& maps)
{
	bool bB = maps.count("BBBBB");	// 支线程运行标志
	string strAxis = maps.at("Axis").toStdString();
	Axis eAxis = enum_cast<Axis>(strAxis).value();


	bool bDouble;
	double dPos = maps.at("Pos").toDouble(&bDouble);
	if (!bDouble)
	{
		QString qstrPos;
		map<QString, CompValue>	mapComps = m_pService->GetCompDevice()->GetComps();
		ErrorCode eCode = Expression::GetResult(maps.at("Pos"), mapComps, qstrPos);
		if (eCode != ErrorCode::ERROR_NONE)
		{
			SHOW_PROCESS_ERROR(eCode);
			if (bB)
			{
				g_bGroupError.store(true);
				return;// 支线程直接返回
			}
			g_bStop.store(true);
			return;
		}
		dPos = qstrPos.toDouble();
	}

	double dAbsolutePos = dPos;
	bool bRelative = false;
	if (maps.at("Mode").toInt() == 1)	//0绝对位置，1相对位置，相对位置需要转绝对位置供重启节点使用
	{
		bRelative = true;
		m_pService->GetMotionControl()->GetActualPos(eAxis, dAbsolutePos);
		dAbsolutePos += dPos;
	}

	double dVel = 0;
	switch (maps.at("Speed").toInt())
	{
	case 0:		SETTINGS->GetKeyValue("fLowSpeed",		dVel, SettingSection::Axis, strAxis);	break;
	case 1:		SETTINGS->GetKeyValue("fMediumSpeed",	dVel, SettingSection::Axis, strAxis);	break;
	case 2:		SETTINGS->GetKeyValue("fHighSpeed",		dVel, SettingSection::Axis, strAxis);	break;
	}

	if (!m_pService->GetMotionControl()->IsReachPos(eAxis, bRelative, dPos))
	{
		SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_AXISOUTOFLIMIT);
		if (bB)
		{
			g_bGroupError.store(true);
			return;// 支线程直接返回
		}
		g_bStop.store(true);
		return;
	}

	// 支线程运动保护
	if (bB)
	{
		while (m_pService->GetMotionControl()->IsAxisMoving(eAxis))
		{
			boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
		}
	}
	LOG_PROCESS_INFO(QObject::tr("%1[Axis] %2 move to %3").arg(bB ? "[B]" : "").arg(strAxis.c_str()).arg(dPos).toUtf8().data());
	

	while (g_bPause.load() && !g_bStop.load())
	{
		boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
		continue;
	}

	if (g_bStop.load()) return;

	if (bRelative)
		m_pService->GetMotionControl()->MoveRelative(eAxis, dPos, dVel);
	else
		m_pService->GetMotionControl()->MoveAbsolute(eAxis, dAbsolutePos, dVel);

	do
	{
		if (g_bPause.load())
		{
			m_pService->GetMotionControl()->StopMotion(eAxis);

			while (g_bPause.load() && !g_bStop.load())
			{
				boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
				continue;
			}

			if (g_bStop.load()) return;

			// 恢复后重新开始运动
			m_pService->GetMotionControl()->MoveAbsolute(eAxis, dAbsolutePos, dVel);
		}

		if (g_bStop.load())
		{
			m_pService->GetMotionControl()->StopMotion(eAxis);
			return;
		}

		boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
	} while (m_pService->GetMotionControl()->IsAxisMoving(eAxis));

	double dActualPos;
	m_pService->GetMotionControl()->GetActualPos(eAxis, dActualPos);
	if (abs(dActualPos - dAbsolutePos) > 0.01)
	{
		SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_AXISDISABLED);
		if (bB) {
			g_bGroupError.store(true);
			return;// 支线程直接返回
		}
		g_bStop.store(true);
		return;
	}

	if (bB)		return;// 支线程直接返回
	g_bFinish.store(true);
	return;
}

void ProcessModule::ItemAxesMove(const map<QString, QString>& maps)
{
	string strSpeed;
	switch (maps.at("Speed").toInt())	{
	case 0:		strSpeed = "fLowSpeed";		break;
	case 1:		strSpeed = "fMediumSpeed";	break;
	case 2:		strSpeed = "fHighSpeed";	break;
	default:	strSpeed = "fMediumSpeed";	break;
	}

	struct AxisMoveInfo { Axis axis = Axis::X; QString sAxis = ""; bool b = false; double dPos = 0.0; };
	AxisMoveInfo moves[10];
	switch (maps.at("Mode").toInt())	{
	case 0: {	// 上料位
		table tL = SETTINGS->GetTable(SettingSection::LoadingPos, "LoadingPos");
		moves[0] = { Axis::Z,  "ZIdle",	 tL["bLoadingPosZIdle"].as_boolean(),	tL["fLoadingPosZIdle"].as_floating() };
		moves[1] = { Axis::Z1, "Z1Idle", tL["bLoadingPosZ1Idle"].as_boolean(),	tL["fLoadingPosZ1Idle"].as_floating()};
		moves[2] = { Axis::X,  "X",		 tL["bLoadingPosX"].as_boolean(),		tL["fLoadingPosX"].as_floating()	 };
		moves[3] = { Axis::Y,  "Y",		 tL["bLoadingPosY"].as_boolean(),		tL["fLoadingPosY"].as_floating()	 };
		moves[4] = { Axis::A,  "A",		 tL["bLoadingPosA"].as_boolean(),		tL["fLoadingPosA"].as_floating()	 };
		moves[5] = { Axis::A1, "A1",	 tL["bLoadingPosA1"].as_boolean(),		tL["fLoadingPosA1"].as_floating()	 };
		moves[6] = { Axis::X1, "X1",	 tL["bLoadingPosX1"].as_boolean(),		tL["fLoadingPosX1"].as_floating()	 };
		moves[7] = { Axis::Y1, "Y1",	 tL["bLoadingPosY1"].as_boolean(),		tL["fLoadingPosY1"].as_floating()	 };
		moves[8] = { Axis::Z,  "Z",		 tL["bLoadingPosZ"].as_boolean(),		tL["fLoadingPosZ"].as_floating()	 };
		moves[9] = { Axis::Z1, "Z1",	 tL["bLoadingPosZ1"].as_boolean(),		tL["fLoadingPosZ1"].as_floating()	 };
		break;	}
	
	case 1:	{	// 下料位
		table tB = SETTINGS->GetTable(SettingSection::LoadingPos, "BlankingPos");
		moves[0] = { Axis::Z,  "ZIdle",	 tB["bBlankingPosZIdle"].as_boolean(),	tB["fBlankingPosZIdle"].as_floating()	};
		moves[1] = { Axis::Z1, "Z1Idle", tB["bBlankingPosZ1Idle"].as_boolean(),	tB["fBlankingPosZ1Idle"].as_floating()	};
		moves[2] = { Axis::X,  "X",		 tB["bBlankingPosX"].as_boolean(),		tB["fBlankingPosX"].as_floating()		};
		moves[3] = { Axis::Y,  "Y",		 tB["bBlankingPosY"].as_boolean(),		tB["fBlankingPosY"].as_floating()		};
		moves[4] = { Axis::A,  "A",		 tB["bBlankingPosA"].as_boolean(),		tB["fBlankingPosA"].as_floating()		};
		moves[5] = { Axis::A1, "A1",	 tB["bBlankingPosA1"].as_boolean(),		tB["fBlankingPosA1"].as_floating()		};
		moves[6] = { Axis::X1, "X1",	 tB["bBlankingPosX1"].as_boolean(),		tB["fBlankingPosX1"].as_floating()		};
		moves[7] = { Axis::Y1, "Y1",	 tB["bBlankingPosY1"].as_boolean(),		tB["fBlankingPosY1"].as_floating()		};
		moves[8] = { Axis::Z,  "Z",		 tB["bBlankingPosZ"].as_boolean(),		tB["fBlankingPosZ"].as_floating()		};
		moves[9] = { Axis::Z1, "Z1",	 tB["bBlankingPosZ1"].as_boolean(),		tB["fBlankingPosZ1"].as_floating()		};
		break;	}

	case 2: {	// 手动模式
		moves[0] = { Axis::Z,  "ZIdle",	 (bool)maps.at("ZIdle").toInt(),	maps.at("ZIdlePos").toDouble() 	};
		moves[1] = { Axis::Z1, "Z1Idle", (bool)maps.at("Z1Idle").toInt(),	maps.at("Z1IdlePos").toDouble() };
		moves[2] = { Axis::X,  "X",		 (bool)maps.at("X").toInt(),		maps.at("XPos").toDouble() 		};
		moves[3] = { Axis::Y,  "Y",		 (bool)maps.at("Y").toInt(),		maps.at("YPos").toDouble() 		};
		moves[4] = { Axis::A,  "A",		 (bool)maps.at("A").toInt(),		maps.at("APos").toDouble() 		};
		moves[5] = { Axis::A1, "A1",	 (bool)maps.at("A1").toInt(),		maps.at("A1Pos").toDouble() 	};
		moves[6] = { Axis::X1, "X1",	 (bool)maps.at("X1").toInt(),		maps.at("X1Pos").toDouble() 	};
		moves[7] = { Axis::Y1, "Y1",	 (bool)maps.at("Y1").toInt(),		maps.at("Y1Pos").toDouble() 	};
		moves[8] = { Axis::Z,  "Z",		 (bool)maps.at("Z").toInt(),		maps.at("ZPos").toDouble() 		};
		moves[9] = { Axis::Z1, "Z1",	 (bool)maps.at("Z1").toInt(),		maps.at("Z1Pos").toDouble() 	};
		break;	}

	default:	break;	}

	for (const AxisMoveInfo& m : moves)
	{
		if (!m.b)					continue;
		if (!DT::IsAxisUse(m.axis))	continue;
		LOG_PROCESS_INFO(QObject::tr("[Loading Pos] Axis %1 move to %2").arg(m.sAxis).arg(m.dPos).toUtf8().data());
		if (!AxisMoveToPos(m.axis, m.dPos, m_strSpeedMode))
			return;
	}
	g_bFinish.store(true);
	return;
}

void ProcessModule::ItemIO(const map<QString, QString>& maps)
{
	bool bB = maps.count("BBBBB");	// 支线程运行标志

	LOG_PROCESS_INFO(QObject::tr("%1[IO]Set %2 = %3").arg(bB ? "[B]" : "")
		.arg(QObject::tr(maps.at("IO").toUtf8().data())).arg(maps.at("Value")).toUtf8().data());
	
	if (maps.at("Type").toInt())
	{
		if (maps.at("IO") != "NONE")
		{
			if (!m_pService->GetMotionControl()->DigitalOutputSet(maps.at("IO"), maps.at("Value").toInt(), true))
			{
				LOG_PROCESS_INFO(QObject::tr("%1[IO]Set %2 faild").arg(bB ? "[B]" : "")
					.arg(QObject::tr(maps.at("IO").toUtf8().data())).toUtf8().data());
				if (bB)
				{
					g_bGroupError.store(true);
					return;// 支线程直接返回
				}
				g_bStop.store(true);
				return;
			}
		}
		else
		{
			DigitalIOData IOData;
			string index = maps.at("Value").toStdString();
			int iPos = 0;
			// 检查有无"-"设置信号取反
			if (index.find('-') != std::string::npos)
			{
				IOData.bInversion = true;
				index = index.substr(1);
			}
			// 检查有无"N"走拓展模块
			if (index.find('N') != std::string::npos)
			{
				IOData.bExpand = true;
				iPos++;
				IOData.strPort = "NJHT_DO" + index.substr(iPos, index.find('.') - 1);
				IOData.iIO = stoi(index.substr(iPos + 2, index.find('=') - iPos - 2));
			}
			else
			{
				IOData.iPort = stoi(index.substr(iPos, 1));
				IOData.iIO = stoi(index.substr(iPos + 2, index.find('=') - iPos - 2));
			}
			int iValue = stoi(index.substr(index.size() - 1, 1));

			if (!m_pService->GetMotionControl()->DigitalOutputSet(IOData, iValue, true))
			{
				LOG_PROCESS_INFO(QObject::tr("%1[IO]Set %2 faild").arg(bB ? "[B]" : "")
					.arg(IOData.strPort.c_str()).toUtf8().data());
				if (bB)
				{
					g_bGroupError.store(true);
					return;// 支线程直接返回
				}
				g_bStop.store(true);
				return;
			}
		}
	}
	else
	{
		if (maps.at("IO") != "NONE")
		{
			if (m_pService->GetMotionControl()->AnalogOutputSet(maps.at("IO"), maps.at("Value").toDouble(), true))
			{
				LOG_PROCESS_INFO(QObject::tr("%1[IO]Set %2 faild").arg(bB ? "[B]" : "")
					.arg(QObject::tr(maps.at("IO").toUtf8().data())).toUtf8().data());
				if (bB)
				{
					g_bGroupError.store(true);
					return;// 支线程直接返回
				}
				g_bStop.store(true);
				return;
			}
		}
		else
		{
			AnalogIOData IOData;
			string index = maps.at("Value").toStdString();
			int iPos = 0;
			// 检查有无"N"走拓展模块
			if (index.find('N') != std::string::npos)
			{
				IOData.bExpand = true;
				iPos++;
				IOData.strPort = "NJHT_AOUT" + index.substr(iPos, index.find('=') - iPos);
			}
			else
			{
				IOData.iPort = stoi(index.substr(iPos, index.find('=') - iPos));
			}
			double dValue = stod(index.substr(index.find('=') + 1));

			if (!m_pService->GetMotionControl()->AnalogOutputSet(IOData, dValue, true))
			{
				LOG_PROCESS_INFO(QObject::tr("%1[IO]Set %2 faild").arg(bB ? "[B]" : "")
					.arg(IOData.strPort.c_str()).toUtf8().data());
				if (bB)
				{
					g_bGroupError.store(true);
					return;// 支线程直接返回
				}
				g_bStop.store(true);
				return;
			}
		}
	}
	if (bB)	return;// 支线程直接返回

	g_bFinish.store(true);
	return;
}

void ProcessModule::ItemCommands(const map<QString, QString>& maps)
{
	string strCommands = "";
	QFile file(maps.at("File"));
	QByteArray data = file.readAll();
	file.close();
	strCommands = std::string(data.constData(), data.length());
	LOG_PROCESS_INFO(QObject::tr("[Commands]\n%1").arg(data.constData()).toUtf8().data());

	if (m_pService->GetMotionControl()->CheckBuffer(9, strCommands))
	{
		if (g_bPause.load())
		{
			while (g_bPause.load() && !g_bStop.load())
			{
				boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
				continue;
			}

			if (g_bStop.load()) return;
		}

		if (g_bStop.load()) return;

		LOG_PROCESS_INFO(QObject::tr("[Commands]Run commands").toUtf8().data());
		m_pService->GetMotionControl()->RunBuffer(9);
	}
	else
	{
		g_bStop.store(true);
		SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_COMMANDSINVALID);
		return;
	}

	do
	{
		if (g_bPause.load())
		{
			m_pService->GetMotionControl()->PauseBuffer(9);

			while (g_bPause.load() && !g_bStop.load())
			{
				boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
				continue;
			}

			if (g_bStop.load()) return;

			// 恢复节点
			m_pService->GetMotionControl()->RunBuffer(9);
		}

		if (g_bStop.load()) return;

		boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
	} while (m_pService->GetMotionControl()->IsBufferRunning(9));

	LOG_PROCESS_INFO(QObject::tr("[Commands]End of run.").toUtf8().data());
	g_bFinish.store(true);
}

void ProcessModule::ItemMeasurement(const map<QString, QString>& maps)
{
	bool bB = maps.count("BBBBB");	// 支线程运行标志
	CompValue cValue, cResult;
	ErrorCode eCode = m_pService->GetCompDevice()->MeasurementComp(cValue, maps.at("Source").toInt());
	if (eCode != ErrorCode::ERROR_NONE)
	{
		SHOW_PROCESS_ERROR(eCode);
		if (bB)
		{
			g_bGroupError.store(true);
			return;// 支线程直接返回
		}
		g_bStop.store(true);
		return;
	}
	LOG_PROCESS_INFO(QObject::tr("%1[Measurement]Original index \"%2\", value X=%3 Y=%4")
		.arg(bB ? "[B]" : "").arg(maps.at("Index")).arg(cValue.X).arg(cValue.Y).toUtf8().data());

	// 这里对结果校验，
	// 核算结果有一个0为正常的情况。结果判定在计算节点吧。

	bool bNumber;
	maps.at("X").toDouble(&bNumber);
	if (bNumber)
		cResult.X = maps.at("X");
	else
	{
		eCode = Expression::GetResult(maps.at("X"), cValue, cResult.X);
		if (eCode != ErrorCode::ERROR_NONE)
		{
			SHOW_PROCESS_ERROR(eCode);
			if (bB)
			{
				g_bGroupError.store(true);
				return;// 支线程直接返回
			}
			g_bStop.store(true);
			return;
		}
	}

	maps.at("Y").toDouble(&bNumber);
	if (bNumber)
		cResult.Y = maps.at("Y");
	else
	{
		eCode = Expression::GetResult(maps.at("Y"), cValue, cResult.Y);
		if (eCode != ErrorCode::ERROR_NONE)
		{
			SHOW_PROCESS_ERROR(eCode);
			if (bB)
			{
				g_bGroupError.store(true);
				return;// 支线程直接返回
			}
			g_bStop.store(true);
			return;
		}
	}

	m_pService->GetCompDevice()->SetCompValue(maps.at("Index"), cResult);
	LOG_PROCESS_INFO(QObject::tr("%1[Measurement]Resuclt index \"%2\", value X=%3 Y=%4")
		.arg(bB ? "[B]" : "").arg(maps.at("Index")).arg(cResult.X).arg(cResult.Y).toUtf8().data());
	
	if (bB)	return;// 支线程直接返回
	g_bFinish.store(true);
	return;
}

void ProcessModule::ItemCalculation(const map<QString, QString>& maps)
{
	bool bB = maps.count("BBBBB");	// 支线程运行标志
	ErrorCode eCode;
	CompValue cResult;
	map<QString, CompValue>	mapComps = m_pService->GetCompDevice()->GetComps();

	LOG_PROCESS_INFO(QObject::tr("%1[Calculation]Original index \"%2\", value X=%3 Y=%4")
		.arg(bB ? "[B]" : "").arg(maps.at("Index")).arg(maps.at("X")).arg(maps.at("Y")).toUtf8().data());

	bool bNumber, bResult = true;
	QString qstrX = maps.at("X");
	if (qstrX[0] == '<' || qstrX[0] == '=')
	{	// 误差及条件判定
		m_pService->GetCompDevice()->GetCompValue(maps.at("Index"), cResult);
		double dResultX = abs(cResult.X.toDouble()); // 取绝对值
		double dValue = qstrX.mid(1).toDouble();
		if (qstrX[0] == '<' && dResultX > dValue)
			bResult = false;
		else if (qstrX[0] == '=' && dResultX != dValue)
			bResult = false;
		
		if (!bResult)
		{
			LOG_PROCESS_INFO(QObject::tr("%1[Calculation]Resuclt index \"%2\", X %3 !%4")
				.arg(bB ? "[B]" : "").arg(maps.at("Index")).arg(cResult.X).arg(qstrX).toUtf8().data());
			if (bB)
			{
				g_bGroupError.store(true);
				return;// 支线程直接返回
			}
			SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_OUTOFTOLERANCE);
			g_bStop.store(true);
			return;
		}
	}
	else
	{	// 公式/数字
		qstrX.toDouble(&bNumber);
		if (bNumber)
			cResult.X = qstrX;
		else
		{
			eCode = Expression::GetResult(qstrX, mapComps, cResult.X);
			if (eCode != ErrorCode::ERROR_NONE)
			{
				SHOW_PROCESS_ERROR(eCode);
				if (bB)
				{
					g_bGroupError.store(true);
					return;// 支线程直接返回
				}
				g_bStop.store(true);
				return;
			}
		}
		LOG_PROCESS_INFO(QObject::tr("%1[Calculation]Resuclt index \"%2\", value X=%3")
			.arg(bB ? "[B]" : "").arg(maps.at("Index")).arg(cResult.X).toUtf8().data());
	}

	QString qstrY = maps.at("Y");
	if (qstrY[0] == '<' || qstrY[0] == '=')
	{	// 误差及条件判定
		m_pService->GetCompDevice()->GetCompValue(maps.at("Index"), cResult);
		double dResultY = abs(cResult.Y.toDouble()); // 取绝对值
		double dValue = qstrY.mid(1).toDouble();
		if (qstrY[0] == '<' && dResultY > dValue)
			bResult = false;
		else if (qstrY[0] == '=' && dResultY != dValue)
			bResult = false;

		if (!bResult)
		{
			LOG_PROCESS_INFO(QObject::tr("%1[Calculation]Resuclt index \"%2\", Y %3 !%4")
				.arg(bB ? "[B]" : "").arg(maps.at("Index")).arg(cResult.Y).arg(qstrY).toUtf8().data());
			if (bB)
			{
				g_bGroupError.store(true);
				return;// 支线程直接返回
			}
			SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_OUTOFTOLERANCE);
			g_bStop.store(true);
			return;
		}
	}
	else
	{	// 公式/数字
		qstrY.toDouble(&bNumber);
		if (bNumber)
			cResult.Y = qstrY;
		else
		{
			eCode = Expression::GetResult(qstrY, mapComps, cResult.Y);
			if (eCode != ErrorCode::ERROR_NONE)
			{
				SHOW_PROCESS_ERROR(eCode);
				if (bB)
				{
					g_bGroupError.store(true);
					return;// 支线程直接返回
				}
				g_bStop.store(true);
				return;
			}
		}
		LOG_PROCESS_INFO(QObject::tr("%1[Calculation]Resuclt index \"%2\", value Y=%3")
			.arg(bB ? "[B]" : "").arg(maps.at("Index")).arg(cResult.Y).toUtf8().data());
	}
	m_pService->GetCompDevice()->SetCompValue(maps.at("Index"), cResult);
	
	if (bB)	return;// 支线程直接返回
	g_bFinish.store(true);
	return;
}

void ProcessModule::ItemRunGroupCheck(const map<QString, QString>& maps)
{
	int iWait = 0;
	string strLOG = "";
	switch (maps.at("Unit").toInt())
	{
	case 0:	// ms
		iWait = maps.at("Value").toInt();
		strLOG = "Timeout waiting " + maps.at("Value").toStdString() + " ms";
		break;
	case 1: // s
		iWait = maps.at("Value").toInt() * 1000;
		strLOG = "Timeout waiting " + maps.at("Value").toStdString() + " s";
		break;
	case 2: // min
		iWait = maps.at("Value").toInt() * 60000;
		strLOG = "Timeout waiting " + maps.at("Value").toStdString() + " min";
		break;
	}

	bool bGroupThread = false;
	while (true)
	{
		auto tpStart = std::chrono::steady_clock::now();
		auto tpDeadline = tpStart + std::chrono::milliseconds(iWait);
		long long iPausedMs = 0;
		while (true)
		{
			if (g_bPause.load())
			{
				auto tpPauseStart = std::chrono::steady_clock::now();
				while (g_bPause.load() && !g_bStop.load())
				{
					boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
					continue;
				}

				if (g_bStop.load())		return;

				const auto tpPauseDuration = std::chrono::steady_clock::now() - tpPauseStart;
				tpDeadline += tpPauseDuration;
				iPausedMs += std::chrono::duration_cast<std::chrono::milliseconds>(tpPauseDuration).count();
			}

			if (g_bStop.load())			return;

			const auto tpNow = std::chrono::steady_clock::now();
			if (tpNow >= tpDeadline)
				break;

			const auto iRemain = std::chrono::duration_cast<std::chrono::milliseconds>(
				tpDeadline - tpNow).count();
			const auto iWaitSlice = (std::min)(static_cast<long long>(20), iRemain);

			// 支线程运行状态
			bGroupThread = m_GroupThread.try_join_for(boost::chrono::milliseconds(iWaitSlice));
			const auto iElapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
				std::chrono::steady_clock::now() - tpStart).count() - iPausedMs;
			// 标志结束 线程运行结束，正常状态
			if (!g_bGroupRuning.load() && bGroupThread)	
			{
				LOG_PROCESS_INFO(QObject::tr("[RunGroupCheck]Wait Thread of %1 ms").arg(iElapsedMs).toUtf8().data());
				g_bFinish.store(true);
				return;
			}
			// 标志未结束 线程运行结束，线程执行异常
			else if (g_bGroupRuning.load() && bGroupThread)
			{
				// 子线程异常
				LOG_PROCESS_INFO(QObject::tr("[RunGroupCheck]Thread timeout of %1 ms").arg(iElapsedMs).toUtf8().data());
				SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_GROUPTIMEOUT);
				g_bStop.store(true);
				return;
			}

		}
		
		// 超时后进入暂停，直到停止/继续
		LOG_PROCESS_INFO(QObject::tr("[RunGroupCheck]%1").arg(strLOG.c_str()).toUtf8().data());
		SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_GROUPTIMEOUT);
		g_bPause.store(true);
		// 恢复的等待时间逐渐翻倍
		iWait *= 2;
		boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
	}
}

void ProcessModule::ItemFeeding(const map<QString, QString>& maps)
{
	QG_CuttingInfoWidget::RestartSingleCutTimer();
	QG_CuttingInfoWidget::ResetCompletedCutCount();

	m_pService->GetMotionControl()->DigitalOutputSet(DigitalOUT::Pliers, 1);
	LOG_PROCESS_INFO(QObject::tr("[Feeding] Turn on the pliers and wait for %1 ms").arg(maps.at("OpenPliers")).toUtf8().data());
	if (!WaitTime(maps.at("OpenPliers").toInt())) return;

	boost::this_thread::sleep_for(boost::chrono::milliseconds());
	m_pService->GetMotionControl()->DigitalOutputSet(DigitalOUT::Chuck, 0);
	LOG_PROCESS_INFO(QObject::tr("[Feeding] Turn off the chuck and wait for %1 ms").arg(maps.at("CloseChuck")).toUtf8().data());
	if (!WaitTime(maps.at("CloseChuck").toInt())) return;

	// 计算目标位置
	double dAbsolutePos = m_pService->GetCuttingDevice()->GetEntitysLeftLimitPos() + maps.at("Compensation").toDouble();
	if (!m_pService->GetMotionControl()->IsReachPos(Axis::X, false, dAbsolutePos))
	{
		SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_AXISOUTOFLIMIT);
		g_bStop.store(true);
		return;
	}

	double dVel = 0;
	switch (maps.at("AxisSpeed").toInt())
	{
	case 0:		SETTINGS->GetKeyValue("fLowSpeed",		dVel, SettingSection::Axis, "X");	break;
	case 1:		SETTINGS->GetKeyValue("fMediumSpeed",	dVel, SettingSection::Axis, "X");	break;
	case 2:		SETTINGS->GetKeyValue("fHighSpeed",		dVel, SettingSection::Axis, "X");	break;
	}

	LOG_PROCESS_INFO(QObject::tr("[Feeding] Axis X move to %1").arg(dAbsolutePos).toUtf8().data());

	while (g_bPause.load() && !g_bStop.load())
	{
		boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
		continue;
	}

	if (g_bStop.load()) return;

	m_pService->GetMotionControl()->MoveAbsolute(Axis::X, dAbsolutePos, dVel);

	do
	{
		if (g_bPause.load())
		{
			m_pService->GetMotionControl()->StopMotion(Axis::X);

			while (g_bPause.load() && !g_bStop.load())
			{
				boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
				continue;
			}

			if (g_bStop.load()) return;

			// 恢复后重新开始运动
			m_pService->GetMotionControl()->MoveAbsolute(Axis::X, dAbsolutePos, dVel);
		}

		if (g_bStop.load())
		{
			m_pService->GetMotionControl()->StopMotion(Axis::X);
			return;
		}

		boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
	} while (m_pService->GetMotionControl()->IsAxisMoving(Axis::X));
	
	m_pService->GetMotionControl()->DigitalOutputSet(DigitalOUT::Chuck, 1);
	LOG_PROCESS_INFO(QObject::tr("[Feeding] Turn on the chuck and wait for %1 ms").arg(maps.at("OpenChuck")).toUtf8().data());
	if (!WaitTime(maps.at("OpenChuck").toInt())) return;

	m_pService->GetMotionControl()->DigitalOutputSet(DigitalOUT::Pliers, 0);
	LOG_PROCESS_INFO(QObject::tr("[Feeding] Turn off the pliers and wait for %1 ms").arg(maps.at("ClosePliers")).toUtf8().data());
	if (!WaitTime(maps.at("ClosePliers").toInt())) return;

	QG_CuttingInfoWidget::AddCompletedFeedCount();
	g_bFinish.store(true);
	return;
}

void ProcessModule::ItemCutting(const map<QString, QString>& maps)
{
	m_pService->SetCuttingDevice("NormalCutting");
	m_pService->GetCuttingDevice()->SetCuttingTestFlag(false);
	if (m_eRunMode == RunMode::CuttingTest)
		m_pService->GetCuttingDevice()->SetCuttingTestFlag(true);

	g_eState.store(SystemStatus::LaserProcessing);
	
	if (!StartBlow())
	{
		g_bStop.store(true);
		return;
	}

	if (!m_pService->GetCuttingDevice()->StartCutting(maps))
	{
		g_bStop.store(true);
		return;
	}

	if (QVariant(maps.at("Count")).toBool())
	{
		QG_CuttingInfoWidget::AddCurrentBatchCount();
		CUTTINGINFO->SaveAccumulatedProductionCount();
		LOG_PROCESS_INFO(QObject::tr("[CuttingInfo] CBC:%1\t APC:%2").arg(QG_CuttingInfoWidget::m_iCurrentBatchCount)
			.arg(QG_CuttingInfoWidget::m_iAccumulatedProductionCount).toUtf8().data());
	}

	g_bFinish.store(true);
	return;
}

void ProcessModule::ItemOverCutting(const map<QString, QString>& maps)
{
	m_pService->SetCuttingDevice("OverCutting");
	m_pService->GetCuttingDevice()->SetCuttingTestFlag(false);
	if (m_eRunMode == RunMode::CuttingTest)
		m_pService->GetCuttingDevice()->SetCuttingTestFlag(true);

	g_eState.store(SystemStatus::LaserProcessing);
	main_window->update1();

	if (!StartBlow())
	{
		g_bStop.store(true);
		return;
	}

	if (!m_pService->GetCuttingDevice()->StartCutting(maps))
	{
		g_bStop.store(true);
		return;
	}

	if (QVariant(maps.at("Count")).toBool())
	{
		QG_CuttingInfoWidget::AddCurrentBatchCount();
		CUTTINGINFO->SaveAccumulatedProductionCount();
		LOG_PROCESS_INFO(QObject::tr("[CuttingInfo] CBC:%1\t APC:%2").arg(QG_CuttingInfoWidget::m_iCurrentBatchCount)
			.arg(QG_CuttingInfoWidget::m_iAccumulatedProductionCount).toUtf8().data());
	}
	g_bFinish.store(true);
	return;
}

void ProcessModule::ItemCamera(const map<QString, QString>& maps)
{
	if (!DT::IsUseCamera())
	{
		SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_IOINVALID,
			QObject::tr("[Camera] Camera feature is disabled").toUtf8().data());
		g_bStop.store(true);
		g_bFinish.store(true);
		return;
	}

	QString workflowId;
	if (maps.count("WorkflowID"))
		workflowId = maps.at("WorkflowID").trimmed();

	LOG_PROCESS_INFO(QObject::tr("[Camera] Execute workflow: %1").arg(workflowId).toUtf8().data());

	if (workflowId.isEmpty() || !VisionModule::instance()->isWorkflowReady(workflowId))
	{
		SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_IOINVALID,
			QObject::tr("Vision workflow not ready: %1").arg(workflowId).toUtf8().data());
		g_bStop.store(true);
		g_bFinish.store(true);
		return;
	}

	VisionResult result = VisionModule::instance()->executeWorkflow(workflowId, true);
	if (!result.success)
	{
		SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_IOINVALID,
			QObject::tr("[Camera] Workflow failed: %1").arg(result.message).toUtf8().data());
		g_bStop.store(true);
	}
	else
	{
		LOG_PROCESS_INFO(QObject::tr("[Camera] Workflow done. World=(%1, %2)")
			.arg(result.worldX, 0, 'f', 4)
			.arg(result.worldY, 0, 'f', 4)
			.toUtf8().data());
	}

	g_bFinish.store(true);
}

void ProcessModule::ClearAlignmentSequenceCache(RS_Document* document)
{
	QG_GraphicView* view = nullptr;
	if (!document) {
		getCurrentDocumentContext(main_window, document, view);
	}
	else if (main_window) {
		QC_MDIWindow* mdiWindow = main_window->getMDIWindow();
		if (mdiWindow && mdiWindow->getDocument() == document) {
			view = mdiWindow->getGraphicView();
		}
	}

	clearAlignmentPresentation(this,
		document,
		view,
		m_alignmentSavedSequence,
		m_alignmentSavedSelectedSequence,
		m_alignmentHasTemporaryCuttingScope);
	m_alignmentSavedSequence.clear();
	m_alignmentSavedSelectedSequence.clear();
	m_alignmentHasTemporaryCuttingScope = false;

	if (m_alignmentSequenceCache.isEmpty()) {
		return;
	}

	for (RS_Entity* entity : m_alignmentSequenceCache) {
		delete entity;
	}
	m_alignmentSequenceCache.clear();
}

void ProcessModule::ItemMarkAcquire(const map<QString, QString>& maps)
{
	const bool bB = maps.count("BBBBB");
	const int count = std::max(1, mapIntValue(maps, "Count", 1));

	if (!DT::IsUseCamera())
	{
		SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_IOINVALID,
			QObject::tr("[MarkAcquire] Camera feature is disabled").toUtf8().data());
		if (bB) {
			g_bGroupError.store(true);
			return;
		}
		g_bStop.store(true);
		return;
	}

	for (int row = 0; row < count; ++row)
	{
		const QString markId = rowMapValue(maps, row, "MarkId").trimmed();
		const QString workflowId = rowMapValue(maps, row, "WorkflowID").trimmed();
		if (markId.isEmpty() || workflowId.isEmpty())
		{
			SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_IOINVALID,
				QObject::tr("[MarkAcquire] row %1 config is incomplete").arg(row + 1).toUtf8().data());
			if (bB) {
				g_bGroupError.store(true);
				return;
			}
			g_bStop.store(true);
			return;
		}

		MarkPoint mark;
		if (!VisionModule::instance()->getMarkPoint(markId, mark))
		{
			SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_IOINVALID,
				QObject::tr("[MarkAcquire] mark not found: %1").arg(markId).toUtf8().data());
			if (bB) {
				g_bGroupError.store(true);
				return;
			}
			g_bStop.store(true);
			return;
		}

		if (!VisionModule::instance()->isWorkflowReady(workflowId))
		{
			SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_IOINVALID,
				QObject::tr("[MarkAcquire] workflow not ready: %1").arg(workflowId).toUtf8().data());
			if (bB) {
				g_bGroupError.store(true);
				return;
			}
			g_bStop.store(true);
			return;
		}

		double safeZ = 0.0;
		double captureZ = 0.0;
		double stabilizeDelayMs = 500.0;
		ErrorCode errorCode = ErrorCode::ERROR_NONE;
		if (!resolveExpressionValue(m_pService, rowMapValue(maps, row, "SafeZ"), safeZ, errorCode) ||
			!resolveExpressionValue(m_pService, rowMapValue(maps, row, "CaptureZ"), captureZ, errorCode))
		{
			SHOW_PROCESS_ERROR(errorCode);
			if (bB) {
				g_bGroupError.store(true);
				return;
			}
			g_bStop.store(true);
			return;
		}

		const QString stabilizeDelayText = rowMapValue(maps, row, "StabilizeMs").trimmed();
		if (!stabilizeDelayText.isEmpty() && !resolveExpressionValue(m_pService, stabilizeDelayText, stabilizeDelayMs, errorCode))
		{
			SHOW_PROCESS_ERROR(errorCode);
			if (bB) {
				g_bGroupError.store(true);
				return;
			}
			g_bStop.store(true);
			return;
		}
		const int stabilizeDelay = stabilizeDelayMs < 0.0 ? 0 : static_cast<int>(std::lround(stabilizeDelayMs));

		LOG_PROCESS_INFO(QObject::tr("[MarkAcquire] row %1 mark=%2 workflow=%3")
			.arg(row + 1).arg(markId).arg(workflowId).toUtf8().data());

		const QPointF captureOffset = resolveMarkAcquireCaptureOffset(workflowId);
		const double captureWorldX = mark.worldX - captureOffset.x();
		const double captureWorldY = mark.worldY - captureOffset.y();

		if (!AxisMoveToPos(Axis::Z, safeZ, "fMediumSpeed") ||
			!AxisMoveToPos(Axis::X, captureWorldX, "fMediumSpeed") ||
			!AxisMoveToPos(Axis::Y, captureWorldY, "fMediumSpeed") ||
			!AxisMoveToPos(Axis::Z, captureZ, "fMediumSpeed"))
		{
			if (bB) {
				g_bGroupError.store(true);
				return;
			}
			g_bStop.store(true);
			return;
		}
		if (stabilizeDelay > 0 && !WaitTime(stabilizeDelay))
		{
			return;
		}

		VisionResult result = VisionModule::instance()->executeWorkflow(workflowId, true);
		if (!result.success)
		{
			SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_IOINVALID,
				QObject::tr("[MarkAcquire] workflow failed: %1").arg(result.message).toUtf8().data());
			if (bB) {
				g_bGroupError.store(true);
				return;
			}
			g_bStop.store(true);
			return;
		}

		if (!result.hasWorldCoordinate)
		{
			SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_IOINVALID,
				QObject::tr("[MarkAcquire] workflow has no calibrated measurement: %1").arg(workflowId).toUtf8().data());
			if (bB) {
				g_bGroupError.store(true);
				return;
			}
			g_bStop.store(true);
			return;
		}

		VisionModule::instance()->updateMarkDetected(
			markId,
			result.measuredX,
			result.measuredY,
			result.hasAngle ? result.angle : 0.0,
			result.hasScore ? result.score : 0.0);
	}

	if (!bB)
		g_bFinish.store(true);
}

void ProcessModule::ItemAlignment(const map<QString, QString>& maps)
{
	const bool bB = maps.count("BBBBB");
	const int mode = mapIntValue(maps, "Mode", 0);
	const int groupCount = std::max(1, mapIntValue(maps, "GroupCount", 1));

	RS_Document* document = nullptr;
	QG_GraphicView* view = nullptr;
	if (!getCurrentDocumentContext(main_window, document, view))
	{
		SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_IOINVALID,
			QObject::tr("[Alignment] no active document").toUtf8().data());
		if (bB) {
			g_bGroupError.store(true);
			return;
		}
		g_bStop.store(true);
		return;
	}

	ClearAlignmentSequenceCache(document);
	const AlignmentSourceContext sourceContext = collectAlignmentSourceContext(document, view);
	if (sourceContext.sourceEntities.empty())
	{
		SHOW_PROCESS_ERROR(ErrorCode::ERROR_CUTTING_LISTEMPTY,
			QObject::tr("[Alignment] no source entities for array compensation").toUtf8().data());
		if (bB) {
			g_bGroupError.store(true);
			return;
		}
		g_bStop.store(true);
		return;
	}

	QList<RS_Entity*> alignedSequence;
	int validGroupCount = 0;
	bool referenceMidpointReady = false;
	RS_Vector referenceWorldMidpoint;

	for (int row = 0; row < groupCount; ++row)
	{
		const QString markId1 = rowMapValue(maps, row, "MarkId1").trimmed();
		const QString markId2 = rowMapValue(maps, row, "MarkId2").trimmed();
		const bool allowRotate = rowMapValue(maps, row, "AllowRotate") != "0";
		const bool allowScale = mode == 1 && rowMapValue(maps, row, "AllowScale") != "0";

		if (markId1.isEmpty())
			continue;

		MarkPoint mark1;
		if (!VisionModule::instance()->getMarkPoint(markId1, mark1) || !mark1.detected)
			continue;

		AlignmentTransform transform;
		RS_Vector groupWorldMidpoint;

		if (mode == 0)
		{
			groupWorldMidpoint = alignmentWorldMidpoint(mark1, nullptr);
			if (!referenceMidpointReady) {
				referenceWorldMidpoint = groupWorldMidpoint;
				referenceMidpointReady = true;
			}
			const RS_Vector groupBaseOffset = groupWorldMidpoint - referenceWorldMidpoint;

			transform.basePoint = RS_Vector(mark1.measuredX, mark1.measuredY);
			transform.offset = RS_Vector(mark1.measuredX - mark1.worldX, mark1.measuredY - mark1.worldY) + groupBaseOffset;
			if (allowRotate)
				transform.angleRad = mark1.angle * kPi / 180.0;
		}
		else
		{
			if (markId2.isEmpty())
				continue;

			MarkPoint mark2;
			if (!VisionModule::instance()->getMarkPoint(markId2, mark2) || !mark2.detected)
				continue;

			const RS_Vector filePoint1(mark1.worldX, mark1.worldY);
			const RS_Vector filePoint2(mark2.worldX, mark2.worldY);
			const RS_Vector measuredPoint1(mark1.measuredX, mark1.measuredY);
			const RS_Vector measuredPoint2(mark2.measuredX, mark2.measuredY);
			groupWorldMidpoint = alignmentWorldMidpoint(mark1, &mark2);
			if (!referenceMidpointReady) {
				referenceWorldMidpoint = groupWorldMidpoint;
				referenceMidpointReady = true;
			}
			const RS_Vector groupBaseOffset = groupWorldMidpoint - referenceWorldMidpoint;
			const RS_Vector fileVec = filePoint2 - filePoint1;
			const RS_Vector measuredVec = measuredPoint2 - measuredPoint1;
			const double fileLen = fileVec.magnitude();
			const double measuredLen = measuredVec.magnitude();
			if (fileLen < 1e-9 || measuredLen < 1e-9)
				continue;

			transform.basePoint = measuredPoint1;
			transform.offset = measuredPoint1 - filePoint1 + groupBaseOffset;
			if (allowRotate)
			{
				transform.angleRad = std::atan2(measuredVec.y, measuredVec.x) - std::atan2(fileVec.y, fileVec.x);
			}
			if (allowScale)
			{
				transform.scale = measuredLen / fileLen;
			}
		}

		LOG_PROCESS_INFO(QObject::tr("[Alignment] build row %1 offset=(%2,%3) angle=%4 scale=%5")
			.arg(row + 1)
			.arg(transform.offset.x, 0, 'f', 4)
			.arg(transform.offset.y, 0, 'f', 4)
			.arg(transform.angleRad * 180.0 / kPi, 0, 'f', 4)
			.arg(transform.scale, 0, 'f', 6)
			.toUtf8().data());

		for (RS_Entity* sourceEntity : sourceContext.sourceEntities)
		{
			RS_Entity* cloneEntity = createAlignedClone(sourceEntity, transform, document);
			if (!cloneEntity) {
				continue;
			}
			m_alignmentSequenceCache.append(cloneEntity);
			alignedSequence.append(cloneEntity);
		}

		++validGroupCount;
	}

	if (alignedSequence.empty() || validGroupCount == 0)
	{
		SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_IOINVALID,
			QObject::tr("[Alignment] no valid mark group").toUtf8().data());
		if (bB) {
			g_bGroupError.store(true);
			return;
		}
		g_bStop.store(true);
		return;
	}

	m_alignmentSavedSequence = sourceContext.savedSequence;
	m_alignmentSavedSelectedSequence = sourceContext.savedSelectedSequence;
	m_alignmentHasTemporaryCuttingScope = true;
	updateAlignmentPresentation(this, document, view, alignedSequence);

	LOG_PROCESS_INFO(QObject::tr("[Alignment] generated array sequence groups=%1 entities=%2")
		.arg(validGroupCount)
		.arg(alignedSequence.size())
		.toUtf8().data());

	if (!bB)
		g_bFinish.store(true);
}

void ProcessModule::ItemAutoFocus(const map<QString, QString>& maps)
{
	m_pService->SetCuttingDevice("FocusCutting");
	m_pService->GetCuttingDevice()->SetCuttingTestFlag(false);
	if (m_eRunMode == RunMode::CuttingTest)
		m_pService->GetCuttingDevice()->SetCuttingTestFlag(true);

	g_eState.store(SystemStatus::LaserProcessing);

	if (!StartBlow())
	{
		g_bStop.store(true);
		return;
	}

	if (!m_pService->GetCuttingDevice()->StartCutting(maps))
	{
		g_bStop.store(true);
		return;
	}

	g_bFinish.store(true);
	return;
}

bool ProcessModule::StartBlow()
{
	if (m_eRunMode == RunMode::CuttingTest)
		return true;

	bool bBlow = false;
	SETTINGS->GetKeyValue("bBlow", bBlow, SettingSection::Gas, "Gas");
	double dBlowDelay = 0.0;
	SETTINGS->GetKeyValue("fBlowDelay", dBlowDelay, SettingSection::Gas, "Gas");
	int iBlow = 0;
	m_pService->GetMotionControl()->DigitalOutputGet(DigitalOUT::Blow, iBlow);
	if (bBlow && !iBlow)
	{
		if (!WaitTime((int)dBlowDelay)) return false;
		m_pService->GetMotionControl()->DigitalOutputSet(DigitalOUT::Blow, 1);
	}
	return true;
}

bool ProcessModule::AxisMoveToPos(Axis eAxis, double dPos, string strSpeedMode)
{
	string strAxis = enum_name(eAxis).data();
	double dVel;
	SETTINGS->GetKeyValue(strSpeedMode, dVel, SettingSection::Axis, strAxis);

	if (!m_pService->GetMotionControl()->IsReachPos(eAxis, false, dPos))
	{
		SHOW_OPER_WARN(WarnCode::WARN_MC_OUTOFLIMIT);
		return false;
	}

	if (!m_pService->GetMotionControl()->MoveAbsolute(eAxis, dPos, dVel))
	{
		SHOW_OPER_WARN(ErrorCode::ERROR_MONITOR_AXISDISABLED);
		return false;
	}

	do
	{
		if (g_bPause.load())
		{
			m_pService->GetMotionControl()->StopMotion(eAxis);

			while (g_bPause.load() && !g_bStop.load())
			{
				boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
				continue;
			}

			if (g_bStop.load()) return false;

			// 恢复后重新开始运动
			m_pService->GetMotionControl()->MoveAbsolute(eAxis, dPos, dVel);
		}

		if (g_bStop.load())
		{
			m_pService->GetMotionControl()->StopMotion(eAxis);
			g_bRunning = false;
			return false;
		}

		boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
	} while (m_pService->GetMotionControl()->IsAxisMoving(eAxis));
	
	return true;

}
