#include "Cutting_Normal.h"
#include "qc_applicationwindow.h"
#include "rs_line.h"
#include "rs_arc.h"
#include "rs_circle.h"
#include "rs_polyline.h"
#include "rs_selection.h"
#include "qc_mdiwindow.h"
#include "qg_CuttingInfoWidget.h"

CuttingNormal::CuttingNormal()
{
}

bool CuttingNormal::StartCutting(const map<QString, QString>& maps)
{
	int iStart = -1, iEnd = -1;

	// 非框选切且有切割顺序情况下应用切割刀号
	if (!IsSelectionCutting() && SequenceListSize())
	{
		if (maps.at("StartNumber").size())
			iStart = maps.at("StartNumber").toInt();

		if (maps.at("EndNumber").size())
			iEnd = maps.at("EndNumber").toInt();
	}

	m_iCurrentNum = 0;
	m_iTotalNum = 0;
	
	CreatCuttingData(iStart, iEnd);
	QG_CuttingInfoWidget::SetTotalCutCount(m_iTotalNum);
	if (m_CuttingList.isEmpty())
	{
		SHOW_PROCESS_ERROR(ErrorCode::ERROR_CUTTING_LISTEMPTY);
		g_bStop.store(true);
		g_bFinish.store(true);
		g_eState.store(SystemStatus::Error);
		return false;
	}
	m_itrCurList = m_CuttingList.begin();

	CompValue cValue;
	QString qstrCompIndex = maps.at("CompensationIndex");
	if (!qstrCompIndex.isEmpty())
	{
		if (QC_ApplicationWindow::getAppWindow()->getService()->GetCompDevice() &&
			!QC_ApplicationWindow::getAppWindow()->getService()->GetCompDevice()->GetCompValue(qstrCompIndex, cValue))
		{
			SHOW_PROCESS_ERROR(ErrorCode::ERROR_CUTTING_GETCOMPENSATIONFAILED);
			g_bStop.store(true);
			g_bFinish.store(true);
			g_eState.store(SystemStatus::Error);
			return false;
		}
		LOG_PROCESS_INFO(QObject::tr("[Cutting]Compensation index \"%1\", value X=%2 Y=%3")
			.arg(qstrCompIndex).arg(cValue.X).arg(cValue.Y).toUtf8().data());
	}

	bool bRetuen = CuttingThread(cValue.X.toDouble(), cValue.Y.toDouble());
	return bRetuen;
}

bool CuttingNormal::CuttingThread(double dOffsetX, double dOffsetY)
{
	
	if (m_CuttingList.empty())
	{
		g_bStop.store(true);
		g_bFinish.store(true);
		return false;
	}

	// 切割完成
	if (m_itrCurList == m_CuttingList.end())
		return true;

	bool bFirst = true;
	QList<RS_Entity*>::iterator itrLastPolyline = m_itrCurList;
	MotionControl* pMC = QC_ApplicationWindow::getAppWindow()->getService()->GetMotionControl();
	bool bIsACS = (pMC->GetName() == "ACS");
	bool bACSCommandBuilt = false; // 标记是否已预构建下一特征的切割指令（ACS流水线优化）
	Tool* lastTool = nullptr;

	std::string strLastFlightToolName = ""; // 记录当前飞行切割块的工具名
	for (; m_itrCurList != m_CuttingList.end(); m_itrCurList++)
	{
		// 激光器参数的下发，加工边界更改
		Tool* curTool = ToolFactory::GetTool((*m_itrCurList)->getLayer()->getToolNamelFromLayer());
		const std::string& strCurToolName = curTool->m_strName;
		PROCESSMODULE->RecordUsedTool(strCurToolName);
		
		// 飞切
		if (curTool->m_bFlightCutting)
		{
			if (!bFirst && lastTool && !lastTool->m_bFlightCutting)
			{
				QG_CuttingInfoWidget::AddCompletedCutCount();
				ChangeEntityState(EntityState::CUTTED, *itrLastPolyline);
			}
			// 如果当前飞行切割块的工具与上一个不同，或块为空，则先处理旧的块
			if (!m_FlightCuttingList.empty() && strCurToolName != strLastFlightToolName) {
				if (!ProcessAndSendFlightCuttingBlock(dOffsetX, dOffsetY)) {
					return false;
				}
			}

			m_FlightCuttingList.push_back(*m_itrCurList);
			ChangeEntityState(EntityState::CUTTING, *m_itrCurList);
			// 记录当前块的工具名
			if (m_FlightCuttingList.size() == 1) {
				strLastFlightToolName = strCurToolName;
			}

			//遍历到最后的情况
			if (m_FlightCuttingList.size() >= MAX_FLIGHT_CUTTING_BATCH_SIZE || (m_itrCurList + 1) == m_CuttingList.end())
			{
				if (!ProcessAndSendFlightCuttingBlock(dOffsetX, dOffsetY)) {
					return false;
				}
			}
			itrLastPolyline = m_itrCurList;
			lastTool = curTool;
			continue;
		}

		if (!m_FlightCuttingList.empty()) 
		{
			if (!ProcessAndSendFlightCuttingBlock(dOffsetX, dOffsetY))
				return false;
		}

		// 构建切割指令（ACS下若已在上一轮SendCommand后预构建则跳过）
		if (!bIsACS || !bACSCommandBuilt)
			BuildCuttingCommand(*m_itrCurList, *curTool, dOffsetX, dOffsetY);
		bACSCommandBuilt = false;
			
		// Polyline 色彩的切换
		if (!bFirst)
		{
			QG_CuttingInfoWidget::AddCompletedCutCount();
			ChangeEntityState(EntityState::CUTTED, *itrLastPolyline);
			//(*itrLastPolyline)->update();
		}

		
		if (!m_bCuttingTest)
			LaserValueChange(*curTool);
		AxisBoundUpdate(*curTool);

		// 发送切割指令
		if (!pMC->SendCommand())
		{
			SHOW_PROCESS_ERROR(ErrorCode::ERROR_CUTTING_SENDCOMMAND);
			g_bStop.store(true);
			g_eState.store(SystemStatus::Error);
			return false;
		}
		
		// ACS流水线优化：在ACS执行当前特征期间，预构建下一个非飞切特征的切割指令
		if (bIsACS)
		{
			auto itrNext = m_itrCurList + 1;
			if (itrNext != m_CuttingList.end())
			{
				Tool* nextTool = ToolFactory::GetTool((*itrNext)->getLayer()->getToolNamelFromLayer());
				if (!nextTool->m_bFlightCutting)
				{
					BuildCuttingCommand(*itrNext, *nextTool, dOffsetX, dOffsetY);
					bACSCommandBuilt = true;
				}
			}
		}

		// 等待切割完成
		if (!IsOffsetCutting())
			return false;
		pMC->PrfTrapAxis();//更改轴为点位运动，只适应固高
		// 更新线条状态/颜色
		ChangeEntityState(EntityState::CUTTING, *m_itrCurList);
		//(*m_itrCurList)->update();
		itrLastPolyline = m_itrCurList;
		lastTool = curTool;
		bFirst = false;
	}

	if (pMC->GetName() == "ACS")
	{
		// 等待切割完成
		if (!IsOffsetCutting())
			return false;
	}

	QG_CuttingInfoWidget::AddCompletedCutCount();
	//切过的Polyline恢复原色
	for (auto e : m_CuttingList)
	{
		ChangeEntityState(CuttingDevice::NONE, e);
	}

	QC_ApplicationWindow::getAppWindow()->getGraphicView()->redraw();
	return true;
}

void CuttingNormal::CreatCuttingData(const int iStart, const int iEnd)
{
	int iStartNum = 0, iEndNum = 1;
	m_CuttingList.clear();

	RS_GraphicView* g = QC_ApplicationWindow::getAppWindow()->getGraphicView();
	QC_MDIWindow*	m = QC_ApplicationWindow::getAppWindow()->getMDIWindow();
	if (g && m && m->getDocument())
	{
		RS_Selection s((RS_EntityContainer&)*m->getDocument(), g);
		QList<RS_Entity*> qlistCut;

		if (s.GetSelectSequenceList().empty())
		{
			if (s.GetSequenceList().empty())
			{
				//全切
				qlistCut = QC_ApplicationWindow::getAppWindow()->getDocument()->getEntityList();
				iEndNum = qlistCut.size();
			}
			else
			{
				// 按顺序切
				qlistCut = s.GetSequenceList();
				iStartNum = iStart == -1 ? 0 : iStart - 1;
				iEndNum = iEnd == -1 ? qlistCut.size() : iEnd;
			}
		}
		else
		{
			// 框选切
			qlistCut = s.GetSelectSequenceList();
			iEndNum = qlistCut.size();
		}

		// 构建切割链表
		for (; iStartNum < iEndNum; iStartNum++)
		{
			auto e = qlistCut[iStartNum];
			if (e->isUndone() || !e->isVisible() || e->isLocked())
			{
				continue;
			}
			switch (e->rtti())
			{
			case RS2::EntityLine:
			case RS2::EntityArc:
			case RS2::EntityCircle:
			case RS2::EntityPolyline:
			{
				if (!CheckEntityLimit(e))
					return;

				m_CuttingList.append((RS_Entity*)e);
				ChangeEntityState(CuttingDevice::SELECT, e);
				break;
			}
			default:
				break;
			}
		}

		m_iTotalNum = m_CuttingList.size();
		g->redraw(RS2::RedrawDrawing);
	}
}

bool CuttingNormal::BuildCuttingCommand(RS_Entity* curEntity, Tool& curTool, double dOffsetX, double dOffsetY)
{
	MotionControl* pMC = QC_ApplicationWindow::getAppWindow()->getService()->GetMotionControl();
	switch (curEntity->rtti())
	{
	case RS2::EntityLine:
	case RS2::EntityArc:
	case RS2::EntityCircle:
	{
		if (pMC->GetName() == "GTN")
			EntityCommandGTN(CuttingDevice::COMMAND_ALL, curEntity, curTool, dOffsetX, dOffsetY);
		else
			EntityCommand(CuttingDevice::COMMAND_ALL, curEntity, curTool, dOffsetX, dOffsetY);
		break;
	}

	case RS2::EntityPolyline:
	{
		RS_EntityContainer* ec = (RS_EntityContainer*)curEntity;
		CuttingDevice::COMMAND_TYPE polylineType;
		for (auto dd : *ec)
		{
			if (ec->count() == 1)
				polylineType = CuttingDevice::COMMAND_ALL;
			else if (dd == ec->first())
				polylineType = CuttingDevice::COMMAND_BEGIN;
			else if (dd == ec->last())
				polylineType = CuttingDevice::COMMAND_END;
			else
				polylineType = CuttingDevice::COMMADN_MID;
			if (pMC->GetName() == "GTN")
				EntityCommandGTN(polylineType, dd, curTool, dOffsetX, dOffsetY);
			else
				EntityCommand(polylineType, dd, curTool, dOffsetX, dOffsetY);
		}
		break;
	}

	default:
		break;
	}
	return true;
}

void CuttingNormal::BuildFlightCuttingCommand(const QList<RS_Entity*>& entityList, double dOffsetX, double dOffsetY)
{
	//处理流程 上一个RS_Entity末点与下一个RS_Entity首点以直线相连，暂时不考虑转角过大、圆弧过度的情况
	MotionControl* pMC = QC_ApplicationWindow::getAppWindow()->getService()->GetMotionControl();
	pMC->ResetProgramCommand();
	QList<RS_Entity*>::const_iterator itr = entityList.begin();
	RS_Vector rsStart = (*itr)->getStartpoint();
	RS2::EntityType curtype = (*itr)->rtti();
	if (curtype == RS2::EntityCircle)
	{
		RS_Circle* cir = (RS_Circle*)(*itr);
		rsStart.x = cir->getCenter().x - cir->getRadius();
		rsStart.y = cir->getCenter().y;
	}
	Tool* curTool = ToolFactory::GetTool((*itr)->getLayer()->getToolNamelFromLayer());
	pMC->SetJumpAccJerk(*curTool);
	pMC->JumpToSetAFPos(*curTool);
	pMC->JumpToIdleHeight(*curTool);
	pMC->JumpToIdleXYPosition(rsStart.x + dOffsetX, rsStart.y + dOffsetY, *curTool);
	pMC->JumpToCuttingHeight(*curTool);

	int i = 0;
	for (; itr != entityList.end(); itr++, i++)
	{
		rsStart = (*itr)->getStartpoint();
		//转角角度、速度等参数，只需设置一次
		if (itr == entityList.begin())
		{
			pMC->BeginACSSegmentForFlightCutting(*curTool);	// ACS需设置
		}
		else
		{
			pMC->OffsetFlightLineTo(rsStart.x + dOffsetX, rsStart.y + dOffsetY, *curTool, true);
		}
		pMC->SetShutterOnOffWaitTime((*curTool).m_dBeforeOn, (*curTool).m_dAfterOn, (*curTool).m_dBeforeOff, (*curTool).m_dAfterOff, (*curTool).m_dBlowDelay);
		switch ((*itr)->rtti())
		{
		case RS2::EntityLine:
		case RS2::EntityArc:
		case RS2::EntityCircle:
		{
			EntityFlightCuttingCommand(CuttingDevice::COMMAND_ALL, *itr, *curTool, dOffsetX, dOffsetY);
			break;
		}

		case RS2::EntityPolyline:
		{
			RS_EntityContainer* ec = (RS_EntityContainer*)(*itr);
			CuttingDevice::COMMAND_TYPE polylineType;
			for (auto dd : *ec)
			{
				if (ec->count() == 1)
					polylineType = CuttingDevice::COMMAND_ALL;
				else if (dd == ec->first())
					polylineType = CuttingDevice::COMMAND_BEGIN;
				else if (dd == ec->last())
					polylineType = CuttingDevice::COMMAND_END;
				else
					polylineType = CuttingDevice::COMMADN_MID;

				EntityFlightCuttingCommand(polylineType, dd, *curTool, dOffsetX, dOffsetY);
			}
			break;
		}

		default:
			break;
		}
	}
	pMC->EndProgramCommandForFlightCutting(*curTool);
}

bool CuttingNormal::EntityCommand(CuttingDevice::COMMAND_TYPE ctype, RS_Entity* curEntity, Tool& curTool, double dOffsetX, double dOffsetY)
{
	MotionControl* pMC = QC_ApplicationWindow::getAppWindow()->getService()->GetMotionControl();
	RS2::EntityType curtype = curEntity->rtti();
	if (ctype == CuttingDevice::COMMAND_BEGIN || ctype == CuttingDevice::COMMAND_ALL)
	{
		RS_Vector rsStart = curEntity->getStartpoint();
		if (curtype == RS2::EntityCircle)
		{
			RS_Circle* cir = (RS_Circle*)curEntity;
			rsStart.x = cir->getCenter().x - cir->getRadius();
			rsStart.y = cir->getCenter().y;
		}
		//指令构建，
		pMC->ResetProgramCommand();
		pMC->SetJumpAccJerk(curTool);
		pMC->JumpToSetAFPos(curTool);
		if (curTool.m_bCuttingHead)
		{
			MovingCuttingHead(curTool, rsStart.x + dOffsetX, rsStart.y + dOffsetY);
		}
		else
		{
			pMC->JumpToIdleHeight(curTool);
			pMC->JumpToIdleXYPosition(rsStart.x + dOffsetX, rsStart.y + dOffsetY, curTool);
			pMC->JumpToCuttingHeight(curTool);
		}
		
		pMC->SetShutterOnOffWaitTime(curTool.m_dBeforeOn, curTool.m_dAfterOn, curTool.m_dBeforeOff, curTool.m_dAfterOff, curTool.m_dBlowDelay);
		if (!m_bCuttingTest)
			pMC->ProLaserControl(true, false, curTool, false);
		pMC->BeginACSSegment(curTool);
	}

	switch (curtype)
	{
	case RS2::EntityLine:
	{
		RS_Line* l = (RS_Line*)curEntity;
		if (l->getStartpoint() == l->getEndpoint())
			break;
		pMC->OffsetLineTo(l->getEndpoint().x + dOffsetX, l->getEndpoint().y + dOffsetY, curTool);
		break;
	}
	
	case RS2::EntityArc:
	{
		RS_Arc* a = (RS_Arc*)curEntity;
		if (std::abs(a->getRadius() < 0.001))
			break;
		bool bclockwise	= a->isReversed();
		double dValue1	= a->getAngle1();
		double dValue2	= a->getAngle2();
		double dValueAngle = 0;
		if (bclockwise) 
		{
			if (dValue2 - dValue1 > 0)
				dValueAngle = dValue2 - dValue1 - (2 * M_PI);
			else
				dValueAngle = dValue2 - dValue1;
		}
		else
		{
			if (dValue2 - dValue1 < 0)
				dValueAngle = dValue2 + (2 * M_PI) - dValue1;
			else
				dValueAngle = dValue2 - dValue1;
		}
		
		pMC->OffsetArc2To(a->getEndpoint().x, a->getEndpoint().y, a->getCenter().x + dOffsetX, a->getCenter().y + dOffsetY, dValueAngle, curTool);
		break;
	}
	
	case RS2::EntityCircle:
	{
		RS_Circle* c = (RS_Circle*)curEntity;
		if (std::abs(c->getRadius() < 0.001))
			break;
		pMC->OffsetArc2To(c->getEndpoint().x, c->getEndpoint().y, c->getCenter().x + dOffsetX, c->getCenter().y + dOffsetY, 2 * M_PI, curTool);
		break;
	}
	
	default:
		break;
	}
	if (ctype == CuttingDevice::COMMAND_END || ctype == CuttingDevice::COMMAND_ALL)
	{
		if (!m_bCuttingTest)
			pMC->ProLaserControl(false, true, curTool, false);
		pMC->EndProgramCommand(curTool);
	}
	return true;
}

bool CuttingNormal::EntityCommandGTN(CuttingDevice::COMMAND_TYPE ctype, RS_Entity* curEntity, Tool& curTool, double dOffsetX /*= 0*/, double dOffsetY /*= 0*/)
{
	MotionControl* pMC = QC_ApplicationWindow::getAppWindow()->getService()->GetMotionControl();
	RS2::EntityType curtype = curEntity->rtti();
	if (ctype == CuttingDevice::COMMAND_BEGIN || ctype == CuttingDevice::COMMAND_ALL)
	{
		RS_Vector rsStart = curEntity->getStartpoint();
		if (curtype == RS2::EntityCircle)
		{
			RS_Circle* cir = (RS_Circle*)curEntity;
			rsStart.x = cir->getCenter().x - cir->getRadius();
			rsStart.y = cir->getCenter().y;
		}
		//指令构建，
		pMC->SetJumpAccJerk(curTool);
		pMC->JumpToSetAFPos(curTool);
		if (curTool.m_bCuttingHead)
		{
			MovingCuttingHead(curTool, rsStart.x + dOffsetX, rsStart.y + dOffsetY);
		}
		else
		{
			pMC->JumpToIdleHeight(curTool);
			pMC->JumpToIdleXYPosition(rsStart.x + dOffsetX, rsStart.y + dOffsetY, curTool);
			pMC->JumpToCuttingHeight(curTool);
		}

		pMC->SetShutterOnOffWaitTime(curTool.m_dBeforeOn, curTool.m_dAfterOn, curTool.m_dBeforeOff, curTool.m_dAfterOff, curTool.m_dBlowDelay);
		//创建前瞻之前m_iGTNCuttingCount置零
		m_iGTNCuttingCount = 0;
		m_bStartGTNCommand = false;
		//创建前瞻
		if (!pMC->InitCrd(curTool))
			return false;
		if (!m_bCuttingTest)
			pMC->ProLaserControl(true, false, curTool, true);
	}

	switch (curtype)
	{
	case RS2::EntityLine:
	{
		RS_Line* l = (RS_Line*)curEntity;
		if (l->getStartpoint() == l->getEndpoint())
			break;
		pMC->OffsetLineTo(l->getEndpoint().x + dOffsetX, l->getEndpoint().y + dOffsetY, curTool);
		m_iGTNCuttingCount++;
		if (m_iGTNCuttingCount > 1000 && !m_bStartGTNCommand)
			GTNStartCommand();
		break;
	}

	case RS2::EntityArc:
	{
		RS_Arc* a = (RS_Arc*)curEntity;
		if (std::abs(a->getRadius() < 0.001))
			break;
		bool bclockwise = a->isReversed();
		pMC->OffsetArcTo(a->getEndpoint().x, a->getEndpoint().y, a->getCenter().x + dOffsetX, a->getCenter().y + dOffsetY,bclockwise, curTool, 0, 0);
		m_iGTNCuttingCount++;
		if (m_iGTNCuttingCount > 1000)
			GTNStartCommand();
		break;
	}

	case RS2::EntityCircle:
	{
		RS_Circle* c = (RS_Circle*)curEntity;
		bool bclockwise = true;
		if (std::abs(c->getRadius() < 0.001))
			break;

		double dR = c->getRadius();
		pMC->OffsetArcTo(c->getCenter().x + dR, c->getCenter().y, c->getCenter().x + dOffsetX, c->getCenter().y + dOffsetY, bclockwise, curTool, 0, 0);
		m_iGTNCuttingCount++;
		if (m_iGTNCuttingCount > 1000)
			GTNStartCommand();
		pMC->OffsetArcTo(c->getCenter().x - dR, c->getCenter().y, c->getCenter().x + dOffsetX, c->getCenter().y + dOffsetY, bclockwise, curTool, 0, 0);
		m_iGTNCuttingCount++;
		if (m_iGTNCuttingCount > 1000)
			GTNStartCommand();
		break;
	}
	default:
		break;
	}
	if (ctype == CuttingDevice::COMMAND_END || ctype == CuttingDevice::COMMAND_ALL)
	{
		if (!m_bCuttingTest)
			pMC->ProLaserControl(false, true, curTool, true);
		GTNStartCommand();
	}
	return true;
}

void CuttingNormal::GTNStartCommand()
{
	if (!m_bStartGTNCommand)
	{
		MotionControl* pMC = QC_ApplicationWindow::getAppWindow()->getService()->GetMotionControl();
		// 先将软件前瞻缓冲区刷入硬件FIFO，再条件性启动插补运动
		// 对应 C# 参考：do{GTN_CrdDataEx}while(Z!=0) + if(pRun==0) GTN_CrdStart
		pMC->SendCommand();
		m_iGTNCuttingCount = 0;
		m_bStartGTNCommand = true;
	}
}

bool CuttingNormal::EntityFlightCuttingCommand(CuttingDevice::COMMAND_TYPE ctype, RS_Entity* curEntity, Tool& curTool, double dOffsetX /*= 0*/, double dOffsetY /*= 0*/)
{
	MotionControl* pMC = QC_ApplicationWindow::getAppWindow()->getService()->GetMotionControl();
	RS2::EntityType curtype = curEntity->rtti();
	if (ctype == CuttingDevice::COMMAND_BEGIN || ctype == CuttingDevice::COMMAND_ALL)
	{
		RS_Vector rsStart = curEntity->getStartpoint();
		if (curtype == RS2::EntityCircle)
		{
			RS_Circle* cir = (RS_Circle*)curEntity;
			rsStart.x = cir->getCenter().x - cir->getRadius();
			rsStart.y = cir->getCenter().y;
		}
	}

	switch (curtype)
	{
	case RS2::EntityLine:
	{
		RS_Line* l = (RS_Line*)curEntity;
		if (l->getStartpoint() == l->getEndpoint())
			break;
		pMC->OffsetFlightLineTo(l->getEndpoint().x + dOffsetX, l->getEndpoint().y + dOffsetY, curTool, false);
		break;
	}

	case RS2::EntityArc:
	{
		RS_Arc* a = (RS_Arc*)curEntity;
		if (std::abs(a->getRadius() < 0.001))
			break;
		bool bclockwise = a->isReversed();
		double dValue1 = a->getAngle1();
		double dValue2 = a->getAngle2();
		double dValueAngle = 0;
		if (bclockwise)
		{
			if (dValue2 - dValue1 > 0)
				dValueAngle = dValue2 - dValue1 - (2 * M_PI);
			else
				dValueAngle = dValue2 - dValue1;
		}
		else
		{
			if (dValue2 - dValue1 < 0)
				dValueAngle = dValue2 + (2 * M_PI) - dValue1;
			else
				dValueAngle = dValue2 - dValue1;
		}
		pMC->OffsetFlightArc2To(a->getCenter().x + dOffsetX, a->getCenter().y + dOffsetY, dValueAngle, curTool, false);
		break;
	}

	case RS2::EntityCircle:
	{
		RS_Circle* c = (RS_Circle*)curEntity;
		if (std::abs(c->getRadius() < 0.001))
			break;
		pMC->OffsetFlightArc2To(c->getCenter().x + dOffsetX, c->getCenter().y + dOffsetY, 2 * M_PI, curTool, false);
		break;
	}

	default:
		break;
	}
	return true;
}

bool CuttingNormal::MovingCuttingHead(Tool& curTool, double ptFirstX, double ptFirstY)
{
	MotionControl* pMC = QC_ApplicationWindow::getAppWindow()->getService()->GetMotionControl();
	if (curTool.m_strName == m_strName)                        
		m_bChangeTool = false;
	else
		m_bChangeTool = true;
	if (curTool.m_bCrossBridge)
	{
		pMC->StopMovingCuttingHead();
		pMC->JumpToIdleHeight(curTool);
		pMC->JumpToIdleXYPosition(ptFirstX, ptFirstY, curTool);
		pMC->JumpToCuttingHeight(curTool);
		pMC->StartMovingCuttingHead(curTool);
	}
	else
	{
		if (m_bChangeTool)
		{
			pMC->JumpToIdleXYPosition(ptFirstX, ptFirstY, curTool);
			pMC->StartMovingCuttingHead(curTool);  
		}
		else
		{
			pMC->JumpToIdleXYPosition(ptFirstX, ptFirstY, curTool);
		}
	} 
	m_strName = curTool.m_strName;
	return true;
}

bool CuttingNormal::ProcessAndSendFlightCuttingBlock(double dOffsetX, double dOffsetY)
{
	MotionControl* pMC = QC_ApplicationWindow::getAppWindow()->getService()->GetMotionControl();
	if (m_FlightCuttingList.empty()) {
		return true; // 无需处理
	}

	// 获取飞行切割块中第一个实体的工具，整个块应使用同一工具
	Tool* curTool = ToolFactory::GetTool(m_FlightCuttingList.front()->getLayer()->getToolNamelFromLayer());

	// 构建飞行切割指令
	BuildFlightCuttingCommand(m_FlightCuttingList, dOffsetX, dOffsetY);

	// 等待上一个偏移切割完成（如果存在）
	if (!IsOffsetCutting()) {
		return false;
	}

	// 更新激光参数和加工边界
	if (!m_bCuttingTest) {
		LaserValueChange(*curTool);
	}
	AxisBoundUpdate(*curTool);

	// 发送切割指令
	if (!pMC->SendCommand()) {
		SHOW_PROCESS_ERROR(ErrorCode::ERROR_CUTTING_SENDCOMMAND);
		g_bStop.store(true);
		g_eState.store(SystemStatus::Error);
		return false;
	}

	// 优化点1: 修复了原代码中的Bug，正确更新每个飞行切割实体的状态
	for (auto* pEntity : m_FlightCuttingList) {
		QG_CuttingInfoWidget::AddCompletedCutCount();
		ChangeEntityState(EntityState::CUTTED, pEntity);
	}

	m_FlightCuttingList.clear();
	return true;
}
