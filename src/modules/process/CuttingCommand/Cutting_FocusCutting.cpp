#include "Cutting_FocusCutting.h"
#include "qc_applicationwindow.h"
#include "rs_line.h"
#include "rs_arc.h"
#include "rs_circle.h"
#include "rs_polyline.h"
#include "rs_selection.h"
#include "qc_mdiwindow.h"
#include "qg_CuttingInfoWidget.h"

FocusCutting::FocusCutting()
{
}

bool FocusCutting::StartCutting(const map<QString, QString>& maps)
{
	m_iFocus0Sum		= 0;
	m_iCurrentNum		= 0;
	m_iTotalNum			= 0;
	m_iCurCuttingNumber = 1;
	m_dHighCompensate	= 0;
	m_iStepInterval		= maps.at("StepInterval").toInt();
	m_dEnergyStep		= maps.at("EnergyStep").toDouble();
	m_dFrequencyStep	= maps.at("FrequencyStep").toDouble();
	m_dPulseWidthStep	= maps.at("PulseWidthStep").toDouble();
	m_dCuttingHighStep	= maps.at("CuttingHighStep").toDouble();

	CreatCuttingListDucument();
	if (m_iFocus0Sum % 2 == 0 && m_iFocus0Sum != 0)
	{
		SHOW_PROCESS_ERROR(ErrorCode::ERROR_AUTOFOCUS_NOODDNUMBER);
		g_bStop.store(true);
		g_bFinish.store(true);
		return false;
	}

	if (m_CuttingList.isEmpty())
	{
		SHOW_PROCESS_ERROR(ErrorCode::ERROR_CUTTING_LISTEMPTY);
		g_bStop.store(true);
		g_bFinish.store(true);
		g_eState.store(SystemStatus::Error);
		return false;
	}

	QG_CuttingInfoWidget::SetTotalCutCount(m_iTotalNum);
	m_itrCurList = m_CuttingList.begin();

	bool bRetuen = CuttingThread();
	return bRetuen;
}

bool FocusCutting::CuttingThread(double dOffsetX, double dOffsetY)
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
	bool bFirstCount = true;
	QList<RS_Entity*>::iterator itrLastPolyline = m_itrCurList;
	MotionControl* pMC = QC_ApplicationWindow::getAppWindow()->getService()->GetMotionControl();
	for (; m_itrCurList != m_CuttingList.end(); m_itrCurList++)
	{	
		// 激光器参数的下发
		string strToolName = (*m_itrCurList)->getLayer()->getToolNamelFromLayer().toStdString();
		Tool* curTool = ToolFactory::GetTool(strToolName);
		PROCESSMODULE->RecordUsedTool(strToolName);
		if ((*m_itrCurList)->getLayer()->getName() == "Focus|")
		{
			// 补偿为逐步递增至高度再递增，
			// 如短竖线总数为9，则中位为5。
			// 如果总数/2取余为1。则商+1为标准的高度。
			// 下的刀数序号与标准的插值，向下取整即为补偿次数。
			int iSteps = (m_iCurCuttingNumber - ((m_iFocus0Sum / 2) + 1)) / m_iStepInterval;
			double dCurEnergy		= curTool->m_dLaserEnergy		+ iSteps * m_dEnergyStep;
			double dCurFrequency	= curTool->m_dLaserFrequency	+ iSteps * m_dFrequencyStep;
			double dCurPulseWidth	= curTool->m_dLaserPulseWidth	+ iSteps * m_dPulseWidthStep;
			m_dHighCompensate = iSteps * m_dCuttingHighStep;
			LOG_PROCESS_INFO(QObject::tr("[AutoFocus] Set cutting high compensate %1").arg(m_dHighCompensate).toUtf8().data());
			if (!m_bCuttingTest)
			{
				LaserParameter parameter(dCurEnergy, dCurFrequency, dCurPulseWidth,
					curTool->m_dLaserAttenuatorPercentage, curTool->m_dLaserPpDivider,
					curTool->m_iLaserDelay);
				LaserValueChange(parameter);
			}
			m_iCurCuttingNumber++;
		}
		else if ((*m_itrCurList)->getLayer()->getName() == "Focus-")
		{
			if (!m_bCuttingTest)
				LaserValueChange(*curTool);
			m_dHighCompensate = 0;
		}

		// 构建切割指令
		BuildCuttingCommand(*m_itrCurList, dOffsetX, dOffsetY);

		if (pMC->GetName() == "ACS")
		{
			// 等待切割完成
			if (!IsOffsetCutting())
				return false;
		}
		
		if (bFirstCount)
			bFirstCount = false;
		else
			QG_CuttingInfoWidget::AddCompletedCutCount();

		// Polyline 色彩的切换
		if (!bFirst)
		{
			QG_CuttingInfoWidget::AddCompletedCutCount();
			ChangeEntityState(EntityState::CUTTED, *itrLastPolyline);
			//(*itrLastPolyline)->update();
		}

		//加工边界更改
		AxisBoundUpdate(*curTool);

		// 发送切割指令
		if (!pMC->SendCommand())
		{
			SHOW_PROCESS_ERROR(ErrorCode::ERROR_CUTTING_SENDCOMMAND);
			g_bStop.store(true);
			g_eState.store(SystemStatus::Error);
			return false;
		}
		if (pMC->GetName() == "GTN")
		{
			// 等待切割完成
			if (!IsOffsetCutting())
				return false;
		}
		ChangeEntityState(EntityState::CUTTING, *m_itrCurList);
		//(*m_itrCurList)->update();
		itrLastPolyline = m_itrCurList;
		bFirst = false;
	}

	if (pMC->GetName() == "ACS")
	{
		// 等待切割完成
		if (!IsOffsetCutting())
			return false;
	}

	//切过的Polyline恢复原色
	for (auto e : m_CuttingList)
	{
		ChangeEntityState(CuttingDevice::NONE, e);
	}

	QC_ApplicationWindow::getAppWindow()->getGraphicView()->redraw();
	return true;
}

void FocusCutting::CreatCuttingListDucument()
{
	m_CuttingList.clear();
	RS_GraphicView* g = QC_ApplicationWindow::getAppWindow()->getGraphicView();
	QC_MDIWindow*	m = QC_ApplicationWindow::getAppWindow()->getMDIWindow();
	if (g && m && m->getDocument())
	{
		RS_Selection s((RS_EntityContainer&)*m->getDocument(), g);
		QList<RS_Entity*> qlistCut;

		if (s.GetSelectSequenceList().empty()) // 不框选
			qlistCut = QC_ApplicationWindow::getAppWindow()->getDocument()->getEntityList();
		else		// 框选切
			qlistCut = s.GetSelectSequenceList();

		// 构建切割链表
		QList<RS_Entity*>::iterator e = qlistCut.begin();
		for (; e != qlistCut.end(); e++)
		{
			if ((*e)->isUndone() || !(*e)->isVisible() || (*e)->isLocked())
			{
				continue;
			}
			switch ((*e)->rtti())
			{
			case RS2::EntityLine:
			case RS2::EntityArc:
			case RS2::EntityCircle:
			case RS2::EntityPolyline:
			{
				if (!CheckEntityLimit((*e)))
					return;

				// 所在图层判定
				if (!((*e)->getLayer()->getName() == "Focus|" ||
					(*e)->getLayer()->getName() == "Focus-"))
					continue;

				if ((*e)->getLayer()->getName() == "Focus|")	
					m_iFocus0Sum++;

				m_CuttingList.append((RS_Entity*)(*e));
				ChangeEntityState(CuttingDevice::SELECT, *e);
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

bool FocusCutting::BuildCuttingCommand(RS_Entity* curEntity, double dOffsetX, double dOffsetY)
{
	RS2::EntityType curType = curEntity->rtti();
	string strToolName = curEntity->getLayer()->getToolNamelFromLayer().toStdString();
	Tool* curTool = ToolFactory::GetTool(strToolName);
	switch (curType)
	{
	case RS2::EntityLine:
	case RS2::EntityArc:
	case RS2::EntityCircle:
	{
		EntityCommand(CuttingDevice::COMMAND_ALL, curEntity, *curTool, dOffsetX, dOffsetY);
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

			EntityCommand(polylineType, dd, *curTool, dOffsetX, dOffsetY);
		}
		break;
	}

	default:
		break;
	}
	return true;
}

bool FocusCutting::EntityCommand(CuttingDevice::COMMAND_TYPE ctype, RS_Entity* curEntity, Tool& curTool, double dOffsetX, double dOffsetY)
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
		//pMC->JumpToIdleHeight(curTool);
		pMC->JumpToIdleXYPosition(rsStart.x + dOffsetX, rsStart.y + dOffsetY, curTool);
		pMC->JumpToCuttingHeight(curTool, m_dHighCompensate);
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
		pMC->OffsetLineTo(l->getEndpoint().x + dOffsetX, l->getEndpoint().y + dOffsetY, curTool);
		break;
	}
	default: break;
	}

	if (ctype == CuttingDevice::COMMAND_END || ctype == CuttingDevice::COMMAND_ALL)
	{
		pMC->ProLaserControl(false, true, curTool, false);
		pMC->EndProgramCommand(curTool);
	}
	return true;
}

void FocusCutting::SetEntitysLeftLimitPos()
{
	double dMinx = 0;
	RS_GraphicView* g = QC_ApplicationWindow::getAppWindow()->getGraphicView();
	QC_MDIWindow*	m = QC_ApplicationWindow::getAppWindow()->getMDIWindow();
	if (g && m && m->getDocument())
	{
		RS_Selection s((RS_EntityContainer&)*m->getDocument(), g);
		QList<RS_Entity*> qlistCut;

		if (s.GetSelectSequenceList().empty()) // 不框选
			qlistCut = QC_ApplicationWindow::getAppWindow()->getDocument()->getEntityList();
		else		// 框选切
			qlistCut = s.GetSelectSequenceList();

		// 构建切割链表
		bool bFirst = true;
		QList<RS_Entity*>::iterator e = qlistCut.begin();
		for (; e != qlistCut.end(); e++)
		{
			if ((*e)->isUndone() || !(*e)->isVisible())
				continue;

			if ((*e)->rtti() != RS2::EntityPolyline)
				continue;

			if (!((*e)->getLayer()->getName() == "Focus|" ||
				(*e)->getLayer()->getName() == "Focus-"))
				continue;

			if (!CheckEntityLimit((*e)))
				return;

			if (bFirst)
			{
				dMinx = (*e)->getMin().x;
				bFirst = false;
			}
			dMinx = (dMinx < (*e)->getMin().x) ? dMinx : (*e)->getMin().x;
		}
	}
	m_dMinX = dMinx;
	m_bComputeMinX = false;
}
