#include "CuttingDevice.h"
#include "qc_applicationwindow.h"
#include "qc_mdiwindow.h"
#include "qg_graphicview.h"
//#include "rs_graphic.h"
#include "rs_layerlist.h"
#include "rs_selection.h"
#include "Service.h"

void CuttingDevice::AxisBoundUpdate(const Tool& curTool)
{
	if (curTool.m_strDirectionX == "X")
		m_eDirectionX = Axis::X;
	else
		m_eDirectionX = enum_cast<Axis>(curTool.m_strDirectionX.toStdString()).value_or(Axis::X);

	if (curTool.m_strDirectionY == "Y")
		m_eDirectionY = Axis::Y;
	else if (curTool.m_strDirectionY == "A")
		m_eDirectionY = Axis::A;
	else
		m_eDirectionY = enum_cast<Axis>(curTool.m_strDirectionY.toStdString()).value_or(Axis::Y);
}

void CuttingDevice::SetDirectionX(QString qstr)
{
	if (qstr == "X")
		m_eDirectionX = Axis::X;
	else
		m_eDirectionX = enum_cast<Axis>(qstr.toStdString()).value_or(Axis::X);
}

Axis CuttingDevice::GetDirectionX()
{
	return m_eDirectionX;
}

void CuttingDevice::SetDirectionY(QString qstr)
{
	if (qstr == "A")
		m_eDirectionY = Axis::A;
	else if (qstr == "Y")
		m_eDirectionY = Axis::Y;
	else
		m_eDirectionY = enum_cast<Axis>(qstr.toStdString()).value_or(Axis::Y);
}

Axis CuttingDevice::GetDirectionY()
{
	return m_eDirectionY;
}

void CuttingDevice::LaserValueChange(const Tool& curTool)
{
	LaserParameter parameter(curTool.m_dLaserEnergy, curTool.m_dLaserFrequency, curTool.m_dLaserPulseWidth,
		curTool.m_dLaserAttenuatorPercentage, curTool.m_dLaserPpDivider, curTool.m_iLaserDelay);
	LaserValueChange(parameter);
}

void CuttingDevice::LaserValueChange(const LaserParameter& parameter)
{
	MotionControl* pMC = QC_ApplicationWindow::getAppWindow()->getService()->GetMotionControl();
	LaserDevice* pLaserDevice = QC_ApplicationWindow::getAppWindow()->getService()->GetLaserDevice();
	if (pMC->GetName() != "GTN" && !LaserDevice::IsChanged(parameter))
		return;

	LOG_PROCESS_INFO(QObject::tr("[Laser] Set Laser energy %1 frequency %2 pulse width %3")
		.arg(parameter.dEnergy).arg(parameter.dFrequency).arg(parameter.dPulseWidth).toUtf8().data());

	bool bSignalSource = false;
	SETTINGS->GetKeyValue("bSignal", bSignalSource, SettingSection::Laser, "SignalSource");
	if (bSignalSource)
	{
		LaserEnergySet(parameter.dEnergy);
		if (pMC->GetName() == "GTN")
		{
			if (!pMC->GSN_SetLaserParameterApplication(parameter.dFrequency, parameter.dPulseWidth, 0.0))
			{
				SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_LASERSETFAILED);
				g_bStop.store(true);
				return;
			}
		}
		else
		{
			SignalSource* pSignalSource = QC_ApplicationWindow::getAppWindow()->getService()->GetSignalSource();
			if (!pSignalSource->SetFrequencyAndPulseWidth(parameter.dFrequency, parameter.dPulseWidth))
			{
				SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_LASERSETFAILED);
				g_bStop.store(true);
				return;
			}
		}
		LaserDevice::UpdateLaserParameter(parameter);
	}
	else
	{
		bool bOk = pLaserDevice->SetLaserParameter(parameter);

		if (!bOk)
		{
			SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_LASERSETFAILED);
			g_bStop.store(true);
			return;
		}
	}
}

void CuttingDevice::LaserEnergySet(double dEnergy)
{
	LaserDevice* pLaserDevice = QC_ApplicationWindow::getAppWindow()->getService()->GetLaserDevice();
	double dResolution = 0.0;
	SETTINGS->GetKeyValue("fResolution", dResolution, SettingSection::Laser, "Laser");
	if (pLaserDevice->GetName() == "AnalogControl" && dResolution > 0.0)
	{
		//根据百分比计算下发的激光模拟量参数
		double dValue = dEnergy / 100.0 * dResolution;
		MotionControl* pMC = QC_ApplicationWindow::getAppWindow()->getService()->GetMotionControl();
		if (!pMC->AnalogOutputSet(AnalogOUT::Laser, dValue, true))
		{
			SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_LASERSETFAILED);
			g_bStop.store(true);
			return;
		}
		LaserDevice::UpdateEnergy(dEnergy);
	}
	else if (!pLaserDevice->SetEnergy(dEnergy))
	{
		SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_LASERSETFAILED);
		g_bStop.store(true);
		return;
	}
}

bool CuttingDevice::WaitTime(int iWaitTime)
{
	int iWaitTimes = 0;
	int iWaitTimesTotal = iWaitTime / 30;

	do
	{
		if (g_bPause.load())
		{
			while (g_bPause.load() && !g_bStop.load())
			{
				boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
				continue;
			}

			if (g_bStop.load())	return false;
		}

		if (g_bStop.load())		return false;

		boost::this_thread::sleep_for(boost::chrono::milliseconds(30));
		iWaitTimes++;
	} while (iWaitTimes < iWaitTimesTotal);
	return true;
}

bool CuttingDevice::IsOffsetCutting(int iOverTime)
{
	MotionControl* pMC = QC_ApplicationWindow::getAppWindow()->getService()->GetMotionControl();
	int iOverTimes = iOverTime * 20;	// 转50ms次数，太低也影响暂停、停止按钮的响应
	do
	{
		if (g_bPause.load())
		{
			while (g_bPause.load() && !g_bStop.load())
			{
				if (pMC->IsOffsetCutting())
					g_bPausing.store(true);
				else
					g_bPausing.store(false);

				boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
				continue;
			}

			g_bPausing.store(false);
			if (g_bStop.load())
			{
				pMC->StopAllBuffer();
				pMC->StopMotion();
				return false;
			}
		}

		if (g_bStop.load())
		{
			pMC->StopAllBuffer();
			pMC->StopMotion();
			return false;
		}

		boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
		if (iOverTime) 
		{
			if (!iOverTimes)
			{
				SHOW_PROCESS_ERROR(ErrorCode::ERROR_PROCESS_LASERSETFAILED);
				g_bStop.store(true);
				return false;
			}
			iOverTimes--;
		}

	} while (pMC->IsOffsetCutting());
	return true;
}

void CuttingDevice::ResetLeftLimitPos()
{
	m_bComputeMinX = true;
	m_dMinX = 0;
}

void CuttingDevice::SetEntitysLeftLimitPos()
{
	if (m_bComputeMinX)
	{
		RS_Vector vecMin, vecMax;
		GetCuttingBound(vecMin, vecMax);
		m_dMinX = vecMin.x;
		m_bComputeMinX = false;
	}
}

double CuttingDevice::GetEntitysLeftLimitPos()
{
	SetEntitysLeftLimitPos();
	return m_dMinX;
}

void CuttingDevice::GetCuttingBound(RS_Vector& vecMin, RS_Vector& vecMax)
{
	// 计算最大Bound的范围，分两种情况：
	// 1、框选切割，  计算框选部分的范围；
	// 2、非框选切割，有顺序链表时，以链表为准，没链表时，计算整个图纸的范围；
	QList<RS_Entity*> qlistCut;
	GetCuttingBound(vecMin, vecMax, qlistCut);
}

void CuttingDevice::GetCuttingBound(RS_Vector& vecMin, RS_Vector& vecMax, QList<RS_Entity*>& qlistCut)
{
	if (qlistCut.empty())
	{
		RS_GraphicView*		g = QC_ApplicationWindow::getAppWindow()->getGraphicView();
		QC_MDIWindow*		m = QC_ApplicationWindow::getAppWindow()->getMDIWindow();
		if (g && m && m->getDocument())
		{
			RS_Selection s((RS_EntityContainer&)*m->getDocument(), g);
			QList<RS_Entity*> qlistSelect = s.GetSelectSequenceList();
			if (qlistSelect.empty())
			{
				QList<RS_Entity*> qlistSquence = s.GetSequenceList();
				qlistCut = qlistSquence.empty() ?
					QC_ApplicationWindow::getAppWindow()->getDocument()->getEntityList() : qlistSquence;
			}
			else
			{
				qlistCut = qlistSelect;
			}
		}
	}
	GetBound(vecMin, vecMax, qlistCut);
}

void CuttingDevice::GetBound(RS_Vector& vecMin, RS_Vector& vecMax, const QList<RS_Entity*>& qlist)
{
	if (qlist.empty())
	{
		vecMin = RS_Vector(0, 0);
		vecMax = RS_Vector(0, 0);
		return;
	}

	QList<RS_Entity*>::const_iterator itor = qlist.begin();
	RS_Vector spLL = (*itor)->getMin();
	RS_Vector spUR = (*itor)->getMax();

	for (; itor != qlist.end(); itor++)
	{
		spUR.x = (spUR.x > (*itor)->getMax().x) ? spUR.x : (*itor)->getMax().x;
		spUR.y = (spUR.y > (*itor)->getMax().y) ? spUR.y : (*itor)->getMax().y;
		spLL.x = (spLL.x < (*itor)->getMin().x) ? spLL.x : (*itor)->getMin().x;
		spLL.y = (spLL.y < (*itor)->getMin().y) ? spLL.y : (*itor)->getMin().y;
	}
	vecMin = spLL;
	vecMax = spUR;
}

bool CuttingDevice::CheckEntityLimit(RS_Entity* e)
{
	Tool* entityTool = ToolFactory::GetTool(e->getLayer(false)->getToolNamelFromLayer());
	Axis eDirectionX = enum_cast<Axis>(entityTool->m_strDirectionX).value();
	Axis eDirectionY = enum_cast<Axis>(entityTool->m_strDirectionY).value();
	RS_Vector entityLL = e->getMin();
	RS_Vector entityRR = e->getMax();
	MotionControl* pMC = QC_ApplicationWindow::getAppWindow()->getService()->GetMotionControl();
	double lx, ly, rx, ry;
	pMC->ReadAxisSoftLimit(eDirectionX, lx, rx);
	pMC->ReadAxisSoftLimit(eDirectionY, ly, ry);
	if ((entityLL.x < lx || entityLL.y < ly || 
		 entityRR.x > rx || entityRR.y > ry))
	{
		SHOW_PROCESS_ERROR(ErrorCode::ERROR_CUTTING_OUTOFLIMIT);
		g_bStop.store(true);
		g_bFinish.store(true);
		return false;
	}
	return true;
}

void CuttingDevice::ChangeEntityState(EntityState eState, RS_Entity* e)
{
	if (!e)		return;

	switch (eState)
	{
	case CuttingDevice::NONE:
		e->setPen(RS_Pen(RS_Color(RS2::FlagByLayer), RS2::WidthByLayer, RS2::LineByLayer));
		break;
	case CuttingDevice::SELECT:
		// 构建切割数据时候不做图纸刷新重绘
		e->setPen(RS_Pen(RS_Color(255, 0, 0), RS2::WidthByLayer, RS2::LineByLayer));
		break;
	case CuttingDevice::CUTTING:
	{
		e->setPen(RS_Pen(RS_Color(0, 255, 0), RS2::WidthByLayer, RS2::LineByLayer));
		QC_MDIWindow* m = QC_ApplicationWindow::getAppWindow()->getMDIWindow();
		if (m)
		{
			QG_GraphicView* g = m->getGraphicView();
			if (g)	g->RedrawDrawing();
		}
		break;
	}
	case CuttingDevice::CUTTED:
	{
		e->setPen(RS_Pen(RS_Color(255, 255, 255), RS2::WidthByLayer, RS2::LineByLayer));
		QC_MDIWindow* m = QC_ApplicationWindow::getAppWindow()->getMDIWindow();
		if (m)
		{
			QG_GraphicView* g = m->getGraphicView();
			if (g)	g->RedrawDrawing();
		}
		break;
	}
	default:
		break;
	}
}

bool CuttingDevice::IsEntityList()
{
	return QC_ApplicationWindow::getAppWindow()->getDocument()->getEntityList().size();
}

bool CuttingDevice::IsSelectionCutting()
{
	RS_EntityContainer* d = QC_ApplicationWindow::getAppWindow()->getDocument();
	QList<RS_Entity*> qlistSelect = d->GetSelectSequenceList();
	if (qlistSelect.empty())
		return false;
	else
	{
		for (auto e : qlistSelect)
		{
			if (!e->isUndone() && e->isSelected())
			{
				return true;
			}
		}
		return false;
	}
}

int CuttingDevice::SequenceListSize()
{
	RS_GraphicView* gv	= QC_ApplicationWindow::getAppWindow()->getGraphicView();
	QC_MDIWindow*	m	= QC_ApplicationWindow::getAppWindow()->getMDIWindow();
	if (gv && m && m->getDocument())
	{
		RS_Selection s((RS_EntityContainer&)*m->getDocument(), gv);
		return s.GetSequenceList().size();
	}
	return 0;
}
