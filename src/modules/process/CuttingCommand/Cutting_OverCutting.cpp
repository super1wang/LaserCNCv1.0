#include "Cutting_OverCutting.h"
#include "LogModule.h"
#include "qc_applicationwindow.h"
#include "qc_mdiwindow.h"
#include "rs_graphic.h"
#include "rs_layerlist.h"
#include "qg_CuttingInfoWidget.h"

#include <algorithm>
#include <cmath>

namespace
{
	const double EPS = 1e-6;

	bool SamePosition(double lhs, double rhs)
	{
		return std::fabs(lhs - rhs) <= EPS;
	}

	void ReportOverCuttingError(ErrorCode errorCode)
	{
		SHOW_PROCESS_ERROR(errorCode);
		g_bStop.store(true);
		g_bFinish.store(true);
		g_eState.store(SystemStatus::Error);
	}
}

OverCutting::OverCutting()
{
}

bool OverCutting::StartCutting(const map<QString, QString>& maps)
{
	//数据的初始化
	m_iCuttingPiece = 0;
	m_CuttingList.clear();
	m_vSortedPointX.clear();
	m_vCuttingSolenoid.clear();

	if (!CreatOverCuttingData(maps))
		return false;

	if (!OverLengthCuttingThread(maps))
		return false;
	
	return true;
}

bool OverCutting::OverLengthCuttingThread(const map<QString, QString>& maps)
{
	m_itrOverLength = m_vCuttingSolenoid.begin();
	RS_Vector ptLL0, ptUR0;
	GetBound(ptLL0, ptUR0, m_vCuttingSolenoid[0]);
	double dPos = ptLL0.x;		//图形起点

	for (; m_itrOverLength != m_vCuttingSolenoid.end(); m_itrOverLength++)
	{
		if (m_CuttingList.empty())
		{
			m_CuttingList = *m_itrOverLength;
			m_itrCurList = m_CuttingList.begin();
		}

		if (!IsOffsetCutting())
			return false;

		if (!OverLengthCuttingFeeding(m_iCuttingPiece, maps))
			return false;

		// 切割完成
		if (m_CuttingList.size() == 0)
		{
			m_iCuttingPiece++;
			m_CuttingList.clear();
			continue;
		}

		double dValue = 0;
		if (m_iCuttingPiece == 0)
		{
			dValue = 0;
		}
		else
		{
			if (m_iCuttingPiece > 0 && static_cast<size_t>(m_iCuttingPiece - 1) < m_vSortedPointX.size())
			{
				dValue = m_vSortedPointX[static_cast<size_t>(m_iCuttingPiece - 1)] - m_dStartPoint;
			}
		}
		CuttingThread(-dValue, 0.0);

		m_CuttingList.clear();
		m_iCuttingPiece++;
	}
	return true;
}

bool OverCutting::CreatOverCuttingData(const map<QString, QString>& maps)
{
	MotionControl*		pMC = QC_ApplicationWindow::getAppWindow()->getService()->GetMotionControl();
	RS_GraphicView*		g	= QC_ApplicationWindow::getAppWindow()->getGraphicView();
	QC_MDIWindow*		m	= QC_ApplicationWindow::getAppWindow()->getMDIWindow();
	RS_EntityContainer* d	= QC_ApplicationWindow::getAppWindow()->getDocument();
	if (g && m && m->getDocument())
	{
		double lx, rx;
		pMC->ReadAxisSoftLimit(Axis::X, lx, rx);
		RS_Vector vecMin, vecMax;
		QList<RS_Entity*> qlistCut;
		GetCuttingBound(vecMin, vecMax, qlistCut);
		m_dStartPoint	= vecMin.x;
		m_dEndPoint		= vecMax.x;
		double dLimited = rx - m_dStartPoint;
		if (dLimited <= 0)
		{
			ReportOverCuttingError(ErrorCode::ERROR_OVERCUTTING_OUTOFLIMIT);
			return false;
		}

		QString qstrDivision = maps.at("Layer");
		for (auto t : *d)
		{
			if (!t || t->isUndone() || !t->getLayer() || qstrDivision != t->getLayer()->getName())
				continue;

			const double divisionX = t->getMin().x;
			if (divisionX <= m_dStartPoint + EPS || divisionX >= m_dEndPoint - EPS)
				continue;

			m_vSortedPointX.push_back(divisionX);
		}

		if (m_vSortedPointX.empty())
		{
			ReportOverCuttingError(ErrorCode::ERROR_OVERCUTTING_DIVISIONEMPTY);
			return false;
		}
		sort(m_vSortedPointX.begin(), m_vSortedPointX.end());		//从小到大排序
		m_vSortedPointX.erase(std::unique(m_vSortedPointX.begin(), m_vSortedPointX.end(), SamePosition), m_vSortedPointX.end());

		vector<double>::const_iterator iterposition = m_vSortedPointX.begin();
		double dLastPoint = m_dStartPoint;
		for (; iterposition != m_vSortedPointX.end(); iterposition++)
		{
			double dLength = *iterposition - dLastPoint;

			if (dLength <= EPS)
			{
				ReportOverCuttingError(ErrorCode::ERROR_OVERCUTTING_SEGTOOSHORT);
				return false;
			}
			if (dLength > dLimited + EPS)
			{
				ReportOverCuttingError(ErrorCode::ERROR_OVERCUTTING_OUTOFLIMIT);
				return false;
			}
			dLastPoint = *iterposition;
		}
		double dLastLength = m_dEndPoint - dLastPoint;
		if (dLastLength <= EPS)
		{
			ReportOverCuttingError(ErrorCode::ERROR_OVERCUTTING_SEGTOOSHORT);
			return false;
		}
		if (dLastLength > dLimited + EPS)
		{
			ReportOverCuttingError(ErrorCode::ERROR_OVERCUTTING_OUTOFLIMIT);
			return false;
		}
		LOG_PROCESS_INFO(QString("[OverCutting] Loaded %1 division points from layer %2.")
			.arg(static_cast<int>(m_vSortedPointX.size())).arg(qstrDivision).toUtf8().data());

		int iTotalNums = GraphDivisionByLine(qlistCut, m_vCuttingSolenoid);
		QG_CuttingInfoWidget::SetTotalCutCount(iTotalNums);
	}

	return true;
}

int OverCutting::GraphDivisionByLine(QList<RS_Entity*> qlistCut, vector<QList<RS_Entity*>>& newCuttingSolenoidVec)
{
	int iNumber = 0;
	int ipiece  = m_vSortedPointX.size() + 1;
	double dLL, dRR, dLastPoint;
	for (int i = 0; i < ipiece; i++)
	{
		if (ipiece == 1)
		{
			dLL = m_dStartPoint;
			dRR = m_dEndPoint;
		}
		else if (i == 0)
		{
			dLL = m_dStartPoint;
			dRR = m_vSortedPointX[i];
		}
		else if (i == (ipiece - 1))
		{
			dLL = m_vSortedPointX[i - 1];
			dRR = m_dEndPoint;
		}
		else
		{
			dLL = m_vSortedPointX[i - 1];
			dRR = m_vSortedPointX[i];
		}
		QList<RS_Entity*> vecTemp;
		QList<RS_Entity*>::const_iterator iterPolyline = qlistCut.begin();
		while (iterPolyline != qlistCut.end())
		{
			RS_Entity* pEntity = *iterPolyline;
			RS_Vector rsCenter = (pEntity->getMin() + pEntity->getMax()) * 0.5;
			if (i == ipiece - 1)
			{
				if (rsCenter.x >= dLL && rsCenter.x <= dRR)
				{
					vecTemp.push_back(*iterPolyline);
					iNumber++;
				}
				iterPolyline++;
			}
			else
			{
				if (rsCenter.x >= dLL && rsCenter.x < dRR)
				{
					vecTemp.push_back(*iterPolyline);
					iNumber++;
				}
				iterPolyline++;
			}
		}
		newCuttingSolenoidVec.push_back(vecTemp);
	}
	return iNumber;
}

bool OverCutting::OverLengthCuttingFeeding(int ipiece, const map<QString, QString>& maps)
{
	double dVelX = 0;
	switch (maps.at("XSpeed").toInt())
	{
	case 0:		SETTINGS->GetKeyValue("fLowSpeed",		dVelX, SettingSection::Axis, "X");	break;
	case 1:		SETTINGS->GetKeyValue("fMediumSpeed",	dVelX, SettingSection::Axis, "X");	break;
	case 2:		SETTINGS->GetKeyValue("fHighSpeed",		dVelX, SettingSection::Axis, "X");	break;
	}

	double dVelY = 0;
	switch (maps.at("YSpeed").toInt())
	{
	case 0:		SETTINGS->GetKeyValue("fLowSpeed",		dVelY, SettingSection::Axis, "A");	break;
	case 1:		SETTINGS->GetKeyValue("fMediumSpeed",	dVelY, SettingSection::Axis, "A");	break;
	case 2:		SETTINGS->GetKeyValue("fHighSpeed",		dVelY, SettingSection::Axis, "A");	break;
	}

	MotionControl* pMC = QC_ApplicationWindow::getAppWindow()->getService()->GetMotionControl();
	if (ipiece == 1)
	{
		double dSetPoint = m_vSortedPointX[ipiece - 1];
		pMC->MoveAbsolute(Axis::X, dSetPoint,dVelX);   //移动到图形位置切
		//确保X轴移动到位
		do { Sleep(100); } while (pMC->IsAxisMoving());
	}
	else if (ipiece >= 2)
	{
		double dSetPoint = m_vSortedPointX[ipiece - 1] - m_vSortedPointX[ipiece - 2];
		pMC->MoveAbsolute(Axis::X, dSetPoint + m_dStartPoint, dVelX);   //移动到图形位置切
		//确保X轴移动到位
		do { Sleep(100); } while (pMC->IsAxisMoving());
	}

	pMC->DigitalOutputSet(DigitalOUT::Pliers, 1);
	LOG_PROCESS_INFO(QObject::tr("[OverCutting] Turn on the pliers and wait for %1 ms").arg(maps.at("OpenPliers")).toUtf8().data());
	if (!WaitTime(maps.at("OpenPliers").toInt())) return false;

	pMC->DigitalOutputSet(DigitalOUT::Chuck, 0);

	LOG_PROCESS_INFO(QObject::tr("[OverCutting] Turn off the chuck and wait for %1 ms").arg(maps.at("CloseChuck")).toUtf8().data());
	if (!WaitTime(maps.at("CloseChuck").toInt())) return false;

	double dOverLengthCompensateX = maps.at("XCompensation").toDouble();
	double dOverLengthCompensateY = maps.at("YCompensation").toDouble();
	
	LOG_PROCESS_INFO(QObject::tr("[OverCutting] Axis X move to %1").arg(m_dStartPoint + dOverLengthCompensateX).toUtf8().data());
	LOG_PROCESS_INFO(QObject::tr("[OverCutting] Axis A move %1").arg(dOverLengthCompensateY).toUtf8().data());
	
	pMC->MoveAbsolute(Axis::X, m_dStartPoint + dOverLengthCompensateX, dVelX);   //移动到图形位置切
	pMC->MoveRelative(Axis::A, dOverLengthCompensateY, dVelY);   //移动到图形位置切

	//确保X轴移动到位
	do { 
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
	
	} while (pMC->IsAxisMoving());

	pMC->DigitalOutputSet(DigitalOUT::Chuck, 1);
	LOG_PROCESS_INFO(QObject::tr("[OverCutting] Turn on the chuck and wait for %1 ms").arg(maps.at("OpenChuck")).toUtf8().data());
	if (!WaitTime(maps.at("OpenChuck").toInt())) return false;

	pMC->DigitalOutputSet(DigitalOUT::Pliers, 0);
	LOG_PROCESS_INFO(QObject::tr("[OverCutting] Turn off the pliers and wait for %1 ms").arg(maps.at("ClosePliers")).toUtf8().data());
	if (!WaitTime(maps.at("ClosePliers").toInt())) return false;
	return true;
}
