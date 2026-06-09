#ifndef _CUTTING_OVERCUTTING_
#define _CUTTING_OVERCUTTING_

#include "Cutting_Normal.h"
#include <QList>

class OverCutting : public CuttingNormal
{
public:
	OverCutting();

	virtual bool	StartCutting(const map<QString, QString>& maps) override;

private:
	//手动自定义分割
	int		GraphDivisionByLine(QList<RS_Entity*> qlistCut, vector<QList<RS_Entity*>>& newCuttingSolenoidVec);
	bool	OverLengthCuttingFeeding(int ipiece, const map<QString, QString>& maps);	//超行程切割自动feeding流程
	bool	CreatOverCuttingData(const map<QString, QString>& maps);						//创建超行程切割数据
	bool	OverLengthCuttingThread(const map<QString, QString>& maps);					//超行程切割

private:
	vector<double>								m_vSortedPointX;				//分隔线排序过后的X点坐标集合
	vector<QList<RS_Entity*>>					m_vCuttingSolenoid;             //螺线管或者大幅面切割容器
	vector<QList<RS_Entity*>>::const_iterator	m_itrOverLength;				//超行程切割的迭代器
	double										m_dStartPoint;					//图形起始点
	double										m_dEndPoint;					//图形终止点
	int											m_iCuttingPiece;				//超行程切割（非图形分割）中piece计数
};

#endif
