#ifndef _OVERCUTTING_DIVISION_PLANNER_
#define _OVERCUTTING_DIVISION_PLANNER_

#include "MessageCode.h"
#include <QList>
#include <vector>

class RS_Entity;

struct OverCuttingDivisionParams
{
	double startX{0.0};
	double endX{0.0};
	double maxLength{0.0};
	double suggestedLength{0.0};
	bool equalLength{true};
};

struct OverCuttingDivisionResult
{
	std::vector<double> divisions;
	double effectiveLength{0.0};
	double capacity{0.0};
};

class OverCuttingDivisionPlanner
{
public:
	static ErrorCode Plan(const QList<RS_Entity*>& entityList,
		const OverCuttingDivisionParams& params,
		OverCuttingDivisionResult& result);
};

#endif
