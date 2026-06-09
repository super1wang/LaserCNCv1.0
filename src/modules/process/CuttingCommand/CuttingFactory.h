#ifndef _CUTTINGFACTORY_
#define _CUTTINGFACTORY_

#include "CuttingDevice.h"
#include <string>
#include <vector>
#include "Cutting_Normal.h"
#include "Cutting_OverCutting.h"
#include "Cutting_FocusCutting.h"

using std::string;

class CuttingFactory
{
public:
	static CuttingDevice*	GetCuttingModel(const string& sCuttingName);
	static void				GetAllCuttingName(vector<string>& vecName);
private:
	static CuttingNormal	s_NormalCutting;
	static OverCutting		s_OverCutting;
	static FocusCutting		s_FocusCutting;
};

#endif // _CUTTINGFACTORY_