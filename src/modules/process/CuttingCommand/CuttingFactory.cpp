#include "CuttingFactory.h"

CuttingNormal	CuttingFactory::s_NormalCutting;
OverCutting		CuttingFactory::s_OverCutting;
FocusCutting	CuttingFactory::s_FocusCutting;

CuttingDevice* CuttingFactory::GetCuttingModel(const string& sCuttingName)
{
	if (sCuttingName == "NormalCutting")
	{
		return &s_NormalCutting;
	}
	else if (sCuttingName == "OverCutting")
	{
		return &s_OverCutting;
	}
	else if (sCuttingName == "FocusCutting")
	{
		return &s_FocusCutting;
	}
	return &s_NormalCutting;
}

void CuttingFactory::GetAllCuttingName(vector<string>& vecName)
{

}
