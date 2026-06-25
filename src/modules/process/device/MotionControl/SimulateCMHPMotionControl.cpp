#include "SimulateCMHPMotionControl.h"
#include <windows.h>

#define MAX_VALUE 100

SimulateCMHPMotionControl::SimulateCMHPMotionControl()
{
	m_strName = "SimulatorCMHP";
}

bool SimulateCMHPMotionControl::Connect()
{
	if (m_hHandle == ACSC_INVALID)
	{
		DeleteOtherConnections();
		m_hHandle = acsc_OpenCommSimulator();
		if (m_hHandle == ACSC_INVALID)
			return false;
	}
	int iRes = acsc_StopBuffer(m_hHandle, ACSC_NONE, NULL);

	wchar_t Configurefile_path[MAX_VALUE];
	char filePath_exe[MAX_VALUE];
	GetModuleFileNameW(NULL, Configurefile_path, MAX_VALUE);

	DWORD dwNum = WideCharToMultiByte(CP_OEMCP, NULL, Configurefile_path, -1, NULL, 0, NULL, NULL);
	WideCharToMultiByte(CP_OEMCP, NULL, Configurefile_path, -1, filePath_exe, dwNum, NULL, NULL);

	std::string debug_filepath(filePath_exe);
	int index = debug_filepath.find_last_of('\\');
	std::string sub = debug_filepath.substr(0, static_cast<size_t>(index));
	std::string addfilepath = "\\Simulator.prg";
	sub += addfilepath;
	const char* cpFilename = sub.c_str();
	char* pFilename = new char[strlen(cpFilename) + 1];
	strcpy_s(pFilename, strlen(cpFilename) + 1, cpFilename);
	if (!acsc_LoadBuffersFromFile(m_hHandle, pFilename, NULL))
	{
		delete[] pFilename;
		return false;
	}
	delete[] pFilename;
	if (!AfterOpenComm())
		return false;
	m_bErrorOccurred = false;
	return true;
}

bool SimulateCMHPMotionControl::Disconnect()
{
	if (!acsc_StopBuffer(m_hHandle, 0, NULL))
		return false;

	if (!acsc_CloseComm(m_hHandle))
	{
		return false;
	}
	if (!acsc_CloseSimulator())
	{
		return false;
	}
	m_bConnectFlag = false;
	m_hHandle = ACSC_INVALID;
	return true;
}

SimulateCMHPMotionControl::~SimulateCMHPMotionControl()
{
	if (m_hHandle != ACSC_INVALID)
	{
		acsc_CloseComm(m_hHandle);
		acsc_CloseSimulator();
	}
}
