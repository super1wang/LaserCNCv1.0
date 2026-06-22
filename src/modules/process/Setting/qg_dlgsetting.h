#pragma once

#include <QDialog>
#include "ui_qg_dlgSetting.h"

#include "Setting_MotionControl.h"
#include "Setting_Digital.h"
#include "Setting_Analog.h"
#include "Setting_Laser.h"
#include "Setting_Internet.h"
#include "Setting_ProcessPlugins.h"

#include "Setting_Tool.h"
#include "Setting_Gas.h"
#include "Setting_Water.h"
#include "Setting_Monitor.h"
#include "Setting_LoadingPos.h"
#include "Setting_Camera.h"

#define DLGSETTING QG_dlgSetting::instance()

class QG_dlgSetting : public QDialog
{
	// 菜单的枚举，名称同索引
	enum Menu
	{
		EXTERNAL,
			MOTION_CONTROLLER,
			AXIS_SPEED,   // 轴参数已合并至运动控制器页面，菜单项保留用于权限兼容
			IO_INDEX,
				DIGITAL_IO,
				ANALOG_IO,
			LASER,
			INTERNET,
			PROCESS_PLUGINS,

		PROCESSING,
			TOOL,
				MOTION_LASER,
				GENERAL,
				SERVO,
				SPECIAL,	// 需要更名，与拓展的参数项重名了
			GAS,
			WATER,
			MONITOR,
			LOADINGPOS,
			CAMERA
	};

	// 页面的枚举，名称同类名
	// 注意：枚举值直接用作 stackedWidget_Setting_Content 的索引，
	// 必须与 QG_dlgSetting 构造函数中 insertWidget 的顺序严格对应，
	// 不允许出现"占位但不插入"的项，否则后续页面索引会整体错位。
	enum Page
	{
		//EXTERNAL,
		MotionController,
		// 轴参数已合并至运动控制器页面，原 Axis 页面项已移除
		// IO_INDEX 容器页已删除：Digital/Analog 直接作为顶层项呈现
		Digital,
		Analog,
		Laser,
		Internet,
		ProcessPlugins,
		//PROCESSING,
		Tool,
		//MOTION_LASER,
		//GENERAL,
		//SPECIAL,
		Gas,
		Water,
		Monitor,
		LoadingPos,
		Camera
	};

	Q_OBJECT

public:
	QG_dlgSetting(QWidget *parent = nullptr);
	~QG_dlgSetting();
	static QG_dlgSetting* instance(QG_dlgSetting* pdlgSetting = nullptr);
	void SetService(Service*);
	void CreateMenu();							// 创建菜单
	void UpdateMenu(int iPermissionLevel);		// 权限更新结构
	bool CustomerMenu(int iPermissionLevel);	// 客户权限
	void InitSetting();							// 初始化参数
	void UpdatePage();							// 刷新页面
	void GetChanged();							// 获取修改
	void RebuildToolList();						// 刷新工具
	void AppSettings(int iMode = 0);			// 应用参数，默认为全部参数

public slots:
	void SwitchItem(QTreeWidgetItem* item, int column);

	void clickApply();
	void clickOK();
	void clickCancel();
	void clickExportConfig();
	void clickImportConfig();

private:
	void LoadSetting();
	void LoadToolConfig(string FilePath);	// 读入工具
	void ChooseDialog(const QString& qstrTitle, map<string, bool>& mapObject);

private:
	Dialog_Setting_MotionControl*	dlgMotionControlSetting;
	Dialog_Setting_Digital*			dlgDigitalSetting;
	Dialog_Setting_Analog*			dlgAnalogSetting;
	Dialog_Setting_Laser*			dlgLaserSetting;
	Dialog_Setting_Internet*		dlgInternetSetting;
	lcnc::process::Dialog_Setting_ProcessPlugins* dlgProcessPluginsSetting;

	Dialog_Setting_Tool*			dlgToolSetting;
	Dialog_Setting_Gas*				dlgGasSetting;
	Dialog_Setting_Water*			dlgWaterSetting;
	Dialog_Setting_Monitor*			dlgMonitorSetting;
	Dialog_Setting_LoadingPos*		dlgLoadingPosSetting;
	Dialog_Setting_Camera*			dlgCameraSetting;

	//old
	//QG_dlgCuttingProcessSetting* dlgCuttingProcessSetting;
	//QG_dlgAutomationSetting* dlgAutomationSetting;

private:
	Ui::qg_dlgSetting			ui;
	static QG_dlgSetting*		uniqueInstance;
	Service*					m_pService;

	map<Menu, QTreeWidgetItem*> m_mapMenu;			// 菜单映射

	table						m_tableSettings;	// 全界面参数缓存，结构同SETTING中UserSetting
	table						m_tableChanged;		// 仅修改值的table
};
