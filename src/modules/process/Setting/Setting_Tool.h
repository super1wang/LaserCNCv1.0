#pragma once

#include <QDialog>
#include <QInputDialog>
#include <QMessageBox>
#include "ui_Setting_Tool.h"
#include "Service.h"

class Dialog_Setting_Tool : public QDialog
{
	Q_OBJECT

public:
	Dialog_Setting_Tool(QWidget* parent = nullptr);
	~Dialog_Setting_Tool();
	void SetService(Service* pService);

public:
	void setUI();
	void ClearChange() { set_Changed.clear(); table_Temp.clear(); };
	void InitSetting();
	void SetPage(table table_Set = {});
	void GetPage(table& table_Page);
	bool GetChanged(table table_Page, table& table_Changed);
	void RebuildToolIndex(bool bLoad);	// true以setting中ToolIndex为准，false以vec为准，去更新另一个
	bool IsSaved();

private:
	void setupLineEditValidators(QWidget* dialog);

public  slots:
	void UpdatePage();
	void LinkedModeChanged();
	void CreatTool(string qToolName = "");
	void MotionControlTypeChanged();

private slots:
	void CopyTool();
	void DeleteTool();
	void RenameTool();
	void ExportTool();
	void ImportTool();
	void lineEditChanged();
	void comboBoxChanged();
	void checkBoxChanged();

public:
	Ui::Dialog_Setting_Tool		ui;
	vector<string>				vec_ToolNames;		// 所有的工具名称

private:
	set<pair<string, string>>	set_Changed;		// 记录修改值的Tabale及Key
	string						str_ToolName;		// 记录变化前的工具名称
	table						table_Temp;			// 临时记录修改内容

	QStringList					qstrlist_DirectionsX;
	QStringList					qstrlist_DirectionsY;

	Service*					m_pService;
};
