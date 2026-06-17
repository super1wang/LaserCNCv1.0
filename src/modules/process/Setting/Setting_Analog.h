#pragma once

#include <QDialog>
#include "ui_Setting_Analog.h"
#include "Service.h"

/**
 * @brief 模拟量 IO 配置页 — 单张动态表，统一显示 IN/OUT。
 *
 * 列：名称 / 索引 / 类型(IN|OUT) / 启用 / 操作。
 * 与数字量页相比少了"有效电平"和"显示到主界面"两列。
 */
class Dialog_Setting_Analog : public QDialog
{
    Q_OBJECT
public:
    explicit Dialog_Setting_Analog(QWidget* parent = nullptr);
    ~Dialog_Setting_Analog() override;

    void ClearChange() { m_dirty = false; }
    void InitSetting();
    void SetPage(table table_Set = {});
    void GetPage(table& table_Page);
    bool GetChanged(table table_Page, table& table_Changed);
    void SetIDEnabled(bool bEnabled);
    void SetIndexEnabled(bool bEnabled);

private:
    struct RowDescriptor
    {
        QString tomlKey;
        QString name;
        QString index;
        bool    isInput{false};
        bool    enabled{true};
        bool    builtin{false};
    };

    void setupTable();
    void rebuildRows(const QList<RowDescriptor>& rows);
    void appendRow(const RowDescriptor& row);
    QList<RowDescriptor> readRowsFromUi() const;
    void onAddIN();
    void onAddOUT();
    void onDeleteRow(int row);
    QString allocateNewKey(bool isInput) const;

private slots:
    void markDirty();

private:
    Ui::Dialog_Setting_Analog ui;
    bool m_dirty{false};
};
