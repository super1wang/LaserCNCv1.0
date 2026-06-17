#pragma once

#include <QDialog>
#include "ui_Setting_Digital.h"
#include "Service.h"

/**
 * @brief 数字量 IO 配置页 — 单张动态表，统一显示 IN/OUT。
 *
 * 列：名称 / 索引 / 类型(IN|OUT) / 有效电平(高|低) / 启用 / 显示到主界面 / 操作。
 * 预设行（builtin=true）的名称和类型只读，仅可改索引/启用/有效电平/显示到主界面。
 * 扩展行可通过工具栏的 "+ 添加输入/输出" 按钮新增；非预设行的"操作"列含删除按钮。
 *
 * SetPage / GetPage 走完整 round-trip：每次都将整段 [DigitalIN]/[DigitalOUT]
 * 重写到 settings，避免对动态行做 diff。
 */
class Dialog_Setting_Digital : public QDialog
{
    Q_OBJECT
public:
    explicit Dialog_Setting_Digital(QWidget* parent = nullptr);
    ~Dialog_Setting_Digital() override;

    void ClearChange() { m_dirty = false; }
    void InitSetting();
    void SetPage(table table_Set = {});
    void GetPage(table& table_Page);
    bool GetChanged(table table_Page, table& table_Changed);
    void SetIDEnabled(bool bEnabled);
    void SetIndexEnabled(bool bEnabled);

private:
    enum Column { ColName, ColIndex, ColType, ColActive, ColEnabled, ColShowMain, ColOp, ColCount };

    struct RowDescriptor
    {
        QString tomlKey;
        QString name;
        QString index;
        bool    isInput{false};
        bool    activeHigh{true};
        bool    enabled{true};
        bool    showInMain{false};
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
    Ui::Dialog_Setting_Digital ui;
    bool m_dirty{false};
};
