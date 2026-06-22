#pragma once

#include <QDialog>

class QTableWidget;

namespace lcnc::process {

class Dialog_Setting_ProcessPlugins : public QDialog
{
    Q_OBJECT
public:
    explicit Dialog_Setting_ProcessPlugins(QWidget* parent = nullptr);

    void ClearChange() { m_dirty = false; }
    void InitSetting();
    void SetPage();
    void GetPage();
    bool GetChanged();

private:
    void rebuildRows();

    QTableWidget* m_table{nullptr};
    bool m_dirty{false};
};

} // namespace lcnc::process
