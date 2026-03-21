#pragma once

#include <QDialog>
#include <QList>
#include <QMap>
#include <QString>

class LcncDocument;
class MachineKinematics;
class QComboBox;
class QGridLayout;
class QScrollArea;

/**
 * @brief Dialog for manually assigning machine body shapes to kinematic axes.
 *
 * Layout:
 *   ┌─ info ──────────────────────────────────────────────┐
 *   │  机台构型:  <configType>                             │
 *   ├─ scroll ────────────────────────────────────────────┤
 *   │  零件名称           │ 所属轴系                       │
 *   │  ─────────────────────────────────────────────────  │
 *   │  MachineBody        │ [固定基座 ▼]                   │
 *   │  X_Slide            │ [X 轴（线性） ▼]              │
 *   │  ...                │ ...                            │
 *   ├─────────────────────────────────────────────────────┤
 *   │  [自动识别]          [确定]  [取消]                  │
 *   └─────────────────────────────────────────────────────┘
 *
 * On acceptance the dialog writes all combo selections back into the
 * MachineKinematics object via assignShape().
 */
class DialogMarkAxes : public QDialog
{
    Q_OBJECT
public:
    explicit DialogMarkAxes(LcncDocument*     doc,
                            MachineKinematics* kin,
                            QWidget*           parent = nullptr);

private slots:
    void onAutoDetect();
    void accept() override;

private:
    void buildUi();
    void populateRows();
    void applyAssignments(const QMap<QString,QString>& entry2axis);

    LcncDocument*     m_doc;
    MachineKinematics* m_kin;

    struct Row {
        QString   entry;
        QString   name;
        QComboBox* combo{nullptr};
    };

    QList<Row>   m_rows;
    QWidget*     m_rowContainer{nullptr};
    QGridLayout* m_grid{nullptr};
};
