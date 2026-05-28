#pragma once

#include "core/kinematics/i_motion_controller.h"

#include <QHash>
#include <QObject>
#include <QString>

namespace lcnc::process {

class GtnMotionControllerAdapter : public QObject, public lcnc::IMotionController
{
    Q_OBJECT
public:
    explicit GtnMotionControllerAdapter(QObject* parent = nullptr);
    ~GtnMotionControllerAdapter() override;

    QString id() const override { return QStringLiteral("GTN"); }
    bool start() override;
    void stop() override;
    bool isRunning() const override { return m_running; }
    bool jog(const QString& axis, double delta) override;
    bool moveTo(const QString& axis, double absolutePos) override;
    bool home(const QString& axis = QString()) override;
    void emergencyStop() override;

private:
    short axisIndex(const QString& axis) const;
    bool moveAxis(short axis, double absolutePos);

    bool m_running{false};
    short m_core{1};
    QHash<short, double> m_axisPositions;
};

} // namespace lcnc::process