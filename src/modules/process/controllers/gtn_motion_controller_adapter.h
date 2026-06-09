#pragma once

#include <QHash>
#include <QObject>
#include <QString>

namespace lcnc::process {

class GtnMotionControllerAdapter : public QObject
{
    Q_OBJECT
public:
    explicit GtnMotionControllerAdapter(QObject* parent = nullptr);
    ~GtnMotionControllerAdapter() override;

    QString id() const { return QStringLiteral("GTN"); }
    bool start();
    void stop();
    bool isRunning() const { return m_running; }
    bool jog(const QString& axis, double delta);
    bool moveTo(const QString& axis, double absolutePos);
    bool home(const QString& axis = QString());
    void emergencyStop();
    bool supportsProgramPause() const { return false; }

private:
    short axisIndex(const QString& axis) const;
    bool moveAxis(short axis, double absolutePos);

    bool m_running{false};
    short m_core{1};
    QHash<short, double> m_axisPositions;
};

} // namespace lcnc::process