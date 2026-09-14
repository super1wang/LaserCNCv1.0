#pragma once

#include <QWidget>
#include <QString>
#include <QStringList>
#include <QHash>

class QGridLayout;

namespace lcnc::app {

class StartGuideWidget : public QWidget
{
    Q_OBJECT
public:
    explicit StartGuideWidget(QWidget* parent = nullptr);

    void setRecentFiles(const QStringList& files,
                        const QHash<QString, QString>& thumbnailPaths = {});

signals:
    void fileActivated(const QString& filePath);

private:
    void rebuild();

    QGridLayout* m_grid{nullptr};
    QStringList m_recentFiles;
    QHash<QString, QString> m_thumbnailPaths;
};

} // namespace lcnc::app
