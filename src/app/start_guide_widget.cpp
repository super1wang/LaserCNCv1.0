#include "app/start_guide_widget.h"

#include <QFileInfo>
#include <QGridLayout>
#include <QIcon>
#include <QLabel>
#include <QPainter>
#include <QScrollArea>
#include <QStyle>
#include <QToolButton>
#include <QVBoxLayout>

namespace {

constexpr int kCardWidth = 190;
constexpr int kCardHeight = 178;
constexpr int kIconWidth = 168;
constexpr int kIconHeight = 118;
constexpr int kColumns = 4;

QIcon placeholderIcon(const QString& suffix)
{
    QPixmap pix(kIconWidth, kIconHeight);
    pix.fill(QColor(245, 247, 250));

    QPainter painter(&pix);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(QPen(QColor(205, 214, 224), 1));
    painter.setBrush(QColor(255, 255, 255));
    painter.drawRoundedRect(QRectF(18, 14, kIconWidth - 36, kIconHeight - 28), 8, 8);

    painter.setPen(QColor(82, 94, 112));
    QFont f = painter.font();
    f.setBold(true);
    f.setPointSize(16);
    painter.setFont(f);
    painter.drawText(pix.rect(), Qt::AlignCenter, suffix.toUpper());
    return QIcon(pix);
}

QIcon fileIcon(const QString& filePath, const QString& thumbnailPath)
{
    if (!thumbnailPath.isEmpty() && QFileInfo::exists(thumbnailPath)) {
        QPixmap pix(thumbnailPath);
        if (!pix.isNull())
            return QIcon(pix.scaled(kIconWidth, kIconHeight,
                                    Qt::KeepAspectRatioByExpanding,
                                    Qt::SmoothTransformation));
    }

    const QString suffix = QFileInfo(filePath).suffix().isEmpty()
        ? QStringLiteral("LCNC")
        : QFileInfo(filePath).suffix();
    return placeholderIcon(suffix);
}

} // namespace

namespace lcnc::app {

StartGuideWidget::StartGuideWidget(QWidget* parent)
    : QWidget(parent)
{
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(24, 22, 24, 22);
    root->setSpacing(14);

    auto* title = new QLabel(tr("开始"), this);
    QFont titleFont = title->font();
    titleFont.setPointSize(22);
    titleFont.setBold(true);
    title->setFont(titleFont);
    root->addWidget(title);

    auto* hint = new QLabel(tr("最近打开的工程和 STEP 文件"), this);
    hint->setStyleSheet(QStringLiteral("color: #687385;"));
    root->addWidget(hint);

    auto* scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    auto* content = new QWidget(scroll);
    m_grid = new QGridLayout(content);
    m_grid->setContentsMargins(0, 10, 0, 10);
    m_grid->setHorizontalSpacing(14);
    m_grid->setVerticalSpacing(16);
    scroll->setWidget(content);
    root->addWidget(scroll, 1);
}

void StartGuideWidget::setRecentFiles(const QStringList& files,
                                      const QHash<QString, QString>& thumbnailPaths)
{
    m_recentFiles = files;
    m_thumbnailPaths = thumbnailPaths;
    rebuild();
}

void StartGuideWidget::rebuild()
{
    while (QLayoutItem* item = m_grid->takeAt(0)) {
        if (QWidget* widget = item->widget())
            widget->deleteLater();
        delete item;
    }

    if (m_recentFiles.isEmpty()) {
        auto* empty = new QLabel(tr("还没有最近文件。通过“打开”载入 .lcnc 工程或 STEP 文件后会显示在这里。"), this);
        empty->setAlignment(Qt::AlignCenter);
        empty->setMinimumHeight(220);
        empty->setStyleSheet(QStringLiteral("color: #7B8794;"));
        m_grid->addWidget(empty, 0, 0, 1, kColumns);
        return;
    }

    int row = 0;
    int column = 0;
    for (const QString& path : m_recentFiles) {
        const QFileInfo info(path);
        auto* btn = new QToolButton(this);
        btn->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        btn->setIcon(fileIcon(path, m_thumbnailPaths.value(path)));
        btn->setIconSize(QSize(kIconWidth, kIconHeight));
        btn->setText(info.fileName());
        btn->setToolTip(path);
        btn->setFixedSize(kCardWidth, kCardHeight);
        btn->setAutoRaise(false);
        btn->setStyleSheet(QStringLiteral(
            "QToolButton { border: 1px solid #D8DEE8; border-radius: 8px; padding: 8px; background: #FFFFFF; }"
            "QToolButton:hover { border-color: #2A6FDB; background: #F5F8FF; }"));

        connect(btn, &QToolButton::clicked, this, [this, path]() {
            emit fileActivated(path);
        });

        m_grid->addWidget(btn, row, column);
        ++column;
        if (column >= kColumns) {
            column = 0;
            ++row;
        }
    }
    m_grid->setRowStretch(row + 1, 1);
    m_grid->setColumnStretch(kColumns, 1);
}

} // namespace lcnc::app
