#pragma once

#include "app/project_explorer_model.h"

#include <QList>
#include <QStringList>

#include <cstdint>
#include <optional>

class QTreeWidget;

namespace lcnc::app {

class ProjectExplorerController
{
public:
    struct ContourSelection {
        std::uint64_t contourId{0};
        int contourIndex{-1};
    };
    struct ContourOrder {
        QList<int> indexes;
        QList<std::uint64_t> ids;
        std::uint64_t selectedId{0};
        int selectedRow{-1};
        bool hasStableIds{true};
    };

    explicit ProjectExplorerController(QTreeWidget* tree);

    void rebuild(const ProjectExplorerSnapshot& snapshot);
    [[nodiscard]] std::optional<ContourSelection> selectContour(
        std::uint64_t contourId, int fallbackIndex);
    [[nodiscard]] std::optional<ContourSelection> selectContours(
        const QList<std::uint64_t>& contourIds, const QList<int>& fallbackIndexes);
    void selectEntries(DocumentId documentId, const QStringList& entries);
    [[nodiscard]] std::optional<ContourOrder> contourOrder() const;

private:
    QTreeWidget* m_tree{nullptr};
};

} // namespace lcnc::app
