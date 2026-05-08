#include "modules/cam/services/cam_data_manager.h"

#include <QSet>

namespace lcnc::cam {

void CamDataManager::clearToolpath()
{
    m_toolpath.clear();
    m_nextContourId = 1;
    m_dirty = true;
}

ContourId CamDataManager::contourIdAt(int contourIdx) const
{
    if (contourIdx < 0 || contourIdx >= m_toolpath.contourCount())
        return 0;

    return static_cast<ContourId>(m_toolpath.contour(contourIdx).contourId);
}

int CamDataManager::contourIndexById(ContourId contourId) const
{
    if (contourId == 0)
        return -1;

    for (int index = 0; index < m_toolpath.contourCount(); ++index) {
        if (m_toolpath.contour(index).contourId == contourId)
            return index;
    }
    return -1;
}

void CamDataManager::ensureContourIds()
{
    for (LaserContour& contour : m_toolpath.contours()) {
        if (contour.contourId == 0)
            contour.contourId = nextContourId();
        if (contour.contourId >= m_nextContourId)
            m_nextContourId = static_cast<ContourId>(contour.contourId + 1);
    }
}

bool CamDataManager::reorderContours(const QList<int>& order)
{
    if (order.size() != m_toolpath.contourCount())
        return false;

    QSet<int> seen;
    std::vector<LaserContour> current = std::move(m_toolpath.contours());
    std::vector<LaserContour> reordered;
    reordered.reserve(current.size());

    for (int index : order) {
        if (index < 0 || index >= static_cast<int>(current.size()) || seen.contains(index)) {
            m_toolpath.contours() = std::move(current);
            return false;
        }

        seen.insert(index);
        reordered.push_back(std::move(current[index]));
    }

    m_toolpath.contours() = std::move(reordered);
    ensureContourIds();
    m_dirty = true;
    return true;
}

bool CamDataManager::reorderContoursById(const QList<ContourId>& order)
{
    if (order.size() != m_toolpath.contourCount())
        return false;

    QList<int> indexOrder;
    indexOrder.reserve(order.size());
    for (ContourId contourId : order) {
        const int index = contourIndexById(contourId);
        if (index < 0)
            return false;
        indexOrder.append(index);
    }

    return reorderContours(indexOrder);
}

ContourId CamDataManager::nextContourId()
{
    return m_nextContourId++;
}

} // namespace lcnc::cam