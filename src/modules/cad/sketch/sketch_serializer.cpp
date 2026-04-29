#include "modules/cad/sketch/sketch_serializer.h"

#include <QVariantList>

namespace lcnc::cad {
namespace {

QVariantMap elementToMap(const SketchElement& element)
{
    QVariantMap map;
    map.insert(QStringLiteral("id"), element.id);
    map.insert(QStringLiteral("kind"), static_cast<int>(element.kind));
    map.insert(QStringLiteral("label"), element.label);
    QVariantList params;
    for (double value : element.params)
        params.append(value);
    map.insert(QStringLiteral("params"), params);
    return map;
}

SketchElement elementFromMap(const QVariantMap& map)
{
    SketchElement element;
    element.id = map.value(QStringLiteral("id")).toInt();
    element.kind = static_cast<SketchToolKind>(map.value(QStringLiteral("kind")).toInt());
    element.label = map.value(QStringLiteral("label")).toString();
    const QVariantList params = map.value(QStringLiteral("params")).toList();
    for (const QVariant& value : params)
        element.params.append(value.toDouble());
    return element;
}

} // namespace

QVariantMap SketchSerializer::toVariantMap(const SketchRecord& record)
{
    QVariantMap map;
    map.insert(QStringLiteral("id"), record.id);
    map.insert(QStringLiteral("name"), record.name);
    map.insert(QStringLiteral("plane"), static_cast<int>(record.plane));
    map.insert(QStringLiteral("visible"), record.visible);
    map.insert(QStringLiteral("usage"), static_cast<int>(record.usage));
    map.insert(QStringLiteral("featureEntry"), record.featureEntry);
    QVariantList elements;
    for (const auto& element : record.elements)
        elements.append(elementToMap(element));
    map.insert(QStringLiteral("elements"), elements);
    return map;
}

SketchRecord SketchSerializer::fromVariantMap(const QVariantMap& map)
{
    SketchRecord record;
    record.id = map.value(QStringLiteral("id")).toInt();
    record.name = map.value(QStringLiteral("name")).toString();
    record.plane = static_cast<SketchPlaneKind>(map.value(QStringLiteral("plane")).toInt());
    record.visible = map.value(QStringLiteral("visible"), true).toBool();
    record.usage = static_cast<SketchUsageState>(map.value(QStringLiteral("usage")).toInt());
    record.featureEntry = map.value(QStringLiteral("featureEntry")).toString();
    const QVariantList elements = map.value(QStringLiteral("elements")).toList();
    for (const QVariant& value : elements)
        record.elements.append(elementFromMap(value.toMap()));
    return record;
}

} // namespace lcnc::cad