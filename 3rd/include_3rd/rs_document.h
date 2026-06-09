#pragma once
// Stub for missing rs_document.h / RS_Vector types from QCad/librecad

#include <QString>
#include <QList>

class RS_Vector {
public:
    double x{0.0}, y{0.0}, z{0.0};
    RS_Vector() = default;
    RS_Vector(double x, double y, double z = 0.0) : x(x), y(y), z(z) {}
};

class RS_Entity {
public:
    virtual ~RS_Entity() = default;
};

class RS_Layer {
public:
    QString name;
};

class RS_Pen {
public:
    int width{1};
};

class RS_Document {
public:
    virtual ~RS_Document() = default;
};

enum class EntityState {
    Normal,
    Selected,
    Hidden
};
