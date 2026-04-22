#pragma once

#include <QList>
#include <QString>

#include <TopoDS_Shape.hxx>

class TaskProgress;

class MachineModelCompressor
{
public:
    enum class Strategy {
        FilledSolid,
        ExteriorShell,
        SewingShell,
        BoundingBoxProxy,
    };

    struct Options {
        Strategy strategy{Strategy::FilledSolid};
    };

    struct GroupInput {
        QString axisName;
        QString displayName;
        QList<TopoDS_Shape> shapes;
    };

    struct GroupOutput {
        QString axisName;
        QString displayName;
        TopoDS_Shape shape;
    };

    struct Result {
        QList<GroupOutput> groups;
        QString error;
        bool aborted{false};

        bool success() const
        {
            return !aborted && error.isEmpty();
        }
    };

    static Result compressGroups(const QList<GroupInput>& groups,
                                 const Options& options = {},
                                 TaskProgress* progress = nullptr);
};