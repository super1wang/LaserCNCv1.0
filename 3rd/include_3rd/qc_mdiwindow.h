#pragma once
#include <QWidget>
class QG_GraphicView;
class QC_MDIWindow : public QWidget {
public:
    QG_GraphicView* getGraphicView();
};
