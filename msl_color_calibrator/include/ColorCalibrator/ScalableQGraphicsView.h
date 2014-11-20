#ifndef SCALABLEQGRAPHICSVIEW_H
#define SCALABLEQGRAPHICSVIEW_H

#include <QGraphicsView>

class ScalableQGraphicsView : public QGraphicsView
{
public:
    ScalableQGraphicsView(QWidget *parent = 0);

    void setScene(QGraphicsScene *scene);
    void wheelEvent(QWheelEvent *event);

    int getZoomState();

private:
    int zoomState;
};

#endif // SCALABLEQGRAPHICSVIEW_H
