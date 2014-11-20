#include "ScalableQGraphicsView.h"

#include <QTimeLine>
#include <QWheelEvent>

#include <iostream>

ScalableQGraphicsView::ScalableQGraphicsView(QWidget *parent) :
    QGraphicsView(parent)
{
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    zoomState = 0;
}

void ScalableQGraphicsView::setScene(QGraphicsScene *scene) {
    while (zoomState > 0) {
        qreal factor = 1.0 / 1.2;
        scale(factor, factor);
        zoomState--;
    }
    QGraphicsView::setScene(scene);
}

void ScalableQGraphicsView::wheelEvent(QWheelEvent *event) {
    qreal factor = 1.2;
    if (event->delta() < 0) {
        if (zoomState > 0) {
            factor = 1.0 / factor;
            zoomState--;
        } else {
            factor = 1.0;
        }
    } else if (zoomState < 19) {
        zoomState++;
    } else {
        factor = 1.0;
    }
    scale(factor, factor);
}

int ScalableQGraphicsView::getZoomState() {
    return zoomState;
}
