#include "PolygonQGraphicsScene.h"

#include <QGraphicsView>

#include <iostream>

using namespace std;

PolygonQGraphicsScene::PolygonQGraphicsScene(QObject *parent) :
    QGraphicsScene(parent)
{
    m_draw = false;
    m_drag = false;
    m_drawingMode = RECT;

    addItems();
}

QPolygon PolygonQGraphicsScene::getPolygon() {
    return m_polygon;
}

void PolygonQGraphicsScene::resetItems() {
	if (m_draw) {
        m_polygonItem->setVisible(false);
    }
    m_draw = false;
    m_drag = false;
}

void PolygonQGraphicsScene::addItems() {
    m_polygonItem = new QGraphicsPolygonItem();
    m_polygonItem->setZValue(1.0);
    addItem(m_polygonItem);

    m_polygonItem->setVisible(false);
}

void PolygonQGraphicsScene::updateItems(QGraphicsSceneMouseEvent *event) {
	switch (m_drawingMode) {
		case RECT:
		{
			int rect_x, rect_y, rect_width, rect_height;
			if (m_startX <= event->scenePos().x()) {
				rect_x = m_startX;
				rect_width = event->scenePos().x()-m_startX;
			} else {
				rect_x = event->scenePos().x();
				rect_width = m_startX-event->scenePos().x();
			}

			if (m_startY <= event->scenePos().y()) {
				rect_y = m_startY;
				rect_height = event->scenePos().y()-m_startY;
			} else {
				rect_y = event->scenePos().y();
				rect_height = m_startY-event->scenePos().y();
			}

			if (rect_x < 0) {
				rect_width += rect_x;
				rect_x = 0;
			}
			if (rect_y < 0) {
				rect_height += rect_y;
				rect_y = 0;
			}
			if (rect_x + rect_width >= sceneRect().width()) {
				rect_width = sceneRect().width() - rect_x;// - 1; // 1 = line width
			}
			if (rect_y + rect_height >= sceneRect().height()) {
				rect_height = sceneRect().height() - rect_y;// - 1;
			}

			m_polygon = QPolygon();
			m_polygon.append(QPoint(rect_x, rect_y));
			m_polygon.append(QPoint(rect_x + rect_width, rect_y));
			m_polygon.append(QPoint(rect_x + rect_width, rect_y + rect_height));
			m_polygon.append(QPoint(rect_x, rect_y + rect_height));
			break;
		}
		case POLYGON:
		{
			int poly_x, poly_y;
			poly_x = event->scenePos().x();
			if (poly_x < 0) {
				poly_x = 0;
			}
			if (poly_x >= sceneRect().width()) {
				poly_x = sceneRect().width();
			}
			poly_y = event->scenePos().y();
			if (poly_y < 0) {
				poly_y = 0;
			}
			if (poly_y >= sceneRect().height()) {
				poly_y = sceneRect().height();
			}
			m_polygon.append(QPoint(poly_x, poly_y));
			break;
		}
	}

	m_polygonItem->setPolygon(m_polygon);
}

void PolygonQGraphicsScene::setDrawingMode(DrawingMode drawingMode) {
    m_drawingMode = drawingMode;
}

DrawingMode PolygonQGraphicsScene::getDrawingMode() {
    return m_drawingMode;
}

bool PolygonQGraphicsScene::checkModifiers(Qt::KeyboardModifier modifier) {
    return m_modifiers & modifier;
}

void PolygonQGraphicsScene::mousePressEvent(QGraphicsSceneMouseEvent *event) {
	if (items().size() > 1) {
		m_polygonItem->setVisible(true);

        m_startX = event->scenePos().x();
        m_startY = event->scenePos().y();

		m_polygon = QPolygon(QVector<QPoint>() << QPoint(m_startX, m_startY));

		m_pressedButton = event->button();
		if (m_pressedButton == Qt::LeftButton) {
			m_polygonItem->setPen(QPen(Qt::green));
		} else if (m_pressedButton == Qt::RightButton) {
			m_polygonItem->setPen(QPen(Qt::red));
		} else {
			m_polygonItem->setPen(QPen(Qt::white));
		}
		m_polygonItem->setBrush(Qt::NoBrush);

        updateItems(event);

        m_draw = true;
    }
}

void PolygonQGraphicsScene::mouseMoveEvent(QGraphicsSceneMouseEvent *event) {
    if (m_draw) {
        updateItems(event);
        m_drag = true;
    }
}

void PolygonQGraphicsScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    if (m_draw) {
        m_modifiers = event->modifiers();
        if (m_drag) {
            if (m_pressedButton == Qt::LeftButton) {
                emit leftDrawn(this);
            } else if (m_pressedButton == Qt::RightButton) {
                emit rightDrawn(this);
            } else {

            }
        } else {
            if (m_pressedButton == Qt::LeftButton) {
                emit leftClicked(this, m_startX, m_startY);
            } else if (m_pressedButton == Qt::RightButton) {
                emit rightClicked(this, m_startX, m_startY);
            } else {

            }
        }
    }
    resetItems();
}

void PolygonQGraphicsScene::clear() {
    QGraphicsScene::clear();
    addItems();
}
