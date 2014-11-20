#ifndef POLYGONQGRAPHICSSCENE_H
#define POLYGONQGRAPHICSSCENE_H

#include <QGraphicsSceneMouseEvent>
#include <QGraphicsRectItem>
#include <QGraphicsScene>

enum DrawingMode
{
    RECT,
    POLYGON
};

class PolygonQGraphicsScene : public QGraphicsScene
{
    Q_OBJECT

public:
    PolygonQGraphicsScene(QObject *parent = 0);

	QPolygon getPolygon();
    void updateItems(QGraphicsSceneMouseEvent *event);

    void setDrawingMode(DrawingMode drawingMode);
    DrawingMode getDrawingMode();

    bool checkModifiers(Qt::KeyboardModifier modifier);

    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

public Q_SLOTS:
	void clear();

private:
    DrawingMode m_drawingMode;

    int m_startX, m_startY;

    bool m_draw, m_drag;
	Qt::MouseButton m_pressedButton;

    Qt::KeyboardModifiers m_modifiers;

	QPolygon m_polygon;
    QGraphicsPolygonItem* m_polygonItem;

    void addItems();
    void resetItems();

Q_SIGNALS:
    void leftDrawn(PolygonQGraphicsScene* scn);
    void rightDrawn(PolygonQGraphicsScene* scn);
    void leftClicked(PolygonQGraphicsScene* scn, int x, int y);
    void rightClicked(PolygonQGraphicsScene* scn, int x, int y);
};

#endif // POLYGONQGRAPHICSSCENE_H
