#ifndef FLOWEDITOR_FLOWVIEW_H
#define FLOWEDITOR_FLOWVIEW_H

#include <QGraphicsView>

class FlowScene;

class FlowView : public QGraphicsView
{
    Q_OBJECT
public:
    explicit FlowView(FlowScene *scene, QWidget *parent = nullptr);

    QPointF centerScenePos() const;
    void frameItems();

signals:
    void zoomChanged(qreal factor);

protected:
    void dragEnterEvent(QDragEnterEvent *event) override;
    void dragMoveEvent(QDragMoveEvent *event) override;
    void dropEvent(QDropEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;
    void paintEvent(QPaintEvent *event) override;
    bool eventFilter(QObject *watched, QEvent *event) override;

private:
    bool acceptStepDrag(QDropEvent *event) const;

    FlowScene *m_scene = nullptr;
    bool m_panning = false;
    QPoint m_panAnchor;
};

#endif
