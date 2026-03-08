#include "visualization/viewer.hpp"
#include "geometry/transforms.hpp"
#include <QGraphicsPolygonItem>
#include <QGraphicsPathItem>
#include <QPen>
#include <QBrush>
#include <QColor>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>
#include "config/constants.hpp"

namespace robot_cleaner {

Viewer::Viewer(const Path& path,
               const RobotModel& robot,
               const CleaningGadget& gadget,
               const std::vector<double>& curvatures,
               const MultiPolygon& swept_area,
               QWidget* parent)
    : QGraphicsView(parent),
      m_path(path),
      m_robot(robot),
      m_gadget(gadget),
      m_curvatures(curvatures),
      m_swept_area(swept_area)
{
    m_scene = new QGraphicsScene(this);
    setScene(m_scene);
    setRenderHint(QPainter::Antialiasing);
    setDragMode(QGraphicsView::ScrollHandDrag);
    setBackgroundBrush(Qt::white);

    // Coordinate system: Qt y is down, let's flip it so y is up for math conventions
    scale(1, -1);

    drawSweptArea();
    drawRobotFootprints();
    drawPath();

    // Determine bounding rect to fit in view
    QRectF bounds = m_scene->itemsBoundingRect();
    bounds.adjust(-100, -100, 100, 100);
    setSceneRect(bounds);
    fitInView(bounds, Qt::KeepAspectRatio);
}

void Viewer::wheelEvent(QWheelEvent* event) {
    auto angle = event->angleDelta().y();
    double factor = std::pow(1.0015, angle);
    scale(factor, factor);
}

void Viewer::drawSweptArea() {
    QColor areaColor(0, 150, 255, 100); // Semi-transparent blue
    QBrush brush(areaColor);
    QPen pen(Qt::NoPen);

    for (const auto& ring : m_swept_area.outer_rings) {
        QPolygonF qpoly;
        for (const auto& pt : ring) {
            qpoly << QPointF(pt.x * m_scale, pt.y * m_scale);
        }
        m_scene->addPolygon(qpoly, pen, brush);
    }
}

void Viewer::drawRobotFootprints() {
    QPen robotPen(Qt::darkGray, 1.0);
    QPen gadgetPen(Qt::red, 3.0);
    
    // Draw robot footprints at sampled poses (e.g. every point for a small path)
    for (size_t i = 1; i < m_path.points.size(); ++i) {
        const auto& p1 = m_path.points[i-1];
        const auto& p2 = m_path.points[i];
        double heading = compute_heading(p1, p2);
        
        // Draw at p2
        Pose2D pose{p2.x, p2.y, heading};
        
        QPolygonF qpoly;
        for (const auto& pt : m_robot.polygon) {
            Point2D t_pt = transform_point(pt, pose);
            qpoly << QPointF(t_pt.x * m_scale, t_pt.y * m_scale);
        }
        
        m_scene->addPolygon(qpoly, robotPen, QBrush(Qt::NoBrush));

        // Draw gadget
        Point2D g0 = transform_point(m_gadget.p0, pose);
        Point2D g1 = transform_point(m_gadget.p1, pose);
        m_scene->addLine(g0.x * m_scale, g0.y * m_scale, g1.x * m_scale, g1.y * m_scale, gadgetPen);
    }
}

void Viewer::drawPath() {
    if (m_path.points.size() < 2) return;

    double max_curv = 0.0;
    for (double c : m_curvatures) max_curv = std::max(max_curv, c);
    
    for (size_t i = 1; i < m_path.points.size(); ++i) {
        const auto& p1 = m_path.points[i-1];
        const auto& p2 = m_path.points[i];
        
        double avg_curv = (m_curvatures[i-1] + m_curvatures[i]) / 2.0;
        
        // Map curvature to color (green = straight (low curvature), red = sharp (high curvature))
        double ratio = std::min(1.0, avg_curv / config::K_MAX); // Using K_MAX as max mapping
        int r = static_cast<int>(255 * ratio);
        int g = static_cast<int>(255 * (1.0 - ratio));
        int b = 0;
        QColor segColor(r, g, b);

        QPen pathPen(segColor, 3.0);
        pathPen.setCapStyle(Qt::RoundCap);
        
        m_scene->addLine(p1.x * m_scale, p1.y * m_scale, p2.x * m_scale, p2.y * m_scale, pathPen);
    }
}

} // namespace robot_cleaner


