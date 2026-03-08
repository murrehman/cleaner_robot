#pragma once

#include "geometry/transforms.hpp"
#include "cleaning/cleaning_area.hpp"
#include <vector>
#include <QWidget>
#include <QGraphicsView>
#include <QGraphicsScene>

namespace robot_cleaner {

class Viewer : public QGraphicsView {
    Q_OBJECT

public:
    Viewer(const Path& path,
           const RobotModel& robot,
           const CleaningGadget& gadget,
           const std::vector<double>& curvatures,
           const MultiPolygon& swept_area,
           QWidget* parent = nullptr);

protected:
    void wheelEvent(QWheelEvent* event) override;

private:
    void drawSweptArea();
    void drawPath();
    void drawRobotFootprints();

    QGraphicsScene* m_scene;
    Path m_path;
    RobotModel m_robot;
    CleaningGadget m_gadget;
    std::vector<double> m_curvatures;
    MultiPolygon m_swept_area;

    // Scale factor to map physical meters to screen pixels
    const double m_scale = 100.0; 
};

} // namespace robot_cleaner
