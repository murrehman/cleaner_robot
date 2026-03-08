#include "io/json_loader.hpp"
#include "geometry/path_metrics.hpp"
#include "geometry/curvature.hpp"
#include "cleaning/cleaning_area.hpp"
#include "visualization/viewer.hpp"
#include <QApplication>
#include <iostream>
#include <iomanip>

using namespace robot_cleaner;

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);

    std::string data_path = "data/short.json";
    if (argc > 1) {
        data_path = argv[1];
    }

    try {
        InputData data = load_json(data_path);

        // Compute metrics
        double path_length = compute_total_path_length(data.path);
        
        std::vector<double> curvatures = compute_path_curvature(data.path);
        
        VelocityProfileParams params;
        double traversal_time = compute_traversal_time(data.path, curvatures, params);
        
        double cleaned_area = compute_cleaned_area(data.path, data.gadget);

        // Print results (Console output requirement)
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "Path length: " << path_length << " m\n";
        std::cout << "Cleaned area: " << cleaned_area << " m²\n";
        std::cout << "Traversal time: " << traversal_time << " s\n";

        // Optional: Compute explicit polygon for visualization
        MultiPolygon swept_poly = compute_swept_polygon(data.path, data.gadget);

        // Visualization
        Viewer viewer(data.path, data.robot, data.gadget, curvatures, swept_poly);
        viewer.setWindowTitle("Robot Cleaner Visualization");
        viewer.resize(800, 600);
        viewer.show();

        return app.exec();

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
