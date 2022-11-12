//
// Created by Abdullah Al Redwan Newaz on 11/10/22.
//

#ifndef BT_CONTROLLER_GEN_WAYPOINTS_H
#define BT_CONTROLLER_GEN_WAYPOINTS_H
#include "COM.h"


using namespace BT;

typedef std::vector<std::vector<double>> COORDS;
struct Pose2D
{
    double x, y, theta;
};

/**""
 * @brief Dummy action that generates a list of poses.
 */
class GenerateWaypoints : public SyncActionNode
{
public:
    GenerateWaypoints(const std::string& name, const NodeConfig& config, const string& filename) :
            SyncActionNode(name, config), filename_(filename)
    {}

    NodeStatus tick() override
    {
        auto queue = std::make_shared<ProtectedQueue<Pose2D>>();
        auto roomba20 = get_coordinates(filename_);
        auto angles = get_angles(roomba20);
        int index = 0;
        for (const auto& point: roomba20)
        {
            queue->items.push_back(Pose2D{point[0], point[1], angles[index++]});
        }
        setOutput("waypoints", queue);
        return NodeStatus::SUCCESS;
    }

    static PortsList providedPorts()
    {
        return {OutputPort<std::shared_ptr<ProtectedQueue<Pose2D>>>("waypoints")};
    }
private:
    std::vector<double> get_angles(const COORDS& points)
    {
        std::vector<double> angles(points.size());
//        for (int i = 1; i < points.size(); ++i) {
//            double dx = points[i][0] - points[i-1][0];
//            double dy = points[i][1] - points[i-1][1];
//            angles[i] = atan2(dy, dx);
//        }

//        angles[0] = angles[1];
        for (int i = 0; i < points.size(); ++i) {
            angles[i] = atan2(points[i][1], points[i][0]);
        }
        return angles;

    }
    COORDS get_coordinates(const std::string& filename)
    {
        rapidcsv::Document doc(filename, rapidcsv::LabelParams(-1, -1));
        COORDS coords;
        auto x = doc.GetColumn<double>(0);
        auto y = doc.GetColumn<double>(1);
        for (int i = 0; i < x.size(); ++i) {
            coords.push_back({x[i], y[i]});
        }
        return coords;
    }
private:
    std::string filename_;

};

#endif //BT_CONTROLLER_GEN_WAYPOINTS_H
