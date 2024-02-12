#include "geometryMsgs.hpp"

PoseWithCovariance::PoseWithCovariance()
    : position(0,0,0)
    , orientation(0,0,0,0)
    , covariance(Eigen::Matrix<double,6,6>::Zero())
{

}

PoseWithCovariance::PoseWithCovariance(
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& orientation,
        const Eigen::Matrix<double,6,6>& covariance
        )
    : position(position)
    , orientation(orientation)
    , covariance(covariance)
{

}

std::ostream& operator<<(std::ostream& os, const PoseWithCovariance& pose)
{
    os << "PoseWithCovariance {" << std::endl;
    os << "  position: " << pose.position.transpose() << std::endl;
    os << "  orientation: " << pose.orientation << std::endl;
    os << "  covariance: " << std::endl << pose.covariance << std::endl;
    os << "}";
    return os;
}

PredictedObject::PredictedObject(size_t length)
    : predictedPath(length)
{

}

PredictedObject::PredictedObject(
        int object_id,
        std::vector<PoseWithCovariance> predictedPath,
        double length,
        double width
)
    : object_id(object_id)
    , length(length)
    , width(width)
    , predictedPath(predictedPath)
{
}

std::ostream& operator<<(std::ostream& os, const PredictedObject& obj)
{
    os << "PredictedObject {" << std::endl;
    os << "  object_id: " << obj.object_id << std::endl;
    os << "  length: " << obj.length << std::endl;
    os << "  width: " << obj.width << std::endl;
    os << "  predictedPath: [" << std::endl;
    for (const auto& pose : obj.predictedPath)
    {
        os << "    " << pose << std::endl;
    }
    os << "  ]" << std::endl;
    os << "}" << std::endl;
    return os;
}
