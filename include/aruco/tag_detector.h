#include <deque>
#include <mutex>
#include <chrono>
#include <vector>
#include <memory>
#include <utility>
#include <shared_mutex>
#include <unordered_map>

#include <Eigen/Core>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>


struct TagDescription
{
    using Ptr = std::shared_ptr<TagDescription>;
    using ConstPtr = std::shared_ptr<const TagDescription>;

    std::array<cv::Point3f, 4>
        world_corners,
        rel_corners;

    union
    {
        struct{ double x, y, z; };
        double translation[3];
    };
    union
    {
        struct{ double qw, qx, qy, qz; };
        double rotation[4];
    };
    union
    {
        struct{ double a, b, c, d; };
        double plane[4];
    };

    static Ptr fromRaw(const std::vector<double>& world_corner_pts);
};

class TagDetector
{
public:
    struct Detection
    {
        using Ptr = std::shared_ptr<Detection>;
        using ConstPtr = std::shared_ptr<const Detection>;

        union
        {
            struct
            {
                double x, y, z, qw, qx, qy, qz, qww;
            };
            struct
            {
                double translation[3], rotation[5];
            };
        };

        double tags_area, avg_range, rms;
    };

    using DetectionQueue = std::deque<std::pair<std::chrono::system_clock::time_point, std::vector<Detection::ConstPtr>>>;

public:
    TagDetector(int aruco_predefined_dict = cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_APRILTAG_36h11);

    bool addTag(int id, const std::vector<double>& world_corners);

    cv::Ptr<cv::aruco::Dictionary> accessDictionary(std::unique_lock&);
    cv::Ptr<cv::aruco::DetectorParameters> accessParams(std::unique_lock&);

    void process(
        const cv::Mat& frame,
        cv::InputArray calib,
        cv::InputArray distort,
        const Eigen::Isometry3d& c2base,
        cv::Mat& debug);

    // queue access

protected:
private:
    std::unordered_map<int, TagDescription::ConstPtr> obj_tag_corners;
    cv::Ptr<cv::aruco::Dictionary> aruco_dict;
    cv::Ptr<cv::aruco::DetectorParameters> aruco_params;

    DetectionQueue detections;

    std::shared_mutex params_mtx, output_mtx;

};