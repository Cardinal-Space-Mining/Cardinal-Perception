#include <string>
#include <sstream>
#include <memory>
#include <chrono>
#include <vector>
#include <array>
#include <utility>
#include <format>
#include <deque>
#include <functional>
#include <unordered_map>
#include <type_traits>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/float64.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>

#include <opencv2/core/quaternion.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>


namespace util {

	template<typename T>
	struct identity { typedef T type; };

	template<typename T>
	void declare_param(rclcpp::Node* node, const std::string param_name, T& param, const typename identity<T>::type& default_value) {
		node->declare_parameter(param_name, default_value);
		node->get_parameter(param_name, param);
	}

	template<typename ros_T, typename primitive_T>
	inline constexpr ros_T& to_ros(primitive_T& v)
	{
		static_assert(
			std::is_same<typename ros_T::_data_type, primitive_T>::value &&
			sizeof(ros_T) == sizeof(primitive_T) &&
			alignof(ros_T) == alignof(primitive_T) );

		return reinterpret_cast<ros_T&>(v);
	}

	inline constexpr std_msgs::msg::Bool& to_ros(bool& v)
	{
		return reinterpret_cast<std_msgs::msg::Bool&>(v);
	}
	inline constexpr std_msgs::msg::Int64& to_ros(int64_t& v)
	{
		return reinterpret_cast<std_msgs::msg::Int64&>(v);
	}
	inline constexpr std_msgs::msg::Float64& to_ros(double& v)
	{
		return reinterpret_cast<std_msgs::msg::Float64&>(v);
	}

	template<typename ros_T, typename primitive_T>
	inline ros_T to_ros_val(primitive_T v)
	{
		static_assert(std::is_same<typename ros_T::_data_type, primitive_T>::value);

		return ros_T{}.set__data(v);
	}

	inline std_msgs::msg::Bool to_ros_val(bool v)
	{
		return std_msgs::msg::Bool{}.set__data(v);
	}
	inline std_msgs::msg::Int64 to_ros_val(int64_t v)
	{
		return std_msgs::msg::Int64{}.set__data(v);
	}
	inline std_msgs::msg::Float64 to_ros_val(double v)
	{
		return std_msgs::msg::Float64{}.set__data(v);
	}

}


class ArucoServer : public rclcpp::Node
{
public:
	ArucoServer();

protected:
	void publish_debug_frame();
	void publish_alignment();

	struct ImageSource
	{
		ArucoServer* ref;

		image_transport::Subscriber image_sub;
		rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;

		cv::Mat last_frame;
		cv::Mat1d
			calibration = cv::Mat1d::zeros(3, 3),
			// undistorted_calib = cv::Mat1f::zeros(3, 3),
			distortion = cv::Mat1d::zeros(1, 5);

		bool valid_calib_data = false;

		void img_callback(const sensor_msgs::msg::Image::ConstSharedPtr& img);
		void info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info);
	};

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

private:
	std::unordered_map<int, TagDescription::ConstPtr> obj_tag_corners;
	std::vector<ImageSource> sources;
	std::deque<std::pair<std::chrono::system_clock::time_point, Eigen::Vector<double, 6>>> covariance_samples;

	image_transport::ImageTransport img_tp;
	image_transport::Publisher debug_pub;
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub;
	rclcpp::TimerBase::SharedPtr tf_pub;

	tf2_ros::Buffer tfbuffer;
	tf2_ros::TransformListener tflistener;
	tf2_ros::TransformBroadcaster tfbroadcaster;

	geometry_msgs::msg::TransformStamped last_alignment;

	std::chrono::system_clock::time_point last_img_publish, last_nav_publish;

	cv::Ptr<cv::aruco::Dictionary> aruco_dict;
	cv::Ptr<cv::aruco::DetectorParameters> aruco_params;

	struct
	{
		std::vector<std::vector<cv::Point2f>> tag_corners;
		std::vector<int32_t> tag_ids;
		std::vector<cv::Point2f> img_points;
		std::vector<cv::Point3f> obj_points;
		std::vector<cv::Vec3d> tvecs, rvecs;
		std::vector<double> eerrors;
	}
	_detect;

	struct
	{
		Eigen::AlignedBox3d filter_bbox;

		std::chrono::duration<double>
			debug_pub_duration,
			covariance_sample_history;

		std::string
			tags_frame_id,
			odom_frame_id,
			base_frame_id;

		double
			fitness_linear_diff_velocity_weight,
			fitness_angular_diff_velocity_weight,
			fitness_oob_weight,
			fitness_rms_weight,
			thresh_max_linear_diff_velocity,
			thresh_max_angular_diff_velocity,
			thresh_min_tags_per_range,
			thresh_max_rms_per_tag,
			covariance_linear_base_coeff,
			covariance_linear_range_coeff,
			covariance_angular_base_coeff,
			covariance_angular_range_coeff;

		bool
			enable_nav_tf,
			enable_live_covariance;
	}
	_param;

	struct
	{
		rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
			// best_in_bounds,
			did_nav_alignment;
		rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr
			tag_matches;
		rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
			// best_score,
			highest_rms,
			sum_area,
			linear_variance,
			angular_variance,
			best_rms,
			tags_per_range,
			rms_per_tag,
			range,
			alignment_time_diff;
	}
	_pub;
	

};





ArucoServer::ArucoServer():
	Node{ "aruco_server" },
	img_tp{ std::shared_ptr<ArucoServer>(this, [](auto*){}) },
	tfbuffer{ std::make_shared<rclcpp::Clock>(RCL_ROS_TIME) },
	tflistener{ tfbuffer },
	tfbroadcaster{ *this }
{
	std::vector<std::string> img_topics, info_topics;
	util::declare_param<std::vector<std::string>>(this, "img_topics", img_topics, {});
	util::declare_param<std::vector<std::string>>(this, "info_topics", info_topics, {});

	const size_t
		n_img_t = img_topics.size(),
		n_info_t = info_topics.size();
	if(n_img_t < 1 || n_info_t < n_img_t)
	{
		RCLCPP_ERROR(this->get_logger(), "No image topics provided or mismatched image/info topics");
		return;	// exit?
	}

	std::string dbg_topic;
	util::declare_param(this, "debug_topic", dbg_topic, "aruco_server/debug/image");
	this->debug_pub = this->img_tp.advertise(dbg_topic, 1);

	int pub_freq;
	util::declare_param(this, "debug_pub_freq", pub_freq, 30);
	this->_param.debug_pub_duration = std::chrono::duration<double>{ 1. / pub_freq };

	std::string pose_topic;
	util::declare_param(this, "pose_pub_topic", pose_topic, "/aruco_server/pose");
	this->pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic, 10);

	util::declare_param(this, "tags_frame_id", this->_param.tags_frame_id, "world");
	util::declare_param(this, "odom_frame_id", this->_param.odom_frame_id, "odom");
	util::declare_param(this, "base_frame_id", this->_param.base_frame_id, "base_link");

	util::declare_param(this, "enable_nav_tf_alignment", this->_param.enable_nav_tf, false);
	if(this->_param.enable_nav_tf)
	{
		this->tf_pub = this->create_wall_timer(std::chrono::milliseconds(20), std::bind( &ArucoServer::publish_alignment, this ));
	}

	std::vector<double> min, max;
	util::declare_param(this, "filtering.bounds_min", min, {});
	util::declare_param(this, "filtering.bounds_max", max, {});
	if(min.size() > 2 && max.size() > 2)
	{
		this->_param.filter_bbox = Eigen::AlignedBox3d{ *reinterpret_cast<Eigen::Vector3d*>(min.data()), *reinterpret_cast<Eigen::Vector3d*>(max.data()) };
	}

	util::declare_param(this, "filtering.fitness.lienar_diff_velocity_weight", this->_param.fitness_linear_diff_velocity_weight, 0.);
	util::declare_param(this, "filtering.fitness.angular_diff_velocity_weight", this->_param.fitness_angular_diff_velocity_weight, 0.);
	util::declare_param(this, "filtering.fitness.oob_weight", this->_param.fitness_oob_weight, 100.);
	util::declare_param(this, "filtering.fitness.rms_weight", this->_param.fitness_rms_weight, 10.0);

	util::declare_param(this, "filtering.tf_pub_threshold.max_linear_diff_velocity", this->_param.thresh_max_linear_diff_velocity, 5.);
	util::declare_param(this, "filtering.tf_pub_threshold.max_angular_diff_velocity", this->_param.thresh_max_angular_diff_velocity, 5.);
	util::declare_param(this, "filtering.tf_pub_threhsold.min_tags_per_range", this->_param.thresh_min_tags_per_range, 0.5);
	util::declare_param(this, "filtering.tf_pub_threshold.max_rms_per_tag", this->_param.thresh_max_rms_per_tag, 0.1);

	util::declare_param(this, "covariance.use_live_sampler", this->_param.enable_live_covariance, false);
	double covariance_sampling_history;
	util::declare_param(this, "covariance.sampling_history", covariance_sampling_history, 1.0);
	this->_param.covariance_sample_history = std::chrono::duration<double>{ covariance_sampling_history };

	util::declare_param(this, "covariance.linear_base_coeff", this->_param.covariance_linear_base_coeff, 0.001);
	util::declare_param(this, "covariance.linear_range_coeff", this->_param.covariance_linear_range_coeff, 0.001);
	util::declare_param(this, "covariance.angular_base_coeff", this->_param.covariance_angular_base_coeff, 0.001);
	util::declare_param(this, "covariance.angular_range_coeff", this->_param.covariance_angular_range_coeff, 0.001);

	int aruco_dict_id = cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_APRILTAG_36h11;
	util::declare_param(this, "aruco_predefined_family_idx", aruco_dict_id, aruco_dict_id);

	// TODO: expose detector params

	this->aruco_dict = cv::aruco::getPredefinedDictionary(aruco_dict_id);
	this->aruco_params = cv::aruco::DetectorParameters::create();

	std::vector<double> tag_ids;
	util::declare_param(this, "tag_ids", tag_ids, {});

	const size_t n_tags = tag_ids.size();
	if(n_tags < 1)
	{
		RCLCPP_ERROR(this->get_logger(), "No tag ids enabled!");
		return;	// exit?
	}

	std::vector<double> param_buff;
	for(size_t i = 0; i < n_tags; i++)
	{
		param_buff.clear();
		const int id = static_cast<int>(tag_ids[i]);

		std::stringstream topic;
		topic << "tag" << id;
		util::declare_param(this, topic.str(), param_buff, {});

		auto ptr = this->obj_tag_corners.insert({ id, TagDescription::fromRaw(param_buff) });
		if(ptr.second && ptr.first->second)	// successful insertion and successful parse
		{
			auto desc = ptr.first->second;

			RCLCPP_INFO(this->get_logger(),
				"Successfully added description for tag %d :\n"
				"Corners -- [\n\t(%f, %f, %f),\n\t(%f, %f, %f),\n\t(%f, %f, %f),\n\t(%f, %f, %f)\n]\n"
				"Pose -- (%f, %f, %f) ~ [%f, %f, %f, %f]",
				id,
				desc->world_corners[0].x,
				desc->world_corners[0].y,
				desc->world_corners[0].z,
				desc->world_corners[1].x,
				desc->world_corners[1].y,
				desc->world_corners[1].z,
				desc->world_corners[2].x,
				desc->world_corners[2].y,
				desc->world_corners[2].z,
				desc->world_corners[3].x,
				desc->world_corners[3].y,
				desc->world_corners[3].z,
				desc->x,
				desc->y,
				desc->z,
				desc->qw,
				desc->qx,
				desc->qy,
				desc->qz);
		}
		else
		{
			this->obj_tag_corners.erase(id);	// remove since invalid
			RCLCPP_ERROR(this->get_logger(), "Failed to setup description for tag %d", id);
		}
	}

	this->sources.resize(n_img_t);
	for(size_t i = 0; i < n_img_t; i++)
	{
		RCLCPP_DEBUG(this->get_logger(), "Source %ld using image topic [%s] and info topic [%s]", i, img_topics[i].c_str(), info_topics[i].c_str());

		this->sources[i].ref = this;
		this->sources[i].image_sub =
			this->img_tp.subscribe( img_topics[i], 1,
				std::bind( &ArucoServer::ImageSource::img_callback, &this->sources[i], std::placeholders::_1 ) );
		this->sources[i].info_sub =
			this->create_subscription<sensor_msgs::msg::CameraInfo>( info_topics[i], 10,
				std::bind( &ArucoServer::ImageSource::info_callback, &this->sources[i], std::placeholders::_1 ) );
	}

	{
		// this->_pub.best_in_bounds = this->create_publisher<std_msgs::msg::Bool>("/aruco_server/debug/best_is_in_bounds", 1);
		this->_pub.did_nav_alignment = this->create_publisher<std_msgs::msg::Bool>("/aruco_server/debug/did_nav_alignment", 1);
		this->_pub.tag_matches = this->create_publisher<std_msgs::msg::Int64>("/aruco_server/debug/tag_matches", 1);
		// this->_pub.best_score = this->create_publisher<std_msgs::msg::Float64>("/aruco_server/debug/best/score", 1);
		this->_pub.highest_rms = this->create_publisher<std_msgs::msg::Float64>("/aruco_server/debug/highest_rms", 1);
		this->_pub.sum_area = this->create_publisher<std_msgs::msg::Float64>("/aruco_server/debug/sum_area", 1);
		this->_pub.linear_variance = this->create_publisher<std_msgs::msg::Float64>("/aruco_server/debug/linear_variance", 1);
		this->_pub.angular_variance = this->create_publisher<std_msgs::msg::Float64>("/aruco_server/debug/angular_variance", 1);
		this->_pub.best_rms = this->create_publisher<std_msgs::msg::Float64>("/aruco_server/debug/best_rms", 1);
		this->_pub.tags_per_range = this->create_publisher<std_msgs::msg::Float64>("/aruco_server/debug/tags_per_range", 1);
		this->_pub.rms_per_tag = this->create_publisher<std_msgs::msg::Float64>("/aruco_server/debug/rms_per_tag", 1);
		this->_pub.range = this->create_publisher<std_msgs::msg::Float64>("/aruco_server/debug/range", 1);
		this->_pub.alignment_time_diff = this->create_publisher<std_msgs::msg::Float64>("/aruco_server/debug/alignment_diff_time", 1);
	}
}

void ArucoServer::publish_debug_frame()
{
	std::vector<cv::Mat> frames;
	frames.reserve(this->sources.size());
	for(const ImageSource& s : this->sources)
	{
		if(s.last_frame.size().area() > 0)
		{
			frames.insert(frames.end(), s.last_frame);
		}
	}

	if(frames.size() > 0)
	{
		try {
			cv::Mat pub;
			cv::hconcat(frames, pub);

			std_msgs::msg::Header hdr;
			hdr.frame_id = this->_param.base_frame_id;
			hdr.stamp = this->get_clock()->now();
			this->debug_pub.publish(cv_bridge::CvImage(hdr, "bgr8", pub).toImageMsg());
		}
		catch(const std::exception& e)
		{
			RCLCPP_ERROR(this->get_logger(), "Failed to publish debug image concatenation: %s", e.what());
		}
	}
}

void ArucoServer::publish_alignment()
{
	auto _now = std::chrono::system_clock::now();
	if(_now - this->last_nav_publish > std::chrono::milliseconds(20))
	{
		this->last_alignment.header.frame_id = this->_param.tags_frame_id;
		this->last_alignment.header.stamp = this->get_clock()->now();
		this->last_alignment.child_frame_id = this->_param.odom_frame_id;

		this->tfbroadcaster.sendTransform(this->last_alignment);
		this->last_nav_publish = _now;

		// RCLCPP_INFO(this->get_logger(), "Manually republished tags->odom TF.");
	}
}

void ArucoServer::ImageSource::img_callback(const sensor_msgs::msg::Image::ConstSharedPtr& img)
{
	if(!this->valid_calib_data) return;

	cv_bridge::CvImageConstPtr cv_img = cv_bridge::toCvCopy(*img, "mono8");
	cv::cvtColor(cv_img->image, this->last_frame, CV_GRAY2BGR);

	ref->_detect.tag_corners.clear();
	ref->_detect.tag_ids.clear();

	// cv::Mat undistorted;
	// try {
	// 	cv::undistort(
	// 		cv_img->image,
	// 		undistorted,
	// 		this->calibration,
	// 		this->distortion);
	// }
	// catch(const std::exception& e)
	// {
	// 	RCLCPP_ERROR(ref->get_logger(), "Error undistorting image: %s", e.what());
	// 	// detect markers with original image...
	// 	undistorted = cv_img->image;
	// }

	try {
		cv::aruco::detectMarkers(
			cv_img->image,
			ref->aruco_dict,
			ref->_detect.tag_corners,
			ref->_detect.tag_ids,
			ref->aruco_params);
	}
	catch(const std::exception& e)
	{
		RCLCPP_ERROR(ref->get_logger(), "Error detecting markers: %s", e.what());
	}

	cv::aruco::drawDetectedMarkers(this->last_frame, ref->_detect.tag_corners, ref->_detect.tag_ids, cv::Scalar{0, 255, 0});

	if(const size_t n_detected = ref->_detect.tag_ids.size(); (n_detected > 0 && n_detected == ref->_detect.tag_corners.size()))	// valid detection(s)
	{
		ref->_detect.rvecs.clear();
		ref->_detect.tvecs.clear();
		ref->_detect.eerrors.clear();

		ref->_detect.obj_points.clear();
		ref->_detect.obj_points.reserve(n_detected * 4);
		ref->_detect.img_points.clear();
		ref->_detect.img_points.reserve(n_detected * 4);

		size_t matches = 0;
		bool all_coplanar = true;
		cv::Vec4d _plane = cv::Vec4d::zeros();
		TagDescription::ConstPtr primary_desc;
		std::vector<double> ranges;
		double
			sum_area = 0.,
			highest_individual_rms = 0.;

		ranges.reserve(n_detected * 2);
		for(size_t i = 0; i < n_detected; i++)
		{
			auto search = ref->obj_tag_corners.find(ref->_detect.tag_ids[i]);
			if(search != ref->obj_tag_corners.end())
			{
				auto& result = search->second;
				auto& obj_corners = result->world_corners;
				auto& rel_obj_corners = result->rel_corners;
				auto& img_corners = ref->_detect.tag_corners[i];

				ref->_detect.obj_points.insert(ref->_detect.obj_points.end(), obj_corners.begin(), obj_corners.end());
				ref->_detect.img_points.insert(ref->_detect.img_points.end(), img_corners.begin(), img_corners.end());

				if(++matches == 1) primary_desc = result;	// first match

				sum_area += cv::contourArea(img_corners);

				// for(size_t x = 0; x < obj_corners.size(); x++)
				// {
				// 	cv::putText(this->last_frame, std::format("({}, {}, {})", obj_corners[x].x, obj_corners[x].y, obj_corners[x].z), static_cast<cv::Point>(img_corners[x]), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(255, 100, 0), 1, cv::LINE_AA);
				// }

				cv::solvePnPGeneric(
					rel_obj_corners,
					img_corners,
					this->calibration,
					this->distortion,
					ref->_detect.rvecs,
					ref->_detect.tvecs,
					false,
					cv::SOLVEPNP_IPPE_SQUARE,
					cv::noArray(),
					cv::noArray(),
					ref->_detect.eerrors);

				// cv::drawFrameAxes(this->last_frame, this->calibration, this->distortion, _rvec, _tvec, 0.2f, 3);

				if(ref->_detect.eerrors[0] > highest_individual_rms) highest_individual_rms = ref->_detect.eerrors[0];

				for(size_t s = 0; s < ref->_detect.rvecs.size(); s++)
				{
					cv::Vec3d
						&_rvec = ref->_detect.rvecs[s],
						&_tvec = ref->_detect.tvecs[s];

					ranges.push_back(cv::norm(_tvec));

					cv::Quatd q = cv::Quatd::createFromRvec(_rvec);

					geometry_msgs::msg::TransformStamped tfs;
					tfs.header = cv_img->header;
					tfs.child_frame_id = std::format("tag_{}_s{}", ref->_detect.tag_ids[i], s);
					tfs.transform.translation = reinterpret_cast<geometry_msgs::msg::Vector3&>(_tvec);
					tfs.transform.rotation.w = q.w;
					tfs.transform.rotation.x = q.x;
					tfs.transform.rotation.y = q.y;
					tfs.transform.rotation.z = q.z;

					ref->tfbroadcaster.sendTransform(tfs);
				}

				if(all_coplanar)
				{
					const cv::Vec4d& _p = reinterpret_cast<const cv::Vec4d&>(result->plane);
					if(matches == 1) _plane = _p;
					else if(1.0 - std::abs(_plane.dot(_p)) > 1e-6) all_coplanar = false;
				}
			}
		}

		// RCLCPP_INFO(ref->get_logger(), "MEGATAG solve params -- obj points: %lu, img points: %lu", ref->_detect.obj_points.size(), ref->_detect.img_points.size());

		if(matches == 1)	// reuse debug tvecs and rvecs ^^
		{
			Eigen::Isometry3d tf = ( Eigen::Quaterniond{ primary_desc->qw, primary_desc->qx, primary_desc->qy, primary_desc->qz } *
				Eigen::Translation3d{ -reinterpret_cast<const Eigen::Vector3d&>(primary_desc->translation) } );

			const size_t n_solutions = ref->_detect.tvecs.size();
			for(size_t i = 0; i < n_solutions; i++)
			{
				cv::Quatd r = cv::Quatd::createFromRvec(ref->_detect.rvecs[i]);
				Eigen::Isometry3d _tf = reinterpret_cast<Eigen::Translation3d&>(ref->_detect.tvecs[i]) * (Eigen::Quaterniond{ r.w, r.x, r.y, r.z } * tf);

				Eigen::Vector3d _t;
				_t = _tf.translation();
				ref->_detect.tvecs[i] = reinterpret_cast<cv::Vec3d&>(_t);

				Eigen::Quaterniond _r;
				_r = _tf.rotation();
				ref->_detect.rvecs[i] = cv::Quatd{ _r.w(), _r.x(), _r.y(), _r.z() }.toRotVec();
			}

			RCLCPP_INFO(ref->get_logger(), "SolvePnP successfully yielded %lu solution(s) using 1 [COPLANAR] tag match.", n_solutions);
		}
		else if(matches > 1)
		{
			try
			{
				cv::solvePnPGeneric(
					ref->_detect.obj_points,
					ref->_detect.img_points,
					this->calibration,
					this->distortion,
					ref->_detect.rvecs,
					ref->_detect.tvecs,
					false,
					(all_coplanar ? cv::SOLVEPNP_IPPE : cv::SOLVEPNP_SQPNP),	// if coplanar, we want all the solutions so we can manually filter the best
					cv::noArray(),
					cv::noArray(),
					ref->_detect.eerrors);

				RCLCPP_INFO(ref->get_logger(),
					"SolvePnP successfully yielded %lu solution(s) using %lu [%s] tag matches.",
					ref->_detect.tvecs.size(),
					matches,
					all_coplanar ? "COPLANAR" : "non-coplanar");
			}
			catch(const std::exception& e)
			{
				RCLCPP_ERROR(ref->get_logger(), "SolvePnP failed with %lu tag matches: %s", matches, e.what());
			}
		}

		const size_t n_solutions = ref->_detect.tvecs.size();
		if(n_solutions > 0 && n_solutions == ref->_detect.tvecs.size())
		{
			const tf2::TimePoint time_point = tf2::TimePoint{
				std::chrono::seconds{cv_img->header.stamp.sec} +
				std::chrono::nanoseconds{cv_img->header.stamp.nanosec} };

			geometry_msgs::msg::TransformStamped cam2base/*, prev_w2base*/;
			bool valid_cam2base = true, valid_prev_w2base = true;
			try
			{
				cam2base = ref->tfbuffer.lookupTransform(cv_img->header.frame_id, ref->_param.base_frame_id, time_point);		// camera to base link
			}
			catch(const std::exception& e)
			{
				RCLCPP_ERROR(ref->get_logger(), "No transform available from camera frame to base frame!");
				valid_cam2base = false;
			}
			// try
			// {
			// 	// TODO: this might not be what we want (tp really old?)...
			// 	prev_w2base = ref->tfbuffer.lookupTransform(ref->_param.tags_frame_id, ref->_param.base_frame_id, tf2::TimePoint{});
			// }
			// catch(const std::exception& e)
			// {
			// 	RCLCPP_INFO(ref->get_logger(), "Could not fetch previous transform from world to base frame - filtering may be degraded.");
			// 	valid_prev_w2base = false;
			// }

			geometry_msgs::msg::TransformStamped best_tf;
			bool best_is_in_bounds = true;
			double
				best_score = std::numeric_limits<double>::infinity(),
				// best_lv = 0.,
				// best_rv = 0.,
				best_rms = 0.;

			// calculate the best solution
			for(size_t i = 0; i < n_solutions; i++)
			{
				cv::Vec3d
					&_rvec = ref->_detect.rvecs[i],
					&_tvec = ref->_detect.tvecs[i];

				cv::drawFrameAxes(this->last_frame, this->calibration, this->distortion, _rvec, _tvec, 0.5f, 5);

				cv::Quatd r = cv::Quatd::createFromRvec(_rvec);
				Eigen::Translation3d& t = reinterpret_cast<Eigen::Translation3d&>(_tvec);

				Eigen::Quaterniond q{ r.w, r.x, r.y, r.z };
				Eigen::Isometry3d _w2cam = (t * q).inverse();	// world to camera
				Eigen::Quaterniond qi;
				Eigen::Vector3d vi;
				qi = _w2cam.rotation();
				vi = _w2cam.translation();

				geometry_msgs::msg::TransformStamped cam2w;		// camera to tags origin
				cam2w.header = cv_img->header;
				cam2w.child_frame_id = std::format("tags_odom_{}", i);
				cam2w.transform.translation = reinterpret_cast<geometry_msgs::msg::Vector3&>(t);
				cam2w.transform.rotation = reinterpret_cast<geometry_msgs::msg::Quaternion&>(q);

				ref->tfbroadcaster.sendTransform(cam2w);

				if(!valid_cam2base) continue;

				geometry_msgs::msg::TransformStamped w2cam;	// tags origin to camera
				w2cam.transform.translation = reinterpret_cast<geometry_msgs::msg::Vector3&>(vi);
				w2cam.transform.rotation = reinterpret_cast<geometry_msgs::msg::Quaternion&>(qi);
				w2cam.header.stamp = cv_img->header.stamp;
				w2cam.header.frame_id = ref->_param.tags_frame_id;
				w2cam.child_frame_id = cv_img->header.frame_id;

				geometry_msgs::msg::TransformStamped w2base;	// full transform -- world to base
				tf2::doTransform(cam2base, w2base, w2cam);

				// filter based on w2base
				const Eigen::Vector3d&
					current_translation = reinterpret_cast<Eigen::Vector3d&>(w2base.transform.translation)/*,
					prev_translation = reinterpret_cast<const Eigen::Vector3d&>(prev_w2base.transform.translation)*/;
				const Eigen::Quaterniond&
					current_rotation = reinterpret_cast<Eigen::Quaterniond&>(w2base.transform.rotation)/*,
					prev_rotation = reinterpret_cast<const Eigen::Quaterniond&>(prev_w2base.transform.rotation)*/;
				// const double
				// 	t_diff = std::abs((w2base.header.stamp.sec + w2base.header.stamp.nanosec * 1e-9) - (prev_w2base.header.stamp.sec + prev_w2base.header.stamp.nanosec * 1e-9));

				// double _lv = 0., _rv = 0.;
				// if(valid_prev_w2base)	// this is getting confused with dlo jittering
				// {
				// 	_lv = (current_translation - prev_translation).norm() / t_diff,
				// 	_rv = current_rotation.angularDistance(prev_rotation) / t_diff;
				// }
				const bool in_bounds = ref->_param.filter_bbox.isEmpty() || ref->_param.filter_bbox.contains(current_translation);

				const double score =
					// _lv * ref->_param.fitness_linear_diff_velocity_weight +
					// _rv * ref->_param.fitness_angular_diff_velocity_weight +
					(in_bounds ? 0. : ref->_param.fitness_oob_weight) + 
					ref->_detect.eerrors[i] * ref->_param.fitness_rms_weight;

				// RCLCPP_INFO(ref->get_logger(),
				// 	"Pose candidate [%lu] -- RMS: %f -- In bounds?: %d -- Translational(cm/s): %f -- Rotational(deg/s): %f -- SCORE: %f",
				// 	i,
				// 	ref->_detect.eerrors[i],
				// 	(int)in_bounds,
				// 	_lv * 100.,
				// 	_rv * 180. / M_PI,
				// 	score);

				if(score < best_score || i == 0)
				{
					best_tf = w2base;
					best_score = score;
					// best_lv = _lv;
					// best_rv = _rv;
					best_rms = ref->_detect.eerrors[i];
					best_is_in_bounds = in_bounds;
				}
			}

			double avg_range = 0.;
			for(double r : ranges)
			{
				avg_range += r;
			}
			avg_range /= ranges.size();

			double
				tags_per_range = matches / avg_range,
				rms_per_tag = best_rms / matches;

			// publish alignment tf
			bool did_pub_tf = false;
			double base2odom_time = 0.;
			if(ref->_param.enable_nav_tf)
			{
				if( best_is_in_bounds &&
					// best_lv <= ref->_param.thresh_max_linear_diff_velocity &&
					// best_rv <= ref->_param.thresh_max_angular_diff_velocity &&
					tags_per_range >= ref->_param.thresh_min_tags_per_range &&
					rms_per_tag <= ref->_param.thresh_max_rms_per_tag )
				{
					// alignment = full tf + inverse of odom tf ("full - odom")
					geometry_msgs::msg::TransformStamped base2odom;
					bool valid_base2odom = true;
					try
					{
						if(ref->tfbuffer.canTransform(ref->_param.base_frame_id, ref->_param.odom_frame_id, time_point))
							base2odom = ref->tfbuffer.lookupTransform(ref->_param.base_frame_id, ref->_param.odom_frame_id, time_point);
						else if(ref->tfbuffer.canTransform(ref->_param.base_frame_id, ref->_param.odom_frame_id, tf2::TimePoint{}))
							base2odom = ref->tfbuffer.lookupTransform(ref->_param.base_frame_id, ref->_param.odom_frame_id, tf2::TimePoint{});
						else valid_base2odom = false;
					}
					catch(const std::exception& e)
					{
						RCLCPP_INFO(ref->get_logger(), "Failed to fetch transform from base to odom frames: %s", e.what());
						valid_base2odom = false;
					}

					if(valid_base2odom)
					{
						tf2::doTransform(base2odom, ref->last_alignment, best_tf);
						ref->last_alignment.child_frame_id = ref->_param.odom_frame_id;

						ref->tfbroadcaster.sendTransform(ref->last_alignment);
						ref->last_nav_publish = std::chrono::system_clock::now();

						did_pub_tf = true;
						base2odom_time = base2odom.header.stamp.sec + base2odom.header.stamp.nanosec * 1e-9;
					}
				}
				else
				{
					// RCLCPP_INFO(ref->get_logger(),
					// 	"TF Alignment candidate rejected -- Tags/range: %f -- RMS/tag: %f",
					// 	tags_per_range,
					// 	rms_per_tag);
				}
			}

			Eigen::Vector<double, 6> variance = Eigen::Vector<double, 6>::Zero();
			{
				using Vec6d = Eigen::Vector<double, 6>;

				auto _now = std::chrono::system_clock::now();
				while(!ref->covariance_samples.empty())	// remove old samples from back
				{
					if(_now - ref->covariance_samples.back().first > ref->_param.covariance_sample_history)
					{
						ref->covariance_samples.pop_back();
					}
					else break;
				}

				// push new data
				ref->covariance_samples.push_front( { _now, Vec6d::Zero() } );
				*reinterpret_cast<Eigen::Vector3d*>(ref->covariance_samples.front().second.data() + 0) =
					reinterpret_cast<const Eigen::Vector3d&>(best_tf.transform.translation);
				*reinterpret_cast<Eigen::Vector3d*>(ref->covariance_samples.front().second.data() + 3) =
					reinterpret_cast<const Eigen::Quaterniond&>(best_tf.transform.rotation).toRotationMatrix().eulerAngles(0, 1, 2);

				RCLCPP_INFO(ref->get_logger(), "Current covariance samples: %lu", ref->covariance_samples.size());

				// perform calculation using remaining samples
				const size_t samples = ref->covariance_samples.size();

				if(samples > 1)
				{
					Vec6d mean = Vec6d::Zero();
					for(const auto& s : ref->covariance_samples)
					{
						mean += s.second;
					}
					mean /= samples;
					for(const auto& s : ref->covariance_samples)
					{
						Vec6d _v = s.second - mean;
						variance += _v.cwiseProduct(_v);
					}
					variance /= (samples - 1);
				}
			}

			// debug publishers
			{
				ref->_pub.did_nav_alignment->publish(util::to_ros(did_pub_tf));
				ref->_pub.tag_matches->publish(util::to_ros_val((int64_t)matches));
				ref->_pub.highest_rms->publish(util::to_ros(highest_individual_rms));
				ref->_pub.sum_area->publish(util::to_ros(sum_area));
				ref->_pub.linear_variance->publish(util::to_ros_val(cv::norm(*reinterpret_cast<cv::Vec3d*>(variance.data() + 0))));
				ref->_pub.angular_variance->publish(util::to_ros_val(cv::norm(*reinterpret_cast<cv::Vec3d*>(variance.data() + 3))));
				ref->_pub.best_rms->publish(util::to_ros(best_rms));
				ref->_pub.tags_per_range->publish(util::to_ros(tags_per_range));
				ref->_pub.rms_per_tag->publish(util::to_ros(rms_per_tag));
				ref->_pub.range->publish(util::to_ros(avg_range));
				ref->_pub.alignment_time_diff->publish(util::to_ros_val(std::abs((best_tf.header.stamp.sec + best_tf.header.stamp.nanosec * 1e-9) - base2odom_time)));
			}

			// covariance
			if(!ref->_param.enable_live_covariance)
			{
				// covariance from range
				variance[0] = variance[1] = variance[2] = ref->_param.covariance_linear_base_coeff + ref->_param.covariance_linear_range_coeff * avg_range;
				variance[3] = variance[4] = variance[5] = ref->_param.covariance_angular_base_coeff + ref->_param.covariance_angular_range_coeff * avg_range;
			}

			RCLCPP_INFO(ref->get_logger(),
				"Calulcated variance: [%f, %f, %f, %f, %f, %f]",
				variance[0],
				variance[1],
				variance[2],
				variance[3],
				variance[4],
				variance[5]);

			geometry_msgs::msg::PoseWithCovarianceStamped p;
			p.pose.pose.orientation = best_tf.transform.rotation;
			p.pose.pose.position = reinterpret_cast<geometry_msgs::msg::Point&>(best_tf.transform.translation);
			p.header = best_tf.header;

			for(size_t i = 0; i < 6; i++)
			{
				p.pose.covariance[i * 7] = variance[i];
			}

			ref->pose_pub->publish(p);
		}
	}
	else
	{
		// RCLCPP_INFO(ref->get_logger(), "No tags detected in source frame.");
	}

	auto t = std::chrono::system_clock::now();
	if(t - ref->last_img_publish > ref->_param.debug_pub_duration)
	{
		ref->last_img_publish = t;
		ref->publish_debug_frame();
	}
}

void ArucoServer::ImageSource::info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info)
{
	if(
		this->valid_calib_data ||
		info->k.size() != 9 ||
		info->d.size() < 5
	) return;

	this->calibration = cv::Mat(info->k, true);
	this->calibration = this->calibration.reshape(0, 3);
	this->distortion = cv::Mat(info->d, true);
	this->distortion = this->distortion.reshape(0, 1);

	this->valid_calib_data = true;

	RCLCPP_INFO(ref->get_logger(), "calib: [%dx%d], distort: [%dx%d] -- %s",
		this->calibration.rows,
		this->calibration.cols,
		this->distortion.rows,
		this->distortion.cols,
		(std::stringstream{} << this->distortion).str().c_str() );

	// try
	// {
	// 	this->undistorted_calib = cv::getOptimalNewCameraMatrix(
	// 		this->calibration,
	// 		this->distortion,
	// 		cv::Size(info->width, info->height),
	// 		1.0,
	// 		cv::Size(info->width, info->height));
	// }
	// catch(const std::exception& e)
	// {
	// 	RCLCPP_ERROR(ref->get_logger(), "Error getting undistorted camera matrix: %s", e.what());
	// }
}


ArucoServer::TagDescription::Ptr ArucoServer::TagDescription::fromRaw(const std::vector<double>& pts)
{
	if(pts.size() < 12) return nullptr;
	Ptr _desc = std::make_shared<TagDescription>();

	const cv::Point3d
		*_0 = reinterpret_cast<const cv::Point3d*>(pts.data() + 0),
		*_1 = reinterpret_cast<const cv::Point3d*>(pts.data() + 3),
		*_2 = reinterpret_cast<const cv::Point3d*>(pts.data() + 6),
		*_3 = reinterpret_cast<const cv::Point3d*>(pts.data() + 9);

	cv::Matx33d rmat;	// rotation matrix from orthogonal axes
	cv::Vec3d			// fill in each row
		*a = reinterpret_cast<cv::Vec3d*>(rmat.val + 0),
		*b = reinterpret_cast<cv::Vec3d*>(rmat.val + 3),
		*c = reinterpret_cast<cv::Vec3d*>(rmat.val + 6);

	*a = *_1 - *_0;	// x-axis

	const double
		len = cv::norm(*a),
		half_len = len / 2.;

	*b = *_1 - *_2;	// y-axis
	*c = ( *a /= len ).cross( *b /= len );	// z-axis can be dervied from x and y

	_desc->world_corners = {
		static_cast<cv::Point3f>(*_0),
		static_cast<cv::Point3f>(*_1),
		static_cast<cv::Point3f>(*_2),
		static_cast<cv::Point3f>(*_3)
	};
	_desc->rel_corners = {
		cv::Point3f{ -half_len, +half_len, 0.f },
		cv::Point3f{ +half_len, +half_len, 0.f },
		cv::Point3f{ +half_len, -half_len, 0.f },
		cv::Point3f{ -half_len, -half_len, 0.f }
	};
	reinterpret_cast<cv::Point3d&>(_desc->translation) = (*_0 + *_2) / 2.;
	reinterpret_cast<cv::Quatd&>(_desc->rotation) = cv::Quatd::createFromRotMat(rmat);
	reinterpret_cast<cv::Vec3d&>(_desc->plane) = *c;
	_desc->d = c->dot(reinterpret_cast<cv::Vec3d&>(_desc->translation));
	reinterpret_cast<cv::Vec4d&>(_desc->plane) /= cv::norm(reinterpret_cast<cv::Vec4d&>(_desc->plane));		// Eigen cast and normalize() causes crash :|

	return _desc;
}


int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ArucoServer>());
	rclcpp::shutdown();

	return 0;
}
