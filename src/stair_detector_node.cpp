/**
 * @file    stair_detector_node.cpp
 * @brief   ROS 2 node: Stair geometry detection via iterative RANSAC plane
 *          fitting on filtered point clouds, with EMA temporal tracking.
 *
 * OVERVIEW
 * 
 *  /depth_camera/points/filtered  (downsampled by voxel_filter_node)
 *        │
 *        ▼
 *   ┌─────────────────────────────────────────────────────────┐
 *   │  Stage 1 – ROI Crop (PassThrough filters)               │
 *   │     X: [0.05, 3.00] m  →  forward depth window         │
 *   │     Z: [−0.60, 0.50] m →  vertical band (floor+stairs) │
 *   └────────────────────┬────────────────────────────────────┘
 *                        ▼
 *   ┌─────────────────────────────────────────────────────────┐
 *   │  Stage 2 – Iterative RANSAC Plane Fitting               │
 *   │     Model : PERPENDICULAR_PLANE (axis = Z, ε = 0.30 r) │
 *   │     Repeats up to max_planes times, stripping each      │
 *   │     dominant horizontal plane from the residual cloud.  │
 *   │     Per-plane checks:                                   │
 *   │       • |n_z| ≥ min_nz_horizontal  → near-horizontal   │
 *   │       • tread_depth ≥ min_tread_depth  → real surface   │
 *   │       • tread_width ≥ min_tread_width  → not a pillar   │
 *   └────────────────────┬────────────────────────────────────┘
 *                        ▼
 *   │  Stage 3 – EMA Temporal Tracker                         │
 *   │     Each detected plane is matched to an existing       │
 *   │     TrackedStep by height proximity (h_match_tol).      │
 *   │     Matched steps: geometry blended with α=0.25 EMA.   │
 *   │     Unmatched: confidence decays → pruned at 0.          │
 *   │     New detections: seeded at confidence = 1.           │
 *   └────────────────────┬────────────────────────────────────┘
 *                        ▼
 *   ┌─────────────────────────────────────────────────────────┐
 *   │  Stage 4 – Regularity Check                             │
 *   │     Coefficient of Variation (CV) of riser heights.     │
 *   │     CV ≤ regularity_max_cv → is_staircase = true.      │
 *   └────────────────────┬────────────────────────────────────┘
 *                        ▼
 *           /step_state (Int32)   /stair_info (Float64MultiArray)
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * PUBLISHED TOPICS
 * ═══════════════════════════════════════════════════════════════════════════
 *   /step_state   [std_msgs/Int32]
 *       0 = NONE        – no staircase, or distance out of range
 *       1 = FIRST_STEP  – staircase detected, first step 0.90–1.50 m away
 *       2 = INTERMEDIATE– first step 0.25–0.45 m away (already climbing)
 *       3 = REACHED_PLATFORM – platform detected (large last tread + close)
 *
 *   /stair_info  [std_msgs/Float64MultiArray]  (8 floats)
 *       [0] n              – number of confident steps visible
 *       [1] mean_rise      – average riser height [m]
 *       [2] mean_run       – average tread depth  [m]
 *       [3] mean_width     – average tread width  [m]
 *       [4] dist_first     – camera-frame X of the nearest tread centroid
 *       [5] height_last    – camera-frame Z of the highest tread centroid
 *       [6] dist_last      – camera-frame X of the highest tread centroid
 *
 *   /stair_edges [visualization_msgs/MarkerArray]
 *       LINE_STRIP markers for RViz: GREEN = front (convex) edge,
 *                                     RED  = back  (concave) edge.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * KEY PARAMETERS  (all tunable at runtime)
 * ═══════════════════════════════════════════════════════════════════════════
 *   distance_threshold  0.015 m  – RANSAC inlier tolerance (tight for 60 mm steps)
 *   max_planes          10       – max RANSAC iterations per frame
 *   min_plane_inliers   20       – minimum points to accept a plane
 *   min_nz_horizontal   0.85     – |cos θ| threshold for horizontal planes
 *   max_tread_depth     0.35 m   – rejects floors (large horizontal surfaces)
 *   min_tread_depth     0.10 m   – rejects noise / thin spurious planes
 *   min_tread_width     0.30 m   – rejects wall patches / narrow ledges
 *   min_step_height     0.04 m   – minimum riser height accepted
 *   max_step_height     0.25 m   – maximum riser height accepted
 *   roi_x_min/max       0.05–3.00 m  – forward depth ROI
 *   roi_z_min/max       −0.60–0.50 m – vertical ROI
 *   min_confidence      4        – frames a step must persist before publishing
 *   height_match_tol    0.025 m  – EMA tracker height-matching window
 *                                  (must be < half of one riser ≈ 0.030 m)
 *   regularity_max_cv   0.40     – max CV of riser heights for staircase flag
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/print.h>

#include <vector>
#include <algorithm>
#include <cmath>
#include <numeric>

struct PlaneInfo {
    double        height;        ///< Centroid Z [m] – gravity-aligned "step height"
    double        distance;      ///< Centroid X [m] – forward distance from camera
    double        tread_depth;   ///< X-axis span of the plane [m]
    double        tread_width;   ///< Y-axis span of the plane [m]
    int           num_inliers;   ///< Number of RANSAC inlier points
    pcl::PointXYZ min_pt;        ///< Axis-aligned bounding box min corner
    pcl::PointXYZ max_pt;        ///< Axis-aligned bounding box max corner
    double        cx, cy, cz;   ///< Centroid (plain doubles, no Eigen dependency)
};

struct TrackedStep {
    double        height;
    double        distance;
    double        tread_depth;
    double        tread_width;
    double        cx, cy, cz;        
    pcl::PointXYZ min_pt;
    pcl::PointXYZ max_pt;
    int           confidence;       
    bool          matched_this_frame; 
};

static constexpr int    MAX_CONF  = 12;    // Maximum confidence a track can accumulate
static constexpr double EMA_ALPHA = 0.25;  // EMA weight on new measurement (lower = smoother)

class StairDetectorNode : public rclcpp::Node
{
public:
    StairDetectorNode() : Node("stair_detector_node")
    {
        this->declare_parameter("distance_threshold", 0.015);
        this->declare_parameter("max_planes",         10);
        this->declare_parameter("min_plane_inliers",  20);
        this->declare_parameter("min_nz_horizontal",  0.85);
        this->declare_parameter("max_tread_depth",    0.35);
        this->declare_parameter("min_tread_depth",    0.10);
        this->declare_parameter("min_tread_width",    0.30);
        this->declare_parameter("min_step_height",    0.04);
        this->declare_parameter("max_step_height",    0.25);

        this->declare_parameter("roi_x_min",  0.05); // Lowered to capture first step when backing close
        this->declare_parameter("roi_x_max",  3.00);
        this->declare_parameter("roi_z_min", -0.60);
        this->declare_parameter("roi_z_max",  0.50);

        this->declare_parameter("min_confidence",   4);
        this->declare_parameter("height_match_tol", 0.025); 

        this->declare_parameter("regularity_max_cv", 0.40); 

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/depth_camera/points/filtered", rclcpp::SensorDataQoS(),
            std::bind(&StairDetectorNode::cloudCallback, this, std::placeholders::_1));

        step_state_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/step_state", 10);
        stair_info_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/stair_info", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/stair_edges", 10);

        pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
        RCLCPP_INFO(this->get_logger(), "StairDetectorNode started");
    }

private:
    static double ema(double prev, double next) {
        return EMA_ALPHA * next + (1.0 - EMA_ALPHA) * prev;
    }

    static float emaf(float prev, float next) {
        return static_cast<float>(EMA_ALPHA * next + (1.0 - EMA_ALPHA) * prev);
    }

    static void passthrough(const pcl::PointCloud<pcl::PointXYZ>::Ptr &in,
                            const std::string &field, float lo, float hi,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr &out)
    {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(in);
        pass.setFilterFieldName(field);
        pass.setFilterLimits(lo, hi);
        pass.filter(*out);
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        const double dist_thresh = this->get_parameter("distance_threshold").as_double();
        const int    max_planes  = this->get_parameter("max_planes").as_int();
        const int    min_inliers = this->get_parameter("min_plane_inliers").as_int();
        const float  min_nz      = static_cast<float>(
                                       this->get_parameter("min_nz_horizontal").as_double());
        const double max_tread   = this->get_parameter("max_tread_depth").as_double();
        const double min_tread   = this->get_parameter("min_tread_depth").as_double();
        const double min_tread_w = this->get_parameter("min_tread_width").as_double();
        const double roi_x_min   = this->get_parameter("roi_x_min").as_double();
        const double roi_x_max   = this->get_parameter("roi_x_max").as_double();
        const double roi_z_min   = this->get_parameter("roi_z_min").as_double();
        const double roi_z_max   = this->get_parameter("roi_z_max").as_double();
        const int    min_conf    = this->get_parameter("min_confidence").as_int();
        const double h_match_tol = this->get_parameter("height_match_tol").as_double();
        const double reg_max_cv  = this->get_parameter("regularity_max_cv").as_double();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        if (static_cast<int>(cloud->size()) < min_inliers) {
            decay_all_tracks();
            return;
        }

        // ROI Crop
        // Removes floor points behind the robot (x < 0.05) and ceiling / very high points (z > 0.50). 
        pcl::PointCloud<pcl::PointXYZ>::Ptr roi(new pcl::PointCloud<pcl::PointXYZ>);
        passthrough(cloud, "x",
                    static_cast<float>(roi_x_min), static_cast<float>(roi_x_max), roi);
        passthrough(roi, "z",
                    static_cast<float>(roi_z_min), static_cast<float>(roi_z_max), roi);
        if (static_cast<int>(roi->size()) < min_inliers) {
            decay_all_tracks();
            return;
        }

        // Iterative RANSAC plane fitting 
        // Each iteration finds the dominant horizontal plane in the residual
        // cloud, extracts its inliers, and removes them before the next pass.
        std::vector<PlaneInfo> candidates;

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f)); 
        seg.setEpsAngle(0.30f);                         
        seg.setOptimizeCoefficients(true);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(dist_thresh);
        seg.setMaxIterations(500);

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::PointCloud<pcl::PointXYZ>::Ptr remaining(
            new pcl::PointCloud<pcl::PointXYZ>(*roi));

        for (int iter = 0; iter < max_planes; ++iter) {
            if (static_cast<int>(remaining->size()) < min_inliers) break;

            pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr      inliers(new pcl::PointIndices);
            seg.setInputCloud(remaining);
            seg.segment(*inliers, *coeffs);
            if (static_cast<int>(inliers->indices.size()) < min_inliers) break;

            pcl::PointCloud<pcl::PointXYZ>::Ptr plane_pts(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr next_rem (new pcl::PointCloud<pcl::PointXYZ>);
            extract.setInputCloud(remaining);
            extract.setIndices(inliers);
            extract.setNegative(false); extract.filter(*plane_pts);  // inliers
            extract.setNegative(true);  extract.filter(*next_rem);   // residual
            remaining = next_rem;

            if (std::fabs(coeffs->values[2]) < min_nz) continue;

            // Compute 3-D bounding box of this plane's inlier set
            pcl::PointXYZ min_p, max_p;
            pcl::getMinMax3D(*plane_pts, min_p, max_p);
            const double depth = std::fabs(max_p.x - min_p.x);  // forward extent
            const double width = std::fabs(max_p.y - min_p.y);  // lateral extent

            if (depth < min_tread)   continue;
            if (width < min_tread_w) continue;

            // Compute centroid of the inlier cloud
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*plane_pts, centroid);

            PlaneInfo pi;
            pi.height      = centroid[2];
            pi.distance    = centroid[0];
            pi.tread_depth = depth;
            pi.tread_width = width;
            pi.num_inliers = static_cast<int>(inliers->indices.size());
            pi.min_pt      = min_p;
            pi.max_pt      = max_p;
            pi.cx = centroid[0]; pi.cy = centroid[1]; pi.cz = centroid[2];
            candidates.push_back(pi);
        }

        std::sort(candidates.begin(), candidates.end(),
                  [](const PlaneInfo &a, const PlaneInfo &b) {
                      return a.height < b.height;
                  });

        if (!candidates.empty() && candidates.front().tread_depth > max_tread) {
            candidates.erase(candidates.begin());
        }

        for (auto it = candidates.begin(); it != candidates.end(); ) {
            if (it->tread_depth > max_tread && it != (candidates.end() - 1)) {
                it = candidates.erase(it);
            } else {
                ++it;
            }
        }

        // EMA temporal tracker 
        update_tracks(candidates, h_match_tol);

        // Collect tracks that have reached minimum confidence 
        std::vector<TrackedStep *> visible;
        for (auto &ts : tracked_steps_)
            if (ts.confidence >= min_conf) visible.push_back(&ts);

        std::sort(visible.begin(), visible.end(),
                  [](const TrackedStep *a, const TrackedStep *b) {
                      return a->height < b->height;
                  });

        const bool is_staircase = check_regularity(visible, reg_max_cv);

        publishMarkers(msg->header, visible);
        publishResults(msg->header, visible, is_staircase);

        RCLCPP_INFO(this->get_logger(),
            "visible steps: %zu | tracks: %zu",
            visible.size(), tracked_steps_.size());
    }

    void update_tracks(const std::vector<PlaneInfo> &detections, double h_tol)
    {
        for (auto &ts : tracked_steps_) ts.matched_this_frame = false;

        for (const auto &det : detections) {
            TrackedStep *best      = nullptr;
            double       best_dist = h_tol;
            for (auto &ts : tracked_steps_) {
                if (ts.matched_this_frame) continue;
                double d = std::fabs(ts.height - det.height);
                if (d < best_dist) { best_dist = d; best = &ts; }
            }

            if (best) {
                best->height      = ema(best->height,      det.height);
                best->distance    = ema(best->distance,    det.distance);
                best->tread_depth = ema(best->tread_depth, det.tread_depth);
                best->tread_width = ema(best->tread_width, det.tread_width);
                best->cx          = ema(best->cx,          det.cx);
                best->cy          = ema(best->cy,          det.cy);
                best->cz          = ema(best->cz,          det.cz);
                best->min_pt.x    = emaf(best->min_pt.x,   det.min_pt.x);
                best->min_pt.y    = emaf(best->min_pt.y,   det.min_pt.y);
                best->min_pt.z    = emaf(best->min_pt.z,   det.min_pt.z);
                best->max_pt.x    = emaf(best->max_pt.x,   det.max_pt.x);
                best->max_pt.y    = emaf(best->max_pt.y,   det.max_pt.y);
                best->max_pt.z    = emaf(best->max_pt.z,   det.max_pt.z);
                best->confidence  = std::min(best->confidence + 2, MAX_CONF);
                best->matched_this_frame = true;
            } else {
                TrackedStep ns{};
                ns.height      = det.height;     ns.distance    = det.distance;
                ns.tread_depth = det.tread_depth; ns.tread_width = det.tread_width;
                ns.cx = det.cx; ns.cy = det.cy;  ns.cz = det.cz;
                ns.min_pt      = det.min_pt;      ns.max_pt      = det.max_pt;
                ns.confidence  = 1;               ns.matched_this_frame = true;
                tracked_steps_.push_back(ns);
            }
        }

        for (auto &ts : tracked_steps_)
            if (!ts.matched_this_frame) ts.confidence--;

        tracked_steps_.erase(
            std::remove_if(tracked_steps_.begin(), tracked_steps_.end(),
                [](const TrackedStep &ts) { return ts.confidence <= 0; }),
            tracked_steps_.end());
    }

    void decay_all_tracks()
    {
        for (auto &ts : tracked_steps_) ts.confidence--;
        tracked_steps_.erase(
            std::remove_if(tracked_steps_.begin(), tracked_steps_.end(),
                [](const TrackedStep &ts) { return ts.confidence <= 0; }),
            tracked_steps_.end());
    }

    bool check_regularity(const std::vector<TrackedStep *> &steps,
                          double max_cv) const
    {
        if (steps.size() < 2) return false;
        if (max_cv <= 0.0)    return true;  // check disabled

        std::vector<double> deltas;
        for (size_t i = 1; i < steps.size(); ++i) {
            double dh = steps[i]->height - steps[i-1]->height;
            if (dh > 0.02) deltas.push_back(dh);  // ignore non-rising pairs
        }
        if (deltas.empty()) return false;

        double mean = std::accumulate(deltas.begin(), deltas.end(), 0.0)
                      / static_cast<double>(deltas.size());
        if (mean < 0.02) return false;

        double sq = 0.0;
        for (double d : deltas) sq += (d - mean) * (d - mean);
        double cv = std::sqrt(sq / static_cast<double>(deltas.size())) / mean;
        return cv <= max_cv;
    }

    /** 
     * RViz marker publisher
     * Visual convention:
     *   GREEN  = front (convex) 
     *   RED    = back  (concave) 
    */
    void publishMarkers(const std_msgs::msg::Header &header,
                        const std::vector<TrackedStep *> &steps)
    {
        visualization_msgs::msg::MarkerArray markers;

        visualization_msgs::msg::Marker del;
        del.header = header;
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        markers.markers.push_back(del);

        int mid = 0;

        auto mk_line = [&](const std::string &ns,
                           geometry_msgs::msg::Point a,
                           geometry_msgs::msg::Point b,
                           float r, float g, float bl,
                           float thickness) {
            visualization_msgs::msg::Marker m;
            m.header   = header; m.ns = ns; m.id = mid++;
            m.type     = visualization_msgs::msg::Marker::LINE_STRIP;
            m.action   = visualization_msgs::msg::Marker::ADD;
            m.scale.x  = thickness;
            m.color.r  = r; m.color.g = g; m.color.b = bl; m.color.a = 1.0f;
            m.lifetime = rclcpp::Duration::from_seconds(0.15);
            m.points   = {a, b};
            markers.markers.push_back(m);
        };

        for (size_t i = 0; i < steps.size(); ++i) {
            const TrackedStep &ts = *steps[i];
            const double z = ts.min_pt.z + 0.015; 

            geometry_msgs::msg::Point p_fl, p_fr, p_bl, p_br;
            p_fl.x = ts.min_pt.x; p_fl.y = ts.min_pt.y; p_fl.z = z;  // front-left
            p_fr.x = ts.min_pt.x; p_fr.y = ts.max_pt.y; p_fr.z = z;  // front-right
            p_bl.x = ts.max_pt.x; p_bl.y = ts.min_pt.y; p_bl.z = z;  // back-left
            p_br.x = ts.max_pt.x; p_br.y = ts.max_pt.y; p_br.z = z;  // back-right

            const float thick = 0.01f;

            // Front (convex) edge: GREEN
            mk_line("front", p_fl, p_fr, 0.0f, 1.0f, 0.0f, thick);
            // Back (concave) edge: RED
            mk_line("back",  p_bl, p_br, 1.0f, 0.0f, 0.0f, thick);
        }
        marker_pub_->publish(markers);
    }

    void publishResults(const std_msgs::msg::Header &header,
                        const std::vector<TrackedStep *> &steps,
                        bool is_staircase)
    {
        (void)header;     
        (void)is_staircase;

        const int n = static_cast<int>(steps.size());

        std_msgs::msg::Int32 step_state_msg;
        step_state_msg.data = 0; 

        std_msgs::msg::Float64MultiArray info;
        info.data.resize(8, 0.0);
        info.data[0] = static_cast<double>(n);

        if (n >= 1) {
            double sum_rise = 0.0;
            int    rc       = 0;
            for (int i = 0; i < n; ++i) {
                if (i > 0) { sum_rise += steps[i]->height - steps[i-1]->height; ++rc; }
                double rise = (i > 0) ? (steps[i]->height - steps[i-1]->height)
                                      : steps[i]->height;
                RCLCPP_INFO(this->get_logger(),
                    "  Step %d: rise=%.3f, run=%.3f, width=%.3f, dist=%.3f",
                    i+1, rise, steps[i]->tread_depth, steps[i]->tread_width,
                    steps[i]->distance);
            }
            info.data[1] = rc > 0 ? sum_rise / rc : 0.0;

            double sum_run = 0.0, sum_w = 0.0;
            for (const auto *s : steps) { sum_run += s->tread_depth; sum_w += s->tread_width; }
            info.data[2] = sum_run / n;
            info.data[3] = sum_w   / n;

            info.data[4] = steps.front()->distance;   // dist to nearest step
            info.data[5] = steps.back()->height;       // height of top step
            info.data[6] = steps.back()->distance;     // dist to top step
            info.data[7] = 0.0;

            double first_dist = steps.front()->distance;

            if (n > 1 && steps.back()->tread_depth >= 0.35
                      && first_dist >= 0.50 && first_dist <= 0.89) {
                step_state_msg.data = 3;  // top platform
                RCLCPP_INFO(this->get_logger(), "Last Step Detected!!!");
            } else if (first_dist <= 1.50 && first_dist >= 0.90 && n >= 1) {
                step_state_msg.data = 1;  // first-step
            } else if (n > 1 && first_dist >= 0.25 && first_dist <= 0.45) {
                step_state_msg.data = 2;  // mid-climb
            } else {
                step_state_msg.data = 0;  // none
            }

            RCLCPP_INFO(this->get_logger(), "first_dist: %.3f | Step State Mode: %d",
                        first_dist, step_state_msg.data);
        }

        step_state_pub_->publish(step_state_msg);
        stair_info_pub_->publish(info);
    }

    int count_{0};                         
    std::vector<TrackedStep> tracked_steps_; 

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr    sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr                 step_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr     stair_info_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StairDetectorNode>());
    rclcpp::shutdown();
    return 0;
}