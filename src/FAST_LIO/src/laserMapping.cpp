#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <chrono>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <utility> // [新增] for std::move
#include "IMU_Processing.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <atomic>

#define INIT_TIME (0.1)
#define LASER_POINT_COV (0.001)
#define MAXN (720000)
#define PUBFRAME_PERIOD (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;
string odom_frame = "odom";
string base_footprint_frame = "base_footprint";
string base_frame = "base_link";
string lidar_frame = "livox_frame";
V3D base_to_lidar_T(0.0, 0.0, 0.10);

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool point_selected_surf[100000] = {0};
bool lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
// publish.* 开关仅使用 LaserMappingNode 成员变量；勿在此处再声明同名全局，
// 否则 publish_frame_world 等游离函数会一直读到错的 false（导致 /cloud_registered 无输出）。
bool is_first_lidar = true;

// ==================== 时钟桥：lidar 内部时钟 -> ROS 时钟 ====================
// 在 livox_pcl_cbk 第一次接收到 lidar 数据时初始化
// IMU callback 用同一个 offset，保证 lidar/IMU 相对时间关系不变
// 后续所有对外发布的 stamp 都是 ROS 时钟，nav2/tf2 全程一致
bool   clock_offset_initialized   = false;
double lidar_to_ros_clock_offset  = 0.0;
// ===========================================================================

/** 由节点构造时根据 diagnostics.time_diag_file 设置；为 false 时不写 time_diag.log */
static std::atomic<bool> g_fastlio_time_diag_file_en{false};

static std::mutex mtx_fastlio_diag_file;

/** 时间诊断写入 root_dir/Log/fastlio_diag/time_diag.log，避免刷屏 ros 控制台 */
static void fastlio_diag_file_append(const char *fmt, ...)
{
    if (!g_fastlio_time_diag_file_en.load(std::memory_order_relaxed))
        return;

    char buf[2048];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    std::lock_guard<std::mutex> lk(mtx_fastlio_diag_file);
    std::error_code ec;
    const std::filesystem::path dir = std::filesystem::path(root_dir) / "Log" / "fastlio_diag";
    std::filesystem::create_directories(dir, ec);
    const std::string path = (dir / "time_diag.log").string();
    FILE *fp = fopen(path.c_str(), "a");
    if (!fp)
        return;

    static bool s_wrote_header = false;
    if (!s_wrote_header)
    {
        s_wrote_header = true;
        fprintf(fp,
                "# recv_ros_now=RCL_ROS_TIME in callback; msg_stamp=header; (recv-stamp)=recv_ros_now-msg_stamp "
                "(meaningful if stamps are ROS-time); cb_dt=steady_clock since previous same-topic callback "
                "(large => executor/subscription starvation)\n");
    }
    const auto tp = std::chrono::system_clock::now();
    const std::time_t tt = std::chrono::system_clock::to_time_t(tp);
    std::tm tm_buf{};
#if defined(_WIN32)
    localtime_s(&tm_buf, &tt);
#else
    localtime_r(&tt, &tm_buf);
#endif
    char tstr[32];
    std::strftime(tstr, sizeof(tstr), "%Y-%m-%d %H:%M:%S", &tm_buf);
    fprintf(fp, "[%s] %s\n", tstr, buf);
    fflush(fp);
    fclose(fp);
}

vector<vector<int>> pointSearchInd_surf;
vector<BoxPointType> cub_needrm;
vector<PointVector> Nearest_Points;
vector<double> extrinT(3, 0.0);
vector<double> extrinR(9, 0.0);
deque<double> time_buffer;
deque<PointCloudXYZI::Ptr> lidar_buffer;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::Quaternion geoQuat;
geometry_msgs::msg::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

class LaserMappingNode;
static LaserMappingNode *g_fastlio_diag_node = nullptr;
static void fastlio_diag_log_recv_lidar(double msg_stamp_sec, double recv_now_sec, double cb_dt_steady_sec);
static void fastlio_diag_log_recv_imu(double raw_msg_stamp_sec, double used_stamp_sec, double recv_now_sec,
                                       double cb_dt_steady_sec);
static void fastlio_diag_log_recv_lidar_impl(double msg_stamp_sec, double recv_now_sec, const char *src_tag,
                                              double cb_dt_steady_sec);

void SigHandle(int sig)
{
    flg_exit = true;
    std::cout << "catch sig %d" << sig << std::endl;
    sig_buffer.notify_all();
    rclcpp::shutdown();
}

inline void dump_lio_state_to_log(FILE *fp)
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                            // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2));    // Pos
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                                 // omega
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2));    // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                                 // Acc
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));       // Bias_g
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));       // Bias_a
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a
    fprintf(fp, "\r\n");
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const *const pi, PointType *const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void pointBodyToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template <typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I * p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized)
    {
        for (int i = 0; i < 3; i++)
        {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++)
    {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            need_move = true;
    }
    if (!need_move)
        return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD - 1)));
    for (int i = 0; i < 3; i++)
    {
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
        else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if (cub_needrm.size() > 0)
        kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
    // 此处不可调用 g_fastlio_diag_node->now()：LaserMappingNode 尚未完整定义（仅前置声明）
    const double recv_ros_now = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
    static std::chrono::steady_clock::time_point prev_lidar_cb{};
    static bool prev_lidar_cb_inited = false;
    const auto steady_now = std::chrono::steady_clock::now();
    double lidar_cb_dt = 0.0;
    if (prev_lidar_cb_inited)
        lidar_cb_dt = std::chrono::duration<double>(steady_now - prev_lidar_cb).count();
    prev_lidar_cb_inited = true;
    prev_lidar_cb = steady_now;
    mtx_buffer.lock();
    scan_count++;
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer, dt="
                  << (cur_time - last_timestamp_lidar) << "s" << std::endl;
        lidar_buffer.clear();
        time_buffer.clear();
    }
    if (is_first_lidar)
    {
        is_first_lidar = false;
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(cur_time);
    last_timestamp_lidar = cur_time;
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    fastlio_diag_log_recv_lidar_impl(cur_time, recv_ros_now, "PointCloud2", lidar_cb_dt);
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg)
{
    // 同上：回调早于 LaserMappingNode 定义，用全局 ROS 时钟取接收时刻
    const double recv_ros_now = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
    static std::chrono::steady_clock::time_point prev_lidar_cb{};
    static bool prev_lidar_cb_inited = false;
    const auto steady_now = std::chrono::steady_clock::now();
    double lidar_cb_dt = 0.0;
    if (prev_lidar_cb_inited)
        lidar_cb_dt = std::chrono::duration<double>(steady_now - prev_lidar_cb).count();
    prev_lidar_cb_inited = true;
    prev_lidar_cb = steady_now;
    mtx_buffer.lock();

    // ===== 临时止血：禁用 lidar->ROS offset 注入，直接使用原始消息时钟 =====
    // 保留 offset 计算与日志仅用于诊断；数据路径不再使用该 offset，
    // 避免首帧偏差把整条时间轴推到未来（当前观测约 +3s）。
    double lidar_stamp = get_time_sec(msg->header.stamp);
    if (!clock_offset_initialized) {
        rclcpp::Clock ros_clock(RCL_ROS_TIME);
        lidar_to_ros_clock_offset = ros_clock.now().seconds() - lidar_stamp;
        clock_offset_initialized = true;
        printf("[Fast-LIO] Lidar->ROS clock offset initialized: %.6f s\n",
               lidar_to_ros_clock_offset);
    }
    double cur_time = lidar_stamp;
    // ======================================================================

    double preprocess_start_time = omp_get_wtime();
    scan_count++;
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer, dt="
                  << (cur_time - last_timestamp_lidar) << "s" << std::endl;
        lidar_buffer.clear();
        time_buffer.clear();
    }
    if (is_first_lidar)
    {
        is_first_lidar = false;
    }
    last_timestamp_lidar = cur_time;

    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty())
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n", last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);

    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    fastlio_diag_log_recv_lidar(lidar_stamp, recv_ros_now, lidar_cb_dt);
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in)
{
    const double recv_ros_now = rclcpp::Clock(RCL_ROS_TIME).now().seconds(); // 同上，避免不完整类型
    static std::chrono::steady_clock::time_point prev_imu_cb{};
    static bool prev_imu_cb_inited = false;
    const auto steady_now = std::chrono::steady_clock::now();
    double imu_cb_dt = 0.0;
    if (prev_imu_cb_inited)
        imu_cb_dt = std::chrono::duration<double>(steady_now - prev_imu_cb).count();
    prev_imu_cb_inited = true;
    prev_imu_cb = steady_now;
    publish_count++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

    // ===== 临时止血：IMU 不再注入 lidar->ROS offset，直接沿用消息时钟 =====
    double imu_stamp = get_time_sec(msg_in->header.stamp);
    // =====================================================================

    msg->header.stamp = get_ros_time(imu_stamp - time_diff_lidar_to_imu);
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp =
            rclcpp::Time(timediff_lidar_wrt_imu + imu_stamp);
    }

    double timestamp = get_time_sec(msg->header.stamp);
    fastlio_diag_log_recv_imu(imu_stamp, timestamp, recv_ros_now, imu_cb_dt);

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        std::cerr << "imu loop back, clear buffer, dt="
                  << (timestamp - last_timestamp_imu) << "s" << std::endl;
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty())
    {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            std::cerr << "Too few input point cloud!\n";
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = get_time_sec(imu_buffer.front()->header.stamp);
        if (imu_time > lidar_end_time)
            break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point;
            mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            float dist = calc_dist(feats_down_world->points[i], mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min)
            {
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++)
            {
                if (points_near.size() < NUM_MATCH_POINTS)
                    break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add)
                PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI());
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull, bool dense_en)
{
    PointCloudXYZI::Ptr laserCloudFullRes(dense_en ? feats_undistort : feats_down_body);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = odom_frame;
    pubLaserCloudFull->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_frame_body(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i],
                               &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    // Points above are in the FAST-LIO body frame, which is the MID360 built-in IMU frame.
    // Keep this distinct from base_frame/base_link, the quadruped body center.
    laserCloudmsg.header.frame_id = lidar_frame;
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_effect_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld(
        new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i],
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = get_ros_time(lidar_end_time);
    laserCloudFullRes3.header.frame_id = odom_frame;
    pubLaserCloudEffect->publish(laserCloudFullRes3);
}

void publish_map(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap, bool dense_en)
{
    PointCloudXYZI::Ptr laserCloudFullRes(dense_en ? feats_undistort : feats_down_body);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(
        new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyToWorld(&laserCloudFullRes->points[i],
                            &laserCloudWorld->points[i]);
    }
    *pcl_wait_pub += *laserCloudWorld;

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*pcl_wait_pub, laserCloudmsg);
    // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = odom_frame;
    pubLaserCloudMap->publish(laserCloudmsg);
}

void save_to_pcd()
{
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(map_file_path, *pcl_wait_pub);
}

template <typename T>
void set_posestamp(T &out)
{
    const V3D base_pos = state_point.pos - state_point.rot * base_to_lidar_T;
    out.pose.position.x = base_pos(0);
    out.pose.position.y = base_pos(1);
    out.pose.position.z = base_pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
}

void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped, std::unique_ptr<tf2_ros::TransformBroadcaster> &tf_br)
{
    odomAftMapped.header.frame_id = odom_frame;
    odomAftMapped.child_frame_id = base_frame;
    odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped->publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
    }

    geometry_msgs::msg::TransformStamped trans;
    trans.header.frame_id = odom_frame;
    trans.header.stamp = odomAftMapped.header.stamp;
    trans.child_frame_id = base_frame;
    trans.transform.translation.x = odomAftMapped.pose.pose.position.x;
    trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
    trans.transform.translation.z = odomAftMapped.pose.pose.position.z;
    trans.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    trans.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    trans.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    trans.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    tf_br->sendTransform(trans);
}

void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = get_ros_time(lidar_end_time); // ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = odom_frame;

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0)
    {
        path.poses.push_back(msg_body_pose);
        pubPath->publish(path);
    }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear();
    corr_normvect->clear();
    total_residual = 0.0;

/** closest surface search and residual computation **/
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body = feats_down_body->points[i];
        PointType &point_world = feats_down_world->points[i];

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false
                                                                                                                                : true;
        }

        if (!point_selected_surf[i])
            continue;

        VF(4)
        pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }

    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        std::cerr << "No Effective Points!" << std::endl;
        // ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time += omp_get_wtime() - match_start;
    double solve_start_ = omp_get_wtime();

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); // 23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() * norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); // s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}

class LaserMappingNode : public rclcpp::Node
{
public:
    LaserMappingNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("laser_mapping", options)
    {
        // 1. Declare parameters (using ROS 2 style with return value assignment)
        path_en = this->declare_parameter<bool>("publish.path_en", true);
        effect_pub_en = this->declare_parameter<bool>("publish.effect_map_en", false);
        map_pub_en = this->declare_parameter<bool>("publish.map_en", false);
        scan_pub_en = this->declare_parameter<bool>("publish.scan_publish_en", true);
        dense_pub_en = this->declare_parameter<bool>("publish.dense_publish_en", true);
        scan_body_pub_en = this->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
        high_freq_odom_en_ = this->declare_parameter<bool>("publish.high_freq_odom_en", false);
        NUM_MAX_ITERATIONS = this->declare_parameter<int>("max_iteration", 4);
        map_file_path = this->declare_parameter<string>("map_file_path", "");
        lid_topic = this->declare_parameter<string>("common.lid_topic", "/livox/lidar");
        imu_topic = this->declare_parameter<string>("common.imu_topic", "/livox/imu");
        time_sync_en = this->declare_parameter<bool>("common.time_sync_en", false);
        time_diff_lidar_to_imu = this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
        g_fastlio_time_diag_file_en.store(
            this->declare_parameter<bool>("diagnostics.time_diag_file", false), std::memory_order_relaxed);
        odom_frame = this->declare_parameter<string>("frames.odom_frame", "odom");
        base_footprint_frame = this->declare_parameter<string>("frames.base_footprint_frame", "base_footprint");
        base_frame = this->declare_parameter<string>("frames.base_frame", "base_link");
        lidar_frame = this->declare_parameter<string>("frames.lidar_frame", "livox_frame");
        const auto base_to_lidar = this->declare_parameter<vector<double>>(
            "frames.base_to_lidar_T", vector<double>{0.0, 0.0, 0.10});
        if (base_to_lidar.size() == 3)
        {
            base_to_lidar_T = V3D(base_to_lidar[0], base_to_lidar[1], base_to_lidar[2]);
        }
        else
        {
            RCLCPP_WARN(
                this->get_logger(),
                "frames.base_to_lidar_T must have 3 values; using default [0, 0, 0.10]");
            base_to_lidar_T = V3D(0.0, 0.0, 0.10);
        }
        filter_size_corner_min = this->declare_parameter<double>("filter_size_corner", 0.5);
        filter_size_surf_min = this->declare_parameter<double>("filter_size_surf", 0.5);
        filter_size_map_min = this->declare_parameter<double>("filter_size_map", 0.5);
        cube_len = this->declare_parameter<double>("cube_side_length", 200.);
        DET_RANGE = this->declare_parameter<float>("mapping.det_range", 300.);
        fov_deg = this->declare_parameter<double>("mapping.fov_degree", 180.);
        gyr_cov = this->declare_parameter<double>("mapping.gyr_cov", 0.1);
        acc_cov = this->declare_parameter<double>("mapping.acc_cov", 0.1);
        b_gyr_cov = this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
        b_acc_cov = this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
        p_pre->blind = this->declare_parameter<double>("preprocess.blind", 0.01);
        p_pre->lidar_type = static_cast<LID_TYPE>(this->declare_parameter<int>("preprocess.lidar_type", AVIA));
        p_pre->N_SCANS = this->declare_parameter<int>("preprocess.scan_line", 16);
        p_pre->time_unit = static_cast<TIME_UNIT>(this->declare_parameter<int>("preprocess.timestamp_unit", US));
        p_pre->SCAN_RATE = this->declare_parameter<int>("preprocess.scan_rate", 10);
        p_pre->point_filter_num = this->declare_parameter<int>("point_filter_num", 2);
        p_pre->feature_enabled = this->declare_parameter<bool>("feature_extract_enable", false);
        runtime_pos_log = this->declare_parameter<bool>("runtime_pos_log_enable", 0);
        extrinsic_est_en = this->declare_parameter<bool>("mapping.extrinsic_est_en", true);
        pcd_save_en = this->declare_parameter<bool>("pcd_save.pcd_save_en", false);
        pcd_save_interval = this->declare_parameter<int>("pcd_save.interval", -1);
        extrinT = this->declare_parameter<vector<double>>("mapping.extrinsic_T", vector<double>());
        extrinR = this->declare_parameter<vector<double>>("mapping.extrinsic_R", vector<double>());

        RCLCPP_INFO(this->get_logger(), "p_pre->lidar_type %d", p_pre->lidar_type);

        // 与 MultiThreadedExecutor 配合：订阅与定时器分回调组，减轻 Nav2 同机负载下 IMU/雷达饥饿
        cb_group_subscriptions_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        cb_group_timers_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        path.header.stamp = this->get_clock()->now();
        path.header.frame_id = odom_frame;

        FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
        HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

        _featsArray.reset(new PointCloudXYZI());

        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));
        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));

        Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
        p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
        p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
        p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

        fill(epsi, epsi + 23, 0.001);
        kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

        /*** debug record ***/
        string pos_log_dir = root_dir + "/Log/pos_log.txt";
        fp = fopen(pos_log_dir.c_str(), "w");

        fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), ios::out);
        fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);
        fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"), ios::out);
        if (fout_pre && fout_out)
            cout << "~~~~" << ROOT_DIR << " file opened" << endl;
        else
            cout << "~~~~" << ROOT_DIR << " doesn't exist" << endl;

        g_fastlio_diag_node = this;

        if (g_fastlio_time_diag_file_en.load(std::memory_order_relaxed))
        {
            std::error_code ec;
            const std::filesystem::path diag_dir = std::filesystem::path(root_dir) / "Log" / "fastlio_diag";
            std::filesystem::create_directories(diag_dir, ec);
            RCLCPP_INFO(this->get_logger(), "Fast-LIO time diagnostics log: %s",
                        (diag_dir / "time_diag.log").string().c_str());
        }

        /*** ROS subscribe initialization ***/
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = cb_group_subscriptions_;

        if (p_pre->lidar_type == AVIA)
        {
            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                lid_topic, 20, livox_pcl_cbk, sub_options);
        }
        else
        {
            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                lid_topic, rclcpp::SensorDataQoS(), standard_pcl_cbk, sub_options);
        }
        // 默认 QoS 为 RELIABLE+小深度，高率 IMU 在 CPU 紧张时易在 DDS 侧排队，表现为 recv_now - header.stamp 达秒级
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, rclcpp::SensorDataQoS().keep_last(100), imu_cbk, sub_options);
        pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 20);
        pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 20);
        pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 20);
        pubLaserCloudMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 20);

        pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_3d", 20);
        pubOdomAftMapped_2d_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_2d", 20);
        pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path", 20);
        // 低频发布 + 晚订阅（如 wait_for_nav_ready）需能拿到最近值，避免首条在订阅前丢失导致 lag=inf
        pub_fastlio_lag_sec_ = this->create_publisher<std_msgs::msg::Float64>(
            "/fast_lio/time_lag_sec", rclcpp::QoS(1).reliable().transient_local());
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        //------------------------------------------------------------------------------------------------------
        auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 50.0));
        timer_ = rclcpp::create_timer(
            this, this->get_clock(), period_ms, std::bind(&LaserMappingNode::timer_callback, this), cb_group_timers_);

        auto map_period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0));
        map_pub_timer_ = rclcpp::create_timer(
            this, this->get_clock(), map_period_ms, std::bind(&LaserMappingNode::map_publish_callback, this),
            cb_group_timers_);

        map_save_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "map_save", std::bind(&LaserMappingNode::map_save_callback, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, cb_group_timers_);

        RCLCPP_INFO(this->get_logger(), "Node init finished (callback groups: sub=Reentrant, timer/srv=MutuallyExclusive).");
    }

    ~LaserMappingNode()
    {
        if (g_fastlio_diag_node == this)
        {
            g_fastlio_diag_node = nullptr;
        }
        fout_out.close();
        fout_pre.close();
        if (fp)
            fclose(fp);
    }

private:
    void timer_callback()
    {
        static uint64_t processed_scans = 0;
        static int last_scan_count_snapshot = 0;
        static uint64_t last_processed_snapshot = 0;
        static auto last_rate_tp = std::chrono::steady_clock::now();
        bool processed_scan_this_cycle = false;

        // 尝试同步雷达和IMU数据
        if (sync_packages(Measures))
        {
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                return;
            }

            double t0, t1, t2, t3, t4, t5, match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time = 0;
            t0 = omp_get_wtime();

            p_imu->Process(Measures, kf, feats_undistort);
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;

            lasermap_fov_segment();
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();

            if (ikdtree.Root_Node == nullptr)
            {
                if (feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for (int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                return;
            }

            if (feats_down_size < 5)
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }

            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose() << " " << ext_euler.transpose() << " " << state_point.offset_T_L_I.transpose() << " " << state_point.vel.transpose()
                     << " " << state_point.bg.transpose() << " " << state_point.ba.transpose() << " " << state_point.grav << endl;

            if (0) 
            {
                PointVector().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            // EKF 更新
            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int rematch_num = 0;
            bool nearest_search_en = true;

            t2 = omp_get_wtime();
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];
            double t_update_end = omp_get_wtime();

            // 注释掉原有的低频 publish_odometry 调用，由高频函数接管 TF 发布
            // publish_odometry(pubOdomAftMapped_, tf_broadcaster_); 

            t3 = omp_get_wtime();
            map_incremental();
            t5 = omp_get_wtime();

            if (path_en)
                publish_path(pubPath_);
            if (scan_pub_en)
                publish_frame_world(pubLaserCloudFull_, dense_pub_en);
            if (scan_pub_en && scan_body_pub_en)
                publish_frame_body(pubLaserCloudFull_body_);
            if (effect_pub_en)
                publish_effect_world(pubLaserCloudEffect_);
            // if (map_pub_en) publish_map(pubLaserCloudMap_);

            processed_scans++;
            processed_scan_this_cycle = true;

            // 诊断1&2：队列长度 + mapping 单帧耗时
            size_t lidar_q = 0;
            size_t imu_q = 0;
            {
                std::lock_guard<std::mutex> lk(mtx_buffer);
                lidar_q = lidar_buffer.size();
                imu_q = imu_buffer.size();
            }
            const double mapping_cost_ms = (t5 - t0) * 1000.0;
            const double map_lag_sec = this->now().seconds() - Measures.lidar_end_time;
            const double ros_now_sec = this->now().seconds();
            {
                static std::chrono::steady_clock::time_point last_map_diag{};
                static bool map_diag_inited = false;
                const auto st_now = std::chrono::steady_clock::now();
                if (!map_diag_inited || std::chrono::duration<double>(st_now - last_map_diag).count() >= 2.0)
                {
                    map_diag_inited = true;
                    last_map_diag = st_now;
                    fastlio_diag_file_append(
                        "[FastlioMap] post_reg lidar_beg=%.6f lidar_end=%.6f ros_now=%.6f "
                        "(now-lidar_end)=%.3fs (now-lidar_beg)=%.3fs map_cost=%.1fms",
                        Measures.lidar_beg_time, Measures.lidar_end_time, ros_now_sec, map_lag_sec,
                        ros_now_sec - Measures.lidar_beg_time, mapping_cost_ms);
                }
            }

            // 诊断3：输入频率 vs 处理频率（窗口统计）
            const auto now_tp = std::chrono::steady_clock::now();
            const double dt_rate = std::chrono::duration<double>(now_tp - last_rate_tp).count();
            if (dt_rate >= 2.0)
            {
                const int in_delta = scan_count - last_scan_count_snapshot;
                const uint64_t out_delta = processed_scans - last_processed_snapshot;
                const double in_hz = in_delta / dt_rate;
                const double out_hz = out_delta / dt_rate;
                fastlio_diag_file_append(
                    "[LoadDiag] in_hz=%.2f out_hz=%.2f map_cost=%.1fms lag=%.3fs q(lidar=%zu, imu=%zu)",
                    in_hz, out_hz, mapping_cost_ms, map_lag_sec, lidar_q, imu_q);
                last_scan_count_snapshot = scan_count;
                last_processed_snapshot = processed_scans;
                last_rate_tp = now_tp;
            }

            if (runtime_pos_log)
            {
                frame_num++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1) / frame_num + (kdtree_incremental_time) / frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time + solve_H_time) / frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n", t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu, aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose() << " " << ext_euler.transpose() << " " << state_point.offset_T_L_I.transpose() << " " << state_point.vel.transpose()
                         << " " << state_point.bg.transpose() << " " << state_point.ba.transpose() << " " << state_point.grav << " " << feats_undistort->points.size() << endl;
                dump_lio_state_to_log(fp);
            }
        }
        
        // By default publish odom/TF only after a LiDAR EKF update.
        // Enable publish.high_freq_odom_en to additionally extrapolate with buffered IMU.
        if (flg_EKF_inited && (processed_scan_this_cycle || high_freq_odom_en_))
        {
            publish_high_freq_odom(high_freq_odom_en_);

        }
    }

    void map_publish_callback()
    {
        if (map_pub_en)
            publish_map(pubLaserCloudMap_, dense_pub_en);
    }

    void map_save_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        RCLCPP_INFO(this->get_logger(), "Saving map to %s...", map_file_path.c_str());
        if (pcd_save_en)
        {
            save_to_pcd();
            res->success = true;
            res->message = "Map saved.";
        }
        else
        {
            res->success = false;
            res->message = "Map save disabled.";
        }
    }

    // ================= [新增函数] 高频 IMU 里程计发布 =================
    void publish_high_freq_odom(bool use_imu_prediction)
    {
        const auto hf_start = std::chrono::steady_clock::now();
        if (!flg_EKF_inited)
            return;
        if (use_imu_prediction && imu_buffer.empty())
            return;

        state_ikfom state_tmp = state_point;
        double last_update_time = lidar_end_time; 

        mtx_buffer.lock();
        if (use_imu_prediction)
        {
        for (const auto &imu_msg : imu_buffer)
        {
            double imu_time = get_time_sec(imu_msg->header.stamp);

            if (imu_time <= last_update_time)
                continue;

            double dt = imu_time - last_update_time;
            if (dt <= 0)
                continue;

            V3D acc_raw(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
            V3D gyr_raw(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);

            V3D acc_unbiased = acc_raw - state_tmp.ba;
            V3D gyr_unbiased = gyr_raw - state_tmp.bg;

            // 姿态更新: 显式定义 V3D 类型的角增量，解决 Eigen 表达式匹配失败的问题
            V3D angle_inc = gyr_unbiased * dt; 
            state_tmp.rot = state_tmp.rot * Exp(std::move(angle_inc));

            // 速度位置更新
            V3D grav_vec(state_tmp.grav[0], state_tmp.grav[1], state_tmp.grav[2]);
            V3D acc_world = state_tmp.rot * acc_unbiased + grav_vec;

            state_tmp.pos = state_tmp.pos + state_tmp.vel * dt + 0.5 * acc_world * dt * dt;
            state_tmp.vel = state_tmp.vel + acc_world * dt;

            last_update_time = imu_time;
        }
        }
        mtx_buffer.unlock();

        // 4. 发布逻辑
        nav_msgs::msg::Odometry odom_high_freq;
        odom_high_freq.header.frame_id = odom_frame;
        odom_high_freq.child_frame_id = base_frame;
        odom_high_freq.header.stamp = get_ros_time(last_update_time);

        geometry_msgs::msg::Quaternion geoQuat_high;
        geoQuat_high.x = state_tmp.rot.coeffs()[0];
        geoQuat_high.y = state_tmp.rot.coeffs()[1];
        geoQuat_high.z = state_tmp.rot.coeffs()[2];
        geoQuat_high.w = state_tmp.rot.coeffs()[3];

        // FAST-LIO tracks the MID360/IMU pose. Publish base_link as the robot body center.
        const V3D base_pos = state_tmp.pos - state_tmp.rot * base_to_lidar_T;

        odom_high_freq.pose.pose.position.x = base_pos(0);
        odom_high_freq.pose.pose.position.y = base_pos(1);
        odom_high_freq.pose.pose.position.z = base_pos(2);

        odom_high_freq.pose.pose.orientation = geoQuat_high;
        // 用当前高频预测姿态(geoQuat_high)计算 yaw，避免与旧状态混用
        double yaw = atan2(
            2.0 * (geoQuat_high.w * geoQuat_high.z + geoQuat_high.x * geoQuat_high.y),
            1.0 - 2.0 * (geoQuat_high.y * geoQuat_high.y + geoQuat_high.z * geoQuat_high.z));

        // 二阶段诊断：观测 Fast-LIO 时间滞后与时钟桥 offset（便于定位 1s 级延迟根因）
        const auto odom_stamp = get_ros_time(last_update_time);
        const auto nav_now = this->now();
        const double lag_sec = nav_now.seconds() - last_update_time;
        {
            const auto tsteady = std::chrono::steady_clock::now();
            double dt = 0.0;
            if (fastlio_lag_pub_inited_)
            {
                dt = std::chrono::duration<double>(tsteady - fastlio_lag_pub_last_).count();
            }
            if (!fastlio_lag_pub_inited_ || dt >= kFastlioLagPublishPeriodSec)
            {
                fastlio_lag_pub_inited_ = true;
                fastlio_lag_pub_last_ = tsteady;
                std_msgs::msg::Float64 lag_msg;
                lag_msg.data = lag_sec;
                pub_fastlio_lag_sec_->publish(lag_msg);
            }
        }
        {
            static std::chrono::steady_clock::time_point last_td{};
            static bool td_inited = false;
            const auto st_now = std::chrono::steady_clock::now();
            if (!td_inited || std::chrono::duration<double>(st_now - last_td).count() >= 2.0)
            {
                td_inited = true;
                last_td = st_now;
                fastlio_diag_file_append(
                    "[TimeDiag] now=%.6f last_update=%.6f lag=%.3f s lidar_end=%.6f offset=%.6f",
                    nav_now.seconds(), last_update_time, lag_sec, lidar_end_time, lidar_to_ros_clock_offset);
            }
        }

        // TF 标准导航链路: odom -> base_footprint (平面 x/y/yaw)，
        // base_footprint -> base_link（地面以上高度 z 及 roll/pitch，不再叠加 yaw）
        Eigen::Matrix3d R_full = state_tmp.rot.toRotationMatrix();
        Eigen::Matrix3d R_yaw = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Matrix3d R_bf_bl_mat = R_yaw.transpose() * R_full;
        Eigen::Quaterniond q_bf_bl(R_bf_bl_mat);
        q_bf_bl.normalize();
        Eigen::Vector3d t_bf_bl =
            R_yaw.transpose() * Eigen::Vector3d(0.0, 0.0, base_pos(2));

        geometry_msgs::msg::TransformStamped trans_odom_bf;
        trans_odom_bf.header.frame_id = odom_frame;
        trans_odom_bf.child_frame_id = base_footprint_frame;
        trans_odom_bf.header.stamp = odom_stamp;
        trans_odom_bf.transform.translation.x = base_pos(0);
        trans_odom_bf.transform.translation.y = base_pos(1);
        trans_odom_bf.transform.translation.z = 0.0;
        trans_odom_bf.transform.rotation.x = 0.0;
        trans_odom_bf.transform.rotation.y = 0.0;
        trans_odom_bf.transform.rotation.z = sin(yaw / 2.0);
        trans_odom_bf.transform.rotation.w = cos(yaw / 2.0);

        geometry_msgs::msg::TransformStamped trans_bf_bl_msg;
        trans_bf_bl_msg.header.frame_id = base_footprint_frame;
        trans_bf_bl_msg.child_frame_id = base_frame;
        trans_bf_bl_msg.header.stamp = odom_stamp;
        trans_bf_bl_msg.transform.translation.x = t_bf_bl.x();
        trans_bf_bl_msg.transform.translation.y = t_bf_bl.y();
        trans_bf_bl_msg.transform.translation.z = t_bf_bl.z();
        trans_bf_bl_msg.transform.rotation.x = q_bf_bl.x();
        trans_bf_bl_msg.transform.rotation.y = q_bf_bl.y();
        trans_bf_bl_msg.transform.rotation.z = q_bf_bl.z();
        trans_bf_bl_msg.transform.rotation.w = q_bf_bl.w();

        tf_broadcaster_->sendTransform(trans_odom_bf);
        tf_broadcaster_->sendTransform(trans_bf_bl_msg);
        
        // 发布 Odom 消息
        // 填充协方差 (直接复用 KF 的 P)
        auto P = kf.get_P(); 
        for (int i = 0; i < 6; i++)
        {
            int k = i < 3 ? i + 3 : i - 3;
            odom_high_freq.pose.covariance[i * 6 + 0] = P(k, 3);
            odom_high_freq.pose.covariance[i * 6 + 1] = P(k, 4);
            odom_high_freq.pose.covariance[i * 6 + 2] = P(k, 5);
            odom_high_freq.pose.covariance[i * 6 + 3] = P(k, 0);
            odom_high_freq.pose.covariance[i * 6 + 4] = P(k, 1);
            odom_high_freq.pose.covariance[i * 6 + 5] = P(k, 2);
        }
        pubOdomAftMapped_->publish(odom_high_freq);
            nav_msgs::msg::Odometry odom2d;
        odom2d.header.frame_id = odom_frame;
        odom2d.child_frame_id = base_footprint_frame;
        odom2d.header.stamp = odom_stamp;
        
        // 设置2D位置（z设为0）
        odom2d.pose.pose.position.x = base_pos(0);
        odom2d.pose.pose.position.y = base_pos(1);
        odom2d.pose.pose.position.z = 0.0;
        
        // 从四元数提取yaw角度
        // geoQuat的格式是 (x, y, z, w)
        // double yaw = atan2(2.0 * (geoQuat.w * geoQuat.z + geoQuat.x * geoQuat.y),
        //                 1.0 - 2.0 * (geoQuat.y * geoQuat.y + geoQuat.z * geoQuat.z));
        
        // 创建只有yaw的四元数（roll=0, pitch=0）
        odom2d.pose.pose.orientation.x = 0.0;
        odom2d.pose.pose.orientation.y = 0.0;
        odom2d.pose.pose.orientation.z = sin(yaw / 2.0);
        odom2d.pose.pose.orientation.w = cos(yaw / 2.0);
        
        // 获取协方差矩阵
        // auto P = kf.get_P();
        
        // 初始化协方差矩阵为0
        for (int i = 0; i < 36; i++)
        {
            odom2d.pose.covariance[i] = 0.0;
        }
        
        // 设置2D相关的协方差 (x, y, yaw)
        // 协方差矩阵索引: 0=x, 7=y, 35=yaw
        // P矩阵中: P(3,3)=x, P(4,4)=y, P(2,2)=yaw (z rotation)
        odom2d.pose.covariance[0] = P(3, 3);   // x variance
        odom2d.pose.covariance[7] = P(4, 4);   // y variance
        odom2d.pose.covariance[35] = P(2, 2);  // yaw variance
        // Odometry twist is expressed in child_frame_id (base_footprint).
        Eigen::Vector3d linear_vel_world(state_tmp.vel(0), state_tmp.vel(1), state_tmp.vel(2));
        Eigen::Vector3d linear_vel_base = R_yaw.transpose() * linear_vel_world;

        static bool last_twist_sample_inited = false;
        static double last_twist_yaw = 0.0;
        static double last_twist_time = 0.0;
        double yaw_rate = 0.0;
        if (last_twist_sample_inited)
        {
            const double dt_twist = last_update_time - last_twist_time;
            if (dt_twist > 1.0e-4)
            {
                const double yaw_delta = atan2(sin(yaw - last_twist_yaw), cos(yaw - last_twist_yaw));
                yaw_rate = yaw_delta / dt_twist;
            }
        }
        last_twist_sample_inited = true;
        last_twist_yaw = yaw;
        last_twist_time = last_update_time;

        odom2d.twist.twist.linear.x = linear_vel_base.x();
        odom2d.twist.twist.linear.y = linear_vel_base.y();
        odom2d.twist.twist.linear.z = 0.0;
        odom2d.twist.twist.angular.x = 0.0;
        odom2d.twist.twist.angular.y = 0.0;
        odom2d.twist.twist.angular.z = yaw_rate;
        
        // 初始化twist协方差
        for (int i = 0; i < 36; i++)
        {
            odom2d.twist.covariance[i] = 0.0;
        }
        
        // 设置速度协方差（如果需要）
        // odom2d.twist.covariance[0] = ...;  // linear x
        // odom2d.twist.covariance[35] = ...; // angular z
        
        pubOdomAftMapped_2d_->publish(odom2d);

        const auto hf_end = std::chrono::steady_clock::now();
        const double hf_cost_ms = std::chrono::duration<double, std::milli>(hf_end - hf_start).count();
        RCLCPP_DEBUG(
            this->get_logger(), "[PerfDiag] publish_high_freq_odom cost=%.3f ms", hf_cost_ms);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_2d_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fastlio_lag_sec_;
    static constexpr double kFastlioLagPublishPeriodSec = 10.0;
    bool fastlio_lag_pub_inited_{false};
    std::chrono::steady_clock::time_point fastlio_lag_pub_last_{};
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;

    rclcpp::CallbackGroup::SharedPtr cb_group_subscriptions_;
    rclcpp::CallbackGroup::SharedPtr cb_group_timers_;

    bool effect_pub_en = false, map_pub_en = false, path_en = true, scan_pub_en = true, dense_pub_en = true, scan_body_pub_en = true;
    bool high_freq_odom_en_ = false;
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    double epsi[23] = {0.001};

    FILE *fp = nullptr;
    ofstream fout_pre, fout_out, fout_dbg;
};

static void fastlio_diag_log_recv_lidar_impl(double msg_stamp_sec, double recv_now_sec, const char *src_tag,
                                              double cb_dt_steady_sec)
{
    if (!g_fastlio_time_diag_file_en.load(std::memory_order_relaxed))
        return;
    if (!g_fastlio_diag_node)
        return;
    static std::chrono::steady_clock::time_point last{};
    static bool last_inited = false;
    const auto now_tp = std::chrono::steady_clock::now();
    if (last_inited && std::chrono::duration<double>(now_tp - last).count() < 2.0)
        return;
    last_inited = true;
    last = now_tp;
    fastlio_diag_file_append(
        "[FastlioRecv] lidar(%s) msg_stamp=%.6f recv_ros_now=%.6f (recv-stamp)=%.3fs cb_dt=%.4fs", src_tag,
        msg_stamp_sec, recv_now_sec, recv_now_sec - msg_stamp_sec, cb_dt_steady_sec);
}

static void fastlio_diag_log_recv_lidar(double msg_stamp_sec, double recv_now_sec, double cb_dt_steady_sec)
{
    fastlio_diag_log_recv_lidar_impl(msg_stamp_sec, recv_now_sec, "LivoxCustomMsg", cb_dt_steady_sec);
}

static void fastlio_diag_log_recv_imu(double raw_msg_stamp_sec, double used_stamp_sec, double recv_now_sec,
                                      double cb_dt_steady_sec)
{
    if (!g_fastlio_time_diag_file_en.load(std::memory_order_relaxed))
        return;
    if (!g_fastlio_diag_node)
        return;
    static std::chrono::steady_clock::time_point last{};
    static bool last_inited = false;
    const auto now_tp = std::chrono::steady_clock::now();
    if (last_inited && std::chrono::duration<double>(now_tp - last).count() < 2.0)
        return;
    last_inited = true;
    last = now_tp;
    fastlio_diag_file_append(
        "[FastlioRecv] imu raw_stamp=%.6f used_stamp=%.6f recv_ros_now=%.6f "
        "(recv-raw)=%.3fs (recv-used)=%.3fs cb_dt=%.4fs",
        raw_msg_stamp_sec, used_stamp_sec, recv_now_sec, recv_now_sec - raw_msg_stamp_sec,
        recv_now_sec - used_stamp_sec, cb_dt_steady_sec);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    signal(SIGINT, SigHandle);

    auto node = std::make_shared<LaserMappingNode>();
    // 多线程 executor：与 Reentrant 订阅组配合，使雷达/IMU 回调可与定时器并行（缓冲仍由 mtx_buffer 保护）
    constexpr size_t k_executor_threads = 4;
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), k_executor_threads);
    executor.add_node(node);
    executor.spin();

    if (rclcpp::ok())
        rclcpp::shutdown();
    cout << "Total points: " << pcl_wait_pub->size() << endl;
    cout << "pcd_save_en " <<  pcd_save_en << endl;
    if (pcl_wait_pub->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name << endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_pub);
    }

    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(), "w");
        fprintf(fp2, "time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0; i < time_log_counter; i++)
        {
            fprintf(fp2, "%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n", T1[i], s_plot[i], int(s_plot2[i]), s_plot3[i], s_plot4[i], int(s_plot5[i]), s_plot6[i], int(s_plot7[i]), int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    return 0;
}
