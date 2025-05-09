#ifndef _TinyVersatilePoseGraphSLAM_H_
#define _TinyVersatilePoseGraphSLAM_H_

#include <Eigen/Eigen>

class TinyVersatilePoseGraphSLAM
{
public:
    //----------- Tait Bryan ----------
    struct TaitBryanPose
    {
        double px;
        double py;
        double pz;
        double om;
        double fi;
        double ka;
    };

    struct TaitBryanPoseRobustKernelW
    {
        double px_robust_kernel_W = 1.0;
        double py_robust_kernel_W = 1.0;
        double pz_robust_kernel_W = 1.0;
        double om_robust_kernel_W = 1.0;
        double fi_robust_kernel_W = 1.0;
        double ka_robust_kernel_W = 1.0;
    };

    struct EdgeTaitBryan
    {
        unsigned int index_from;
        unsigned int index_to;
        Eigen::Affine3d measurement;
        Eigen::Matrix<double, 6, 6> covariance;
        Eigen::Matrix<double, 6, 6> information_matrix;
        TaitBryanPoseRobustKernelW robust_kernel_W;
    };

    TinyVersatilePoseGraphSLAM() { ; };
    ~TinyVersatilePoseGraphSLAM() { ; };

    static std::pair<Eigen::SparseMatrix<double>, Eigen::SparseMatrix<double>> get_AtPA_AtPB_pose_graph_tait_byan_wc(const std::vector<std::pair<Eigen::Affine3d, bool>> &m_poses, const std::vector<EdgeTaitBryan> &edges);
    static double apply_result_tait_bryan_wc(const Eigen::SparseMatrix<double> &x, std::vector<std::pair<Eigen::Affine3d, bool>> &m_poses);

    static void relative_pose_obs_eq_tait_bryan_wc_case1_AtPA_simplified(Eigen::Matrix<double, 12, 12> &AtPA, const double &tx_1, const double &ty_1, const double &tz_1, const double &om_1, const double &fi_1, const double &ka_1, const double &tx_2, const double &ty_2, const double &tz_2, const double &om_2, const double &fi_2, const double &ka_2, const double &p_x, const double &p_y, const double &p_z, const double &p_om, const double &p_fi, const double &p_ka);
    static void relative_pose_obs_eq_tait_bryan_wc_case1_AtPB_simplified(Eigen::Matrix<double, 12, 1> &AtPB, const double &tx_1, const double &ty_1, const double &tz_1, const double &om_1, const double &fi_1, const double &ka_1, const double &tx_2, const double &ty_2, const double &tz_2, const double &om_2, const double &fi_2, const double &ka_2, const double &tx_m, const double &ty_m, const double &tz_m, const double &om_m, const double &fi_m, const double &ka_m, const double &p_x, const double &p_y, const double &p_z, const double &p_om, const double &p_fi, const double &p_ka);

    static TaitBryanPose pose_tait_bryan_from_affine_matrix(const Eigen::Affine3d &m);
    static Eigen::Affine3d affine_matrix_from_pose_tait_bryan(const TaitBryanPose &pose);

    //----------- Rodrigues ----------
    struct RodriguesPose
    {
        double px;
        double py;
        double pz;
        double sx;
        double sy;
        double sz;
    };

    struct RodriguesPoseRobustKernelW
    {
        double px_robust_kernel_W = 1.0;
        double py_robust_kernel_W = 1.0;
        double pz_robust_kernel_W = 1.0;
        double sx_robust_kernel_W = 1.0;
        double sy_robust_kernel_W = 1.0;
        double sz_robust_kernel_W = 1.0;
    };

    struct EdgeRodrigues
    {
        unsigned int index_from;
        unsigned int index_to;
        Eigen::Affine3d measurement;
        Eigen::Matrix<double, 6, 6> covariance;
        Eigen::Matrix<double, 6, 6> information_matrix;
        RodriguesPoseRobustKernelW robust_kernel_W;
    };

    static std::pair<Eigen::SparseMatrix<double>, Eigen::SparseMatrix<double>> get_AtPA_AtPB_pose_graph_rodrigues_wc(const std::vector<std::pair<Eigen::Affine3d, bool>> &m_poses, const std::vector<EdgeRodrigues> &edges);
    static double apply_result_rodrigues_wc(const Eigen::SparseMatrix<double> &x, std::vector<std::pair<Eigen::Affine3d, bool>> &m_poses);

    static void relative_pose_obs_eq_rodrigues_wc(Eigen::Matrix<double, 6, 1> &delta, double px_1, double py_1, double pz_1, double sx_1, double sy_1, double sz_1, double px_2, double py_2, double pz_2, double sx_2, double sy_2, double sz_2, double px_m, double py_m, double pz_m, double sx_m, double sy_m, double sz_m);
    static void relative_pose_obs_eq_rodrigues_wc_jacobian(Eigen::Matrix<double, 6, 12, Eigen::RowMajor> &j, double px_1, double py_1, double pz_1, double sx_1, double sy_1, double sz_1, double px_2, double py_2, double pz_2, double sx_2, double sy_2, double sz_2);
    static void relative_pose_rodrigues_wc(Eigen::Matrix<double, 6, 1> &relative_pose, double px_1, double py_1, double pz_1, double sx_1, double sy_1, double sz_1, double px_2, double py_2, double pz_2, double sx_2, double sy_2, double sz_2);

    static RodriguesPose pose_rodrigues_from_affine_matrix(const Eigen::Affine3d &m);
    static void orthogonalize_rotation(Eigen::Affine3d &m);
    static Eigen::Affine3d affine_matrix_from_pose_rodrigues(const RodriguesPose &pr);

    //------------Quaternion

    struct QuaternionPose
    {
        double px;
        double py;
        double pz;
        double q0;
        double q1;
        double q2;
        double q3;
    };

    struct QuaternionPoseRobustKernelW
    {
        double px_robust_kernel_W = 1.0;
        double py_robust_kernel_W = 1.0;
        double pz_robust_kernel_W = 1.0;
        double q0_robust_kernel_W = 1.0;
        double q1_robust_kernel_W = 1.0;
        double q2_robust_kernel_W = 1.0;
        double q3_robust_kernel_W = 1.0;
    };

    struct EdgeQuaternion
    {
        unsigned int index_from;
        unsigned int index_to;
        Eigen::Affine3d measurement;
        Eigen::Matrix<double, 7, 7> covariance;
        Eigen::Matrix<double, 7, 7> information_matrix;
        QuaternionPoseRobustKernelW robust_kernel_W;
    };

    static std::pair<Eigen::SparseMatrix<double>, Eigen::SparseMatrix<double>> get_AtPA_AtPB_pose_graph_quaternion_wc(const std::vector<std::pair<Eigen::Affine3d, bool>> &m_poses, const std::vector<EdgeQuaternion> &edges);
    static double apply_result_quaternion_wc(const Eigen::SparseMatrix<double> &x, std::vector<std::pair<Eigen::Affine3d, bool>> &m_poses);

    static void normalize_quaternion(QuaternionPose &pq);
    static Eigen::Affine3d affine_matrix_from_pose_quaternion(const QuaternionPose &pq);
    static QuaternionPose pose_quaternion_from_affine_matrix(const Eigen::Affine3d &m);
    static double normalize_angle(double src_rad);

    static void relative_pose_obs_eq_quaternion_wc(Eigen::Matrix<double, 7, 1> &delta, const double &px_1, const double &py_1, const double &pz_1, const double &q0_1, const double &q1_1, const double &q2_1, const double &q3_1, const double &px_2, const double &py_2, const double &pz_2, const double &q0_2, const double &q1_2, const double &q2_2, const double &q3_2, const double &px_m, const double &py_m, const double &pz_m, const double &q0_m, const double &q1_m, const double &q2_m, const double &q3_m);
    static void relative_pose_obs_eq_quaternion_wc_jacobian(Eigen::Matrix<double, 7, 14, Eigen::RowMajor> &j, const double &px_1, const double &py_1, const double &pz_1, const double &q0_1, const double &q1_1, const double &q2_1, const double &q3_1, const double &px_2, const double &py_2, const double &pz_2, const double &q0_2, const double &q1_2, const double &q2_2, const double &q3_2);
    static void relative_pose_quaternion_wc(Eigen::Matrix<double, 7, 1> &relative_pose, const double &px_1, const double &py_1, const double &pz_1, const double &q0_1, const double &q1_1, const double &q2_1, const double &q3_1, const double &px_2, const double &py_2, const double &pz_2, const double &q0_2, const double &q1_2, const double &q2_2, const double &q3_2);

    static void quaternion_constraint(double &delta, const double &q0, const double &q1, const double &q2, const double &q3);
    static void quaternion_constraint_jacobian(Eigen::Matrix<double, 1, 4> &j, const double &q0, const double &q1, const double &q2, const double &q3);

    ////////////////////////////////uncertainty////////////////////////////
    static void uncertainty_pose_tait_bryan_to_rodrigues(Eigen::Matrix<double, 6, 6> &j, const double &tx_tb, const double &ty_tb, const double &tz_tb, const double &om_tb, const double &fi_tb, const double &ka_tb);
    static void uncertainty_pose_rodrigues_to_tait_bryan(Eigen::Matrix<double, 6, 6> &j, const double &tx_rodrigues, const double &ty_rodrigues, const double &tz_rodrigues, const double &sx_rodrigues, const double &sy_rodrigues, const double &sz_rodrigues);
    
    static Eigen::Matrix<double, 6, 6> rodrigues_covariance_from_tait_bryan_covariance(const Eigen::Matrix<double, 6, 6> &cov_tb, const TinyVersatilePoseGraphSLAM::RodriguesPose &pose_r);
    static Eigen::Matrix<double, 7, 7> quaternion_covariance_from_tait_bryan_covariance(const Eigen::Matrix<double, 6, 6> &cov_tb, const TinyVersatilePoseGraphSLAM::TaitBryanPose &pose_tb);
    
    static Eigen::Matrix<double, 6, 6> quaternion_covariance_to_tait_bryan_covariance(const Eigen::Matrix<double, 7, 7> &cov_q, const TinyVersatilePoseGraphSLAM::QuaternionPose &pose_q);

    static void uncertainty_pose_quaternion_to_tait_bryan(Eigen::Matrix<double, 6, 7> &j, const double &tx, const double &ty, const double &tz, const double &q0, const double &q1, const double &q2, const double &q3);
    static void uncertainty_pose_tait_bryan_to_quaternion(Eigen::Matrix<double, 7, 6> &j, const double &tx_tb, const double &ty_tb, const double &tz_tb, const double &om_tb, const double &fi_tb, const double &ka_tb);
};

#endif