#ifndef _TinyVersatilePoseGraphSLAM_H_
#define _TinyVersatilePoseGraphSLAM_H_

#include <Eigen/Eigen>

class TinyVersatilePoseGraphSLAM{
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

        struct TaitBryanPoseUncertainty
        {
            double px_1_sigma_m = 0.01;
            double py_1_sigma_m = 0.01;
            double pz_1_sigma_m = 0.01;
            double om_1_sigma_deg = 0.1;
            double fi_1_sigma_deg = 0.1;
            double ka_1_sigma_deg = 0.1;
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

        struct EdgeTaitBryan{
            unsigned int index_from;
            unsigned int index_to;
            Eigen::Affine3d measurement;
            TaitBryanPoseUncertainty uncertainty_covariance_information_matrix_inverse;
            TaitBryanPoseRobustKernelW robust_kernel_W;
        };

        TinyVersatilePoseGraphSLAM(){;};
        ~TinyVersatilePoseGraphSLAM() { ; };

        static std::pair<Eigen::SparseMatrix<double>, Eigen::SparseMatrix<double>> get_AtPA_AtPB_pose_graph_tait_byan_wc(const std::vector<Eigen::Affine3d> &m_poses, const std::vector<EdgeTaitBryan> &tb_edges);
        static double apply_result_tait_bryan_wc(const Eigen::SparseMatrix<double> &x, std::vector<Eigen::Affine3d> &m_poses);

        static void relative_pose_obs_eq_tait_bryan_wc_case1_AtPA_simplified(Eigen::Matrix<double, 12, 12> &AtPA, const double &tx_1, const double &ty_1, const double &tz_1, const double &om_1, const double &fi_1, const double &ka_1, const double &tx_2, const double &ty_2, const double &tz_2, const double &om_2, const double &fi_2, const double &ka_2, const double &p_x, const double &p_y, const double &p_z, const double &p_om, const double &p_fi, const double &p_ka);
        static void relative_pose_obs_eq_tait_bryan_wc_case1_AtPB_simplified(Eigen::Matrix<double, 12, 1> &AtPB, const double &tx_1, const double &ty_1, const double &tz_1, const double &om_1, const double &fi_1, const double &ka_1, const double &tx_2, const double &ty_2, const double &tz_2, const double &om_2, const double &fi_2, const double &ka_2, const double &tx_m, const double &ty_m, const double &tz_m, const double &om_m, const double &fi_m, const double &ka_m, const double &p_x, const double &p_y, const double &p_z, const double &p_om, const double &p_fi, const double &p_ka);

        static TaitBryanPose pose_tait_bryan_from_affine_matrix(Eigen::Affine3d m);
        static Eigen::Affine3d affine_matrix_from_pose_tait_bryan(TaitBryanPose pose);

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

        struct RodriguesPoseUncertainty
        {
            double px_1_sigma_m = 0.01;
            double py_1_sigma_m = 0.01;
            double pz_1_sigma_m = 0.01;
            double sx_1_sigma = 1.0;
            double sy_1_sigma = 1.0;
            double sz_1_sigma = 1.0;
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

        struct EdgeRodrigues{
            unsigned int index_from;
            unsigned int index_to;
            Eigen::Affine3d measurement;
            RodriguesPoseUncertainty uncertainty_covariance_information_matrix_inverse;
            RodriguesPoseRobustKernelW robust_kernel_W;
        };

        static std::pair<Eigen::SparseMatrix<double>, Eigen::SparseMatrix<double>> get_AtPA_AtPB_pose_graph_rodrigues_wc(const std::vector<Eigen::Affine3d> &m_poses, const std::vector<EdgeRodrigues> &rodrigues_edges);
        static double apply_result_rodrigues_wc(const Eigen::SparseMatrix<double> &x, std::vector<Eigen::Affine3d> &m_poses);

        static void relative_pose_obs_eq_rodrigues_wc(Eigen::Matrix<double, 6, 1> &delta, double px_1, double py_1, double pz_1, double sx_1, double sy_1, double sz_1, double px_2, double py_2, double pz_2, double sx_2, double sy_2, double sz_2, double px_m, double py_m, double pz_m, double sx_m, double sy_m, double sz_m);
        static void relative_pose_obs_eq_rodrigues_wc_jacobian(Eigen::Matrix<double, 6, 12, Eigen::RowMajor> &j, double px_1, double py_1, double pz_1, double sx_1, double sy_1, double sz_1, double px_2, double py_2, double pz_2, double sx_2, double sy_2, double sz_2);
        static void relative_pose_rodrigues_wc(Eigen::Matrix<double, 6, 1> &relative_pose, double px_1, double py_1, double pz_1, double sx_1, double sy_1, double sz_1, double px_2, double py_2, double pz_2, double sx_2, double sy_2, double sz_2);

        static RodriguesPose pose_rodrigues_from_affine_matrix(Eigen::Affine3d m);
        static void orthogonalize_rotation(Eigen::Affine3d& m);
        static Eigen::Affine3d affine_matrix_from_pose_rodrigues(const RodriguesPose& pr);

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

        struct QuaternionPoseUncertainty
        {
            double px_1_sigma_m = 0.01;
            double py_1_sigma_m = 0.01;
            double pz_1_sigma_m = 0.01;
            double q0_1_sigma = 1.0;
            double q1_1_sigma = 1.0;
            double q2_1_sigma = 1.0;
            double q3_1_sigma = 1.0;
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
            QuaternionPoseUncertainty uncertainty_covariance_information_matrix_inverse;
            QuaternionPoseRobustKernelW robust_kernel_W;
        };
    };

#endif