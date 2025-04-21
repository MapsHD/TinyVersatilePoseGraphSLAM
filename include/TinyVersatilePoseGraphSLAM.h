#ifndef _TinyVersatilePoseGraphSLAM_H_
#define _TinyVersatilePoseGraphSLAM_H_

#include <Eigen/Eigen>

class TinyVersatilePoseGraphSLAM{
    public:
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
            double px_1_sigma_m;
            double py_1_sigma_m;
            double pz_1_sigma_m;
            double om_1_sigma_deg;
            double fi_1_sigma_deg;
            double ka_1_sigma_deg;
        };

        struct EdgeTaitBryan{
            unsigned int index_from;
            unsigned int index_to;
            Eigen::Affine3d measurement;
            TaitBryanPoseUncertainty uncertainty_covariance_information_matrix_inverse;
        };

        TinyVersatilePoseGraphSLAM(){;};
        ~TinyVersatilePoseGraphSLAM() { ; };

        static std::pair<Eigen::SparseMatrix<double>, Eigen::SparseMatrix<double>> get_AtPA_AtPB(const std::vector<Eigen::Affine3d> &m_poses, const std::vector<EdgeTaitBryan> &tb_edges);

        static void relative_pose_obs_eq_tait_bryan_wc_case1_AtPA_simplified(Eigen::Matrix<double, 12, 12> &AtPA, double tx_1, double ty_1, double tz_1, double om_1, double fi_1, double ka_1, double tx_2, double ty_2, double tz_2, double om_2, double fi_2, double ka_2, double p_x, double p_y, double p_z, double p_om, double p_fi, double p_ka);
        static void relative_pose_obs_eq_tait_bryan_wc_case1_AtPB_simplified(Eigen::Matrix<double, 12, 1> &AtPB, double tx_1, double ty_1, double tz_1, double om_1, double fi_1, double ka_1, double tx_2, double ty_2, double tz_2, double om_2, double fi_2, double ka_2, double tx_m, double ty_m, double tz_m, double om_m, double fi_m, double ka_m, double p_x, double p_y, double p_z, double p_om, double p_fi, double p_ka);

        static TaitBryanPose pose_tait_bryan_from_affine_matrix(Eigen::Affine3d m);
        static Eigen::Affine3d affine_matrix_from_pose_tait_bryan(TaitBryanPose pose);
};

#endif