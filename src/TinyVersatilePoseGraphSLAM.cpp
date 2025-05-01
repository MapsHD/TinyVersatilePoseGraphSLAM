#include <TinyVersatilePoseGraphSLAM.h>
#include <math.h>

Eigen::Matrix<double, 6, 6> TinyVersatilePoseGraphSLAM::rodrigues_covariance_from_tait_bryan_covariance(const Eigen::Matrix<double, 6, 6> &cov_tb, const TinyVersatilePoseGraphSLAM::RodriguesPose &pose_r)
{
    Eigen::Matrix<double, 6, 6> j_r;
    TinyVersatilePoseGraphSLAM::uncertainty_pose_tait_bryan_to_rodrigues(j_r, pose_r.px, pose_r.py, pose_r.pz, pose_r.sx, pose_r.sy, pose_r.sz);
    return j_r * cov_tb * j_r.transpose();
}

Eigen::Matrix<double, 7, 7> TinyVersatilePoseGraphSLAM::quaternion_covariance_from_tait_bryan_covariance(const Eigen::Matrix<double, 6, 6> &cov_tb, const TinyVersatilePoseGraphSLAM::TaitBryanPose &pose_tb)
{
    Eigen::Matrix<double, 7, 6> j_q;
    TinyVersatilePoseGraphSLAM::uncertainty_pose_tait_bryan_to_quaternion(j_q, pose_tb.px, pose_tb.py, pose_tb.pz, pose_tb.om, pose_tb.fi, pose_tb.ka);
    return j_q * cov_tb * j_q.transpose();
}

Eigen::Matrix<double, 6, 6> TinyVersatilePoseGraphSLAM::quaternion_covariance_to_tait_bryan_covariance(const Eigen::Matrix<double, 7, 7> &cov_q, const TinyVersatilePoseGraphSLAM::QuaternionPose &pose_q)
{
    Eigen::Matrix<double, 6, 7> j;
    uncertainty_pose_quaternion_to_tait_bryan(j, pose_q.px, pose_q.py, pose_q.pz, pose_q.q0, pose_q.q1, pose_q.q2, pose_q.q3);
    return j * cov_q * j.transpose();
}