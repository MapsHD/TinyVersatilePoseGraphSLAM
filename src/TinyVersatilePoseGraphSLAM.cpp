#include <TinyVersatilePoseGraphSLAM.h>
#include <math.h>

std::pair<Eigen::SparseMatrix<double>, Eigen::SparseMatrix<double>> TinyVersatilePoseGraphSLAM::get_AtPA_AtPB(const std::vector<Eigen::Affine3d> &m_poses, const std::vector<EdgeTaitBryan> &tb_edges)
{
    std::vector<TaitBryanPose> tb_poses;
    for (const auto &m : m_poses)
    {
        tb_poses.emplace_back(pose_tait_bryan_from_affine_matrix(m));
    }

    std::pair<Eigen::SparseMatrix<double>, Eigen::SparseMatrix<double>> AtPA_AtPB;

    Eigen::SparseMatrix<double> AtPA_out(m_poses.size() * 6, m_poses.size() * 6);
    Eigen::SparseMatrix<double> AtPB_out(m_poses.size() * 6, 1);

    for (const auto &e : tb_edges)
    {
        TaitBryanPose tb_measurement = pose_tait_bryan_from_affine_matrix(e.measurement);
        double wx = 1.0 / (e.uncertainty_covariance_information_matrix_inverse.px_1_sigma_m * e.uncertainty_covariance_information_matrix_inverse.px_1_sigma_m);
        double wy = 1.0 / (e.uncertainty_covariance_information_matrix_inverse.py_1_sigma_m * e.uncertainty_covariance_information_matrix_inverse.py_1_sigma_m);
        double wz = 1.0 / (e.uncertainty_covariance_information_matrix_inverse.pz_1_sigma_m * e.uncertainty_covariance_information_matrix_inverse.pz_1_sigma_m);
        double om_rad = e.uncertainty_covariance_information_matrix_inverse.om_1_sigma_deg * M_PI / 180.0;
        double wom = 1.0 / (om_rad * om_rad);
        double fi_rad = e.uncertainty_covariance_information_matrix_inverse.fi_1_sigma_deg * M_PI / 180.0;
        double wfi = 1.0 / (fi_rad * fi_rad);
        double ka_rad = e.uncertainty_covariance_information_matrix_inverse.ka_1_sigma_deg * M_PI / 180.0;
        double wka = 1.0 / (ka_rad * ka_rad);

        Eigen::Matrix<double, 12, 12> AtPA;
        relative_pose_obs_eq_tait_bryan_wc_case1_AtPA_simplified(AtPA,
                                                                 tb_poses[e.index_from].px,
                                                                 tb_poses[e.index_from].py,
                                                                 tb_poses[e.index_from].pz,
                                                                 tb_poses[e.index_from].om,
                                                                 tb_poses[e.index_from].fi,
                                                                 tb_poses[e.index_from].ka,
                                                                 tb_poses[e.index_to].px,
                                                                 tb_poses[e.index_to].py,
                                                                 tb_poses[e.index_to].pz,
                                                                 tb_poses[e.index_to].om,
                                                                 tb_poses[e.index_to].fi,
                                                                 tb_poses[e.index_to].ka,
                                                                 wx,
                                                                 wy,
                                                                 wz,
                                                                 wom,
                                                                 wfi,
                                                                 wka);
        Eigen::Matrix<double, 12, 1> AtPB;
        relative_pose_obs_eq_tait_bryan_wc_case1_AtPB_simplified(AtPB,
                                                                 tb_poses[e.index_from].px,
                                                                 tb_poses[e.index_from].py,
                                                                 tb_poses[e.index_from].pz,
                                                                 tb_poses[e.index_from].om,
                                                                 tb_poses[e.index_from].fi,
                                                                 tb_poses[e.index_from].ka,
                                                                 tb_poses[e.index_to].px,
                                                                 tb_poses[e.index_to].py,
                                                                 tb_poses[e.index_to].pz,
                                                                 tb_poses[e.index_to].om,
                                                                 tb_poses[e.index_to].fi,
                                                                 tb_poses[e.index_to].ka,
                                                                 tb_measurement.px,
                                                                 tb_measurement.py,
                                                                 tb_measurement.pz,
                                                                 tb_measurement.om,
                                                                 tb_measurement.fi,
                                                                 tb_measurement.ka,
                                                                 wx,
                                                                 wy,
                                                                 wz,
                                                                 wom,
                                                                 wfi,
                                                                 wka);
        int ic_1 = e.index_from * 6;
        int ic_2 = e.index_to * 6;

        for (int row = 0; row < 6; row++)
        {
            for (int col = 0; col < 6; col++)
            {
                AtPA_out.coeffRef(ic_1 + row, ic_1 + col) += AtPA(row, col);
                AtPA_out.coeffRef(ic_1 + row, ic_2 + col) += AtPA(row, col + 6);
                AtPA_out.coeffRef(ic_2 + row, ic_1 + col) += AtPA(row + 6, col);
                AtPA_out.coeffRef(ic_2 + row, ic_2 + col) += AtPA(row + 6, col + 6);
            }
        }

        for (int row = 0; row < 6; row++)
        {
            AtPB_out.coeffRef(ic_1 + row, 0) -= AtPB(row, 0);
            AtPB_out.coeffRef(ic_2 + row, 0) -= AtPB(row + 6, 0);
        }
    }

    AtPA_AtPB.first = AtPA_out;
    AtPA_AtPB.second = AtPB_out;

    return AtPA_AtPB;
}

TinyVersatilePoseGraphSLAM::TaitBryanPose TinyVersatilePoseGraphSLAM::pose_tait_bryan_from_affine_matrix(Eigen::Affine3d m)
{
    TaitBryanPose pose;

    pose.px = m(0, 3);
    pose.py = m(1, 3);
    pose.pz = m(2, 3);

    if (m(0, 2) < 1)
    {
        if (m(0, 2) > -1)
        {
            // case 1
            pose.fi = asin(m(0, 2));
            pose.om = atan2(-m(1, 2), m(2, 2));
            pose.ka = atan2(-m(0, 1), m(0, 0));

            return pose;
        }
        else // r02 = −1
        {
            // case 2
            //  not a unique solution: thetaz − thetax = atan2 ( r10 , r11 )
            pose.fi = -M_PI / 2.0;
            pose.om = -atan2(m(1, 0), m(1, 1));
            pose.ka = 0.0;
            return pose;
        }
    }
    else
    {
        // case 3
        //  r02 = +1
        //  not a unique solution: thetaz + thetax = atan2 ( r10 , r11 )
        pose.fi = M_PI / 2.0;
        pose.om = atan2(m(1, 0), m(1, 1));
        pose.ka = 0.0;
        return pose;
    }
    return pose;
}

Eigen::Affine3d TinyVersatilePoseGraphSLAM::affine_matrix_from_pose_tait_bryan(TaitBryanPose pose)
{
	Eigen::Affine3d m = Eigen::Affine3d::Identity();

	double sx = sin(pose.om);
	double cx = cos(pose.om);
	double sy = sin(pose.fi);
	double cy = cos(pose.fi);
	double sz = sin(pose.ka);
	double cz = cos(pose.ka);

	m(0,0) = cy * cz;
	m(1,0) = cz * sx * sy + cx * sz;
	m(2,0) = -cx * cz * sy + sx * sz;

	m(0,1) = -cy * sz;
	m(1,1) = cx * cz - sx * sy * sz;
	m(2,1) = cz * sx + cx * sy * sz;

	m(0,2) = sy;
	m(1,2) = -cy * sx;
	m(2,2) = cx * cy;

	m(0,3) = pose.px;
	m(1,3) = pose.py;
	m(2,3) = pose.pz;

	return m;
}