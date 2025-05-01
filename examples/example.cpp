#include <GL/freeglut.h>
#include <iostream>
#include <TinyVersatilePoseGraphSLAM.h>
#include <fstream>

const unsigned int window_width = 800;
const unsigned int window_height = 600;
int mouse_old_x, mouse_old_y;
int mouse_buttons = 0;
float rotate_x = 0.0, rotate_y = 0.0;
float translate_z = -10.0;
float translate_x, translate_y = 0.0;

bool initGL(int *argc, char **argv);
void display();
void keyboard(unsigned char key, int x, int y);
void mouse(int button, int state, int x, int y);
void motion(int x, int y);
void reshape(int w, int h);
void printHelp();

#ifndef MAX
#define MAX(a, b) a > b ? a : b
#endif

#include <random>

static std::random_device rd;
static std::mt19937 gen(rd());

inline double random(double low, double high)
{
    std::uniform_real_distribution<double> dist(low, high);
    return dist(gen);
}

std::vector<Eigen::Affine3d> m_poses;
std::vector<TinyVersatilePoseGraphSLAM::EdgeTaitBryan> tb_edges;
std::vector<TinyVersatilePoseGraphSLAM::EdgeRodrigues> rodrigues_edges;
std::vector<TinyVersatilePoseGraphSLAM::EdgeQuaternion> quaternion_edges;

int main(int argc, char *argv[])
{
    if (argc == 1)
    {

        for (size_t i = 0; i < 100; i++)
        {
            TinyVersatilePoseGraphSLAM::TaitBryanPose p;
            p.px = i;
            p.py = -1;
            p.pz = random(-0.01, 0.01);
            p.om = random(-0.01, 0.01);
            p.fi = random(-0.01, 0.01);
            p.ka = random(-0.01, 0.01);

            Eigen::Affine3d m = TinyVersatilePoseGraphSLAM::affine_matrix_from_pose_tait_bryan(p);
            m_poses.push_back(m);
        }
        for (size_t i = 0; i < 100; i++)
        {
            TinyVersatilePoseGraphSLAM::TaitBryanPose p;
            p.px = i;
            p.py = 1;
            p.pz = random(-0.01, 0.01);
            p.om = random(-0.01, 0.01);
            p.fi = random(-0.01, 0.01);
            p.ka = random(-0.01, 0.01);

            Eigen::Affine3d m = TinyVersatilePoseGraphSLAM::affine_matrix_from_pose_tait_bryan(p);
            m_poses.push_back(m);
        }

        for (size_t i = 1; i < 100; i++)
        {
            TinyVersatilePoseGraphSLAM::EdgeTaitBryan edge;
            edge.index_from = i - 1;
            edge.index_to = i;
            edge.measurement = m_poses[edge.index_from].inverse() * m_poses[edge.index_to];
            edge.uncertainty_covariance_information_matrix_inverse.covariance = Eigen::Matrix<double, 6, 6>::Identity();
            edge.uncertainty_covariance_information_matrix_inverse.covariance(0, 0) = 0.01;
            edge.uncertainty_covariance_information_matrix_inverse.covariance(1, 1) = 0.02;
            edge.uncertainty_covariance_information_matrix_inverse.covariance(2, 2) = 0.03;
            edge.uncertainty_covariance_information_matrix_inverse.covariance(3, 3) = 0.04;
            edge.uncertainty_covariance_information_matrix_inverse.covariance(4, 4) = 0.05;
            edge.uncertainty_covariance_information_matrix_inverse.covariance(5, 5) = 0.06;

            edge.robust_kernel_W.px_robust_kernel_W = 1.0; // no robust kernel function
            edge.robust_kernel_W.py_robust_kernel_W = 1.0; // no robust kernel function
            edge.robust_kernel_W.pz_robust_kernel_W = 1.0; // no robust kernel function
            edge.robust_kernel_W.om_robust_kernel_W = 1.0; // no robust kernel function
            edge.robust_kernel_W.fi_robust_kernel_W = 1.0; // no robust kernel function
            edge.robust_kernel_W.ka_robust_kernel_W = 1.0; // no robust kernel function

            tb_edges.emplace_back(edge);

            TinyVersatilePoseGraphSLAM::EdgeRodrigues r_edge;

            r_edge.index_from = i - 1;
            r_edge.index_to = i;
            r_edge.measurement = m_poses[r_edge.index_from].inverse() * m_poses[r_edge.index_to];

            r_edge.uncertainty_covariance_information_matrix_inverse.covariance =
                TinyVersatilePoseGraphSLAM::rodrigues_covariance_from_tait_bryan_covariance(
                    edge.uncertainty_covariance_information_matrix_inverse.covariance,
                    TinyVersatilePoseGraphSLAM::pose_rodrigues_from_affine_matrix(r_edge.measurement));

            r_edge.robust_kernel_W.px_robust_kernel_W = 1.0; // no robust kernel function
            r_edge.robust_kernel_W.py_robust_kernel_W = 1.0; // no robust kernel function
            r_edge.robust_kernel_W.pz_robust_kernel_W = 1.0; // no robust kernel function
            r_edge.robust_kernel_W.sx_robust_kernel_W = 1.0; // no robust kernel function
            r_edge.robust_kernel_W.sy_robust_kernel_W = 1.0; // no robust kernel function
            r_edge.robust_kernel_W.sz_robust_kernel_W = 1.0; // no robust kernel function

            rodrigues_edges.emplace_back(r_edge);

            TinyVersatilePoseGraphSLAM::EdgeQuaternion q_edge;

            q_edge.index_from = i - 1;
            q_edge.index_to = i;
            q_edge.measurement = m_poses[q_edge.index_from].inverse() * m_poses[q_edge.index_to];

            q_edge.uncertainty_covariance_information_matrix_inverse.covariance =
                TinyVersatilePoseGraphSLAM::quaternion_covariance_from_tait_bryan_covariance(
                    edge.uncertainty_covariance_information_matrix_inverse.covariance,
                    TinyVersatilePoseGraphSLAM::pose_tait_bryan_from_affine_matrix(q_edge.measurement));

            q_edge.robust_kernel_W.px_robust_kernel_W = 1.0; // no robust kernel function
            q_edge.robust_kernel_W.py_robust_kernel_W = 1.0; // no robust kernel function
            q_edge.robust_kernel_W.pz_robust_kernel_W = 1.0; // no robust kernel function
            q_edge.robust_kernel_W.q0_robust_kernel_W = 1.0; // no robust kernel function
            q_edge.robust_kernel_W.q1_robust_kernel_W = 1.0; // no robust kernel function
            q_edge.robust_kernel_W.q2_robust_kernel_W = 1.0; // no robust kernel function
            q_edge.robust_kernel_W.q3_robust_kernel_W = 1.0; // no robust kernel function

            quaternion_edges.emplace_back(q_edge);

            // std::cout << "---------------" << std::endl;
            // std::cout << edge.uncertainty_covariance_information_matrix_inverse.covariance << std::endl;
            // std::cout << r_edge.uncertainty_covariance_information_matrix_inverse.covariance << std::endl;
            // std::cout << q_edge.uncertainty_covariance_information_matrix_inverse.covariance << std::endl;
        }

        for (size_t i = 101; i < 200; i++)
        {
            TinyVersatilePoseGraphSLAM::EdgeTaitBryan edge;
            edge.index_from = i - 1;
            edge.index_to = i;
            edge.measurement = m_poses[edge.index_from].inverse() * m_poses[edge.index_to];

            edge.uncertainty_covariance_information_matrix_inverse.covariance = Eigen::Matrix<double, 6, 6>::Identity();

            edge.robust_kernel_W.px_robust_kernel_W = 1.0; // no robust kernel function
            edge.robust_kernel_W.py_robust_kernel_W = 1.0; // no robust kernel function
            edge.robust_kernel_W.pz_robust_kernel_W = 1.0; // no robust kernel function
            edge.robust_kernel_W.om_robust_kernel_W = 1.0; // no robust kernel function
            edge.robust_kernel_W.fi_robust_kernel_W = 1.0; // no robust kernel function
            edge.robust_kernel_W.ka_robust_kernel_W = 1.0; // no robust kernel function

            tb_edges.emplace_back(edge);

            TinyVersatilePoseGraphSLAM::EdgeRodrigues r_edge;

            r_edge.index_from = i - 1;
            r_edge.index_to = i;
            r_edge.measurement = m_poses[r_edge.index_from].inverse() * m_poses[r_edge.index_to];
            r_edge.uncertainty_covariance_information_matrix_inverse.covariance =
                TinyVersatilePoseGraphSLAM::rodrigues_covariance_from_tait_bryan_covariance(
                    edge.uncertainty_covariance_information_matrix_inverse.covariance,
                    TinyVersatilePoseGraphSLAM::pose_rodrigues_from_affine_matrix(r_edge.measurement));

            r_edge.robust_kernel_W.px_robust_kernel_W = 1.0; // no robust kernel function
            r_edge.robust_kernel_W.py_robust_kernel_W = 1.0; // no robust kernel function
            r_edge.robust_kernel_W.pz_robust_kernel_W = 1.0; // no robust kernel function
            r_edge.robust_kernel_W.sx_robust_kernel_W = 1.0; // no robust kernel function
            r_edge.robust_kernel_W.sy_robust_kernel_W = 1.0; // no robust kernel function
            r_edge.robust_kernel_W.sz_robust_kernel_W = 1.0; // no robust kernel function

            rodrigues_edges.emplace_back(r_edge);

            TinyVersatilePoseGraphSLAM::EdgeQuaternion q_edge;

            q_edge.index_from = i - 1;
            q_edge.index_to = i;
            q_edge.measurement = m_poses[q_edge.index_from].inverse() * m_poses[q_edge.index_to];

            q_edge.uncertainty_covariance_information_matrix_inverse.covariance =
                TinyVersatilePoseGraphSLAM::quaternion_covariance_from_tait_bryan_covariance(
                    edge.uncertainty_covariance_information_matrix_inverse.covariance,
                    TinyVersatilePoseGraphSLAM::pose_tait_bryan_from_affine_matrix(q_edge.measurement));

            q_edge.robust_kernel_W.px_robust_kernel_W = 1.0; // no robust kernel function
            q_edge.robust_kernel_W.py_robust_kernel_W = 1.0; // no robust kernel function
            q_edge.robust_kernel_W.pz_robust_kernel_W = 1.0; // no robust kernel function
            q_edge.robust_kernel_W.q0_robust_kernel_W = 1.0; // no robust kernel function
            q_edge.robust_kernel_W.q1_robust_kernel_W = 1.0; // no robust kernel function
            q_edge.robust_kernel_W.q2_robust_kernel_W = 1.0; // no robust kernel function
            q_edge.robust_kernel_W.q3_robust_kernel_W = 1.0; // no robust kernel function

            quaternion_edges.emplace_back(q_edge);
        }

        for (size_t i = 0; i < 100; i += 10)
        {
            TinyVersatilePoseGraphSLAM::EdgeTaitBryan edge;
            edge.index_from = i;
            edge.index_to = i + 100;
            edge.measurement = m_poses[edge.index_from].inverse() * m_poses[edge.index_to];
            edge.uncertainty_covariance_information_matrix_inverse.covariance = Eigen::Matrix<double, 6, 6>::Identity();

            edge.robust_kernel_W.px_robust_kernel_W = 1.0; // no robust kernel function
            edge.robust_kernel_W.py_robust_kernel_W = 1.0; // no robust kernel function
            edge.robust_kernel_W.pz_robust_kernel_W = 1.0; // no robust kernel function
            edge.robust_kernel_W.om_robust_kernel_W = 1.0; // no robust kernel function
            edge.robust_kernel_W.fi_robust_kernel_W = 1.0; // no robust kernel function
            edge.robust_kernel_W.ka_robust_kernel_W = 1.0; // no robust kernel function

            tb_edges.emplace_back(edge);

            TinyVersatilePoseGraphSLAM::EdgeRodrigues r_edge;

            r_edge.index_from = i;
            r_edge.index_to = i + 100;
            r_edge.measurement = m_poses[r_edge.index_from].inverse() * m_poses[r_edge.index_to];

            r_edge.uncertainty_covariance_information_matrix_inverse.covariance =
                TinyVersatilePoseGraphSLAM::rodrigues_covariance_from_tait_bryan_covariance(
                    edge.uncertainty_covariance_information_matrix_inverse.covariance,
                    TinyVersatilePoseGraphSLAM::pose_rodrigues_from_affine_matrix(r_edge.measurement));

            r_edge.robust_kernel_W.px_robust_kernel_W = 1.0; // no robust kernel function
            r_edge.robust_kernel_W.py_robust_kernel_W = 1.0; // no robust kernel function
            r_edge.robust_kernel_W.pz_robust_kernel_W = 1.0; // no robust kernel function
            r_edge.robust_kernel_W.sx_robust_kernel_W = 1.0; // no robust kernel function
            r_edge.robust_kernel_W.sy_robust_kernel_W = 1.0; // no robust kernel function
            r_edge.robust_kernel_W.sz_robust_kernel_W = 1.0; // no robust kernel function

            rodrigues_edges.emplace_back(r_edge);

            TinyVersatilePoseGraphSLAM::EdgeQuaternion q_edge;

            q_edge.index_from = i;
            q_edge.index_to = i + 100;
            q_edge.measurement = m_poses[q_edge.index_from].inverse() * m_poses[q_edge.index_to];

            q_edge.uncertainty_covariance_information_matrix_inverse.covariance =
                TinyVersatilePoseGraphSLAM::quaternion_covariance_from_tait_bryan_covariance(
                    edge.uncertainty_covariance_information_matrix_inverse.covariance,
                    TinyVersatilePoseGraphSLAM::pose_tait_bryan_from_affine_matrix(q_edge.measurement));

            q_edge.robust_kernel_W.px_robust_kernel_W = 1.0; // no robust kernel function
            q_edge.robust_kernel_W.py_robust_kernel_W = 1.0; // no robust kernel function
            q_edge.robust_kernel_W.pz_robust_kernel_W = 1.0; // no robust kernel function
            q_edge.robust_kernel_W.q0_robust_kernel_W = 1.0; // no robust kernel function
            q_edge.robust_kernel_W.q1_robust_kernel_W = 1.0; // no robust kernel function
            q_edge.robust_kernel_W.q2_robust_kernel_W = 1.0; // no robust kernel function
            q_edge.robust_kernel_W.q3_robust_kernel_W = 1.0; // no robust kernel function

            quaternion_edges.emplace_back(q_edge);
        }
    }else{
        std::ifstream g2o_file(argv[1]);

        std::string line;
        while (std::getline(g2o_file, line))
        {
            std::stringstream line_stream(line);
            std::string class_element;
            line_stream >> class_element;
            if (class_element == "VERTEX_SE3:QUAT")
            {
                int id = 0;
                TinyVersatilePoseGraphSLAM::QuaternionPose p;
                line_stream >> id;
                line_stream >> p.px;
                line_stream >> p.py;
                line_stream >> p.pz;
                line_stream >> p.q1;
                line_stream >> p.q2;
                line_stream >> p.q3;
                line_stream >> p.q0;
                TinyVersatilePoseGraphSLAM::normalize_quaternion(p);

                Eigen::Affine3d m = TinyVersatilePoseGraphSLAM::affine_matrix_from_pose_quaternion(p);
                m_poses.push_back(m);
            }
            else if (class_element == "EDGE_SE3:QUAT")
            {
                int pose_id1, pose_id2;
                line_stream >> pose_id1;
                line_stream >> pose_id2;
                TinyVersatilePoseGraphSLAM::QuaternionPose p;
                line_stream >> p.px;
                line_stream >> p.py;
                line_stream >> p.pz;
                line_stream >> p.q1;
                line_stream >> p.q2;
                line_stream >> p.q3;
                line_stream >> p.q0;
                Eigen::Affine3d measurement = TinyVersatilePoseGraphSLAM::affine_matrix_from_pose_quaternion(p);

                //edges_g2o.emplace_back(std::make_tuple(pose_id1, pose_id2, m));



                std::vector<double> w(21);
                for (size_t i = 0; i < 21; i++)
                {
                    line_stream >> w[i];
                }
                std::vector<double> ww(6);
                ww[0] = w[0];
                ww[1] = w[6];
                ww[2] = w[11];
                ww[3] = w[15];
                ww[4] = w[18];
                ww[5] = w[20];
                //edges_g2o_w.push_back(ww);

                ////////////////////////////////////////////////////////
                TinyVersatilePoseGraphSLAM::EdgeTaitBryan edge;
                edge.index_from = pose_id1;
                edge.index_to = pose_id2;
                edge.measurement = measurement;
                edge.uncertainty_covariance_information_matrix_inverse.covariance = Eigen::Matrix<double, 6, 6>::Identity();
                edge.uncertainty_covariance_information_matrix_inverse.covariance(0, 0) = 1.0 / sqrt(ww[0]);
                edge.uncertainty_covariance_information_matrix_inverse.covariance(1, 1) = 1.0 / sqrt(ww[1]);
                edge.uncertainty_covariance_information_matrix_inverse.covariance(2, 2) = 1.0 / sqrt(ww[2]);
                edge.uncertainty_covariance_information_matrix_inverse.covariance(3, 3) = 1.0 / sqrt(ww[3]);
                edge.uncertainty_covariance_information_matrix_inverse.covariance(4, 4) = 1.0 / sqrt(ww[4]);
                edge.uncertainty_covariance_information_matrix_inverse.covariance(5, 5) = 1.0 / sqrt(ww[5]);

                edge.robust_kernel_W.px_robust_kernel_W = 1.0; // no robust kernel function
                edge.robust_kernel_W.py_robust_kernel_W = 1.0; // no robust kernel function
                edge.robust_kernel_W.pz_robust_kernel_W = 1.0; // no robust kernel function
                edge.robust_kernel_W.om_robust_kernel_W = 1.0; // no robust kernel function
                edge.robust_kernel_W.fi_robust_kernel_W = 1.0; // no robust kernel function
                edge.robust_kernel_W.ka_robust_kernel_W = 1.0; // no robust kernel function

                tb_edges.emplace_back(edge);

                TinyVersatilePoseGraphSLAM::EdgeRodrigues r_edge;

                r_edge.index_from = pose_id1;
                r_edge.index_to = pose_id2;
                r_edge.measurement = measurement;

                r_edge.uncertainty_covariance_information_matrix_inverse.covariance =
                    TinyVersatilePoseGraphSLAM::rodrigues_covariance_from_tait_bryan_covariance(
                        edge.uncertainty_covariance_information_matrix_inverse.covariance,
                        TinyVersatilePoseGraphSLAM::pose_rodrigues_from_affine_matrix(r_edge.measurement));

                r_edge.robust_kernel_W.px_robust_kernel_W = 1.0; // no robust kernel function
                r_edge.robust_kernel_W.py_robust_kernel_W = 1.0; // no robust kernel function
                r_edge.robust_kernel_W.pz_robust_kernel_W = 1.0; // no robust kernel function
                r_edge.robust_kernel_W.sx_robust_kernel_W = 1.0; // no robust kernel function
                r_edge.robust_kernel_W.sy_robust_kernel_W = 1.0; // no robust kernel function
                r_edge.robust_kernel_W.sz_robust_kernel_W = 1.0; // no robust kernel function

                rodrigues_edges.emplace_back(r_edge);

                TinyVersatilePoseGraphSLAM::EdgeQuaternion q_edge;

                q_edge.index_from = pose_id1;
                q_edge.index_to = pose_id2;
                q_edge.measurement = measurement;

                q_edge.uncertainty_covariance_information_matrix_inverse.covariance =
                    TinyVersatilePoseGraphSLAM::quaternion_covariance_from_tait_bryan_covariance(
                        edge.uncertainty_covariance_information_matrix_inverse.covariance,
                        TinyVersatilePoseGraphSLAM::pose_tait_bryan_from_affine_matrix(q_edge.measurement));

                q_edge.robust_kernel_W.px_robust_kernel_W = 1.0; // no robust kernel function
                q_edge.robust_kernel_W.py_robust_kernel_W = 1.0; // no robust kernel function
                q_edge.robust_kernel_W.pz_robust_kernel_W = 1.0; // no robust kernel function
                q_edge.robust_kernel_W.q0_robust_kernel_W = 1.0; // no robust kernel function
                q_edge.robust_kernel_W.q1_robust_kernel_W = 1.0; // no robust kernel function
                q_edge.robust_kernel_W.q2_robust_kernel_W = 1.0; // no robust kernel function
                q_edge.robust_kernel_W.q3_robust_kernel_W = 1.0; // no robust kernel function

                quaternion_edges.emplace_back(q_edge);
            }
        }
        g2o_file.close();
    }

    if (false == initGL(&argc, argv))
    {
        return 4;
    }
    printHelp();
    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutMainLoop();
    return 0;
}

bool initGL(int *argc, char **argv)
{
    glutInit(argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
    glutInitWindowSize(window_width, window_height);
    glutCreateWindow("example");
    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);
    glutMotionFunc(motion);

    // default initialization
    glClearColor(1.0, 1.0, 1.0, 1.0);
    glEnable(GL_DEPTH_TEST);

    // viewport
    glViewport(0, 0, window_width, window_height);

    // projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (GLfloat)window_width / (GLfloat)window_height, 0.01,
                   10000.0);
    glutReshapeFunc(reshape);
    return true;
}

void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(translate_x, translate_y, translate_z);
    glRotatef(rotate_x, 1.0, 0.0, 0.0);
    glRotatef(rotate_y, 0.0, 0.0, 1.0);

    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(1.0f, 0.0f, 0.0f);

    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 1.0f, 0.0f);

    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 1.0f);
    glEnd();

    glColor3f(1, 0, 0);
    glBegin(GL_LINES);
    for (size_t i = 0; i < tb_edges.size(); i++)
    {
        glVertex3f(m_poses[tb_edges[i].index_from](0, 3), m_poses[tb_edges[i].index_from](1, 3), m_poses[tb_edges[i].index_from](2, 3));
        glVertex3f(m_poses[tb_edges[i].index_to](0, 3), m_poses[tb_edges[i].index_to](1, 3), m_poses[tb_edges[i].index_to](2, 3));
    }
    glEnd();

    glutSwapBuffers();
}

void keyboard(unsigned char key, int /*x*/, int /*y*/)
{
    switch (key)
    {
    case (27):
    {
        glutDestroyWindow(glutGetWindow());
        return;
    }
    case 'n':
    {
        for (size_t i = 0; i < m_poses.size(); i++)
        {
            TinyVersatilePoseGraphSLAM::TaitBryanPose tb_pose = TinyVersatilePoseGraphSLAM::pose_tait_bryan_from_affine_matrix(m_poses[i]);
            tb_pose.px += random(-0.1, 0.1);
            tb_pose.py += random(-0.1, 0.1);
            tb_pose.pz += random(-0.1, 0.1);
            tb_pose.om += random(-0.01, 0.01);
            tb_pose.fi += random(-0.01, 0.01);
            tb_pose.ka += random(-0.01, 0.01);
            m_poses[i] = TinyVersatilePoseGraphSLAM::affine_matrix_from_pose_tait_bryan(tb_pose);
        }
        break;
        break;
    }
    case 't':
    {
        std::pair<Eigen::SparseMatrix<double>, Eigen::SparseMatrix<double>> AtPA_AtPB = TinyVersatilePoseGraphSLAM::get_AtPA_AtPB_pose_graph_tait_byan_wc(m_poses, tb_edges);

        for (int row = 0; row < 6; row++)
        {
            AtPA_AtPB.first.coeffRef(row, row) += 1;
        }

        std::cout << "start solving AtPA=AtPB" << std::endl;
        Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA_AtPB.first);

        std::cout << "x = solver.solve(AtPB)" << std::endl;
        Eigen::SparseMatrix<double> x = solver.solve(AtPA_AtPB.second);

        double update = TinyVersatilePoseGraphSLAM::apply_result_tait_bryan_wc(x, m_poses);

        std::cout << "update: " << update << std::endl;

        break;
    }
    case 'r':
    {
        std::pair<Eigen::SparseMatrix<double>, Eigen::SparseMatrix<double>> AtPA_AtPB = TinyVersatilePoseGraphSLAM::get_AtPA_AtPB_pose_graph_rodrigues_wc(m_poses, rodrigues_edges);

        for (int row = 0; row < 6; row++)
        {
            AtPA_AtPB.first.coeffRef(row, row) += 1;
        }

        std::cout << "start solving AtPA=AtPB" << std::endl;
        Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA_AtPB.first);

        std::cout << "x = solver.solve(AtPB)" << std::endl;
        Eigen::SparseMatrix<double> x = solver.solve(AtPA_AtPB.second);

        double update = TinyVersatilePoseGraphSLAM::apply_result_rodrigues_wc(x, m_poses);

        std::cout << "update: " << update << std::endl;

        break;
    }
    case 'q':
    {
        std::pair<Eigen::SparseMatrix<double>, Eigen::SparseMatrix<double>> AtPA_AtPB = TinyVersatilePoseGraphSLAM::get_AtPA_AtPB_pose_graph_quaternion_wc(m_poses, quaternion_edges);

        for (int row = 0; row < 7; row++)
        {
            AtPA_AtPB.first.coeffRef(row, row) += 1;
        }

        std::cout << "start solving AtPA=AtPB" << std::endl;
        Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA_AtPB.first);

        std::cout << "x = solver.solve(AtPB)" << std::endl;
        Eigen::SparseMatrix<double> x = solver.solve(AtPA_AtPB.second);

        double update = TinyVersatilePoseGraphSLAM::apply_result_quaternion_wc(x, m_poses);

        std::cout << "update: " << update << std::endl;

        break;
    }
    case 'u':
    {
        TinyVersatilePoseGraphSLAM::TaitBryanPose pose_tb;
        pose_tb.px = 10.01;
        pose_tb.py = 1000.02;
        pose_tb.pz = 110.03;
        pose_tb.om = 3.0 / 180.0 * M_PI;
        pose_tb.fi = 3.0 / 180.0 * M_PI;
        pose_tb.ka = 90.0 / 180.0 * M_PI;

        Eigen::Matrix<double, 6, 6> cov_tb;
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                cov_tb(i, j) = 0.0;
            }
        }
        cov_tb(0, 0) = 0.01;
        cov_tb(1, 1) = 0.02;
        cov_tb(2, 2) = 0.03;
        cov_tb(3, 3) = pow(3.0 / 180.0 * M_PI, 2);
        cov_tb(4, 4) = pow(4.0 / 180.0 * M_PI, 2);
        cov_tb(5, 5) = pow(5.0 / 180.0 * M_PI, 2);

        std::cout << "cov_tb: " << std::endl
                  << cov_tb << std::endl;

        // Eigen::Matrix<double, 7, 7> cov_q = TinyVersatilePoseGraphSLAM::quaternion_covariance_from_tait_bryan_covariance(cov_tb, pose_tb);

        Eigen::Matrix<double, 7, 6> j;
        TinyVersatilePoseGraphSLAM::uncertainty_pose_tait_bryan_to_quaternion(j, pose_tb.px, pose_tb.py, pose_tb.pz, pose_tb.om, pose_tb.fi, pose_tb.ka);

        Eigen::Matrix<double, 7, 7> cov_q = j * cov_tb * j.transpose();
        std::cout << "-------------------" << std::endl;
        std::cout << cov_q << std::endl;
        std::cout << cov_q.inverse() << std::endl;
        std::cout << cov_q * cov_q.inverse() << std::endl;
        std::cout << cov_q.inverse() * cov_q << std::endl;
        std::cout << "-------------------" << std::endl;

        // static void uncertainty_pose_quaternion_to_tait_bryan(Eigen::Matrix<double, 6, 7> & j, const double &tx_tb, const double &ty_tb, const double &tz_tb, const double &q0_quaternion, const double &q1_quaternion, const double &q2_quaternion, const double &q3_quaternion);
        Eigen::Affine3d m = TinyVersatilePoseGraphSLAM::affine_matrix_from_pose_tait_bryan(pose_tb);
        TinyVersatilePoseGraphSLAM::QuaternionPose pose_q =
            TinyVersatilePoseGraphSLAM::pose_quaternion_from_affine_matrix(m);

        Eigen::Matrix<double, 6, 7> j_q;
        TinyVersatilePoseGraphSLAM::uncertainty_pose_quaternion_to_tait_bryan(j_q, pose_q.px, pose_q.py, pose_q.pz, pose_q.q0, pose_q.q1, pose_q.q2, pose_q.q3);

        Eigen::Matrix<double, 6, 6> cov_tb2 = j_q * cov_q * j_q.transpose();
        std::cout << cov_tb2 << std::endl;

        Eigen::Matrix<double, 4, 4> m1;
        m1(0, 0) = 0.000954537;
        m1(0, 1) = -4.97519e-05;
        m1(0, 2) = 3.58705e-05;
        m1(0, 3) = -0.000950625;
        m1(1, 0) = -4.97519e-05;
        m1(1, 1) = 0.000949322;
        m1(1, 2) = 0.000266175;
        m1(1, 3) = -2.03157e-22;
        m1(2, 0) = 3.58705e-05;
        m1(2, 1) = 0.000266175;
        m1(2, 2) = 0.000954537;
        m1(2, 3) = -4.97519e-05;
        m1(3, 0) = -0.000950625;
        m1(3, 1) = -2.03157e-22;
        m1(3, 2) = -4.97519e-05;
        m1(3, 3) = 0.000949322;

        Eigen::Matrix<double, 4, 4> m2 = m1.inverse();

        std::cout << "==============" << std::endl;
        std::cout << m1 << std::endl;
        std::cout << m2 << std::endl;
        std::cout << m1 * m2 << std::endl;
        std::cout << m2 * m1 << std::endl;

        /*Eigen::Affine3d m = TinyVersatilePoseGraphSLAM::affine_matrix_from_pose_tait_bryan(pose_tb);
        TinyVersatilePoseGraphSLAM::RodriguesPose pose_r =
            TinyVersatilePoseGraphSLAM::pose_rodrigues_from_affine_matrix(m);

        Eigen::Matrix<double, 6, 6>
            cov_rodrigues = TinyVersatilePoseGraphSLAM::rodrigues_covariance_from_tait_bryan_covariance(cov_tb, pose_r);

        std::cout
            << "cov_rodrigues: " << std::endl
            << cov_rodrigues << std::endl;

        Eigen::Matrix<double, 6, 6> j_tb;
        TinyVersatilePoseGraphSLAM::uncertainty_pose_rodrigues_to_tait_bryan(j_tb, pose_tb.px, pose_tb.py, pose_tb.pz, pose_tb.om, pose_tb.fi, pose_tb.ka);

        cov_tb = j_tb * cov_rodrigues * j_tb.transpose();

        std::cout << "cov_tb: " << std::endl
                    << cov_tb << std::endl;*/
        /*






            //Eigen::Matrix<double, 6, 6>
            //    j_r;
            //TinyVersatilePoseGraphSLAM::uncertainty_pose_tait_bryan_to_rodrigues(j_r, pose_r.px, pose_r.py, pose_r.pz, pose_r.sx, pose_r.sy, pose_r.sz);

            //std::cout << "j_r: " << std::endl
            //          << j_r << std::endl;




            //Eigen::Affine3d m_pose = TinyVersatilePoseGraphSLAM::affine_matrix_from_pose_tait_bryan(pose_tb);
            //TinyVersatilePoseGraphSLAM::RodriguesPose r_pose = TinyVersatilePoseGraphSLAM::pose_rodrigues_from_affine_matrix(m_pose);


    */
        break;
    }
    }
    printHelp();
    glutPostRedisplay();
}

void mouse(int button, int state, int x, int y)
{
    if (state == GLUT_DOWN)
    {
        mouse_buttons |= 1 << button;
    }
    else if (state == GLUT_UP)
    {
        mouse_buttons = 0;
    }
    mouse_old_x = x;
    mouse_old_y = y;
}

void motion(int x, int y)
{
    float dx, dy;
    dx = (float)(x - mouse_old_x);
    dy = (float)(y - mouse_old_y);

    if (mouse_buttons & 1)
    {
        rotate_x += dy * 0.2f;
        rotate_y += dx * 0.2f;
    }
    else if (mouse_buttons & 4)
    {
        translate_z += dy * 0.05f;
    }
    else if (mouse_buttons & 3)
    {
        translate_x += dx * 0.05f;
        translate_y -= dy * 0.05f;
    }

    mouse_old_x = x;
    mouse_old_y = y;

    glutPostRedisplay();
}

void reshape(int w, int h)
{
    glViewport(0, 0, (GLsizei)w, (GLsizei)h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (GLfloat)w / (GLfloat)h, 0.01, 10000.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void printHelp()
{
    std::cout << "-------help-------" << std::endl;
    std::cout << "n: add noise to poses" << std::endl;
    std::cout << "t: optimize single iteration Gauss-Newton (rotation agle reprezentation Tait-Bryan)" << std::endl;
    std::cout << "r: optimize single iteration Gauss-Newton (rotation agle reprezentation Rodrigues)" << std::endl;
    std::cout << "q: optimize single iteration Gauss-Newton (rotation agle reprezentation Quaternion)" << std::endl;
}

// git submodule add https://gitlab.com/libeigen/eigen.git eigen
// git submodule add https://github.com/freeglut/freeglut.git freeglut