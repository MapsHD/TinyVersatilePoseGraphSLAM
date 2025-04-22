#include <GL/freeglut.h>
#include <iostream>
#include <TinyVersatilePoseGraphSLAM.h>

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
#define MAX(a,b) a>b?a:b
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

int main(int argc, char *argv[]){
    for (size_t i = 0; i < 100; i++)
	{
		TinyVersatilePoseGraphSLAM::TaitBryanPose p;
		p.px = i;
		p.py = -1;
		p.pz = 0.0;
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
		p.pz = 0.0;
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
        edge.uncertainty_covariance_information_matrix_inverse.px_1_sigma_m = 0.001; //1mm
        edge.uncertainty_covariance_information_matrix_inverse.py_1_sigma_m = 0.001; //1mm
        edge.uncertainty_covariance_information_matrix_inverse.pz_1_sigma_m = 0.001; //1mm
        edge.uncertainty_covariance_information_matrix_inverse.om_1_sigma_deg = 1.0; // 1degree
        edge.uncertainty_covariance_information_matrix_inverse.fi_1_sigma_deg = 1.0; // 1degree
        edge.uncertainty_covariance_information_matrix_inverse.ka_1_sigma_deg = 1.0; // 1degree

        tb_edges.emplace_back(edge);
	}

	for (size_t i = 101; i < 200; i++)
	{
		TinyVersatilePoseGraphSLAM::EdgeTaitBryan edge;
        edge.index_from = i - 1;
        edge.index_to = i;
        edge.measurement = m_poses[edge.index_from].inverse() * m_poses[edge.index_to];
        edge.uncertainty_covariance_information_matrix_inverse.px_1_sigma_m = 0.001; //1mm
        edge.uncertainty_covariance_information_matrix_inverse.py_1_sigma_m = 0.001; //1mm
        edge.uncertainty_covariance_information_matrix_inverse.pz_1_sigma_m = 0.001; //1mm
        edge.uncertainty_covariance_information_matrix_inverse.om_1_sigma_deg = 1.0; // 1degree
        edge.uncertainty_covariance_information_matrix_inverse.fi_1_sigma_deg = 1.0; // 1degree
        edge.uncertainty_covariance_information_matrix_inverse.ka_1_sigma_deg = 1.0; // 1degree

        tb_edges.emplace_back(edge);
	}

	for (size_t i = 0; i < 100; i += 10)
	{
        TinyVersatilePoseGraphSLAM::EdgeTaitBryan edge;
        edge.index_from = i;
        edge.index_to = i + 100;
        edge.measurement = m_poses[edge.index_from].inverse() * m_poses[edge.index_to];
        edge.uncertainty_covariance_information_matrix_inverse.px_1_sigma_m = 0.001; //1mm
        edge.uncertainty_covariance_information_matrix_inverse.py_1_sigma_m = 0.001; //1mm
        edge.uncertainty_covariance_information_matrix_inverse.pz_1_sigma_m = 0.001; //1mm
        edge.uncertainty_covariance_information_matrix_inverse.om_1_sigma_deg = 1.0; // 1degree
        edge.uncertainty_covariance_information_matrix_inverse.fi_1_sigma_deg = 1.0; // 1degree
        edge.uncertainty_covariance_information_matrix_inverse.ka_1_sigma_deg = 1.0; // 1degree

        tb_edges.emplace_back(edge);
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
    std::cout << "t: optimize (Tait-Bryan)" << std::endl;
}

// git submodule add https://gitlab.com/libeigen/eigen.git eigen
// git submodule add https://github.com/freeglut/freeglut.git freeglut