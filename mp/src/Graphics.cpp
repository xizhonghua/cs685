#include "Graphics.hpp"
#include <algorithm>

#ifdef __APPLE__
#include <GLUT/glut.h>
#elif defined _WIN32
#include "glutForWindows.h"
#else
#include <GL/glut.h>
#endif

Graphics *m_graphics = NULL;
string filename;
int seed = (1<<30);
int max_nodes = 1<<20;
bool predict;
bool constraint = true;
bool use_best_control = false;


Graphics::Graphics(const char fname[], int method)
{
    m_simulator.SetupFromFile(fname);
    m_planner = new MotionPlanner(&m_simulator, use_best_control, predict, constraint);

    m_selectedCircle = -1;
    m_editRadius     = false;
    m_run            = false;
    m_onstep		 = false;

    m_pathPos = 0;
    m_subPathPos = 0;

    m_drawPlannerVertices = true;

    m_method = method;
}

Graphics::~Graphics(void)
{
    if(m_planner)
	delete m_planner;
}

void Graphics::MainLoop(void)
{	
    m_graphics = this;

//create window    
    static int    argc = 1;	
    static char  *args = (char*)"args";
    glutInit(&argc, &args);    
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);    
    glutInitWindowSize(1000, 600);
    glutInitWindowPosition(0, 0); 
    glutCreateWindow("Planner");	   	


//register callback functions
    glutDisplayFunc(CallbackEventOnDisplay);
    glutMouseFunc(CallbackEventOnMouse);
    glutMotionFunc(CallbackEventOnMouseMotion);
    glutIdleFunc(NULL);
    glutTimerFunc(1, CallbackEventOnTimer, 0); 
    glutKeyboardFunc(CallbackEventOnKeyPress);

//enter main event loop
    glutMainLoop();	
}

void Graphics::HandleEventOnTimer(void)
{
    if(m_run && m_method >= 1 && m_method <= 4)
    {
	for(int i = 0; i < 1000 && !m_planner->IsProblemSolved(); ++i)
	{
	    if(m_method == 1)      m_planner->ExtendRandom();
	    else if(m_method == 2) m_planner->ExtendRRT();
	    else if(m_method == 3) m_planner->findBestControls();
	    else if(m_method == 4) m_planner->ExtendUnitCircle();
	    if(m_onstep)
	    {
	    	m_onstep = false;
	    	m_run = false;
	    	break;
	    }

	    if(m_planner->m_vertices.size()>max_nodes)
	    {
	    	m_run = false;
	    	break;
	    }
	}
	if(!m_planner->IsProblemSolved())
	    fprintf(stderr, "TotalSolveTime = %f [Solved = %d] [NrVertices = %d]\n",
		   m_planner->m_totalSolveTime, m_planner->IsProblemSolved(), m_planner->m_vertices.size());
    }
     
    if(m_path.size() == 0 && m_planner->IsProblemSolved())
    {
		m_pathPos = 0;
		m_subPathPos = 0;
		m_planner->GetPathFromInitToGoal(&m_path);

		fprintf(stderr, "TotalSolveTime = %f [Solved = %d] [NrVertices = %d]\n",
			   m_planner->m_totalSolveTime, m_planner->IsProblemSolved(), m_planner->m_vertices.size());

		auto total_length = 0.0;
		auto total_time = 0.0;
	
		for(auto vid : m_path)
		{
			const auto v = this->m_planner->m_vertices[vid];
			total_time += v->m_path_time;
			total_length += v->m_path_length;
		}
	
		cout<<"path found time = "<<total_time<<" length = "<<total_length<<" avg = "<<total_length/total_time<<"m/s"<<endl;
    }
    
    if(m_path.size() != 0)
    {
    	if(m_pathPos >= m_path.size())
	    m_pathPos = 0;
	
    	auto v = m_planner->m_vertices[m_path[m_pathPos]];

    	if(m_subPathPos < v->m_path.size())
    	{
    		m_simulator.SetRobotState(v->m_path[m_subPathPos]);
    		m_subPathPos++;
    	}
    	else
    	{
    		++m_pathPos;
    		m_subPathPos=0;
    	}
    }
} 

void Graphics::HandleEventOnMouseMotion(const double mousePosX, const double mousePosY)
{
    if(m_planner->m_vertices.size() > 1)
	return;
    

    if(m_selectedCircle >= 0)
    {
	if(m_editRadius)
	{
	    const double cx = m_simulator.m_circles[3 * m_selectedCircle];
	    const double cy = m_simulator.m_circles[3 * m_selectedCircle + 1];
	    
	    m_simulator.m_circles[3 * m_selectedCircle + 2] = sqrt((cx - mousePosX) * (cx - mousePosX) +
								   (cy - mousePosY) * (cy - mousePosY));
	}
	else
	{
	    m_simulator.m_circles[3 * m_selectedCircle] = mousePosX;
	    m_simulator.m_circles[3 * m_selectedCircle + 1] = mousePosY;
	}
	
    }
    
}

void Graphics::HandleEventOnMouseBtnDown(const int whichBtn, const double mousePosX, const double mousePosY)
{   
    if(m_planner->m_vertices.size() > 1)
	return;
 
    m_selectedCircle = -1;
    for(int i = 0; i < m_simulator.m_circles.size() && m_selectedCircle == -1; i += 3)
    {
	const double cx = m_simulator.m_circles[i];
	const double cy = m_simulator.m_circles[i + 1];
	const double r  = m_simulator.m_circles[i + 2];
	const double d  = sqrt((mousePosX - cx) * (mousePosX - cx) + (mousePosY - cy) * (mousePosY - cy));
	
	if(d <= r)
	    m_selectedCircle = i / 3;
    }
    
    if(m_selectedCircle == -1)
    {
	m_simulator.m_circles.push_back(mousePosX);
	m_simulator.m_circles.push_back(mousePosY);
	m_simulator.m_circles.push_back(1.0);
    }    
}

void Graphics::HandleEventOnKeyPress(const int key)
{
    FILE *out;
    
    switch(key)
    {
    case 27: //escape key
	exit(0);
	
    case 'r':
		m_editRadius = !m_editRadius;
		break;
	
    case 'p':
		m_run = !m_run;
		printf("ALLOW RUNNING = %d\n", m_run);
		break;

    case 'n':
    	m_run = true;
    	m_onstep = true;
    	break;


    case 'v':
		m_drawPlannerVertices = !m_drawPlannerVertices;
		break;

    case '1': case '2': case '3': case '4':
	m_run    = true;	
	m_method = key - '0';
	break;
	
    case 's':
	out = fopen("obst.txt", "w");
	fprintf(out, "%d\n", m_simulator.GetNrObstacles());	
	for(int i = 0; i < m_simulator.GetNrObstacles(); ++i)
	    fprintf(out, "%f %f %f\n", 
		    m_simulator.GetObstacleCenterX(i),
		    m_simulator.GetObstacleCenterY(i),
		    m_simulator.GetObstacleRadius(i));
	fclose(out);

    case 'o':
    	auto output_filename = "data/" + filename + (predict?"_p":"") + (!constraint?"_nc":"")  + ".traj";
    	m_planner->ExportPath(output_filename);
    	cerr<<"traj exported to "<<output_filename<<endl;
    	break;
    }
}

void Graphics::HandleEventOnDisplay(void)
{
//draw bounding box
    const double *bbox = m_graphics->m_simulator.m_bbox;
    
    glColor3f(0, 0, 1);    
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glBegin(GL_POLYGON);
    glVertex2d(bbox[0], bbox[1]);
    glVertex2d(bbox[2], bbox[1]);
    glVertex2d(bbox[2], bbox[3]);
    glVertex2d(bbox[0], bbox[3]);
    glEnd();
    
//draw robot, goal, and obstacles
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);



    glColor3f(0, 1, 0);
    DrawCircle2D(m_simulator.GetGoalCenterX(), m_simulator.GetGoalCenterY(), m_simulator.GetGoalRadius());
    glColor3f(0, 0, 1);
    for(int i = 0; i < m_simulator.GetNrObstacles(); ++i)
	DrawCircle2D(m_simulator.GetObstacleCenterX(i),
		     m_simulator.GetObstacleCenterY(i),
		     m_simulator.GetObstacleRadius(i));

//draw planner vertices
    if(m_drawPlannerVertices)
    {
		glPointSize(4.0);

		const int n = m_planner->m_vertices.size();
		glColor3f(0.6, 0.8, 0.3);

		glBegin(GL_POINTS);
		for(int i = 0; i < n; ++i)
			glVertex2d(m_planner->m_vertices[i]->m_state.x, m_planner->m_vertices[i]->m_state.y);
		glEnd();

	
		glLineWidth (1.0);
		// draw trajectories
		for(int i = 1; i < n; ++i)
			this->DrawTrajectory(i);
    }

    glColor3f(1.0f, 0, 0);
	glLineWidth (2.0);
	// draw path
	for(int i=1;i<m_path.size();++i)
	{
		this->DrawTrajectory(m_path[i], 1.0);
	}

	glTranslated(0,0,1);
	for(auto vid : m_path)
	{
		auto v = this->m_planner->m_vertices[vid];
		this->DrawCircle2D(v->m_state.x, v->m_state.y, 0.25);
	}
	glTranslated(0,0,-1);

    // draw robot
    glColor3f(1, 0, 1);
    DrawRobot(m_simulator.GetRobotState());
}


void Graphics::DrawCircle2D(const double cx, const double cy, const double r)
{
    const int    nsides = 50;    
    const double angle  = 2 * M_PI / nsides;
    
    glBegin(GL_POLYGON);
    for(int i = 0; i <= nsides; i++)
	glVertex2d(cx + r * cos(i * angle), cy + r * sin(i * angle));
    glEnd();	
}


#define RADPERDEG 0.0174533

void Arrow(GLdouble x1,GLdouble y1,GLdouble z1,GLdouble x2,GLdouble y2,GLdouble z2,GLdouble D)
{
  double x=x2-x1;
  double y=y2-y1;
  double z=z2-z1;
  double L=sqrt(x*x+y*y+z*z);

    GLUquadricObj *quadObj;

    glPushMatrix ();

      glTranslated(x1,y1,z1);

      if((x!=0.)||(y!=0.)) {
        glRotated(atan2(y,x)/RADPERDEG,0.,0.,1.);
        glRotated(atan2(sqrt(x*x+y*y),z)/RADPERDEG,0.,1.,0.);
      } else if (z<0){
        glRotated(180,1.,0.,0.);
      }

      glTranslatef(0,0,L-4*D);

      quadObj = gluNewQuadric ();
      gluQuadricDrawStyle (quadObj, GLU_FILL);
      gluQuadricNormals (quadObj, GLU_SMOOTH);
      gluCylinder(quadObj, 2*D, 0.0, 4*D, 32, 1);
      gluDeleteQuadric(quadObj);

      quadObj = gluNewQuadric ();
      gluQuadricDrawStyle (quadObj, GLU_FILL);
      gluQuadricNormals (quadObj, GLU_SMOOTH);
      gluDisk(quadObj, 0.0, 2*D, 32, 1);
      gluDeleteQuadric(quadObj);

      glTranslatef(0,0,-L+4*D);

      quadObj = gluNewQuadric ();
      gluQuadricDrawStyle (quadObj, GLU_FILL);
      gluQuadricNormals (quadObj, GLU_SMOOTH);
      gluCylinder(quadObj, D, D, L-4*D, 32, 1);
      gluDeleteQuadric(quadObj);

      quadObj = gluNewQuadric ();
      gluQuadricDrawStyle (quadObj, GLU_FILL);
      gluQuadricNormals (quadObj, GLU_SMOOTH);
      gluDisk(quadObj, 0.0, D, 32, 1);
      gluDeleteQuadric(quadObj);

    glPopMatrix ();

}

void Graphics::DrawRobot(const State& state)
{
	static const auto L = 2.0;
	Arrow(state.x, state.y, 2.0, state.x+cos(state.theta)*L, state.y+sin(state.theta)*L, 2.0, 0.6);
}

void Graphics::DrawTrajectory(const int childVid, const double z)
{
	auto v = m_planner->m_vertices[childVid];
	auto p = m_planner->m_vertices[v->m_parent];

	glBegin(GL_LINE_STRIP);

	auto state = p->m_state;

	glVertex3d(state.x, state.y, z);

	for(const auto& s : v->m_path)
		glVertex3d(s.x, s.y, z);

	glEnd();
}

void Graphics::CallbackEventOnDisplay(void)
{
    if(m_graphics)
    {
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
	glClearDepth(1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);	
	
	glViewport(0, 0, glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));
	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	const double *bbox = m_graphics->m_simulator.m_bbox;
	

	glOrtho(bbox[0] - 1, bbox[2] + 1, bbox[1] - 1, bbox[3] + 1, -1.0, 1.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();	    
	
	m_graphics->HandleEventOnDisplay();
	
	glutSwapBuffers();	    
    }
}

void Graphics::CallbackEventOnMouse(int button, int state, int x, int y)
{
    if(m_graphics &&  state == GLUT_DOWN)
    {
	double mouseX, mouseY;
	MousePosition(x, y, &mouseX, &mouseY);
	m_graphics->HandleEventOnMouseBtnDown(button, mouseX , mouseY);
	glutPostRedisplay();
    }	    
}

void Graphics::CallbackEventOnMouseMotion(int x, int y)
{
    double mouseX, mouseY;
    MousePosition(x, y, &mouseX, &mouseY);
    m_graphics->HandleEventOnMouseMotion(mouseX , mouseY);
    glutPostRedisplay();
}


void Graphics::CallbackEventOnTimer(int id)
{
    if(m_graphics)
    {
	m_graphics->HandleEventOnTimer();
	glutTimerFunc(15, CallbackEventOnTimer, id);
	glutPostRedisplay();	    
    }
}



void Graphics::CallbackEventOnKeyPress(unsigned char key, int x, int y)
{
    if(m_graphics)
	m_graphics->HandleEventOnKeyPress(key);	
}


void Graphics::MousePosition(const int x, const int y, double *posX, double *posY)
{
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    GLfloat winX, winY, winZ;
    GLdouble posZ;
    
    glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
    glGetDoublev( GL_PROJECTION_MATRIX, projection );
    glGetIntegerv( GL_VIEWPORT, viewport );
    
    winX = (float)x;
    winY = (float)viewport[3] - (float)y;
    glReadPixels( x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );
    
    gluUnProject(winX, winY, winZ, modelview, projection, viewport, posX, posY, &posZ);
}

void RunExp(string filename, int method)
{
	Simulator s;
	s.SetupFromFile(filename.c_str());
	MotionPlanner* p;
	p = new MotionPlanner(&s, use_best_control, predict, constraint);

	double last_solve_time = 0.0;

	while( p->GettotalSolveTime() < 30 && !p->IsProblemSolved())
	{
		if(method == 1)      p->ExtendRandom();
		else if(method == 2) p->ExtendRRT();
		else if(method == 3) p->findBestControls();
		else if(method == 4) p->ExtendUnitCircle();

		if(p->GettotalSolveTime() - last_solve_time > 0.5)
		{
			last_solve_time = p->GettotalSolveTime();

			fprintf(stderr, "TotalSolveTime = %f [ Solved = %d ] [ NrVertices = %d ]\n",
					   p->GettotalSolveTime(), p->IsProblemSolved(), p->GetTotalVertices());
		}
	}

	if(p->IsProblemSolved())
	{
		auto total_length = 0.0;
		auto total_time = 0.0;

		auto path = vector<int>();

		p->GetPathFromInitToGoal(&path);

		for(auto vid : path)
		{
			const auto v = p->GetVertex(vid);
			total_time += v->m_path_time;
			total_length += v->m_path_length;
		}

		printf("TotalSolveTime = %f [ Solved = %d ] [ NrVertices = %d ] [Path time = %f ] [Path length = %f ]\n",
			   p->GettotalSolveTime(), p->IsProblemSolved(), p->GetTotalVertices(), total_time, total_length);
	}

	delete p;
}
int main(int argc, char **argv)
{
    // PseudoRandomSeed();
    
    if(argc < 2)
    {
    	fprintf(stderr, "missing arguments\n");
    	fprintf(stderr, "  Planner <file> [-m method]\n");
    	return 0;
    }
    
    bool exp_mode = false;
    int method = 2;
    int repetition = 50;

    for(int i=2;i<argc;i++)
    {
    	auto arg = string(argv[i]);
		if(arg == "-m")
		{
			//exp_mode = true;
			method = atoi(argv[++i]);
		}
		else if(arg == "-n")
		{
			max_nodes = atoi(argv[++i]);

			cerr<<" ! max_nodes set to "<<max_nodes<<endl;
		}
		else if(arg == "-p")
		{
			predict = true;
			cerr<<" ! predict enabled!"<<endl;
		}
		else if(arg == "-bc")
		{
			use_best_control = true;
			cerr<<" ! use_best_control"<<endl;
		}
		else if(arg == "-nc")
		{
			constraint = false;
			cerr<<" ! no dynamic constraint!"<<endl;
		}
		else if(arg == "-exp")
		{
			exp_mode = true;
		}
		else if(arg == "-s")
		{
			seed = atoi(argv[++i]);
		}
		else
		{
			fprintf(stderr, "unknown args %s\n", argv[2]);
			return -1;
		}
    }
    
    // seed not set
    if(seed == (1<<30))
    {
    	seed = PseudoRandomSeed();
    }
    else
    {
    	srandom(seed);
    }

    if(exp_mode)
    {
    	for(int i=0;i<repetition;i++)
    	{
    		PseudoRandomSeed();
    		RunExp(argv[1], method);
    	}
    }
    else
    {
    	if(argc >= 2)
		{
    		cerr<<"seed = "<<seed<<endl;
			filename = string(argv[1]);
			Graphics graphics(argv[1], method);
			graphics.MainLoop();
		}
    }

    return 0;    
}
