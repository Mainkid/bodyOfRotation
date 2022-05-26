#include <iostream>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/mat4x4.hpp"
#include "glm/matrix.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>
#include <ctime>
#include <iostream>
#include <vector>

typedef unsigned int uint;

#pragma comment(lib, "glfw3.lib")
#pragma comment(lib, "glew32.lib")
#pragma comment(lib, "opengl32.lib")


#define EPSILON 1.0e-5
#define IS_ZERO(v) (abs(v) < EPSILON) 
#define SIGN(v) (int)(((v) > EPSILON) - ((v) < -EPSILON))   	
#define RESOLUTION 32
#define C 2.0

using namespace std;

int HEIGHT = 600;
int WIDTH = 800;
const int TRIANGLE_SIZE = 5;
const float angle = 10;
GLFWwindow* g_window;

GLuint g_shaderProgram;
GLuint g_shaderProgramTr;
GLuint g_shaderProgramBody;

GLint g_uMVP, g_uMV, g_uMN;

bool phase2 = false;

class Model {
public:
	GLuint vbo;
	GLuint ibo;
	GLuint vao;
	GLsizei indexCount;
};


class Point3D
{
public:

	double x, y, z;
	double nx, ny, nz;

	Point3D() { x = y = z = 0.0; };


	Point3D(double _x, double _y) { x = _x; y = _y; z = 0.0; };

	Point3D(double _x, double _y, double _z) { x = _x; y = _y; z = _z; };

	Point3D(glm::vec4 v, glm::vec4 n)
	{
		x = v.x; y = v.y; z = v.z;
		nx = n.x; ny = n.y; nz = n.z;
	};

	void setNormal(double _x, double _y)
	{
		nx = -_y;
		ny = _x;
		nz = 0.0;
	}

	Point3D operator +(const Point3D& p) const { return Point3D(x + p.x, y + p.y, z + p.z); };

	Point3D operator -(const Point3D& p) const { return Point3D(x - p.x, y - p.y, z - p.z); };

	Point3D operator *(double v) const { return Point3D(x * v, y * v, z * v); };

	Point3D operator *(glm::mat4 v) const { return Point3D(v * glm::vec4(x, y, z, 1.0), v * glm::vec4(nx, ny, nz, 1.0)); };

	bool operator <(Point3D v) const { return x < v.x; };
	bool operator ==(Point3D v) const { return x == v.x; };


	void normalize()
	{
		double l = sqrt(x * x + y * y + z * z);
		if (IS_ZERO(l))
			x = y = z = 0.0;
		else
		{
			x /= l;
			y /= l;
			z /= l;
		}
	};

	
	static Point3D absMin(const Point3D& p1, const Point3D& p2)
	{
		return Point3D(abs(p1.x) < abs(p2.x) ? p1.x : p2.x, abs(p1.y) < abs(p2.y) ? p1.y : p2.y);
	};
};


class Segment
{
public:
	
	Point3D points[4];

	
	Point3D calc(double t) const
	{
		double t2 = t * t;
		double t3 = t2 * t;
		double nt = 1.0 - t;
		double nt2 = nt * nt;
		double nt3 = nt2 * nt;
		return Point3D(nt3 * points[0].x + 3.0 * t * nt2 * points[1].x + 3.0 * t2 * nt * points[2].x + t3 * points[3].x,
			nt3 * points[0].y + 3.0 * t * nt2 * points[1].y + 3.0 * t2 * nt * points[2].y + t3 * points[3].y);
	};
};

Model g_model;
Model g_model_tr;
Model g_model_body;

chrono::time_point<chrono::system_clock> g_callTime;
glm::mat4 mv = glm::mat4(1.0f);
glm::mat4 projection = glm::mat4(1.0f);
glm::mat3 mn = glm::mat3(1.0f);
vector<float> modelPoints;
vector<Point3D> windowPoints, bodyPoints;
vector<Segment> Curve;



bool tbezierSO1()
{
	int n = windowPoints.size() - 1;

	if (n < 1)
		return false;

	Curve.resize(n);

	Point3D cur, next, tgL, tgR, deltaL, deltaC, deltaR;
	double l1, l2;

	next = windowPoints[1] - windowPoints[0];
	next.normalize();

	for (int i = 0; i < n; ++i)
	{
		tgL = tgR;
		cur = next;

		deltaC = windowPoints[i + 1] - windowPoints[i];

		if (i > 0)
			deltaL = Point3D::absMin(deltaC, windowPoints[i] - windowPoints[i - 1]);
		else
			deltaL = deltaC;

		if (i < n - 1)
		{
			next = windowPoints[i + 2] - windowPoints[i + 1];
			next.normalize();
			if (IS_ZERO(cur.x) || IS_ZERO(cur.y))
				tgR = cur;
			else if (IS_ZERO(next.x) || IS_ZERO(next.y))
				tgR = next;
			else
				tgR = cur + next;
			tgR.normalize();
			deltaR = Point3D::absMin(deltaC, windowPoints[i + 2] - windowPoints[i + 1]);
		}
		else
		{
			tgR = Point3D();
			deltaR = deltaC;
		}

		l1 = IS_ZERO(tgL.x) ? 0.0 : deltaL.x / (C * tgL.x);
		l2 = IS_ZERO(tgR.x) ? 0.0 : deltaR.x / (C * tgR.x);

		if (abs(l1 * tgL.y) > abs(deltaL.y))
			l1 = IS_ZERO(tgL.y) ? 0.0 : deltaL.y / tgL.y;
		if (abs(l2 * tgR.y) > abs(deltaR.y))
			l2 = IS_ZERO(tgR.y) ? 0.0 : deltaR.y / tgR.y;

		Curve[i].points[0] = windowPoints[i];
		Curve[i].points[1] = Curve[i].points[0] + tgL * l1;
		Curve[i].points[3] = windowPoints[i + 1];
		Curve[i].points[2] = Curve[i].points[3] - tgR * l2;
	}

	return true;
}

GLuint createShader(const GLchar* code, GLenum type) {
	GLuint result = glCreateShader(type);

	glShaderSource(result, 1, &code, NULL);
	glCompileShader(result);

	GLint compiled;
	glGetShaderiv(result, GL_COMPILE_STATUS, &compiled);

	if (!compiled) {
		GLint infoLen = 0;
		glGetShaderiv(result, GL_INFO_LOG_LENGTH, &infoLen);
		if (infoLen > 0) {
			char* infoLog = new char[infoLen];
			glGetShaderInfoLog(result, infoLen, NULL, infoLog);

			cout << "Shader compilation error" << endl << infoLog << endl;
		}
		glDeleteShader(result);
		return 0;
	}

	return result;
}

GLuint createProgram(GLuint vsh, GLuint fsh) {
	GLuint result = glCreateProgram();

	glAttachShader(result, vsh);
	glAttachShader(result, fsh);

	glLinkProgram(result);

	GLint linked;
	glGetProgramiv(result, GL_LINK_STATUS, &linked);

	if (!linked) {
		GLint infoLen = 0;
		glGetProgramiv(result, GL_INFO_LOG_LENGTH, &infoLen);
		if (infoLen > 0) {
			char* infoLog = (char*)alloca(infoLen);
			glGetProgramInfoLog(result, infoLen, NULL, infoLog);
			cout << "Shader program linking error" << endl << infoLog << endl;
		}
		glDeleteProgram(result);
		return 0;
	}

	return result;
}

bool createShaderProgram() {
	g_shaderProgram = 0;

	const GLchar vsh[] =
		"#version 330\n"
		""
		"layout(location = 0) in vec2 a_position;"
		""
		""
		"void main()"
		"{"
		"    gl_Position = vec4(a_position, 0.0, 1.0);"
		"}";

	const GLchar fsh[] =
		"#version 330\n"
		""
		"layout(location = 0) out vec4 o_color;"
		""
		"void main()"
		"{"
		"    o_color = vec4(0.0, 0.0, 1.0, 1.0);"
		"}";

	GLuint vertexShader, fragmentShader;

	vertexShader = createShader(vsh, GL_VERTEX_SHADER);
	fragmentShader = createShader(fsh, GL_FRAGMENT_SHADER);

	g_shaderProgram = createProgram(vertexShader, fragmentShader);

	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);

	return g_shaderProgram != 0;
}

bool createShaderProgramTr() {
	g_shaderProgramTr = 0;

	const GLchar vsh[] =
		"#version 330\n"
		""
		"layout(location = 0) in vec2 a_position;"
		""
		""
		"void main()"
		"{"
		"    gl_Position = vec4(a_position, 0.0, 1.0);"
		"}";

	const GLchar fsh[] =
		"#version 330\n"
		""
		"layout(location = 0) out vec4 o_color;"
		""
		"void main()"
		"{"
		"    o_color = vec4(1.0, 0.0, 1.0, 1.0);"
		"}";

	GLuint vertexShader, fragmentShader;

	vertexShader = createShader(vsh, GL_VERTEX_SHADER);
	fragmentShader = createShader(fsh, GL_FRAGMENT_SHADER);

	g_shaderProgramTr = createProgram(vertexShader, fragmentShader);

	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);

	return g_shaderProgramTr != 0;
}

bool createShaderProgramBody()
{
	g_shaderProgramBody = 0;

	const GLchar vsh[] =
		"#version 330\n"
		""
		"layout(location = 0) in vec3 a_pos;"
		"layout(location = 1) in vec3 n;"
		""
		"uniform mat4 u_mvp, u_mv;"
		"uniform mat3 u_mn;"
		"out vec3 v_pos, v_normal;"
		""
		"void main()"
		"{"
		"    vec4 pos = vec4(a_pos.x, a_pos.y, a_pos.z, 1.0);"
		"    gl_Position = u_mvp * pos;"
		"    v_pos = vec3(u_mv * pos);"
		"    v_normal = u_mn * n;"
		"    v_normal = normalize(v_normal);"
		"}"
		;

	const GLchar fsh[] =
		"#version 330\n"
		""
		"in vec3 v_pos, v_normal;"
		""
		"layout(location = 0) out vec4 o_color;"
		""
		"void main()"
		"{"

		"   vec3 L = vec3(3.0, 3.0, 3.0);"
		"   vec3 n = normalize(v_normal);"
		"   vec3 l = normalize(v_pos - L);"
		"   vec3 e = normalize(-v_pos);"
		"   float d = max(dot(n, -l), 0.3);"
		"   vec3 h = normalize(-l + e);"
		"   float s = pow(max(dot(n, h), 0.0), 10.0);"
		"   o_color = vec4(1.0, 0.0, 1.0, 1.0);"
		"   o_color = pow(vec4(d * vec3(o_color) + s * vec3(1.0,1.0,1.0), 1.0),vec4(1/2.2));"
		"}"
		;

	GLuint vertexShader, fragmentShader;
	vertexShader = createShader(vsh, GL_VERTEX_SHADER);
	fragmentShader = createShader(fsh, GL_FRAGMENT_SHADER);

	g_shaderProgramBody = createProgram(vertexShader, fragmentShader);

	g_uMVP = glGetUniformLocation(g_shaderProgramBody, "u_mvp");
	g_uMV = glGetUniformLocation(g_shaderProgramBody, "u_mv");
	g_uMN = glGetUniformLocation(g_shaderProgramBody, "u_mn");

	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);

	return g_shaderProgramBody != 0;
}

GLfloat* genPointsList() {
	GLfloat* grid = new GLfloat[modelPoints.size()];
	for (int i = 0; i < modelPoints.size(); ++i) {
		grid[i] = modelPoints[i];
	}
	return grid;
}

GLuint* genInds() {
	GLuint* inds = new GLuint[(int)modelPoints.size() / 2];
	for (int i = 0; i < (int)modelPoints.size() / 2; ++i) {
		inds[i] = i;
	}
	return inds;
}

bool createModel() {
	const GLfloat* vertices = genPointsList();
	const GLuint* inds = genInds();

	glGenVertexArrays(1, &g_model.vao);
	glBindVertexArray(g_model.vao);

	glGenBuffers(1, &g_model.vbo);
	glBindBuffer(GL_ARRAY_BUFFER, g_model.vbo);
	glBufferData(GL_ARRAY_BUFFER, (modelPoints.size()) * sizeof(GLfloat), vertices, GL_STATIC_DRAW);

	glGenBuffers(1, &g_model.ibo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, g_model.ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, ((int)modelPoints.size() / 2) * sizeof(GLuint), inds, GL_STATIC_DRAW);

	g_model.indexCount = ((int)modelPoints.size() / 2);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(GLfloat), (const GLvoid*)0);
	delete[]vertices;
	delete[]inds;

	return g_model.vbo != 0 && g_model.ibo != 0 && g_model.vao != 0;
}

GLfloat* genPointsListTr() {
	GLfloat* grid = new GLfloat[6 * windowPoints.size()];
	auto it = windowPoints.begin();
	for (int i = 0; i < 6 * windowPoints.size(); i += 6, it++) {
		int x = it->x;
		int y = it->y;
		grid[i] = x;
		grid[i + 1] = y + TRIANGLE_SIZE;
		grid[i + 2] = x - TRIANGLE_SIZE;
		grid[i + 3] = y - TRIANGLE_SIZE;
		grid[i + 4] = x + TRIANGLE_SIZE;
		grid[i + 5] = y - TRIANGLE_SIZE;
	}
	for (int i = 0; i < 6 * windowPoints.size(); i += 2) {
		grid[i] = grid[i] * 2 / WIDTH - 1;
		grid[i + 1] = grid[i + 1] * 2 / HEIGHT - 1;
	}
	return grid;
}

void genBody()
{
	for (int i = 0; i < modelPoints.size(); i += 2)
	{
		bodyPoints.push_back(Point3D(modelPoints[i], modelPoints[i + 1] + 1.0f));
	};

	for (int i = 0; i < bodyPoints.size(); i++)
	{
		Point3D vec;
		if (i == 0)
			vec = bodyPoints[i + 1] - bodyPoints[i];
		else if (i == bodyPoints.size() - 1)
			vec = bodyPoints[i] - bodyPoints[i - 1];
		else
			vec = bodyPoints[i + 1] - bodyPoints[i - 1];
		bodyPoints[i].setNormal(vec.x, vec.y);
	}

	glm::mat4 matRot(1);
	for (int alpha = 10; alpha < 360; alpha += angle)
	{
		matRot = glm::rotate(matRot, glm::radians((float)angle), glm::vec3(1.0f, 0.0f, 0.0f));
		for (int i = 0; i < modelPoints.size() / 2; i++)
		{
			bodyPoints.push_back(bodyPoints[i] * matRot);
		};
	}
}

GLfloat* genVertsBody()
{
	GLfloat* grid = new GLfloat[6 * bodyPoints.size()];
	int m = modelPoints.size() / 2;
	int n = bodyPoints.size() / m;
	for (int i = 0; i < bodyPoints.size(); i++)
	{
		grid[6 * i] = bodyPoints[i].x;
		grid[6 * i + 1] = bodyPoints[i].y;
		grid[6 * i + 2] = bodyPoints[i].z;
		grid[6 * i + 3] = bodyPoints[i].nx;
		grid[6 * i + 4] = bodyPoints[i].ny;
		grid[6 * i + 5] = bodyPoints[i].nz;
	}
	return grid;
}

GLuint* genIndsTr() {
	GLuint* inds = new GLuint[6 * windowPoints.size()];
	for (int i = 0; i < 6 * windowPoints.size(); ++i) {
		inds[i] = i;
	}
	return inds;
}

GLuint* genIndsBody()
{
	int m = modelPoints.size() / 2;
	int n = bodyPoints.size() / m;
	GLuint* inds = new GLuint[6 * (bodyPoints.size() - n)];
	vector<GLuint> vecInds;
	for (int i = 0; i < bodyPoints.size() - m; i++)
	{
		if (i % m == 0) {
			vecInds.push_back(i + m);
			vecInds.push_back(i);
			vecInds.push_back(i + 1);
			continue;
		}
		if ((i + 1) % m == 0) {
			vecInds.push_back(i + m - 1);
			vecInds.push_back(i);
			vecInds.push_back(i + m);
			continue;
		}
		vecInds.push_back(i + m - 1);
		vecInds.push_back(i);
		vecInds.push_back(i + m);
		vecInds.push_back(i + m);
		vecInds.push_back(i);
		vecInds.push_back(i + 1);
	}
	for (int i = bodyPoints.size() - m; i < bodyPoints.size(); i++) {
		if (i % m == 0) {
			vecInds.push_back((i + m) % m);
			vecInds.push_back(i);
			vecInds.push_back(i + 1);
			continue;
		}
		if ((i + 1) % m == 0) {
			vecInds.push_back((i + m - 1) % m);
			vecInds.push_back(i);
			vecInds.push_back((i + m) % m);
			continue;
		}
		vecInds.push_back((i + m) % m);
		vecInds.push_back(i);
		vecInds.push_back(i + 1);
		vecInds.push_back((i + m - 1) % m);
		vecInds.push_back(i);
		vecInds.push_back((i + m) % m);
	}

	assert(vecInds.size() == 6 * (bodyPoints.size() - n));
	for (int i = 0; i < vecInds.size(); i++)
		inds[i] = vecInds[i];
	return inds;
}

bool createModelTr() {

	const GLfloat* vertices = genPointsListTr();
	const GLuint* inds = genIndsTr();

	glGenVertexArrays(1, &g_model_tr.vao);
	glBindVertexArray(g_model_tr.vao);

	glGenBuffers(1, &g_model_tr.vbo);
	glBindBuffer(GL_ARRAY_BUFFER, g_model_tr.vbo);
	glBufferData(GL_ARRAY_BUFFER, 6 * windowPoints.size() * sizeof(GLfloat), vertices, GL_STATIC_DRAW);

	glGenBuffers(1, &g_model_tr.ibo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, g_model_tr.ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, 6 * windowPoints.size() * sizeof(GLuint), inds, GL_STATIC_DRAW);

	g_model_tr.indexCount = 6 * windowPoints.size();

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(GLfloat), (const GLvoid*)0);

	delete[]vertices;
	delete[]inds;

	return g_model_tr.vbo != 0 && g_model_tr.ibo != 0 && g_model_tr.vao != 0;
}

bool createModelBody()
{
	genBody();
	const GLfloat* vertices = genVertsBody();
	const GLuint* inds = genIndsBody();

	glGenVertexArrays(1, &g_model_body.vao);
	glBindVertexArray(g_model_body.vao);

	glGenBuffers(1, &g_model_body.vbo);
	glBindBuffer(GL_ARRAY_BUFFER, g_model_body.vbo);
	glBufferData(GL_ARRAY_BUFFER, 6 * bodyPoints.size() * sizeof(GLfloat), vertices, GL_STATIC_DRAW);

	glGenBuffers(1, &g_model_body.ibo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, g_model_body.ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, 6 * (bodyPoints.size() - bodyPoints.size() / modelPoints.size() / 2) * sizeof(GLuint), inds, GL_STATIC_DRAW);

	g_model_body.indexCount = 6 * (bodyPoints.size() - bodyPoints.size() / modelPoints.size() / 2);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (const GLvoid*)0);

	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (const GLvoid*)(3 * sizeof(GLfloat)));
	delete[]vertices;
	delete[]inds;

	return g_model_body.vbo != 0 && g_model_body.ibo != 0 && g_model_body.vao != 0;
}

void draw() {
	// Clear color buffer.
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (windowPoints.size() > 1) {
		glUseProgram(g_shaderProgram);
		glBindVertexArray(g_model.vao);
		glDrawElements(GL_LINE_STRIP, g_model.indexCount, GL_UNSIGNED_INT, NULL);
	}
	glUseProgram(g_shaderProgramTr);
	glBindVertexArray(g_model_tr.vao);
	glDrawElements(GL_TRIANGLES, g_model_tr.indexCount, GL_UNSIGNED_INT, NULL);
}

void drawBody(float delta)
{
	// Clear color buffer.
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glUseProgram(g_shaderProgramBody);
	glBindVertexArray(g_model_body.vao);

	mv = glm::rotate(mv, glm::radians(delta * 30.0f), glm::vec3(0.0, 1.0, 0.0));
	mn = glm::transpose(glm::inverse(glm::mat3(mv)));
	glUniformMatrix4fv(g_uMVP, 1, GL_FALSE, glm::value_ptr(projection * mv));
	glUniformMatrix4fv(g_uMV, 1, GL_FALSE, glm::value_ptr(mv));
	glUniformMatrix3fv(g_uMN, 1, GL_FALSE, glm::value_ptr(mn));

	glDrawElements(GL_TRIANGLES, g_model_body.indexCount, GL_UNSIGNED_INT, NULL);
}

vector<float> buildModelPts(vector<Segment> splines, vector<Point3D> windowPts)
{
	assert(splines.size() + 1 == windowPts.size());
	vector<float> anspts;

	for (auto s : splines)
	{
		for (int i = 0; i < RESOLUTION; ++i)
		{
			Point3D p = s.calc((double)i / (double)RESOLUTION);
			anspts.push_back(2 * p.x / WIDTH - 1);
			anspts.push_back(2 * p.y / HEIGHT - 1);
		}
	}

	return anspts;
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS && !phase2)
	{
		double xpos, ypos;
		glfwGetCursorPos(window, &xpos, &ypos);
		ypos = HEIGHT - ypos;
		if (find(windowPoints.begin(), windowPoints.end(), Point3D(xpos, ypos)) != windowPoints.end())
			return;
		windowPoints.push_back(Point3D(xpos, ypos));
		sort(windowPoints.begin(), windowPoints.end());
		if (windowPoints.size() >= 2) {
			tbezierSO1();
			modelPoints = buildModelPts(Curve, windowPoints);
			createModel();
		}
		if (windowPoints.size() >= 1) {
			createModelTr();
			draw();
			glfwSwapBuffers(g_window);
			glfwPollEvents();
		}
	}
}

void back_button_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
	if (key == GLFW_KEY_BACKSPACE && action == GLFW_PRESS && !phase2) {
		if (windowPoints.size() > 0)
			windowPoints.pop_back();
		if (windowPoints.size() >= 2) {
			tbezierSO1();
			modelPoints = buildModelPts(Curve, windowPoints);
			createModel();
		}
		if (windowPoints.size() >= 1) {
			createModelTr();
			draw();
			glfwSwapBuffers(g_window);
			glfwPollEvents();
		}
	}
}

bool init() {
	// Set initial color of color buffer to white.
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

	glEnable(GL_DEPTH_TEST);

	glfwSetMouseButtonCallback(g_window, mouse_button_callback);
	glfwSetKeyCallback(g_window, back_button_callback);
	return createShaderProgram() && createShaderProgramTr();
}

bool initBody() {
	// Set initial color of color buffer to white.
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

	glEnable(GL_DEPTH_TEST);

	return createShaderProgramBody() && createModelBody();
}

void reshape(GLFWwindow* window, int width, int height) {
	WIDTH = width;
	HEIGHT = height;
	glViewport(0, 0, width, height);
}

void cleanup() {
	if (g_shaderProgram != 0)
		glDeleteProgram(g_shaderProgram);
	if (g_model.vbo != 0)
		glDeleteBuffers(1, &g_model.vbo);
	if (g_model.ibo != 0)
		glDeleteBuffers(1, &g_model.ibo);
	if (g_model.vao != 0)
		glDeleteVertexArrays(1, &g_model.vao);
	if (g_shaderProgramTr != 0)
		glDeleteProgram(g_shaderProgramTr);
	if (g_model_tr.vbo != 0)
		glDeleteBuffers(1, &g_model_tr.vbo);
	if (g_model_tr.ibo != 0)
		glDeleteBuffers(1, &g_model_tr.ibo);
	if (g_model_tr.vao != 0)
		glDeleteVertexArrays(1, &g_model_tr.vao);
	if (g_shaderProgramBody != 0)
		glDeleteProgram(g_shaderProgramBody);
	if (g_model_body.vbo != 0)
		glDeleteBuffers(1, &g_model_body.vbo);
	if (g_model_body.ibo != 0)
		glDeleteBuffers(1, &g_model_body.ibo);
	if (g_model_body.vao != 0)
		glDeleteVertexArrays(1, &g_model_body.vao);
}

bool initOpenGL() {
	// Initialize GLFW functions.
	if (!glfwInit()) {
		cout << "Failed to initialize GLFW" << endl;
		return false;
	}

	// Request OpenGL 3.3 without obsoleted functions.
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// Create window.
	g_window = glfwCreateWindow(WIDTH, HEIGHT, "OpenGL Test", NULL, NULL);
	if (g_window == NULL) {
		cout << "Failed to open GLFW window" << endl;
		glfwTerminate();
		return false;
	}

	// Initialize OpenGL context with.
	glfwMakeContextCurrent(g_window);

	// Set internal GLEW variable to activate OpenGL core profile.
	glewExperimental = true;

	// Initialize GLEW functions.
	if (glewInit() != GLEW_OK) {
		cout << "Failed to initialize GLEW" << endl;
		return false;
	}

	// Ensure we can capture the escape key being pressed.
	glfwSetInputMode(g_window, GLFW_STICKY_KEYS, GL_TRUE);

	// Set callback for framebuffer resizing event.
	glfwSetFramebufferSizeCallback(g_window, reshape);

	return true;
}

void tearDownOpenGL() {
	// Terminate GLFW.
	glfwTerminate();
}

int main() {

	// Initialize OpenGL
	if (!initOpenGL())
		return -1;

	// Initialize graphical resources.
	bool isOk = init();

	if (isOk) {
		// Main loop until window closed or escape pressed.
		while (glfwGetKey(g_window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
			glfwWindowShouldClose(g_window) == 0 &&
			glfwGetKey(g_window, GLFW_KEY_ENTER) != GLFW_PRESS) {
			// Poll window events.
			glfwPollEvents();
		}
	}
	projection = glm::perspective(glm::radians(45.0f), 4.0f / 3.0f, 0.1f, 100.0f);
	mv = glm::lookAt(glm::vec3(0.0, 1.0, 2.0), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0, 1.0, 0.0));
	mv = glm::scale(mv, glm::vec3(0.5f, 0.5f, 0.5f));
	mv = glm::rotate(mv, glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));

	if (initBody()) {
		phase2 = true;
		g_callTime = chrono::system_clock::now();
		if (glfwGetKey(g_window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
			glfwWindowShouldClose(g_window) == 0) {
			while (glfwGetKey(g_window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
				glfwWindowShouldClose(g_window) == 0) {

				auto callTime = chrono::system_clock::now();
				chrono::duration<double> elapsed = callTime - g_callTime;
				g_callTime = callTime;
				// Draw scene.
				drawBody(elapsed.count());
				// Swap buffers.
				glfwSwapBuffers(g_window);
				// Poll window events.
				glfwPollEvents();
			}
		}
	}

	// Cleanup graphical resources.
	cleanup();

	// Tear down OpenGL.
	tearDownOpenGL();

	return isOk ? 0 : -1;
}