﻿#include "GLViewer.hpp"
#include <random>
#include "cuUtils.h"

#include <opencv2/opencv.hpp>
#if defined(_DEBUG) && defined(_WIN32)
#error "This sample should not be built in Debug mode, use RelWithDebInfo if you want to do step by step."
#endif


GLchar* VERTEX_SHADER =
        "#version 330 core\n"
        "layout(location = 0) in vec3 in_Vertex;\n"
        "layout(location = 1) in vec4 in_Color;\n"
        "uniform mat4 u_mvpMatrix;\n"
        "out vec4 b_color;\n"
        "void main() {\n"
        "   b_color = in_Color;\n"
        "	gl_Position = u_mvpMatrix * vec4(in_Vertex, 1);\n"
        "}";

GLchar* FRAGMENT_SHADER =
        "#version 330 core\n"
        "in vec4 b_color;\n"
        "layout(location = 0) out vec4 out_Color;\n"
        "void main() {\n"
        "   out_Color = b_color;\n"
        "}";

GLViewer* currentInstance_ = nullptr;


float const colors2[5][3] ={
    {.231f, .909f, .69f},
    {.098f, .686f, .816f},
    {.412f, .4f, .804f},
    {1, .725f, .0f},
    {.989f, .388f, .419f}
};


inline sl::float4 generateColorClass(int idx) {
    int const offset = idx % 5;
    sl::float4 clr(colors2[offset][0], colors2[offset][1], colors2[offset][2],0.8);
    return clr;
}

GLViewer::GLViewer() : available(false) {
    currentInstance_ = this;
    clearInputs();
}

GLViewer::~GLViewer() {}

void GLViewer::exit() {
    if (currentInstance_) {
        pointCloud_.close();
        available = false;
    }
}

bool GLViewer::isAvailable() {
    if (available)
        glutMainLoopEvent();
    return available;
}

void CloseFunc(void) { if (currentInstance_) currentInstance_->exit(); }

void GLViewer::init(int argc, char **argv, sl::CameraParameters param) {

    glutInit(&argc, argv);
    int wnd_w = glutGet(GLUT_SCREEN_WIDTH);
    int wnd_h = glutGet(GLUT_SCREEN_HEIGHT);
    int width = wnd_w*0.9;
    int height = wnd_h*0.9;
    if (width > param.image_size.width && height > param.image_size.height) {
        width = param.image_size.width;
        height = param.image_size.height;
    }

    camera_parameters = param;
    windowSize.width = width;
    windowSize.height = height;
    glutInitWindowSize(windowSize.width, windowSize.height);
    glutInitWindowPosition(wnd_w*0.05, wnd_h*0.05);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutCreateWindow("ZED Object Detection Viewer");
    glViewport(0,0,width,height);

    GLenum err = glewInit();
    if (GLEW_OK != err)
        std::cout << "ERROR: glewInit failed: " << glewGetErrorString(err) << "\n";

    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
    glEnable(GL_DEPTH_TEST);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

    pointCloudSize = param.image_size;
    bool err_pc = pointCloud_.initialize(pointCloudSize);
    if (!err_pc)
        std::cout << "ERROR: Failed to initialized point cloud"<<std::endl;

    // Compile and create the shader for 3D objects
    shader.it = Shader(VERTEX_SHADER, FRAGMENT_SHADER);
    shader.MVP_Mat = glGetUniformLocation(shader.it.getProgramId(), "u_mvpMatrix");

    // Create the rendering camera
    setRenderCameraProjection(param,0.5f,200000);

    // Create the bounding box object
    BBox_obj.init();
    BBox_obj.setDrawingType(GL_QUADS);

    // Clear trajectories
#ifdef WITH_TRAJECTORIES
    trajectories.clear();
#endif

    // Set background color (black)
    bckgrnd_clr = sl::float3(0, 0, 0);

    // Set OpenGL settings
    glDisable(GL_DEPTH_TEST); //avoid occlusion with bbox

    // Map glut function on this class methods
    glutDisplayFunc(GLViewer::drawCallback);
    glutReshapeFunc(GLViewer::reshapeCallback);
    glutKeyboardFunc(GLViewer::keyPressedCallback);
    glutKeyboardUpFunc(GLViewer::keyReleasedCallback);
    glutCloseFunc(CloseFunc);

    available = true;
}

void GLViewer::setRenderCameraProjection(sl::CameraParameters params,float znear, float zfar) {
    // Just slightly up the ZED camera FOV to make a small black border
    float fov_y = (params.v_fov+0.5f) * M_PI / 180.f;
    float fov_x = (params.h_fov+0.5f) * M_PI / 180.f;

    projection_(0, 0) = 1.0f / tanf(fov_x * 0.5f);
    projection_(1, 1) = 1.0f / tanf(fov_y * 0.5f);
    projection_(2, 2) = -(zfar + znear) / (zfar - znear);
    projection_(3, 2) = -1;
    projection_(2, 3) = -(2.f * zfar * znear) / (zfar - znear);
    projection_(3, 3) = 0;

    projection_(0, 0) = 1.0f /  tanf(fov_x * 0.5f); //Horizontal FoV.
    projection_(0, 1) = 0;
    projection_(0, 2) = 2.0f * ((params.image_size.width - 1.0f * params.cx) / params.image_size.width) - 1.0f; //Horizontal offset.
    projection_(0, 3) = 0;

    projection_(1, 0) = 0;
    projection_(1, 1) = 1.0f / tanf(fov_y * 0.5f); //Vertical FoV.
    projection_(1, 2) = -(2.0f * ((params.image_size.height - 1.0f * params.cy) / params.image_size.height ) - 1.0f); //Vertical offset.
    projection_(1, 3) = 0;

    projection_(2, 0) = 0;
    projection_(2, 1) = 0;
    projection_(2, 2) = -(zfar + znear) / (zfar - znear); //Near and far planes.
    projection_(2, 3) = -(2.0f * zfar * znear) / (zfar - znear); //Near and far planes.

    projection_(3, 0) = 0;
    projection_(3, 1) = 0;
    projection_(3, 2) = -1;
    projection_(3, 3) = 0.0f;
}

void GLViewer::render() {
    if (available) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(bckgrnd_clr.r, bckgrnd_clr.g, bckgrnd_clr.b, 1.f);
        mtx.lock();
        update();
        draw();
        printText();

        std::cout<<" Out Size : "<<this->windowSize.height<<" / "<<this->windowSize.width<<std::endl;
        cv::Mat image = cv::Mat(1080,1920,CV_8UC4,1);
        glReadPixels(0, 0,1920,1080, GL_RGBA, GL_UNSIGNED_BYTE, image.data);
        cv::cvtColor(image,image,cv::COLOR_RGBA2BGRA);
        cv::flip(image,image,0);
        char name[128];
        sprintf(name,"test_%04d.png",f_count);
        cv::imwrite(name,image);

        f_count++;
        mtx.unlock();
        glutSwapBuffers();
        glutPostRedisplay();
    }
}

void GLViewer::updateData(sl::Mat image, sl::Mat depth, sl::Objects &obj, sl::Timestamp image_ts) {
    //if (mtx.try_lock())
    mtx.lock();
    {

        pointCloud_.pushNewPC(image,depth,camera_parameters);    // Update point cloud

        BBox_obj.clear();

        std::vector<sl::ObjectData> objs = obj.object_list;
        objectsName.clear();


        if (objs.size()>0)
        {
            for (int i = 0; i < objs.size(); i++) {
                if (objs[i].tracking_state == sl::OBJECT_TRACKING_STATE::OK && objs[i].id>=0) {
                    auto bb_ = objs[i].bounding_box;
                    auto pos_ = objs[i].position;
                    ObjectExtPosition ext_pos_;
                    ext_pos_.position = pos_;
                    ext_pos_.timestamp = obj.timestamp;
#ifdef WITH_TRAJECTORIES
                    trajectories[objs[i].id].push_back(ext_pos_);
#endif
                    auto clr_id = generateColorClass(objs[i].id);
                    if (g_showBox && bb_.size()>0) {
                        BBox_obj.addBoundingBox(bb_,clr_id);
                    }

                    if (g_showLabel) {
                        if ( bb_.size()>0) {
                            objectsName.emplace_back();
                            objectsName.back().name_lineA = "ID : "+ std::to_string(objs[i].id);
                            std::stringstream ss_vel;
                            ss_vel << std::fixed << std::setprecision(1) << objs[i].velocity.norm();
                            objectsName.back().name_lineB = ss_vel.str()+" m/s";
                            objectsName.back().color = clr_id;
                            objectsName.back().position = pos_;
                            objectsName.back().position.y =(bb_[0].y + bb_[1].y + bb_[2].y + bb_[3].y)/4.f +0.2f;
                        }
                    }
                }
            }
        }

        //Transform current Map to 3D trajectories
#ifdef WITH_TRAJECTORIES
        std::map<int, std::deque<ObjectExtPosition>>::iterator itT = trajectories.begin();
        while (itT != trajectories.end())
        {
            trajectories_obj[itT->first].clear();
            Simple3DObject tmp = trajectories_obj[itT->first];
            if (!tmp.isInit())
                tmp.init();
            tmp.setDrawingType(GL_LINES);
            std::vector<sl::float3> pts;
            for (int k=0;k<itT->second.size();k++)
            {
                pts.push_back(itT->second.at(k).position);

            }
            auto clr_id = generateColorClass(itT->first);
            if(!pts.empty())
                tmp.addPoints(pts,clr_id);
            trajectories_obj[itT->first] = tmp;
            itT++;
        }

        // Remove old data from trajectories, based on timestamp comparison
        itT = trajectories.begin();
        while (itT != trajectories.end())
        {

            if (itT->second.empty()) {
                trajectories.erase(itT++);
            }
            else{
                if (itT->second.front().timestamp.getMilliseconds()<image_ts.getMilliseconds() - line_fading_time_ms)
                    itT->second.pop_front();
                itT++;
            }
        }
#endif
        mtx.unlock();
    }
}

void GLViewer::update() {
    if (keyStates_['q'] == KEY_STATE::UP || keyStates_['Q'] == KEY_STATE::UP || keyStates_[27] == KEY_STATE::UP) {
        currentInstance_->exit();
        return;
    }

    pointCloud_.update();
    // Update BBox
    BBox_obj.pushToGPU();

    //Clear inputs
    clearInputs();
}

void GLViewer::draw() {
    const sl::Transform vpMatrix = projection_; //simplification : proj * view = proj

    glPointSize(1.0);
    pointCloud_.draw(vpMatrix);

    glUseProgram(shader.it.getProgramId());
    glUniformMatrix4fv(shader.MVP_Mat, 1, GL_TRUE, vpMatrix.m);

#ifdef WITH_TRAJECTORIES
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glLineWidth(3.f);
    for (std::pair<int, Simple3DObject> element : trajectories_obj) {
        element.second.pushToGPU();
        element.second.draw(vpMatrix);
    }
#endif

    glLineWidth(1.f);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    BBox_obj.draw(vpMatrix);
    glUseProgram(0);
}

sl::float2 compute3Dprojection(sl::float3 &pt, const sl::Transform &cam, sl::Resolution wnd_size) {
    sl::float4 pt4d(pt.x, pt.y, pt.z, 1.);
    auto proj3D_cam = pt4d * cam;
    sl::float2 proj2D;
    proj2D.x = ((proj3D_cam.x / pt4d.w) * wnd_size.width) / (2.f * proj3D_cam.w) + wnd_size.width / 2.f;
    proj2D.y = ((proj3D_cam.y / pt4d.w) * wnd_size.height) / (2.f * proj3D_cam.w) + wnd_size.height / 2.f;
    return proj2D;
}

void GLViewer::printText() {
    const sl::Transform vpMatrix = projection_;
    sl::Resolution wnd_size(glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));
    for (auto it : objectsName) {
        auto pt2d = compute3Dprojection(it.position, vpMatrix, wnd_size);
        glColor4f(it.color.r, it.color.g, it.color.b, 1.f);
        const auto *string = it.name_lineA.c_str();
        glWindowPos2f(pt2d.x-40, pt2d.y+20);
        int len = (int)strlen(string);
        for (int i = 0; i < len; i++)
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, string[i]);

        string = it.name_lineB.c_str();
        glWindowPos2f(pt2d.x-40, pt2d.y);
        len = (int)strlen(string);
        for (int i = 0; i < len; i++)
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, string[i]);
    }
}


void GLViewer::clearInputs() {
    for (unsigned int i = 0; i < 256; ++i)
        if (keyStates_[i] != KEY_STATE::DOWN)
            keyStates_[i] = KEY_STATE::FREE;
}

void GLViewer::drawCallback() {
    currentInstance_->render();
}

void GLViewer::reshapeCallback(int width, int height) {
    glViewport(0, 0, width, height);
}

void GLViewer::keyPressedCallback(unsigned char c, int x, int y) {
    currentInstance_->keyStates_[c] = KEY_STATE::DOWN;
    glutPostRedisplay();
}

void GLViewer::keyReleasedCallback(unsigned char c, int x, int y) {
    currentInstance_->keyStates_[c] = KEY_STATE::UP;
}

void GLViewer::idle() {
    glutPostRedisplay();
}

Simple3DObject::Simple3DObject() {
    is_init=false;
}


Simple3DObject::~Simple3DObject() {
    if (vaoID_ != 0) {
        glDeleteBuffers(3, vboID_);
        glDeleteVertexArrays(1, &vaoID_);
        vaoID_=0;
        is_init=false;
    }
}

bool Simple3DObject::isInit()
{
    return is_init;
}
void Simple3DObject::init() {
    vaoID_ = 0;
    isStatic_=false;
    drawingType_ = GL_TRIANGLES;
    rotation_.setIdentity();
    shader.it = Shader(VERTEX_SHADER, FRAGMENT_SHADER);
    shader.MVP_Mat = glGetUniformLocation(shader.it.getProgramId(), "u_mvpMatrix");
    is_init=true;
}


void Simple3DObject::addPoints(std::vector<sl::float3> pts,sl::float4 base_clr)
{
    for (int k=0;k<pts.size();k++) {
        sl::float3 pt = pts.at(k);
        vertices_.push_back(pt.x);
        vertices_.push_back(pt.y);
        vertices_.push_back(pt.z);
        colors_.push_back(base_clr.r);
        colors_.push_back(base_clr.g);
        colors_.push_back(base_clr.b);
        colors_.push_back(1.f);
        int current_size_index = (vertices_.size()/3 -1);
        indices_.push_back(current_size_index);
        indices_.push_back(current_size_index+1);
    }
}

void Simple3DObject::addBoundingBox(std::vector<sl::float3> bbox,sl::float4 base_clr){

    float grad_distance = 0.3;

    for (int p=0;p<4;p++) {

        int indexA = p;
        int indexB = p+1;

        if (indexB>3)
            indexB=0;

        vertices_.push_back(bbox[indexA].x);
        vertices_.push_back(bbox[indexA].y);
        vertices_.push_back(bbox[indexA].z);

        colors_.push_back(base_clr.r);
        colors_.push_back(base_clr.g);
        colors_.push_back(base_clr.b);
        colors_.push_back(base_clr.a);

        vertices_.push_back(bbox[indexB].x);
        vertices_.push_back(bbox[indexB].y);
        vertices_.push_back(bbox[indexB].z);

        colors_.push_back(base_clr.r);
        colors_.push_back(base_clr.g);
        colors_.push_back(base_clr.b);
        colors_.push_back(base_clr.a);

        vertices_.push_back(bbox[indexB].x);
        vertices_.push_back(bbox[indexB].y-grad_distance);
        vertices_.push_back(bbox[indexB].z);

        colors_.push_back(base_clr.r);
        colors_.push_back(base_clr.g);
        colors_.push_back(base_clr.b);
        colors_.push_back(0);

        vertices_.push_back(bbox[indexA].x);
        vertices_.push_back(bbox[indexA].y-grad_distance);
        vertices_.push_back(bbox[indexA].z);

        colors_.push_back(base_clr.r);
        colors_.push_back(base_clr.g);
        colors_.push_back(base_clr.b);
        colors_.push_back(0);

        indices_.push_back((int)indices_.size());
        indices_.push_back((int)indices_.size());
        indices_.push_back((int)indices_.size());
        indices_.push_back((int)indices_.size());
    }

    for (int p=4;p<8;p++) {

        int indexA = p;
        int indexB = p+1;

        if (indexB>7)
            indexB=4;

        vertices_.push_back(bbox[indexA].x);
        vertices_.push_back(bbox[indexA].y);
        vertices_.push_back(bbox[indexA].z);

        colors_.push_back(base_clr.r);
        colors_.push_back(base_clr.g);
        colors_.push_back(base_clr.b);
        colors_.push_back(base_clr.a);

        vertices_.push_back(bbox[indexB].x);
        vertices_.push_back(bbox[indexB].y);
        vertices_.push_back(bbox[indexB].z);

        colors_.push_back(base_clr.r);
        colors_.push_back(base_clr.g);
        colors_.push_back(base_clr.b);
        colors_.push_back(base_clr.a);

        vertices_.push_back(bbox[indexB].x);
        vertices_.push_back(bbox[indexB].y+grad_distance);
        vertices_.push_back(bbox[indexB].z);

        colors_.push_back(base_clr.r);
        colors_.push_back(base_clr.g);
        colors_.push_back(base_clr.b);
        colors_.push_back(0);

        vertices_.push_back(bbox[indexA].x);
        vertices_.push_back(bbox[indexA].y+grad_distance);
        vertices_.push_back(bbox[indexA].z);

        colors_.push_back(base_clr.r);
        colors_.push_back(base_clr.g);
        colors_.push_back(base_clr.b);
        colors_.push_back(0);

        indices_.push_back((int)indices_.size());
        indices_.push_back((int)indices_.size());
        indices_.push_back((int)indices_.size());
        indices_.push_back((int)indices_.size());
    }
}

void Simple3DObject::pushToGPU() {
    if (!isStatic_ || vaoID_ == 0) {
        if (vaoID_ == 0) {
            glGenVertexArrays(1, &vaoID_);
            glGenBuffers(3, vboID_);
        }
        glShadeModel(GL_SMOOTH);
        glBindVertexArray(vaoID_);
        glBindBuffer(GL_ARRAY_BUFFER, vboID_[0]);
        glBufferData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(float), &vertices_[0], isStatic_ ? GL_STATIC_DRAW : GL_DYNAMIC_DRAW);
        glVertexAttribPointer(Shader::ATTRIB_VERTICES_POS, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(Shader::ATTRIB_VERTICES_POS);

        glBindBuffer(GL_ARRAY_BUFFER, vboID_[1]);
        glBufferData(GL_ARRAY_BUFFER, colors_.size() * sizeof(float), &colors_[0], isStatic_ ? GL_STATIC_DRAW : GL_DYNAMIC_DRAW);
        glVertexAttribPointer(Shader::ATTRIB_COLOR_POS, 4, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(Shader::ATTRIB_COLOR_POS);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vboID_[2]);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_.size() * sizeof(unsigned int), &indices_[0], isStatic_ ? GL_STATIC_DRAW : GL_DYNAMIC_DRAW);

        glBindVertexArray(0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
}

void Simple3DObject::clear() {
    vertices_.clear();
    colors_.clear();
    indices_.clear();
}

void Simple3DObject::setDrawingType(GLenum type) {
    drawingType_ = type;
}

void Simple3DObject::draw(const sl::Transform& vp) {
    if (indices_.size() && vaoID_) {
        glBindVertexArray(vaoID_);
        glDrawElements(drawingType_, (GLsizei)indices_.size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
}

void Simple3DObject::translate(const sl::Translation& t) {
    position_ = position_ + t;
}

void Simple3DObject::setPosition(const sl::Translation& p) {
    position_ = p;
}

void Simple3DObject::setRT(const sl::Transform& mRT) {
    position_ = mRT.getTranslation();
    rotation_ = mRT.getOrientation();
}

void Simple3DObject::rotate(const sl::Orientation& rot) {
    rotation_ = rot * rotation_;
}

void Simple3DObject::rotate(const sl::Rotation& m) {
    this->rotate(sl::Orientation(m));
}

void Simple3DObject::setRotation(const sl::Orientation& rot) {
    rotation_ = rot;
}

void Simple3DObject::setRotation(const sl::Rotation& m) {
    this->setRotation(sl::Orientation(m));
}

const sl::Translation& Simple3DObject::getPosition() const {
    return position_;
}

sl::Transform Simple3DObject::getModelMatrix() const {
    sl::Transform tmp;
    tmp.setOrientation(rotation_);
    tmp.setTranslation(position_);
    return tmp;
}

Shader::Shader(GLchar* vs, GLchar* fs) {
    if (!compile(verterxId_, GL_VERTEX_SHADER, vs)) {
        std::cout << "ERROR: while compiling vertex shader" << std::endl;
    }
    if (!compile(fragmentId_, GL_FRAGMENT_SHADER, fs)) {
        std::cout << "ERROR: while compiling fragment shader" << std::endl;
    }

    programId_ = glCreateProgram();

    glAttachShader(programId_, verterxId_);
    glAttachShader(programId_, fragmentId_);

    glBindAttribLocation(programId_, ATTRIB_VERTICES_POS, "in_vertex");
    glBindAttribLocation(programId_, ATTRIB_COLOR_POS, "in_texCoord");

    glLinkProgram(programId_);

    GLint errorlk(0);
    glGetProgramiv(programId_, GL_LINK_STATUS, &errorlk);
    if (errorlk != GL_TRUE) {
        std::cout << "ERROR: while linking Shader :" << std::endl;
        GLint errorSize(0);
        glGetProgramiv(programId_, GL_INFO_LOG_LENGTH, &errorSize);

        char *error = new char[errorSize + 1];
        glGetShaderInfoLog(programId_, errorSize, &errorSize, error);
        error[errorSize] = '\0';
        std::cout << error << std::endl;

        delete[] error;
        glDeleteProgram(programId_);
    }
}

Shader::~Shader() {
    if (verterxId_ != 0)
        glDeleteShader(verterxId_);
    if (fragmentId_ != 0)
        glDeleteShader(fragmentId_);
    if (programId_ != 0)
        glDeleteShader(programId_);
}

GLuint Shader::getProgramId() {
    return programId_;
}

bool Shader::compile(GLuint &shaderId, GLenum type, GLchar* src) {
    shaderId = glCreateShader(type);
    if (shaderId == 0) {
        std::cout << "ERROR: shader type (" << type << ") does not exist" << std::endl;
        return false;
    }
    glShaderSource(shaderId, 1, (const char**)&src, 0);
    glCompileShader(shaderId);

    GLint errorCp(0);
    glGetShaderiv(shaderId, GL_COMPILE_STATUS, &errorCp);
    if (errorCp != GL_TRUE) {
        std::cout << "ERROR: while compiling Shader :" << std::endl;
        GLint errorSize(0);
        glGetShaderiv(shaderId, GL_INFO_LOG_LENGTH, &errorSize);

        char *error = new char[errorSize + 1];
        glGetShaderInfoLog(shaderId, errorSize, &errorSize, error);
        error[errorSize] = '\0';
        std::cout << error << std::endl;

        delete[] error;
        glDeleteShader(shaderId);
        return false;
    }
    return true;
}

GLchar* POINTCLOUD_VERTEX_SHADER =
        "#version 330 core\n"
        "layout(location = 0) in vec4 in_VertexRGBA;\n"
        "uniform mat4 u_mvpMatrix;\n"
        "out vec4 b_color;\n"
        "void main() {\n"
        // Decompose the 4th channel of the XYZRGBA buffer to retrieve the color of the point (1float to 4uint)
        "   uint vertexColor = floatBitsToUint(in_VertexRGBA.w); \n"
        "   vec3 clr_int = vec3((vertexColor & uint(0x000000FF)), (vertexColor & uint(0x0000FF00)) >> 8, (vertexColor & uint(0x00FF0000)) >> 16);\n"
        "   b_color = vec4(clr_int.r / 255.0f, clr_int.g / 255.0f, clr_int.b / 255.0f, 1.f);"
        "	gl_Position = u_mvpMatrix * vec4(in_VertexRGBA.xyz, 1);\n"
        "}";

GLchar* POINTCLOUD_FRAGMENT_SHADER =
        "#version 330 core\n"
        "in vec4 b_color;\n"
        "layout(location = 0) out vec4 out_Color;\n"
        "void main() {\n"
        "   out_Color = b_color;\n"
        "}";

PointCloud::PointCloud() : hasNewPCL_(false) {}

PointCloud::~PointCloud() {
    close();
}

void PointCloud::close() {
    if (matGPU_.isInit()) {
        matGPU_.free();
        cudaGraphicsUnmapResources(1, &bufferCudaID_, 0);
        glDeleteBuffers(1, &bufferGLID_);
    }
}

bool PointCloud::initialize(sl::Resolution res) {
    glGenBuffers(1, &bufferGLID_);
    glBindBuffer(GL_ARRAY_BUFFER, bufferGLID_);
    glBufferData(GL_ARRAY_BUFFER, res.area() * 4 * sizeof(float), 0, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    shader.it = Shader(POINTCLOUD_VERTEX_SHADER, POINTCLOUD_FRAGMENT_SHADER);
    shader.MVP_Mat = glGetUniformLocation(shader.it.getProgramId(), "u_mvpMatrix");
    matGPU_.alloc(res, sl::MAT_TYPE::F32_C4, sl::MEM::GPU);

    cudaError_t err = cudaGraphicsGLRegisterBuffer(&bufferCudaID_, bufferGLID_, cudaGraphicsRegisterFlagsNone);
    err = cudaGraphicsMapResources(1, &bufferCudaID_, 0);
    err = cudaGraphicsResourceGetMappedPointer((void**)&xyzrgbaMappedBuf_, &numBytes_, bufferCudaID_);
    return (err==cudaSuccess);
}

void PointCloud::pushNewPC(sl::Mat &image, sl::Mat& depth, sl::CameraParameters cam_params)
{
    if (matGPU_.isInit()) {
        // CUDA code to convert Image + Z Buffer into single point cloud
        // It was possible to do it also on a GLSL shader, but... just a choice.
        hasNewPCL_ = triangulateImageandZ(matGPU_,image,depth,cam_params);
    }
}

void PointCloud::update() {
    if (hasNewPCL_ && matGPU_.isInit()) {
        cudaMemcpy(xyzrgbaMappedBuf_, matGPU_.getPtr<sl::float4>(sl::MEM::GPU), numBytes_, cudaMemcpyDeviceToDevice);
        hasNewPCL_ = false;
    }
}

void PointCloud::draw(const sl::Transform& vp) {
    if (matGPU_.isInit()) {
        glUseProgram(shader.it.getProgramId());
        glUniformMatrix4fv(shader.MVP_Mat, 1, GL_TRUE, vp.m);

        glBindBuffer(GL_ARRAY_BUFFER, bufferGLID_);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);

        glDrawArrays(GL_POINTS, 0, matGPU_.getResolution().area());
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glUseProgram(0);
    }
}

