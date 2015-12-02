#include "SamplePlugin.hpp"

#include <rws/RobWorkStudio.hpp>

#include <QPushButton>

#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>

using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;

using namespace rws;

using namespace cv;

SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
    setupUi(this);

    _timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

    // now connect stuff from the ui component
    connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_spinBox  ,SIGNAL(valueChanged(int)), this, SLOT(btnPressed()) );

    // robotics test tab - connect gui
    connect(_btn_rotest_computeQ  ,SIGNAL(pressed()), this, SLOT(rotest_computeConfigurations()) );
    connect(_btn_rotest_loadMarker  ,SIGNAL(pressed()), this, SLOT(rotest_loadMarker()) );
    connect(_slider_rotest_Q  ,SIGNAL(valueChanged(int)), this, SLOT(rotest_moveRobot()) );

    // robotics test tab - init values
    _rotest_coordinatesLoaded = false;


    // something
    Image textureImage(300,300,Image::GRAY,Image::Depth8U);
    _textureRender = new RenderImage(textureImage);
    Image bgImage(0,0,Image::GRAY,Image::Depth8U);
    _bgRender = new RenderImage(bgImage,2.5/1000.0);
    _framegrabber = NULL;

    // load absolute path
    std::ifstream ifs;
    ifs.open ("/home/.absolutepath.mypath", std::ifstream::in);

    if(ifs.is_open()){
        ifs >> _myPath;
        log().info() << "My path is: " << _myPath << "\n";
    }else{
        log().info() << "File could not be opened.\n";
    }
}

SamplePlugin::~SamplePlugin()
{
    delete _textureRender;
    delete _bgRender;
}

void SamplePlugin::initialize() {
    log().info() << "INITALIZE" << "\n";

    getRobWorkStudio()->stateChangedEvent().add(boost::bind(&SamplePlugin::stateChangedListener, this, _1), this);

    // Auto load workcell
    WorkCell::Ptr wc = WorkCellLoader::Factory::load(_myPath + "/finalProject/PA10WorkCell/ScenePA10RoVi1.wc.xml");
    getRobWorkStudio()->setWorkCell(wc);

    // Load Lena image
    Mat im, image;
    im = imread(_myPath + "/finalProject/SamplePluginPA10/src/lena.bmp", CV_LOAD_IMAGE_COLOR); // Read the file
    cvtColor(im, image, CV_BGR2RGB); // Switch the red and blue color channels
    if(! image.data ) {
        RW_THROW("Could not open or find the image: please modify the file path in the source code!");
    }
    QImage img(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888); // Create QImage from the OpenCV image
    _label->setPixmap(QPixmap::fromImage(img)); // Show the image at the label in the plugin
}

void SamplePlugin::open(WorkCell* workcell)
{
    log().info() << "OPEN" << "\n";
    _wc = workcell;
    _state = _wc->getDefaultState();

    log().info() << workcell->getFilename() << "\n";

    if (_wc != NULL) {
        // Add the texture render to this workcell if there is a frame for texture
        Frame* textureFrame = _wc->findFrame("MarkerTexture");
        if (textureFrame != NULL) {
            getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage",_textureRender,textureFrame);
        }
        // Add the background render to this workcell if there is a frame for texture
        Frame* bgFrame = _wc->findFrame("Background");
        if (bgFrame != NULL) {
            getRobWorkStudio()->getWorkCellScene()->addRender("BackgroundImage",_bgRender,bgFrame);
        }

        // Create a GLFrameGrabber if there is a camera frame with a Camera property set
        Frame* cameraFrame = _wc->findFrame("CameraSim");
        if (cameraFrame != NULL) {
            if (cameraFrame->getPropertyMap().has("Camera")) {
                // Read the dimensions and field of view
                double fovy;
                int width,height;
                std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
                std::istringstream iss (camParam, std::istringstream::in);
                iss >> fovy >> width >> height;
                // Create a frame grabber
                _framegrabber = new GLFrameGrabber(width,height,fovy);
                SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
                _framegrabber->init(gldrawer);
            }
        }
    }
}

void SamplePlugin::close() {
    log().info() << "CLOSE" << "\n";

    // Stop the timer
    _timer->stop();
    // Remove the texture render
    Frame* textureFrame = _wc->findFrame("MarkerTexture");
    if (textureFrame != NULL) {
        getRobWorkStudio()->getWorkCellScene()->removeDrawable("TextureImage",textureFrame);
    }
    // Remove the background render
    Frame* bgFrame = _wc->findFrame("Background");
    if (bgFrame != NULL) {
        getRobWorkStudio()->getWorkCellScene()->removeDrawable("BackgroundImage",bgFrame);
    }
    // Delete the old framegrabber
    if (_framegrabber != NULL) {
        delete _framegrabber;
    }
    _framegrabber = NULL;
    _wc = NULL;
}

Mat SamplePlugin::toOpenCVImage(const Image& img) {
    Mat res(img.getHeight(),img.getWidth(), CV_8SC3);
    res.data = (uchar*)img.getImageData();
    return res;
}

void SamplePlugin::btnPressed() {
    QObject *obj = sender();
    if(obj==_btn0){
        log().info() << "Button 0\n";
        // Set a new texture (one pixel = 1 mm)
        Image::Ptr image;
        image = ImageLoader::Factory::load(_myPath + "/finalProject/SamplePluginPA10/markers/Marker1.ppm");
        _textureRender->setImage(*image);
        image = ImageLoader::Factory::load(_myPath + "/finalProject/SamplePluginPA10/backgrounds/color1.ppm");
        _bgRender->setImage(*image);
        getRobWorkStudio()->updateAndRepaint();
    } else if(obj==_btn1){
        log().info() << "Button 1\n";
        // Toggle the timer on and off
        if (!_timer->isActive())
            _timer->start(100); // run 10 Hz
        else
            _timer->stop();
    } else if(obj==_spinBox){
        log().info() << "spin value:" << _spinBox->value() << "\n";
    }
}

void SamplePlugin::timer() {
    if (_framegrabber != NULL) {
        // Get the image as a RW image
        Frame* cameraFrame = _wc->findFrame("CameraSim");
        _framegrabber->grab(cameraFrame, _state);
        const Image& image = _framegrabber->getImage();

        // Convert to OpenCV image
        Mat im = toOpenCVImage(image);
        Mat imflip;
        cv::flip(im, imflip, 0);

        // Show in QLabel
        QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
        QPixmap p = QPixmap::fromImage(img);
        unsigned int maxW = 400;
        unsigned int maxH = 800;
        _label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
    }
}


void SamplePlugin::rotest_loadMarker(){
    QString filename = QFileDialog::getOpenFileName(this, "Open file", "", tr("Motion files (*.txt)"));

    _rotest_markerpos.clear();

    // load the data of the file into a vector with the pos (X Y Z Roll Pitch Yaw)

    std::ifstream inCSVFile(filename.toStdString(), std::ifstream::in);; // the input file

    if(inCSVFile.is_open()){
        rw::common::Log::log().info() << "File loaded: " << filename.toStdString() << "\n";
        point6D datapoint;

        while(!inCSVFile.eof()){
            std::string data;
            double number = 0;
            int i = 0;
            while(i < 6 && !inCSVFile.eof()){
                if(i == 5){
                    std::getline (inCSVFile, data);
                } else{
                    std::getline (inCSVFile, data, '\t');
                }
                std::istringstream(data) >> number;

                switch (i) {
                case 0:
                    datapoint.x = number;
                    break;
                case 1:
                    datapoint.y = number;
                    break;
                case 2:
                    datapoint.z = number;
                    break;
                case 3:
                    datapoint.roll = number;
                    break;
                case 4:
                    datapoint.pitch = number;
                    break;
                case 5:
                    datapoint.yaw = number;
                    break;
                default:
                    break;
                }
                i++;
            }
            _rotest_markerpos.emplace_back(datapoint);
        }

        _rotest_coordinatesLoaded = true;
    } else{
        _rotest_coordinatesLoaded = false;
        rw::common::Log::log().error() << "File could not be opened!\n";
    }
    inCSVFile.close();
}

void SamplePlugin::rotest_computeConfigurations(){
    // compute the robot configurations from the data
    if(_rotest_coordinatesLoaded){
        // get the different parameters and pointers
        const std::string deviceName = _line_settings_devName->text().toStdString();
        const std::string toolName = _line_settings_tcp->text().toStdString();

        rw::models::Device::Ptr device = _wc->findDevice(deviceName);
        rw::kinematics::Frame* tool = _wc->findFrame(toolName);
        if (device == NULL) RW_THROW("Device: " << deviceName << " not found!");
        if (tool == NULL) RW_THROW("Tool: " << toolName << " not found!");

        // resize according to the number of entries
        _rotest_robotQ.resize(_rotest_markerpos.size());

        for(int i = 0; i < _rotest_markerpos.size(); i++){

            // Get device to start configuration
            rw::kinematics::State state = _wc->getDefaultState();
            rw::math::Q q = device->getQ(state);

            // get the frames pos
            rw::math::Transform3D<double> T_wTtool = tool->wTf(state);

            // get the pos as seen in camera
            // goal as in world frame
            rw::math::RPY<double> rpy(_rotest_markerpos[i].roll, _rotest_markerpos[i].pitch, _rotest_markerpos[i].yaw);
            rw::math::Transform3D<double> T_wTgoal(rw::math::Vector3D<double>(_rotest_markerpos[i].x, _rotest_markerpos[i].y , _rotest_markerpos[i].z), rpy.toRotation3D());
            //            rw::math::Vector3D<double> dpos(((_rotest_markerpos[i]).x - (T_tool.P())[0]), ((_rotest_markerpos[i]).y - (T_tool.P())[1]), ((_rotest_markerpos[i]).z - (T_tool.P())[2]));

            // T_wTgoal = T_wTtool * T_toolTmarker
            rw::math::Transform3D<double> T_toolTmarker = T_wTtool;
            rw::math::Transform3D<double>::invMult(T_toolTmarker, T_wTgoal);

            /////////////////////////////////////// SOMETHING IS WRONG HERE, NOT THE RIGHT POS GAINED ///////////////////////////////////////////////////////////////////////////////
            rw::math::Vector3D<double> marker_pos = T_toolTmarker.P();

            double x = marker_pos[0];
            double y = marker_pos[1];
            double z = marker_pos[2];
            double f = 823;

            // J_image, image jacobian
            rw::math::Jacobian j_image = visualServoing::imageJacobian(x, y, z, f);

            // T_base_toll
            rw::math::Transform3D<double> T_base_tool = device->baseTframe(tool,state);

            // J(q), manipulator jacobian
            rw::math::Jacobian J_base_tool(6,6);
            J_base_tool = device->baseJframe(tool, state);

            // Z_image
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> z_img = visualServoing::z_image(j_image, T_base_tool.R(), J_base_tool);

            // inverting Z_image
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> z_inv = z_img;
            z_inv = rw::math::LinearAlgebra::pseudoInverse(z_inv);

            // converting du into eigen matrix to do multiplication
            rw::math::Vector2D<double> du = visualServoing::uv(x,y,z,f);
            Eigen::Matrix<double, 2, 1> du_eig;
            du_eig(0,0) = du(0);
            du_eig(1,0) = du(1);

            // compute dq
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> dq_eig = z_inv * du_eig;

            // convert dq into rw::math::Q
            rw::math::Q dq(q.size());

            for(int j = 0; j < q.size(); j++){
                dq[j] = dq_eig(j);
            }

            // find the new configuration
            q += dq;

            // set the state before calculating the next
            device->setQ(q, _state);
            getRobWorkStudio()->setState(_state);

            // find the marker
            std::string markerName = _line_settings_marker->text().toStdString();
            rw::kinematics::MovableFrame* marker = (rw::kinematics::MovableFrame*)(_wc->findFrame(markerName));
            if(marker == NULL) RW_THROW("Device: " << deviceName << " not found!");

            marker->setTransform(T_wTgoal, _state);
            getRobWorkStudio()->setState(_state);

            // store te configuration in the vector
            _rotest_robotQ[i] = q;

        }

        // set slider range according to the length of the vector
        if(_rotest_robotQ.size() > 0){
            _slider_rotest_Q->setRange(0,_rotest_robotQ.size() - 1);
        } else{
            _slider_rotest_Q->setRange(0,0);
        }
    }
}


void SamplePlugin::rotest_moveRobot(){
    // move the robot according to the calculates configurations
    if(_rotest_coordinatesLoaded){
        // set the configuration depending on the value of the marker
        int val = _slider_rotest_Q->value();

        // find device
        const std::string deviceName = _line_settings_devName->text().toStdString();
        rw::models::Device::Ptr device = _wc->findDevice(deviceName);
        if (device == NULL) RW_THROW("Device: " << deviceName << " not found!");

        // set the state
        rw::math::Q config;
        config = _rotest_robotQ[val];
        device->setQ(config, _state);
        getRobWorkStudio()->setState(_state);

        // find the marker
        std::string markerName = _line_settings_marker->text().toStdString();
        rw::kinematics::MovableFrame* marker = (rw::kinematics::MovableFrame*)(_wc->findFrame(markerName));
        if(marker == NULL) RW_THROW("Device: " << deviceName << " not found!");

        rw::math::Vector3D<double> P(_rotest_markerpos[val].x,_rotest_markerpos[val].y,_rotest_markerpos[val].z);
        rw::math::RPY<double> rpy(_rotest_markerpos[val].roll, _rotest_markerpos[val].pitch, _rotest_markerpos[val].yaw);
        rw::math::Rotation3D<double> R = rpy.toRotation3D();
        rw::math::Transform3D<double> T_wTmarker(P, R);

        marker->setTransform(T_wTmarker, _state);
        getRobWorkStudio()->setState(_state);

    }
}



void SamplePlugin::stateChangedListener(const State& state) {
    _state = state;
}

Q_EXPORT_PLUGIN(SamplePlugin);
