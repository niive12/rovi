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

    // robotics test tab - connect gui
    connect(_btn_rotest_computeQP1  ,SIGNAL(pressed()), this, SLOT(rotest_computeConfigurations()) );
    connect(_btn_rotest_computeQP3  ,SIGNAL(pressed()), this, SLOT(rotest_computeConfigurations()) );
    connect(_btn_rotest_loadMarker  ,SIGNAL(pressed()), this, SLOT(rotest_loadMarker()) );
    connect(_slider_rotest_Q        ,SIGNAL(valueChanged(int)), this, SLOT(rotest_moveRobot()) );
    connect(_comboBox_rovi_marker   ,SIGNAL(activated(int)), this, SLOT(rovi_load_markerImage()) );
    connect(_comboBox_rovi_background   ,SIGNAL(activated(int)), this, SLOT(rovi_load_bgImage()) );
    connect(_btn_rovi_processImage   ,SIGNAL(pressed()), this, SLOT(rovi_processImage()) );


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
    Mat res(img.getHeight(),img.getWidth(), CV_8UC3);
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
    QObject *obj = sender();
    // compute the robot configurations from the data
    if(_rotest_coordinatesLoaded){
        // get the different parameters and pointers
        const std::string deviceName = _line_settings_devName->text().toStdString();
        const std::string toolName = _line_settings_tcp->text().toStdString();

        rw::models::Device::Ptr device = _wc->findDevice(deviceName);
        rw::kinematics::Frame* tool = _wc->findFrame(toolName);
        if (device == NULL) RW_THROW("Device: " << deviceName << " not found!");
        if (tool == NULL) RW_THROW("Tool: " << toolName << " not found!");

        // set the state before starting
        rw::math::Q q(7, 0, -0.65, 0, 1.8, 0, 0.42, 0);
        device->setQ(q, _state);
        getRobWorkStudio()->setState(_state);

        // resize according to the number of entries
        _rotest_robotQ.resize(_rotest_markerpos.size());

        double dt = _spinBox_timestep->value();
        rw::common::Log::log().info() << " Timestep size used: " << dt << "\n";
//        rw::common::Log::log().info() << " Vel constraint: " << device->getVelocityLimits() << "\n";
        int constraintsapplied = 0;
        for(unsigned int i = 0; i < _rotest_markerpos.size(); i++){
            // Get device to start configuration
            q = device->getQ(_state);

            // get the frames pos
            rw::math::Transform3D<double> T_wTtool = tool->wTf(_state);

            // get the pos as seen in camera
            // goal as in world frame
            rw::math::RPY<double> rpy(_rotest_markerpos[i].roll, _rotest_markerpos[i].pitch, _rotest_markerpos[i].yaw);
            rw::math::Transform3D<double> T_wTgoal(rw::math::Vector3D<double>(_rotest_markerpos[i].x, _rotest_markerpos[i].y , _rotest_markerpos[i].z), rpy.toRotation3D());

            // T_wTgoal = T_wTtool * T_toolTmarker
            rw::math::Transform3D<double> T_toolTmarker = T_wTtool;
            rw::math::Transform3D<double>::invMult(T_toolTmarker, T_wTgoal);

            // z_actual to find u and v, but z_approx for actual visual servoing
            std::vector< double > x, y, z_actual, z_approx;
            std::vector< cv::Point > mapping;
            double f = 823;

            x.emplace_back(T_toolTmarker.P()[0]);
            y.emplace_back(T_toolTmarker.P()[1]);
            z_actual.emplace_back(T_toolTmarker.P()[2]);
            z_approx.emplace_back(-0.5);
            mapping.emplace_back(0, 0);

            if( obj == _btn_rotest_computeQP3 ){
                // add the point (0.1, 0, 0)
                T_toolTmarker = T_wTtool;
                rw::math::Transform3D<double>::invMult(T_toolTmarker, T_wTgoal * rw::math::Transform3D<double>(rw::math::Vector3D<double>(0.1, 0, 0), rw::math::Rotation3D<double>::identity()));
                x.emplace_back(T_toolTmarker.P()[0]);
                y.emplace_back(T_toolTmarker.P()[1]);
                z_actual.emplace_back(T_toolTmarker.P()[2]);
                z_approx.emplace_back(-0.5);
                mapping.emplace_back(160, 0);

                // add the point (0, 0.1, 0)
                T_toolTmarker = T_wTtool;
                rw::math::Transform3D<double>::invMult(T_toolTmarker, T_wTgoal * rw::math::Transform3D<double>(rw::math::Vector3D<double>(0, 0.1, 0), rw::math::Rotation3D<double>::identity()));
                x.emplace_back(T_toolTmarker.P()[0]);
                y.emplace_back(T_toolTmarker.P()[1]);
                z_actual.emplace_back(T_toolTmarker.P()[2]);
                z_approx.emplace_back(-0.5);
                mapping.emplace_back(0, 160);

//                T_toolTmarker = T_wTtool;
//                rw::math::Transform3D<double>::invMult(T_toolTmarker, T_wTgoal * rw::math::Transform3D<double>(rw::math::Vector3D<double>(0.1, 0.1, 0), rw::math::Rotation3D<double>::identity()));
//                x.emplace_back(-T_toolTmarker.P()[0]);
//                y.emplace_back(-T_toolTmarker.P()[1]);
//                z.emplace_back(-0.5);
//                mapping.emplace_back(20, 20);
            }


            rw::math::Q dq_temp, dq;
            // calculate the perfect u and v
            std::vector< rw::math::Vector2D<double> > uv = visualServoing::uv(x, y, z_actual, f);
            rw::common::Log::log().info() << "uv:\n";
            for(int i = 0; i < uv.size(); i++){
                rw::common::Log::log().info() << uv[i] << "\n";
            }
            // do the visual servoing
            dq_temp = visualServoing::visualServoing(uv, z_approx, f, device, tool, _state, mapping);

            if(visualServoing::velocityConstraint(dq_temp, device, dt, dq)){
                constraintsapplied++;
            }

            // find the new configuration
            q += dq;

            // set the state before calculating the next
            device->setQ(q, _state);

            // find the marker
            std::string markerName = _line_settings_marker->text().toStdString();
            rw::kinematics::MovableFrame* marker = (rw::kinematics::MovableFrame*)(_wc->findFrame(markerName));
            if(marker == NULL) RW_THROW("Device: " << deviceName << " not found!\n");

            marker->setTransform(T_wTgoal, _state);

//            marker = (rw::kinematics::MovableFrame*)(_wc->findFrame("Marker2"));
//            if(marker == NULL) RW_THROW("Device: " << "Marker2" << " not found!\n");rovi_processImage
//            marker->setTransform(T_wTgoal * rw::math::Transform3D<double>(rw::math::Vector3D<double>(0.1, 0, 0), rw::math::Rotation3D<double>::identity()), _state);
//            marker = (rw::kinematics::MovableFrame*)(_wc->findFrame("Marker3"));
//            if(marker == NULL) RW_THROW("Device: " << "Marker3" << " not found!\n");
//            marker->setTransform(T_wTgoal * rw::math::Transform3D<double>(rw::math::Vector3D<double>(0, 0.1, 0), rw::math::Rotation3D<double>::identity()), _state);

            // store te configuration in the vector
            _rotest_robotQ[i] = q;
        }

        rw::common::Log::log().info() << "# of velocity constraints applied: " << constraintsapplied << " out of " << _rotest_robotQ.size() << " movements.\n";

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

        // find the marker
        std::string markerName = _line_settings_marker->text().toStdString();
        rw::kinematics::MovableFrame* marker = (rw::kinematics::MovableFrame*)(_wc->findFrame(markerName));
        if(marker == NULL) RW_THROW("Device: " << deviceName << " not found!");

        rw::math::Vector3D<double> P(_rotest_markerpos[val].x,_rotest_markerpos[val].y,_rotest_markerpos[val].z);
        rw::math::RPY<double> rpy(_rotest_markerpos[val].roll, _rotest_markerpos[val].pitch, _rotest_markerpos[val].yaw);
        rw::math::Rotation3D<double> R = rpy.toRotation3D();
        rw::math::Transform3D<double> T_wTmarker(P, R);

        marker->setTransform(T_wTmarker, _state);

//        marker = (rw::kinematics::MovableFrame*)(_wc->findFrame("Marker2"));
//        if(marker == NULL) RW_THROW("Device: " << deviceName << " not found!\n");
//        marker->setTransform(T_wTmarker * rw::math::Transform3D<double>(rw::math::Vector3D<double>(0.1, 0, 0), rw::math::Rotation3D<double>::identity()), _state);
//        marker = (rw::kinematics::MovableFrame*)(_wc->findFrame("Marker3"));
//        if(marker == NULL) RW_THROW("Device: " << deviceName << " not found!\n");
//        marker->setTransform(T_wTmarker * rw::math::Transform3D<double>(rw::math::Vector3D<double>(0, 0.1, 0), rw::math::Rotation3D<double>::identity()), _state);
        getRobWorkStudio()->setState(_state);


    }
}

void SamplePlugin::rovi_load_markerImage(){
    int marker = _comboBox_rovi_marker->currentIndex();

    Image::Ptr image;
    switch (marker) {
    case 0: // marker 1
        image = ImageLoader::Factory::load(_myPath + "/finalProject/SamplePluginPA10/markers/Marker1.ppm");
        break;
    case 1: // marker 2a
        image = ImageLoader::Factory::load(_myPath + "/finalProject/SamplePluginPA10/markers/Marker2a.ppm");
        break;
    case 2: // marker 2b
        image = ImageLoader::Factory::load(_myPath + "/finalProject/SamplePluginPA10/markers/Marker2b.ppm");
        break;
    case 3: // marker 3
        image = ImageLoader::Factory::load(_myPath + "/finalProject/SamplePluginPA10/markers/Marker3.ppm");
        break;
    default:
        image = ImageLoader::Factory::load(_myPath + "/finalProject/SamplePluginPA10/markers/Marker1.ppm");
        break;
    }

    _textureRender->setImage(*image);
    getRobWorkStudio()->updateAndRepaint();
}

void SamplePlugin::rovi_load_bgImage(){
    int bg = _comboBox_rovi_background->currentIndex();

    Image::Ptr image;
    switch (bg) {
    case 0: // marker 1
        image = ImageLoader::Factory::load(_myPath + "/finalProject/SamplePluginPA10/backgrounds/color1.ppm");
        break;
    case 1: // marker 2a
        image = ImageLoader::Factory::load(_myPath + "/finalProject/SamplePluginPA10/backgrounds/color2.ppm");
        break;
    case 2: // marker 2b
        image = ImageLoader::Factory::load(_myPath + "/finalProject/SamplePluginPA10/backgrounds/color3.ppm");
        break;
    case 3: // marker 3
        image = ImageLoader::Factory::load(_myPath + "/finalProject/SamplePluginPA10/backgrounds/lines1.ppm");
        break;
    case 4: // marker 3
        image = ImageLoader::Factory::load(_myPath + "/finalProject/SamplePluginPA10/backgrounds/texture1.ppm");
        break;
    case 5: // marker 3
        image = ImageLoader::Factory::load(_myPath + "/finalProject/SamplePluginPA10/backgrounds/texture2.ppm");
        break;
    case 6: // marker 3
        image = ImageLoader::Factory::load(_myPath + "/finalProject/SamplePluginPA10/backgrounds/texture3.ppm");
        break;
    default:
        image = ImageLoader::Factory::load(_myPath + "/finalProject/SamplePluginPA10/backgrounds/color1.ppm");
        break;
    }

    _bgRender->setImage(*image);
    getRobWorkStudio()->updateAndRepaint();
}

void SamplePlugin::rovi_processImage(){
    std::vector< cv::Point > points;

    // Get the image as a RW image
    Frame* cameraFrame = _wc->findFrame("CameraSim");
    _framegrabber->grab(cameraFrame, _state);
    const Image& image = _framegrabber->getImage();

    // Convert to OpenCV image
    Mat im = toOpenCVImage(image);
    Mat img;
    cv::flip(im, img, 0);

    cv::cvtColor(img, img, CV_RGB2BGR);

    if(featureextraction::findMarker01(img, points)){
        rw::common::Log::log().info() << "Marker Found!\n";
//        for(int i = 0; i < points.size(); i++){
//            rw::common::Log::log().info()<< points[i] << "\n";
//        }
    } else{
        rw::common::Log::log().info() << "Marker Not Found!\n";
    }
}

void SamplePlugin::stateChangedListener(const State& state) {
    _state = state;
}

Q_EXPORT_PLUGIN(SamplePlugin)
