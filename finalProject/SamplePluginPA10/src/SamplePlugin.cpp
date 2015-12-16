#include "SamplePlugin.hpp"

#include <rws/RobWorkStudio.hpp>

#include <QPushButton>

#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>

SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
    setupUi(this);

    _timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

    // robotics test tab - connect gui
    connect(_comboBox_settings_loadMarker           ,SIGNAL(activated(int)), this, SLOT(loadMarkerMovement()) );
    connect(_slider_rotest_Q                        ,SIGNAL(valueChanged(int)), this, SLOT(rotest_moveRobot()) );
    connect(_comboBox_rovi_marker                   ,SIGNAL(activated(int)), this, SLOT(rovi_load_markerImage()) );
    connect(_comboBox_rovi_background               ,SIGNAL(activated(int)), this, SLOT(rovi_load_bgImage()) );
    connect(_btn_rovi_processImage                  ,SIGNAL(pressed()), this, SLOT(rovi_processImage()) );
    connect(_checkBox_settings_updateCameraview     ,SIGNAL(clicked()), this, SLOT(updateCameraView()) );
    connect(_btn_rovi_saveData                      ,SIGNAL(pressed()), this, SLOT(rovi_saveData()) );
    connect(_btn_rovi_finddt                        ,SIGNAL(pressed()), this, SLOT(find_limits()) );


    // robotics test tab - init values
    _settings_coordinatesLoaded = false;

    // something
    rw::sensor::Image textureImage(300,300,rw::sensor::Image::GRAY,rw::sensor::Image::Depth8U);
    _textureRender = new rwlibs::opengl::RenderImage(textureImage);
    rw::sensor::Image bgImage(0,0,rw::sensor::Image::GRAY,rw::sensor::Image::Depth8U);
    _bgRender = new rwlibs::opengl::RenderImage(bgImage,2.5/1000.0);
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
    loadMarkerMovement();
    _checkBox_settings_updateCameraview->setChecked(true);
    updateCameraView();
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
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(_myPath + "/finalProject/PA10WorkCell/ScenePA10RoVi1.wc.xml");
    getRobWorkStudio()->setWorkCell(wc);

    // Load Lena image
    cv::Mat im, image;
    im = cv::imread(_myPath + "/finalProject/SamplePluginPA10/src/lena.bmp", CV_LOAD_IMAGE_COLOR); // Read the file
    cv::cvtColor(im, image, CV_BGR2RGB); // Switch the red and blue color channels
    if(! image.data ) {
        RW_THROW("Could not open or find the image: please modify the file path in the source code!");
    }
    int w = _label->width()/2;
    int h = _label->height()/2;
    QImage img(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888); // Create QImage from the OpenCV image
    //    _label->setPixmap(QPixmap::fromImage(img)); // Show the image at the label in the plugin
    _label->setPixmap( QPixmap::fromImage(img).scaled(w,h,Qt::KeepAspectRatio) );
}

void SamplePlugin::open(rw::models::WorkCell* workcell)
{
    log().info() << "OPEN" << "\n";
    _wc = workcell;
    _state = _wc->getDefaultState();

    log().info() << workcell->getFilename() << "\n";

    if (_wc != NULL) {
        // Add the texture render to this workcell if there is a frame for texture
        rw::kinematics::Frame* textureFrame = _wc->findFrame("MarkerTexture");
        if (textureFrame != NULL) {
            getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage",_textureRender,textureFrame);
        }
        // Add the background render to this workcell if there is a frame for texture
        rw::kinematics::Frame* bgFrame = _wc->findFrame("Background");
        if (bgFrame != NULL) {
            getRobWorkStudio()->getWorkCellScene()->addRender("BackgroundImage",_bgRender,bgFrame);
        }

        // Create a GLFrameGrabber if there is a camera frame with a Camera property set
        rw::kinematics::Frame* cameraFrame = _wc->findFrame("CameraSim");
        if (cameraFrame != NULL) {
            if (cameraFrame->getPropertyMap().has("Camera")) {
                // Read the dimensions and field of view
                double fovy;
                int width,height;
                std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
                std::istringstream iss (camParam, std::istringstream::in);
                iss >> fovy >> width >> height;
                // Create a frame grabber
                _framegrabber = new rwlibs::simulation::GLFrameGrabber(width,height,fovy);
                rw::graphics::SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
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
    rw::kinematics::Frame* textureFrame = _wc->findFrame("MarkerTexture");
    if (textureFrame != NULL) {
        getRobWorkStudio()->getWorkCellScene()->removeDrawable("TextureImage",textureFrame);
    }
    // Remove the background render
    rw::kinematics::Frame* bgFrame = _wc->findFrame("Background");
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

cv::Mat SamplePlugin::toOpenCVImage(const rw::sensor::Image& img) {
    cv::Mat res(img.getHeight(),img.getWidth(), CV_8UC3);
    res.data = (uchar*)img.getImageData();
    return res;
}

void SamplePlugin::updateCameraView() {
    // Toggle the timer on and off
    if(_checkBox_settings_updateCameraview->isChecked()){
        if (!_timer->isActive()){
            _timer->start(100); // run 10 Hz
        }
    } else{
        _timer->stop();
    }
}

void SamplePlugin::timer() {
    if (_framegrabber != NULL) {
        // Get the image as a RW image
        rw::kinematics::Frame* cameraFrame = _wc->findFrame("CameraSim");
        _framegrabber->grab(cameraFrame, _state);
        const rw::sensor::Image& image = _framegrabber->getImage();

        // Convert to OpenCV image
        cv::Mat im = toOpenCVImage(image);
        cv::Mat imflip;
        cv::flip(im, imflip, 0);

        // Show in QLabel
        QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
        QPixmap p = QPixmap::fromImage(img);
        unsigned int maxW = 200;
        unsigned int maxH = 800;
        _label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
    }
}


void SamplePlugin::loadMarkerMovement(){
    //QString filename = QFileDialog::getOpenFileName(this, "Open file", "", tr("Motion files (*.txt)"));

    _settings_markerpos.clear();

    // load the data of the file into a vector with the pos (X Y Z Roll Pitch Yaw)
    QString file = "";
    int index = _comboBox_settings_loadMarker->currentIndex();

    switch (index) {
    case 2:
        file = QString::fromStdString(_myPath) + "/finalProject/SamplePluginPA10/motions/MarkerMotionSlow.txt";
        break;
    case 1:
        file = QString::fromStdString(_myPath) + "/finalProject/SamplePluginPA10/motions/MarkerMotionMedium.txt";
        break;
    case 0:
        file = QString::fromStdString(_myPath) + "/finalProject/SamplePluginPA10/motions/MarkerMotionFast.txt";
        break;
    default:
        file = QString::fromStdString(_myPath) + "/finalProject/SamplePluginPA10/motions/MarkerMotionSlow.txt";
        break;
    }


    std::ifstream inCSVFile(file.toStdString(), std::ifstream::in);; // the input file

    if(inCSVFile.is_open()){
        rw::common::Log::log().info() << "File loaded: " << file.toStdString() << "\n";
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
            _settings_markerpos.emplace_back(datapoint);
        }

        _settings_coordinatesLoaded = true;
    } else{
        _settings_coordinatesLoaded = false;
        rw::common::Log::log().error() << "File could not be opened!\n";
    }
    inCSVFile.close();
}

void SamplePlugin::rotest_computeFakeUV(int points, std::vector< cv::Point > &uv){
    const std::string toolName = _line_settings_tcp->text().toStdString();
    rw::kinematics::Frame* tool = _wc->findFrame(toolName);
    if (tool == NULL) RW_THROW("Tool: " << toolName << " not found!");


    // get the frames pos
    rw::math::Transform3D<double> T_wTtool = tool->wTf(_state);

    // get the pos as seen in camera
    // goal as in world frame
    const std::string markerName = _line_settings_marker->text().toStdString();
    rw::kinematics::Frame* marker = _wc->findFrame(markerName);
    if (marker == NULL) RW_THROW("Tool: " << markerName << " not found!");

    rw::math::Transform3D<double> T_wTmarker = marker->wTf(_state);

    // T_wTgoal = T_wTtool * T_toolTmarker
    rw::math::Transform3D<double> T_toolTmarker = T_wTtool;
    rw::math::Transform3D<double>::invMult(T_toolTmarker, T_wTmarker);

    // z_actual to find u and v, but z_approx for actual visual servoing
    std::vector< double > x, y, z_actual;
    double f = 823;


    x.emplace_back(T_toolTmarker.P()[0]);
    y.emplace_back(T_toolTmarker.P()[1]);
    z_actual.emplace_back(-T_toolTmarker.P()[2]);

    if( points >= 2 ){
        // add the point (0.1, 0, 0)
        T_toolTmarker = T_wTtool;
        rw::math::Transform3D<double>::invMult(T_toolTmarker, T_wTmarker * rw::math::Transform3D<double>(rw::math::Vector3D<double>(0.1, 0, 0), rw::math::Rotation3D<double>::identity()));
        x.emplace_back(T_toolTmarker.P()[0]);
        y.emplace_back(T_toolTmarker.P()[1]);
        z_actual.emplace_back(-T_toolTmarker.P()[2]);

        if( points == 3){
            // add the point (0, 0.1, 0)
            T_toolTmarker = T_wTtool;
            rw::math::Transform3D<double>::invMult(T_toolTmarker, T_wTmarker * rw::math::Transform3D<double>(rw::math::Vector3D<double>(0, 0.1, 0), rw::math::Rotation3D<double>::identity()));
            x.emplace_back(T_toolTmarker.P()[0]);
            y.emplace_back(T_toolTmarker.P()[1]);
            z_actual.emplace_back(-T_toolTmarker.P()[2]);
        }
    }

//    rw::common::Log::log().info() << "Z: ";
//    for(int i = 0; i < z_actual.size(); i++){
//        rw::common::Log::log().info() << z_actual[i] << ", ";
//    }
//    rw::common::Log::log().info() << "\n";

    // calculate the perfect u and v
    uv = visualServoing::uv(x, y, z_actual, f);
}


void SamplePlugin::rovi_saveData(){
    std::string filepath = _myPath  + "/finalProject/", filename_tool = "toolPose.csv", filename_error = "trackingError.csv", filename_q = "robotConfiguration.csv", filename_dq = "relativeConfVel.csv";

    std::ofstream outCSVFile_tool(filepath + filename_tool, std::ofstream::out); // the input file
    if(outCSVFile_tool.is_open()){
        for(unsigned int i = 0; i < _toolPose.size();i++){
            rw::math::RPY<double> rot(_toolPose[i].R());
            outCSVFile_tool << _toolPose[i].P()[0] << ", " << _toolPose[i].P()[1] << ", " << _toolPose[i].P()[2] << ", " << rot[0] << ", " << rot[1] << ", " << rot[2] << "\n";
        }
    }
    outCSVFile_tool.close();

    std::ofstream outCSVFile_error(filepath + filename_error, std::ofstream::out); // the input file
    if(outCSVFile_error.is_open()){
        for(unsigned int i = 0; i < _trackingError.size();i++){
            outCSVFile_error << _trackingError[i].x << ", " << _trackingError[i].y << "\n";
        }
    }
    outCSVFile_error.close();


    std::ofstream outCSVFile_q(filepath + filename_q, std::ofstream::out); // the input file
    if(outCSVFile_q.is_open()){
        for(unsigned int i = 0; i < _toolPose.size();i++){
            outCSVFile_q << _robotQ[i][0];
            for(unsigned int j = 1; j < _robotQ[i].size(); j++){
                outCSVFile_q << ", " << _robotQ[i][j];
            }
            outCSVFile_q << "\n";
        }
    }
    outCSVFile_q.close();

    std::ofstream outCSVFile_dq(filepath + filename_dq, std::ofstream::out); // the input file
    if(outCSVFile_dq.is_open()){
        for(unsigned int i = 0; i < _dq_relative.size();i++){
            outCSVFile_dq << _dq_relative[i][0];
            for(unsigned int j = 1; j < _dq_relative[i].size(); j++){
                outCSVFile_dq << ", " << _dq_relative[i][j];
            }
            outCSVFile_dq << "\n";
        }
    }
    outCSVFile_q.close();


}

rw::math::Q SamplePlugin::rotest_computeConfigurations(std::vector< cv::Point > &uv, std::vector< cv::Point > &mapping){
    // compute the robot configurations from the data
    // get the different parameters and pointers
    const std::string deviceName = _line_settings_devName->text().toStdString();
    rw::models::Device::Ptr device = _wc->findDevice(deviceName);
    if (device == NULL) RW_THROW("Device: " << deviceName << " not found!");

    const std::string toolName = _line_settings_tcp->text().toStdString();
    rw::kinematics::Frame* tool = _wc->findFrame(toolName);
    if (tool == NULL) RW_THROW("Tool: " << toolName << " not found!");

    // Get device to start configuration
    rw::math::Q dq_temp, q;
    q = device->getQ(_state);

    // z_actual to find u and v, but z_approx for actual visual servoing
    double f = 823;

    std::vector< double > z_approx = {};
    for(size_t i = 0; i < uv.size(); i++){
        z_approx.emplace_back(-0.5);
    }
    // do the visual servoing
    dq_temp = visualServoing::visualServoing(uv, z_approx, f, device, tool, _state, mapping);

    return dq_temp;
}


void SamplePlugin::rotest_moveRobot(){
    // move the robot according to the calculates configurations
    if(_settings_coordinatesLoaded){
        // set the configuration depending on the value of the marker
        int val = _slider_rotest_Q->value();

        // find device
        const std::string deviceName = _line_settings_devName->text().toStdString();
        rw::models::Device::Ptr device = _wc->findDevice(deviceName);
        if (device == NULL) RW_THROW("Device: " << deviceName << " not found!");

        // set the state
        rw::math::Q config;
        config = _robotQ[val];
        device->setQ(config, _state);

        // find the marker
        std::string markerName = _line_settings_marker->text().toStdString();
        rw::kinematics::MovableFrame* marker = (rw::kinematics::MovableFrame*)(_wc->findFrame(markerName));
        if(marker == NULL) RW_THROW("Device: " << deviceName << " not found!");

        rw::math::Vector3D<double> P(_settings_markerpos[val].x,_settings_markerpos[val].y,_settings_markerpos[val].z);
        rw::math::RPY<double> rpy(_settings_markerpos[val].roll, _settings_markerpos[val].pitch, _settings_markerpos[val].yaw);
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
    std::string marker = _comboBox_rovi_marker->currentText().toStdString();

    rw::sensor::Image::Ptr image;
    if (marker == "Marker 1"){
        image = rw::loaders::ImageLoader::Factory::load(_myPath + "/finalProject/SamplePluginPA10/markers/Marker1.ppm");
    } else if( marker == "Marker 2a"){
        image = rw::loaders::ImageLoader::Factory::load(_myPath + "/finalProject/SamplePluginPA10/markers/Marker2a.ppm");
    } else if( marker == "Marker 2b"){
        image = rw::loaders::ImageLoader::Factory::load(_myPath + "/finalProject/SamplePluginPA10/markers/Marker2b.ppm");
    } else if( marker == "Marker 3"){
        image = rw::loaders::ImageLoader::Factory::load(_myPath + "/finalProject/SamplePluginPA10/markers/Marker3.ppm");
    } else {
        image = rw::loaders::ImageLoader::Factory::load(_myPath + "/finalProject/SamplePluginPA10/markers/Marker1.ppm");
    }

    _textureRender->setImage(*image);
    getRobWorkStudio()->updateAndRepaint();
}

void SamplePlugin::rovi_load_bgImage(){
    std::string bg = _comboBox_rovi_background->currentText().toStdString();
    bool setimage = true;

    rw::sensor::Image::Ptr image;
    if (bg == "Many Butterflies"){
        image = rw::loaders::ImageLoader::Factory::load(_myPath + "/finalProject/SamplePluginPA10/backgrounds/color1.ppm");
    } else if( bg == "Color Spots"){
        image = rw::loaders::ImageLoader::Factory::load(_myPath + "/finalProject/SamplePluginPA10/backgrounds/color2.ppm");
    } else if( bg == "One Butterfly"){
        image = rw::loaders::ImageLoader::Factory::load(_myPath + "/finalProject/SamplePluginPA10/backgrounds/color3.ppm");
    } else if( bg == "Metal"){
        image = rw::loaders::ImageLoader::Factory::load(_myPath + "/finalProject/SamplePluginPA10/backgrounds/lines1.ppm");
    } else if( bg == "Carpet"){
        image = rw::loaders::ImageLoader::Factory::load(_myPath + "/finalProject/SamplePluginPA10/backgrounds/texture1.ppm");
    } else if( bg == "Mosaik Window"){
        image = rw::loaders::ImageLoader::Factory::load(_myPath + "/finalProject/SamplePluginPA10/backgrounds/texture2.ppm");
    } else if( bg == "Waterbed"){
        image = rw::loaders::ImageLoader::Factory::load(_myPath + "/finalProject/SamplePluginPA10/backgrounds/texture3.ppm");
    } else {
        if(_bgRender != nullptr){
            delete _bgRender;
            _bgRender = nullptr;
        }
        rw::sensor::Image bgImage(0,0,rw::sensor::Image::GRAY,rw::sensor::Image::Depth8U);
        _bgRender = new rwlibs::opengl::RenderImage(bgImage,2.5/1000.0);
        setimage = false;
    }

    if(setimage){
        _bgRender->setImage(*image);
    }
    getRobWorkStudio()->updateAndRepaint();
}

void SamplePlugin::rovi_processImage(){

    if(_settings_coordinatesLoaded){
        const std::string deviceName = _line_settings_devName->text().toStdString();
        rw::models::Device::Ptr device = _wc->findDevice(deviceName);
        if (device == NULL) RW_THROW("Device: " << deviceName << " not found!");

        // set the state before starting
        rw::math::Q q(7, 0, -0.65, 0, 1.8, 0, 0.42, 0);
        device->setQ(q, _state);

        // use time for something
        bool reduceProcessingTime = _checkBox_settings_useProcessingTime->isChecked();

        // get the timestep
        double dt = _spinBox_timestep->value();

        // set values for automatic find dt
        _rovi_markerNotTracked = false;

        // resize according to the number of entries
        _robotQ.resize(_settings_markerpos.size());
        _trackingError.resize(_settings_markerpos.size());
        _toolPose.resize(_settings_markerpos.size());
        _dq_relative.resize(_settings_markerpos.size());

        // find the marker
        std::string markerName = _line_settings_marker->text().toStdString();
        rw::kinematics::MovableFrame* marker_obj = (rw::kinematics::MovableFrame*)(_wc->findFrame(markerName));
        if(marker_obj == NULL) RW_THROW("Device: " << deviceName << " not found!\n");

        rw::math::Q q_goal, dq, q_next;
        q_goal = q;

        // crop image before getting matches (used for marker 03)
        cv::Point old_position(0,0);
        int accepted_width;
        int accepted_height;
        cv::Mat img_object = cv::imread(_myPath + "/finalProject/SamplePluginPA10/markers/Marker3.ppm");
        featureextraction::init_marker03(img_object);

        rw::math::Q vC = device->getVelocityLimits();
        rw::common::Log::log().info() << "vel lim " << vC << "\n";
        int constraintsapplied = 0;
        int markerNotFound = 0;
        std::chrono::high_resolution_clock::time_point t1, t2;
        double totalTime = 0;
        int n_times = 1;
        if( _comboBox_settings_loadMarker->currentIndex() == 2) {// slow
            n_times = 1;
        }else if( _comboBox_settings_loadMarker->currentIndex() == 1) {// medium
            n_times = 3;
        }else if( _comboBox_settings_loadMarker->currentIndex() == 0) {// fast
            n_times = 1;
            rw::common::Log::log().info() << "I have set the n_times to 1 on fast so it can be tested faster\n";
        }
        for(int trial = 0; trial < n_times; ++trial) {
            for(unsigned int i = 0; i < _settings_markerpos.size(); i++){
                // set the marker as in world frame
                rw::math::RPY<double> rpy(_settings_markerpos[i].roll, _settings_markerpos[i].pitch, _settings_markerpos[i].yaw);
                rw::math::Transform3D<double> T_wTmarker(rw::math::Vector3D<double>(_settings_markerpos[i].x, _settings_markerpos[i].y , _settings_markerpos[i].z), rpy.toRotation3D());
                marker_obj->setTransform(T_wTmarker, _state);

                int markerused = _comboBox_rovi_marker->currentIndex();
                std::vector< cv::Point > mapping, uv;
                uv.clear();
                mapping.clear();

                bool markerFound = false;

                t1 = std::chrono::high_resolution_clock::now();
                if(markerused >= 0 && markerused <=3){ // if tracking an image
                    // Get the image as a RW image
                    getRobWorkStudio()->setState(_state);
                    rw::kinematics::Frame* cameraFrame = _wc->findFrame("CameraSim");
                    _framegrabber->grab(cameraFrame, _state);
                    const rw::sensor::Image& image = _framegrabber->getImage();

                    // Convert to OpenCV image
                    cv::Mat im = toOpenCVImage(image);
                    cv::Mat img;
                    cv::flip(im, img, 0);

                    cv::cvtColor(img, img, CV_RGB2BGR);

                    if(markerused == 0){ // pic 1
                        markerFound = featureextraction::findMarker01(img, uv, false);
                        if(!markerFound){
                            cv::imshow("marker not found", img);
                            cv::waitKey(1);
                        }
                        if(uv.size() == 1){
                            mapping.emplace_back(0,0);
                        }else if(uv.size() > 1){
                            mapping.emplace_back(85,85);
                            mapping.emplace_back(85,-85);
                            mapping.emplace_back(-85,85);
                            mapping.emplace_back(-85,-85);
                        }
                    } else if(markerused == 1){ // pic 2a
                        markerFound = featureextraction::findMarker02(img, uv);
                        if(!markerFound){
                            cv::imshow("marker not found", img);
                            cv::waitKey(1);
                        }
                        mapping.emplace_back(0,0);
                    } else if(markerused == 2){ // pic 2b
                        // ---- this algorithm was not implemented
                        markerFound = false;
                        mapping.emplace_back(0,0);
                    } else if(markerused == 3){ // pic 3
                        int x = 0,y = 0;
                        accepted_width = img.cols;
                        accepted_height = img.rows;
                        if( old_position != cv::Point(0,0) ){
                            accepted_width = img_object.cols * 2;
                            accepted_height = img_object.rows * 2;
                            x = old_position.x - accepted_width/2;
                            y = old_position.y - accepted_height/2;
                            if(x > (img.cols-accepted_width) ){
                                x = img.cols-accepted_width;
                            } else if(x < 0){
                                x = 0;
                            }
                            if(y > (img.rows-accepted_height) ){
                                y = img.rows-accepted_height;
                            } else if(y < 0){
                                y = 0;
                            }
                        }
                        cv::Mat cropped(img, cv::Rect(x,y,accepted_width,accepted_height));

                        markerFound = featureextraction::findMarker03(cropped, uv, false);
                        if( !markerFound ){
                            old_position = cv::Point(0,0);
                        } else if(uv.size() > 0 ){
                            old_position.y = 0;
                            old_position.x = 0;
                            for(unsigned int j = 0; j < uv.size(); j++){
                                old_position.x += uv[j].x + x;
                                old_position.y += uv[j].y + y;
                            }
                            old_position.y /= uv.size();
                            old_position.x /= uv.size();
                        }
                        // adjust uv to standard
                        for(unsigned int p = 0; p < uv.size(); p++){
                            uv[p].x = uv[p].x - img.cols / 2 + x;
                            uv[p].y = img.rows / 2 - uv[p].y - y;
                            //                        rw::common::Log::log().info() << uv[p];
                        }
                        //                    rw::common::Log::log().info() << "\n";

                        if(uv.size() == 1){
                            mapping.emplace_back(0,0);
                        }else if(uv.size() > 1){
                            mapping.emplace_back(212,212);
                            mapping.emplace_back(212,-212);
                            mapping.emplace_back(-212,212);
                            mapping.emplace_back(-212,-212);
                        }

                    }

                } else if(markerused >= 4 && markerused <=6){ // if tracking perfect coords

                    rotest_computeFakeUV(markerused - 3, uv);

                    mapping.emplace_back(0,0);
                    if(markerused - 3 >= 2){
                        mapping.emplace_back(160,0);
                        if(markerused - 3 >= 3){
                            mapping.emplace_back(0,160);
                        }
                    }
                    markerFound = true;

                }else{
                    rw::common::Log::log().info() << "Invalid marker selected\n";
                    markerFound = false;
                }
                t2 = std::chrono::high_resolution_clock::now();
                double algoTime = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
                rw::common::Log::log().info() << "process time: " << algoTime << "\n";
                totalTime += algoTime;
                double robotMoveTime = dt;
                if(reduceProcessingTime){
                    robotMoveTime -= algoTime;
                    if(i == 1 && robotMoveTime < -0.05 ){
                        rw::common::Log::log().info() << "proccess time is : " << algoTime << ", which exceeds Dt\n";
                        _rovi_markerNotTracked = true;
                        return;
                    }
                }
                if(uv.size() > 0 && markerFound && robotMoveTime > 0){
                    // if points where found, use them to compute the new q_goal
                    dq = rotest_computeConfigurations(uv, mapping);
                    q_goal = dq + device->getQ(_state);
                    //                rw::common::Log::log().info() << dq << "\n";
                } else{

                    markerNotFound++;
                }
                // limit dq, calc dq a new for cases where no dq was found but it still has to move from last calculation
                dq = q_goal - device->getQ(_state);
                rw::math::Q dq_new = dq;
                if(robotMoveTime > 0){
                    if(visualServoing::velocityConstraint(dq, device, robotMoveTime, dq_new)){
                        constraintsapplied++;
                    }

                    // move to next q
                    q_next = dq_new + device->getQ(_state);

                    // set the state before calculating the next
                    device->setQ(q_next, _state);

                } else{
                    // set q's to 0
                    q_next = device->getQ(_state);
                }
                // calculate the tracking error
                std::vector< cv::Point > trackingerror;
                rotest_computeFakeUV(1, trackingerror);
                _trackingError[i] = trackingerror[0];

                if(abs(trackingerror[0].x) > 1024/2 || abs(trackingerror[0].y) > 768/2){
                    _rovi_markerNotTracked = true;
                }

                // calculate the tool pose
                rw::math::Transform3D<double> T_wTm = marker_obj->wTf(_state);
                _toolPose[i] = T_wTm;

                // calculate the dq/vC
                _dq_relative[i] = dq_new;
                for(size_t k = 0; k < dq_new.size(); k++){
                    _dq_relative[i](k) = fabs((_dq_relative[i](k) / dt) / vC(k));
                }
                // store the configuration in the vector
                _robotQ[i] = q_next;
            }
            // return to start pose
            device->setQ(q, _state);
            getRobWorkStudio()->setState(_state);
        }
        _rovi_avgTrackingTime = totalTime / ( _robotQ.size() * n_times );
        rw::common::Log::log().info() << "# of constraint applied: " << constraintsapplied / n_times << " / " << _robotQ.size() << "\n";
        rw::common::Log::log().info() << "# of markers not found : " << markerNotFound / n_times << " / " << _robotQ.size() << "\n";
        rw::common::Log::log().info() << "Mean Img. Process time : " << _rovi_avgTrackingTime << "\n";


        // set slider range according to the length of the vector
        if(_robotQ.size() > 0){
            _slider_rotest_Q->setRange(0,_robotQ.size() - 1);
        } else{
            _slider_rotest_Q->setRange(0,0);
        }

    }
}

void SamplePlugin::stateChangedListener(const rw::kinematics::State& state) {
    _state = state;
}

Q_EXPORT_PLUGIN(SamplePlugin)
