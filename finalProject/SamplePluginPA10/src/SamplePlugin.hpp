#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

#include "../build/ui_SamplePlugin.h"
#include "visualServoing.hpp"
#include "vis_marker01.hpp"
#include "vis_marker02.hpp"
#include "vis_marker03.hpp"

#include <opencv2/opencv.hpp>

#include <rws/RobWorkStudioPlugin.hpp>

#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rw/kinematics.hpp>

#include <QSlider>
#include <QString>
#include <QFileDialog>
#include <QCheckBox>

#include <chrono>
#include <ratio>
#include <string>
#include <fstream>      // std::ifstream
#include <vector>

struct point6D{
    double x, y, z, roll, pitch, yaw;
};

class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
public:
	SamplePlugin();
	virtual ~SamplePlugin();

	virtual void open(rw::models::WorkCell* workcell);

	virtual void close();

	virtual void initialize();

private slots:
    void updateCameraView();
	void timer();

	void stateChangedListener(const rw::kinematics::State& state);

    void loadMarkerMovement();

    rw::math::Q rotest_computeConfigurations(std::vector< cv::Point > &uv, std::vector< cv::Point > &mapping);
    void rotest_computeFakeUV(int points, std::vector< cv::Point > &uv);
    void rotest_moveRobot();

    void rovi_load_markerImage();
    void rovi_load_bgImage();
    void rovi_processImage();
    void rovi_saveData();


    void find_limits();


private:
	static cv::Mat toOpenCVImage(const rw::sensor::Image& img);

	QTimer* _timer;

	rw::models::WorkCell::Ptr _wc;
	rw::kinematics::State _state;
	rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
	rwlibs::simulation::GLFrameGrabber* _framegrabber;

    // abosulte path to the projectfolder
    std::string _myPath;

    // settings
    bool _settings_coordinatesLoaded;
    std::vector< point6D > _settings_markerpos;

    // other data to publish for plots
    std::vector< cv::Point > _trackingError;
    std::vector< rw::math::Transform3D<double> > _toolPose;
    std::vector< rw::math::Q > _dq_relative;

    // robot configurations
    std::vector< rw::math::Q > _robotQ;

    // vars for finding best
    bool _rovi_markerNotTracked;
    double _rovi_avgTrackingTime;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
