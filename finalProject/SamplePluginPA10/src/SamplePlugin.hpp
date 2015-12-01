#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

#include "../build/ui_SamplePlugin.h"
#include "visualServoing.hpp"

#include <opencv2/opencv.hpp>

#include <rws/RobWorkStudioPlugin.hpp>

#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>

#include <QSlider>
#include <QString>
#include <QFileDialog>

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
	void btnPressed();
	void timer();

	void stateChangedListener(const rw::kinematics::State& state);


    void rotest_loadMarker();
    void rotest_computeConfigurations();
    void rotest_moveRobot();


private:
	static cv::Mat toOpenCVImage(const rw::sensor::Image& img);

	QTimer* _timer;

	rw::models::WorkCell::Ptr _wc;
	rw::kinematics::State _state;
	rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
	rwlibs::simulation::GLFrameGrabber* _framegrabber;

    // abosulte path to the projectfolder
    std::string _myPath;

    // rotest
    bool _rotest_coordinatesLoaded;
    std::vector< point6D > _rotest_markerpos;
    std::vector< rw::math::Q > _rotest_robotQ;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
