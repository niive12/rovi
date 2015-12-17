#include "SamplePlugin.hpp"

#include <fstream>      // std::fstream

void SamplePlugin::find_trackingerror(){
    // set the use time checkbox thingy
    _checkBox_settings_useProcessingTime->setChecked(true);

//    std::vector< std::string > filenames = {"circle.csv", "lines.csv", "corny.csv"};
//    std::vector< int > markerUsed = {0,1,3};
//    std::vector< double > ts = {0.053, 0.034, 0.366}, te = {0.09, 0.048, 0.42};
//    std::vector< double > dt = {0.00025, 0.00025, 0.001};
  std::vector< int > markerUsed = {3};
    std::vector< std::string > filenames = {"corny.csv"};
    std::vector< double > ts = {0.43}, te = {0.55};
    std::vector< double > dt = {0.02};
    int bg = 5;

    for(int mi = 0; mi < markerUsed.size(); mi++){
        int marker = markerUsed[mi];
        rw::common::Log::log().info() << _comboBox_settings_loadMarker->itemText(marker).toStdString() << " ";
    }

    _comboBox_rovi_background->setCurrentIndex(bg);
    rovi_load_bgImage();
    for(int mi = 0; mi < markerUsed.size(); mi++){
        std::fstream trackingerror_file (_myPath  + "/finalProject/" + filenames[mi], std::fstream::out);
        int marker = markerUsed[mi];
        // set the marker
        _comboBox_rovi_marker->setCurrentIndex(marker);
        rovi_load_markerImage();
        // find the timestep that still detects the marker
        double t = ts[mi];
        while(t <= te[mi]){
            _spinBox_timestep->setValue(t);
            // loop through the marker movements
            // fast, medium, slow
            trackingerror_file << t;
            for(int markerMovement = 0; markerMovement < _comboBox_settings_loadMarker->count(); markerMovement++){
                // load markermovement
                _comboBox_settings_loadMarker->setCurrentIndex(markerMovement);
                loadMarkerMovement();
                // process image
                rovi_processImage();
                // save the data
                if(!_rovi_markerNotTracked){
                    trackingerror_file << ", " << _rovi_maxTrackingError;
                } else{ // inf if it was not tracked.
                    trackingerror_file << ", +Inf";
                }
                trackingerror_file.flush();
            }
            trackingerror_file << "\n";
            t += dt[mi];
        }
        trackingerror_file.close();
    }
}

