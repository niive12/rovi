#include "SamplePlugin.hpp"

//void find_limits(){
//    for(marker)
//            _comboBox_rovi_marker->currentText().toStdString() = "Marker 1";
//            //or
//            _comboBox_rovi_marker->currentIndex() = 0 , 1 , 3
//            rovi_load_markerImage();
//            for(background)
//                    _comboBox_rovi_background->currentText().toStdString();
//                    rovi_load_bgImage();
//                    for(time_step) //0.5 -> 1
//                                    for(marker_movement)
//                                            process_image(0);
//                                            if(lost_track_of_marker){
//                                                    std::cout << background << " & " << time_step << " & " << marker_movement << "\\\n";
//                                                    break;
//                                            }

//}

void SamplePlugin::find_limits(){
    // set the use time checkbox thingy
    _checkBox_settings_useProcessingTime->setChecked(true);

    for(int markerMovement = 0; markerMovement < _comboBox_settings_loadMarker->count(); markerMovement++){
        rw::common::Log::log().error() << _comboBox_settings_loadMarker->currentText().toStdString() << " ";
    }
    rw::common::Log::log().error() << "\n";
    // loop through all markers
    for(int marker = 0; marker < _comboBox_rovi_marker->count(); marker++){
        rw::common::Log::log().error() << _comboBox_rovi_marker->currentText().toStdString() << "\n";
        // set the marker
        _comboBox_rovi_marker->setCurrentIndex(marker);
        rovi_load_markerImage();
        // loop through all backgrounds
        for(int bg = 0; bg < _comboBox_rovi_background->count(); bg++){
            // set bg
            _comboBox_rovi_background->setCurrentIndex(bg);
            rovi_load_bgImage();
            // start the line:
            rw::common::Log::log().error() << _comboBox_rovi_background->currentText().toStdString();
            // loop through the marker movements
            for(int markerMovement = 0; markerMovement < _comboBox_settings_loadMarker->count(); markerMovement++){
                // load markermovement
                _comboBox_settings_loadMarker->setCurrentIndex(markerMovement);
                loadMarkerMovement();
                // find the timestep that still detects the marker
                double dt = 0.0;
                do{
                    dt += 0.05;
                    _spinBox_timestep->setValue(dt);
                    rovi_processImage();
                } while(dt < 1 && _rovi_markerNotTracked);
                rw::common::Log::log().error() << " & " << dt << " avg t:" << _rovi_avgTrackingTime;
            }
            rw::common::Log::log().error() << "\\\\ \\hline \n";
        }

    }

}
