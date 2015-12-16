#include "SamplePlugin.hpp"

#include <fstream>      // std::fstream

void SamplePlugin::find_limits(){
    // set the use time checkbox thingy
    _checkBox_settings_useProcessingTime->setChecked(true);

    std::fstream dt_file (_myPath  + "/finalProject/finddt.txt", std::fstream::out);
    std::fstream avgprocesstime (_myPath  + "/finalProject/findProcessTime.txt", std::fstream::out);


    for(int markerMovement = 0; markerMovement < _comboBox_settings_loadMarker->count(); markerMovement++){
        if(_comboBox_rovi_marker->currentText().lastIndexOf("Ideal") == (-1)){
            dt_file << " & " << _comboBox_settings_loadMarker->itemText(markerMovement).toStdString();
            avgprocesstime << " & " << _comboBox_settings_loadMarker->itemText(markerMovement).toStdString();
        }
    }
    dt_file << "\\\\ \n";
    avgprocesstime << "\\\\ \n";
    // loop through all markers
    for(int marker = 3; marker < _comboBox_rovi_marker->count(); marker++){
        // set the marker
        _comboBox_rovi_marker->setCurrentIndex(marker);
        if(_comboBox_rovi_marker->currentText().lastIndexOf("Ideal") == (-1)){
            //rw::common::Log::log().error() << _comboBox_rovi_marker->currentText().toStdString() << "\n";

            rovi_load_markerImage();
            // loop through all backgrounds
            for(int bg = 5; bg < _comboBox_rovi_background->count(); bg++){
                // set bg
                _comboBox_rovi_background->setCurrentIndex(bg);
                rovi_load_bgImage();
                // start the line:
                dt_file << _comboBox_rovi_background->currentText().toStdString();
                avgprocesstime << _comboBox_rovi_background->currentText().toStdString();
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
                    dt_file << " & " << dt;
                    avgprocesstime << " & " << _rovi_avgTrackingTime;
                }
                dt_file << "\\\\ \\hline \n";
                avgprocesstime << "\\\\ \\hline \n";
            }

        }
        dt_file << "\n\n";
        avgprocesstime << "\n\n";
    }
    dt_file.close();
    avgprocesstime.close();
}

