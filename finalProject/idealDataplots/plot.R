require(stats); 
require(graphics);
require(grDevices);

report_path = "../report/graphics/robotics/";

# name of data tests
datanames = c("robotConfiguration", "toolPose", "trackingError","relativeConfVel");
# name of the speeds and their dt
speeds = c("slow", "medium", "fast");
timeintervals = c(0.05, 0.05, 0.05);
# method used
methods = c("1pt", "3pt");
# file extension
fileextension = ".csv";
# limits for relative vel graphs
velLim = c(0.12,0.28,0.48)

# colors for plotting
color = rainbow(7);

# plot robot config and tracking error
for( dataplot in c(1,2,3,4)){
    for( dataspeed in 1:length(speeds)){
        for( method in 1:length(methods)){
            # prepare name to load file and start of filename for saving
            filename_plot = paste(c(datanames[dataplot], "_", speeds[dataspeed], "_", methods[method]), collapse ="");
            filename_data = paste(c(filename_plot, fileextension), collapse ="");
            #print(filename_data);
            
            # load the data
            data = read.csv(filename_data);
            s = dim(data);
            #print(s);
            maxT = timeintervals[dataspeed] * s[1];
            time = seq(from = 0,to = maxT, length.out = s[1]);
            # plot and save depending on the dataname
            setEPS()
            postscript(paste(c(report_path, filename_plot, ".eps"), collapse =""))
            if(dataplot == 1){
                plottitle = paste(c("Robot Configuration, speed: ", speeds[dataspeed], ", tracking: ", methods[method] ), collapse ="");
                #print("Plotting robot configurations.");
                min = min(data);
                max = max(data);
                plot(time, data[,1], type = "l", xlab = "Time [s]", ylab = "Configuration [rad]", ylim = c(min, max), main = plottitle, col = color[1])
                legend(0,1.7, legend = 0:6, lty = 1, col = color, title="Joint");
                for(i in 2:s[2]){
                    lines(time, data[,i], col = color[i]);
                }
            } else if(dataplot == 2){
                min = min(data[,4:6]);
                max = max(data[,4:6]);
                plot(time, data[,4], type="l" ,xlab = "Angle [rad]", ylab = "Time [s]", main = "Tool Pose Rotation",col = color[1], ylim=c(min,max));
                legend("topleft", legend = c("Roll","Pitch","Yaw"), lty = 1, col = color, title="Angles")
                lines(time, data[,5], col = color[2]);
                lines(time, data[,6], col = color[3]);

            } else if(dataplot == 3){
                #print("Plotting the tracking error.");
                plottitle = paste(c("Tracking Error, speed: ", speeds[dataspeed], ", tracking: ", methods[method] ), collapse ="");
                #print("Plotting robot configurations.");
                min = min(data);
                max = max(data);
                plot(time, data[,1], type = "l", xlab = "Time [s]", ylab = "Error [px]", ylim = c(min, max), main = plottitle, col = color[1])
                legend("topleft", legend = c("u","v"), lty = 1, col = color);
                for(i in 2:s[2]){
                    lines(time, data[,i], col = color[i]);
                }
            }else if(dataplot == 4){
                #print("Plotting the relative link speed.");
                plottitle = paste(c("Relative Joint Speed, speed: ", speeds[dataspeed], ", tracking: ", methods[method] ), collapse ="");
                #print("Plotting robot configurations.");
                max = velLim[dataspeed];
                
                plot(time, data[,1], type = "l", xlab = "Time [s]", ylab = "dq relative [%]", ylim = c(0, max), main = plottitle, col = color[1])
                legend("topleft", legend = 0:(s[2]-1), lty = 1, col = color);
                for(i in 2:s[2]){
                    lines(time, data[,i], col = color[i]);
                }
            }
        dev.off()
        }
    }
}


# colors for plotting
color = rainbow(3);
for( method in 1:length(methods)){
    # plot the tool pose (translation)
    filename_plot = paste(c(datanames[2], "_", speeds[1], "_", methods[method]), collapse ="");
    filename_data = paste(c(filename_plot, fileextension), collapse ="");
    data = read.csv(filename_data);
    setEPS()
    postscript(paste(c(report_path, datanames[2], "_", methods[method], "_", "pos", ".eps"), collapse =""))
    plot(data[,1], data[,3], type="p" ,xlab = "x [m]", ylab = "z [m]", main = "Tool Pose Translation", col = color[1])
    legend(-0.35,1.79, legend = c("Slow","Medium","Fast"), lty = 1, col = color, title="Markerspeed")

    for( dataspeed in 1:length(speeds)){
        # prepare name to load file and start of filename for saving
        filename_plot = paste(c(datanames[2], "_", speeds[dataspeed], "_", methods[method]), collapse ="");
        filename_data = paste(c(filename_plot, fileextension), collapse ="");
        #print(filename_data);
                
        # load the data
        if(!(dataspeed == 1)){
            data = read.csv(filename_data);
            s = dim(data);
            #print(s);
            # plot and save depending on the dataname
            #print("Plotting the tracking error.");
            lines(data[,1], data[,3], col = color[dataspeed], type="p");
        }
    }
    dev.off()
}



