require(grDevices)

report_path = "../report/graphics/"
speed = c("fast","medium","slow")
filename = c("relativeConfVel", "robotConfiguration", "toolPose", "trackingError")
marker = c("marker 1", "marker 2", "marker 3" )
marker_n = c("marker1","marker2","marker3")

velLim = c(0.12,0.28,0.48)

color = rainbow(7);

timeintervals = c(0.050,0.100,0.450)

for(m in 1:length(marker)){
	for(v in 1:length(speed)){
		for(j in 1:length(filename) ){
			if(filename[j] == "robotConfiguration"){
				filename_plot = paste(c("rovi_robot_configuration_", speed[v], "_", marker_n[m]), collapse ="");
				setEPS()
				postscript(paste(c(report_path, filename_plot, ".eps"), collapse =""), height=6, width=8)
				filename_data = paste(c("marker",m,"/",speed[v],"/",filename[j],".csv"), collapse ="")
				data = read.csv(filename_data)
				s = dim(data);
				maxT = timeintervals[m] * s[1];
				time = seq(from = 0,to = maxT, length.out = s[1]);


				plottitle = paste(c("Robot Configuration, speed: ", speed[v], ", ", marker[m]), collapse ="");
				#print("Plotting robot configurations.");
				min = min(data);
				max = max(data);
				plot(time, data[,1], type = "l", xlab = "Time [s]", ylab = "Configuration [rad]", ylim = c(min, max), main = plottitle, col = color[1])
				legend(0,1.7, legend = 0:6, lty = 1, col = color, title="Joint");
				for(i in 2:s[2]){
					lines(time, data[,i], col = color[i]);
				}
				q = dev.off()
			} else if(filename[j] == "relativeConfVel"){
				filename_plot = paste(c("rovi_robot_joint_speed_", speed[v], "_", marker_n[m]), collapse ="");
				setEPS()
				postscript(paste(c(report_path, filename_plot, ".eps"), collapse =""), height=6, width=8)
				filename_data = paste(c("marker",m,"/",speed[v],"/",filename[j],".csv"), collapse ="")
				data = read.csv(filename_data)
				s = dim(data);
				maxT = timeintervals[m] * s[1];
				time = seq(from = 0,to = maxT, length.out = s[1]);
				
				plottitle = paste(c("Relative Joint Speed, speed: ", speed[v], ", ", marker[m] ), collapse ="");
				max = velLim[v]
				
				plot(time, data[,1], type = "l", xlab = "Time [s]", ylab = "dq relative [%]", ylim = c(0, max), main = plottitle, col = color[1])
				legend("topleft", legend = 0:(s[2]-1), lty = 1, col = color, ncol=2);
				for(i in 2:s[2]){
					lines(time, data[,i], col = color[i]);
				}
			}
		}
	}
}