require(stats); 
require(graphics);
require(grDevices);


color = rainbow(3)

datafiles = c("circle","lines","corny")
plottitle = c("Circle","Lines","Corny")
# plot robot config and tracking error
for(i in 1:length(datafiles)){
    data = read.csv(paste(c(datafiles[i],".csv"),collapse=""), na.strings = "+Inf");
    setEPS()
    postscript(paste(c("../report/graphics/robotics/trackingerror_",datafiles[i],".eps"),collapse=""), height=6, width=8)
    min = 0
    max = 640
    plot(data[,1], data[,2], type = "b", xlab = "Time [s]", ylab = "Euclidean Max Error [px]", ylim = c(min, max), main = plottitle[i], col=color[1])
    lines(data[,1], data[,3], type = "b", col=color[2])
    lines(data[,1], data[,4], type = "b", col=color[3])
    legend("topright", c("Fast", "Medium", "Slow"), lty = 1, col = color, title="Marker Speed");
    dev.off()
}
