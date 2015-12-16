require(graphics)

data = read.csv("time.csv")

setEPS()
postscript("../report/graphics/timeplot.eps", height=6, width=8)
plot(data[,1], data[,2], type="l" ,xlab = "Time [s]", ylab = expression(paste(tau,"(t) [s]")), main = expression(paste(tau,"(t) vs t")))
dev.off()
