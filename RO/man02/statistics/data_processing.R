valid_mean <- function(input){
	reduced = input[input != 0]
	return(mean(reduced))
}
valid_var <- function(input){
	reduced = input[input != 0]
	return(var(reduced))
}
mean_bar_plot <- function(input, labels=1, xlab=NULL, ylab=NULL,main=NULL, error=TRUE){
	if(labels[1] == 1){
		labels = 1:dim(input)[2]
	}
	input.mean = apply(input,2,valid_mean)
	input.var = apply(input,2,valid_var)
	limits = c(0, max(input.mean))
	if(error){
		limits = c(0, max(input.mean+input.var))
	}
	mid_bar = barplot(input.mean, names.arg=labels,ylim=limits, col="gray", axis.lty=1, main=main, xlab=xlab, ylab=ylab)
	if(error){
		arrows(mid_bar,input.mean+input.var, mid_bar, input.mean-input.var, angle=90, code=3, length=0.1)
	}
}


time = matrix(0,30,9)
length = matrix(0,30,9)
for(i in 1:9){
	file_name = paste(c("eps0.",i,"00000.csv"),collapse="")
# 	print(file_name)
	y = read.csv(file_name)
	time[,i] = y$time
	length[,i] = y$length
}
mean_bar_plot(time,xlab="epsilon", ylab="Average Time [ms]", labels=seq(0.1,0.9,0.1),error=FALSE)
mean_bar_plot(length,xlab="epsilon", ylab="Average Path Length [m]", labels=seq(0.1,0.9,0.1),error=TRUE)


valid_paths = array(30,9)
for(c in 1:9){
	for(r in 1:30){
		if(length[r,c] == 0){
			valid_paths[c] = valid_paths[c] - 1
		}
	}
}
print(valid_mean(length[,1]))
print(valid_paths)
valid_paths_p = valid_paths/30 * 100
barplot(valid_paths_p, names.arg=seq(0.1,0.9,0.1),ylim=c(0,100), col="gray", axis.lty=1, xlab="epsilon", ylab="Valid outputs [%]")
