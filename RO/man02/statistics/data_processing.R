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

eps_tested = seq(0.1,2.0,0.1)
number_of_tets = 30
time = matrix(0,number_of_tets,length(eps_tested))
length = matrix(0,number_of_tets,length(eps_tested))
for(i in 1:length(eps_tested)){
	file_name = paste(c("eps",format(eps_tested[i],nsmall=6),".csv"),collapse="")
	print(file_name)
	y = read.csv(file_name)
	time[,i] = y$time
	length[,i] = y$length
}
valid_paths = array(number_of_tets,length(eps_tested))
for(c in 1:length(eps_tested)){
	for(r in 1:number_of_tets){
		if(length[r,c] == 0){
			valid_paths[c] = valid_paths[c] - 1
			time[r,c] = 0; #remove time from no solution found results
		}
	}
}

setEPS()
postscript("timeVSepsilon.eps",height = 4, width = 8)
mean_bar_plot(time,xlab="epsilon", ylab="Average Time [ms]", labels=eps_tested,error=FALSE)
q = dev.off()

setEPS()
postscript("distVSepsilon.eps",height = 4, width = 8)
mean_bar_plot(length,xlab="epsilon", ylab="Average Path Length [m]", labels=eps_tested,error=TRUE)
q = dev.off()

cat(c("valid paths", valid_paths, "\n"))
valid_paths_p = valid_paths/number_of_tets * 100
setEPS()
postscript("successfulVSepsilon.eps",height = 4, width = 8)
barplot(valid_paths_p, names.arg=eps_tested,ylim=c(0,100), col="gray", axis.lty=1, xlab="epsilon", ylab="Valid outputs [%]")
q = dev.off()