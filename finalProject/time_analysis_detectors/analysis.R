file = read.csv("marker3_unoptimized.csv",header=FALSE);

sift_keypoints = file[,1] 
sift_descripts = file[,2] + sift_keypoints
sift_homograph = file[,3]

lim = c(0,1)

t = 1:length(sift_keypoints)
setEPS()
postscript("../report/graphics/marker3_timing_unoptimized.eps",width=8,height=4)
plot(t,  sift_homograph, type="l", lty=1, ylim=lim, box.col="white", xlab="Image in set", ylab="Time [S]")
lines(t, sift_descripts, type="l", lty=2)
lines(t, sift_keypoints, type="l", lty=3)
col = c("gray50","gray70","gray90")
polygon(c(t, rev(t)), c(sift_keypoints, array(0,length(t))), col = col[1], border = NA)
polygon(c(t, rev(t)), c(sift_descripts, rev(sift_keypoints)),col = col[2], border = NA)
polygon(c(t, rev(t)), c(sift_homograph, rev(sift_descripts)),col = col[3], border = NA)

legend("topright", legend=rev(c("Keypoints","Descriptors","Homography")), col=rev(col), lty=array("solid",3),lwd=10, bty="n");
q = dev.off()

mean(sift_homograph)

file = read.csv("marker3_good_matches.csv",header=FALSE);

sift_keypoints = file[,1] 
sift_descripts = file[,2] + sift_keypoints
sift_homograph = file[,3]

t = 1:length(sift_keypoints)
setEPS()
postscript("../report/graphics/marker3_timing_good_matches.eps",width=8,height=4)
plot(t,  sift_homograph, type="l", lty=1, ylim=lim, xlab="Image in set", ylab="Time [S]")
lines(t, sift_descripts, type="l", lty=2)
lines(t, sift_keypoints, type="l", lty=3)
col = c("gray50","gray70","gray90")
polygon(c(t, rev(t)), c(sift_keypoints, array(0,length(t))), col = col[1], border = NA)
polygon(c(t, rev(t)), c(sift_descripts, rev(sift_keypoints)),col = col[2], border = NA)
polygon(c(t, rev(t)), c(sift_homograph, rev(sift_descripts)),col = col[3], border = NA)

legend("topright", legend=rev(c("Keypoints","Descriptors","Homography")), col=rev(col), lty=array("solid",3),lwd=10, bty="n");
q = dev.off()

mean(sift_homograph)

file = read.csv("marker3_crop.csv",header=FALSE);

sift_keypoints = file[,1] 
sift_descripts = file[,2] + sift_keypoints
sift_homograph = file[,3]

t = 1:length(sift_keypoints)
setEPS()
postscript("../report/graphics/marker3_timing_crop.eps",width=8,height=4)
plot(t,  sift_homograph, type="l", lty=1, ylim=lim, xlab="Image in set", ylab="Time [S]")
lines(t, sift_descripts, type="l", lty=2)
lines(t, sift_keypoints, type="l", lty=3)
col = c("gray50","gray70","gray90")
polygon(c(t, rev(t)), c(sift_keypoints, array(0,length(t))), col = col[1], border = NA)
polygon(c(t, rev(t)), c(sift_descripts, rev(sift_keypoints)),col = col[2], border = NA)
polygon(c(t, rev(t)), c(sift_homograph, rev(sift_descripts)),col = col[3], border = NA)

legend("topright", legend=rev(c("Keypoints","Descriptors","Homography")), col=rev(col), lty=array("solid",3),lwd=10, bty="n");
q = dev.off()

median(sift_homograph)
mean(sift_homograph)