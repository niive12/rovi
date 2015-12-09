#!/bin/bash
if [ $# -eq 1 ]
then
	dir=$1
	#app=hough-build/Hough
	app=harris-build/Harris

	echo -n $(for img in $dir/*png; do $app $img; done | grep "center" | wc -l)
	echo -n " / " 
	echo $(ls -l $dir/*png | wc -l)
else
	echo "Test if the marker is detected on a set of images."
	echo "Input is a folder containing images"
	echo "Output is the number of successful detections / number of images"
	echo "The output is verified by printing a message with the word center"
fi
