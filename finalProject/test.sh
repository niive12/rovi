#!/bin/bash
dir=$1
app=vision_part-build/VisionPart

echo -n $(for img in $dir/*png; do $app $img; done | grep "center" | wc -l)
echo -n " / " 
echo $(ls -l $dir/*png | wc -l)
