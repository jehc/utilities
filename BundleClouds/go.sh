#!/bin/bash

id=$RANDOM

echo "http://www.cs.cornell.edu/~kmatzen/logs" | mail -s "$id beginning" 789bc4.1327048@push.boxcar.io 

./BundleClouds --bundle /hydra/S2/kmatzen/08April2011_dslr_nodepth/bundle.reposition.out --list /hydra/S2/kmatzen/08April2011_dslr_nodepth/list.reposition.txt --depth_tuning 0.779342 --output ~/NOBACKUP/full --downscale 4 --std_thresh 2 --voxel_size 0.03 --mls_radius 0.1 --output ~/NOBACKUP/output > /home/WIN/kmatzen/MyWeb/logs/log 2> /home/WIN/kmatzen/MyWeb/logs/log_error 

echo "done" | mail -s "$id completed" 789bc4.1327048@push.boxcar.io
