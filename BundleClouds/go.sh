#!/bin/bash

id=$RANDOM

cat go.sh | mail -s "$id begin" kmatzen@gmail.com

./BundleClouds --bundle /hydra/S2/kmatzen/08April2011_dslr_nodepth/bundle.reposition.out --list /hydra/S2/kmatzen/08April2011_dslr_nodepth/list.reposition.txt --depth_tuning 0.779342 --output ~/NOBACKUP/full --downscale 4 > part1 2> part1_error
cat part1 | mail -s "$id part 1 done" kmatzen@gmail.com
cat part1_error | mail -s "$id part 1 errors" kmatzen@gmail.com

./BundleClouds --reprocess --input_cloud ~/NOBACKUP/full.pcd --postprocess --std_thresh 2 --voxel_size 0.01 --mls_radius 0.04 --output ~/NOBACKUP/again > part2 2> part2_error
cat part2 | mail -s "$id part 2 done" kmatzen@gmail.com
cat part2_error | mail -s "$id part 2 errors" kmatzen@gmail.com
