#

A repo for the final project of SLAM study.  

## Project Structure

1. **SlamWork**: Body class of all the features.
2. **DepthGenerator**: Generate the depth from stereo images.  
3. **IcpTranslator**: Generate the R and T between 2 frames.



## TODO 

1. Add backend features.
2. Debug the wrong translation according to the reference.



## Result

![result](/home/t/slam_study/PA8_code/code/result.png)



Input argument: The exact **kitti** data directory of images: **00,01,02,...**

Output now are the infos printed at the console. 

**R_world**: The R matrix from the present to the first.

**t_world**: The t vector from the present to the first.

