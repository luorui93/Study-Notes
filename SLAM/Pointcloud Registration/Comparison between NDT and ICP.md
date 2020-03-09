# Comparison between NDT and ICP
This is a reading note for the following papers:  
>*Magnusson et al, Evaluation of 3D registration reliability and speed-A comparison of ICP and NDT*  
>*Magnusson, Scan registration for autonomous mining vehicles using 3D‐NDT*
>*Magnusson, The Three-Dimensional Normal-Distributions Transform --- an Efficient Representation for Registration, Surface Analysis, and Loop Detection*

>Helpful links to understand the topic:  
>[Iterative Closest Point Algorithm Introduction to Mobile Robotics](http://ais.informatik.uni-freiburg.de/teaching/ss11/robotics/slides/17-icp.pdf)

Currently there are two main strategies to match two pointclouds and find the rigid transformations:  
Iterative Closet Point (ICP) and Normal Distribution Transform (NDT).

ICP is an older one while NDT is more popular in recent years. The original main concepts behind the two algorithms are as follows: 

## ICP

The main idea is to caculate the rigid transformation $`(R,t)`$ so as to minimize the error function:

$$
E(R,t)=\sum_{i=1}^{N_m}\sum_{j=1}^{N_d}w_{i,j}||m_i-(Rd_j+t)||^2
$$


where $N_m$ and $N_d$ is the number of points in model and dataset respectively. $w_{i,j}$ is the weight for a point match. The weights are assigned as follows: $w_{i,j}=1$, if $m_i$ is the closet point to $d_j$ within a proximity threshold, otherwise,$w_{i,j} = 0$


## NDT
The normal-distributions transform can be described as a method for **compactly representing a surface**. NDT uses another representation of the reference scan model instead of using the individual point in a point cloud. The model is represented by a combination of normal distributions, describing the **possiblity of finding a surface point** at certain point in space.  
Because the points in the reference scan are not used directly for matching, there is no need for the computationally expensive nearest-neighbour search of ICP. 