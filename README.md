About:

This project aims to provide an implementation of OBB or simply known as a Box Collider in Unity, however with the addition that it can contain an n number of objects inside and having an implementation to recalculate the most optimal diagonals.
That is done by using the PCA algorithm in order to find the optimal orientation. 
See the comparison with the Green (Unity) vs Blue (our) to see the shortcomings of using a box collider in this situation. Of course, in both situations we use a box collider, however the unity box collider does not by default recalculate the best orientation for the object/objects it represents, in other words, it creates an OBB with the shortcomings of an AABB, since it only calculates the best orientation in space according to its models space.
Aside from the core components of the project being sourced by different implementations by Hew Jun Wei (link below), we have added different optimizations such as vertex caching, calculating the best orientation and size based on the bounding boxes of each object instead of all vertices, etc...
The results on a Ryzen 5700X3D are:

OBB calculation for 176 verts (2 cylinders, all vertices), 200 samples : 0.013ms.. ![2 cylinders, all vertices](https://github.com/shendibrani/Optimal-OBB/blob/main/2capsule(allvertices).png)
For 16 points (Bounding box method, same 2 cylinders) 200 samples: 0.0035ms ![2 cylinders, bounding box based, less accurate](https://github.com/shendibrani/Optimal-OBB/blob/main/2capsule(bb).png)
For 1600 points (3 capsules) 200 samples: 0.39ms   ![3 capsules, all vertices, max accuracy](https://github.com/shendibrani/Optimal-OBB/blob/main/3caps(allverts).png)

3 Capsules reduced to 24 points (3 bounding boxes), 200 samples: 0.02ms avg
![3 capsules, bounding box based, less accuracy](https://github.com/shendibrani/Optimal-OBB/blob/main/3cyls(boundingbox).png)



This suggests that the algorithm has a Big O notation factor of O(n). Scaling with the number of points it has to take into account. Note that the mesh filter vertex collection for all children is not included in these calculations. 

It can be seen in action here:

![Unity Green vs Ours Blue.](https://github.com/shendibrani/Optimal-OBB/blob/main/Screenshot_1.png)

https://github.com/user-attachments/assets/f823cb50-3202-47dc-8948-c9e123601bff

This project was created using the following sources, for the main mathematical explanation follow this site:
https://hewjunwei.wordpress.com/2013/01/26/obb-generation-via-principal-component-analysis/

and unity specific implementations here:
https://blog.csdn.net/qing101hua/article/details/53100112
https://blog.csdn.net/flj135792468/article/details/120759839

