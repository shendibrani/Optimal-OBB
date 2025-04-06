About:

This project aims to provide an implementation of OBB or simply known as a Box Collider in Unity. However, this is with the addition that it can contain an n number of objects inside (it can fit any number of points, or vertices from any objects).
This is done by using the PCA algorithm in order to find the optimal orientation (Hew Jun Wei blog source below). 
See the comparison with the Green (Unity) vs Blue (our) to see the shortcomings of using a box collider in this situation. Of course, in both situations we use a box collider, however the unity box collider does not by default recalculate the best orientation for the object/objects it represents, in other words, it creates an OBB with the shortcomings of an AABB, since it only calculates the best orientation in space based only on the models space it represents, neither does it let us provide a better orientation to it, therefore we generate our own gameobject with custom size and orientation with a default unity box collider to represent the new space of the arbitrary number of points in space.
Aside from the core components of the project being sourced by different implementations by Hew Jun Wei (link below), we have added different optimizations such as vertex caching, calculating the best orientation and size based on the bounding boxes of each object instead of all vertices, etc...
The results on a Ryzen 5700X3D are:

OBB calculation for 176 verts (2 cylinders, all vertices), 200 samples : 0.013ms.. Full Accuracy
![2 cylinders, all vertices](https://github.com/shendibrani/Optimal-OBB/blob/main/2capsule(allvertices).png)
For 16 points (Bounding box method, same 2 cylinders) 200 samples: 0.0035ms Slightly lower accuracy
![2 cylinders, bounding box based, less accurate](https://github.com/shendibrani/Optimal-OBB/blob/main/2capsule(bb).png)
For 1600 points (3 capsules) 200 samples: 0.39ms  Full Accuracy
![3 capsules, all vertices, max accuracy](https://github.com/shendibrani/Optimal-OBB/blob/main/3caps(allverts).png)

3 Capsules reduced to 24 points (3 bounding boxes), 200 samples: 0.02ms avg Lower Accuracy
![3 capsules, bounding box based, less accuracy](https://github.com/shendibrani/Optimal-OBB/blob/main/3caps(bb).png)

For 10,000 points: 2.5ms. 

Unity vs Ours:
![Unity Green vs Ours Blue.](https://github.com/shendibrani/Optimal-OBB/blob/main/Screenshot_1.png)
While Unit wouldn't let you create such a box collider manually at all without going through the trouble of creating it dynamically, we have done that to compare what a normal encapsulation of a unity boxcollider of these children would look like versus our implementation which also finds the best orientation in space by selecting the most optimal diagonals represented by the point cloud (vertices provided).
The accuracy lost by this optimization depends entirely on the shape of the children objects you represent, the tighter their own custom colliders (instead of all their vertices being taken into account) are the more accurate the end representation would be, the one represented here are bad scenarios however the accuracy would still be good in a realtime game environment that requires precise colliders. Therefore, there are quite a lot of optimizations that can be made to this algorithm based on your own use cases, these things include caching all the vertices of the children objects instead of recollecting them every time a child moves, Job Burst system calculations for the heaviest math calculations (Jacobi), re-calculating only part of the PCA (a child who hasn't moved doesn't need to contribute fully to the algorithm recalculation) whenever a child moves, etc.

This suggests that the algorithm has a Big O notation factor of O(n). Scaling linearly with the number of points it has to take into account. Note that the mesh filter vertex collection for all children is not included in these calculations. 



https://github.com/user-attachments/assets/f823cb50-3202-47dc-8948-c9e123601bff

This project was created using the following sources, for the main mathematical explanation follow this site:
https://hewjunwei.wordpress.com/2013/01/26/obb-generation-via-principal-component-analysis/

and unity specific implementations here:
https://blog.csdn.net/qing101hua/article/details/53100112
https://blog.csdn.net/flj135792468/article/details/120759839

