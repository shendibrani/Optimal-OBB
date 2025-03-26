About
This project aims to provide an implementation of OBB or simply known as a Box Collider in Unity, however with the addition that it can contain an n number of objects inside and having an implementation to recalculate the most optimal diagonals. That is done by using the PCA algorithm in order to find the optimal orientation. 
See the comparison with the Green (Unity) vs Blue (our) to see the shortcomings of using a box collider in this situation. Of course, in both situations we use a box collider, however the unity box collider does not by default recalculate the best orientation for the object/objects it represents, in other words, it creates an OBB with the shortcomings of an AABB, which brings us to this project. Aside from the core components of the project being sources by different implementations found online, we have added different optimizations such as vertex caching, and will further add optimization to instead get only the edge vertices, perhaps vectorization of different calculations using SIMD etc. 


It can be seen in action here:

![Unity Green vs Ours Blue.](https://github.com/shendibrani/Optimal-OBB/blob/main/Screenshot_1.png)

https://github.com/user-attachments/assets/f823cb50-3202-47dc-8948-c9e123601bff

This project was created using the following sources, for the main mathematical explanation follow this site:
https://hewjunwei.wordpress.com/2013/01/26/obb-generation-via-principal-component-analysis/

and unity specific implementations here:
https://blog.csdn.net/qing101hua/article/details/53100112
https://blog.csdn.net/flj135792468/article/details/120759839

