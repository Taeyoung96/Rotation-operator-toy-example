# Rotation operator toy example

This project implements rotating a point cloud by randomly generating a rotation angle and unit vector.  
I have implemented a rotation matrix and a quaternion, respectively, and visualized the results.  

## Preliminary knowledge  

Given a rotation angle $\phi$ and a rotation vector (unit vector) $\vec{u}$ as input,  
the following formulas can be utilized to implement a rotation matrix and a quaternion, respectively.  

Given input point cloud of $x$, applying the rotation operator to each of them gives the following representation.


### Rotation matrix  
$$
R = exp(\[ \phi \cdot \vec{u} \]_{\times})
$$

$$
x' = R x
$$

### Quaternion  
$$
q = exp( \phi \cdot \vec{u} / 2)
$$

$$
x' = q \otimes x \otimes q^*
$$

## How to start  

Refer to `requirements.txt` to setup the dependencies and execute the code like below.  
I tested this on ubuntu 18.04.

```
python main.py --input_path ./bunny.ply
```  

## Result  

### Rotation matrix result  

<p align="center"><img src="https://user-images.githubusercontent.com/41863759/233828656-ebfcb1e1-4017-43e0-889a-7d8263faf43b.JPG" width = "500" ></p>  

### Quaternion result

<p align="center"><img src="https://user-images.githubusercontent.com/41863759/233828660-40535a1c-438c-4281-ab03-6e5194a735d6.JPG" width = "500" ></p>  

## Reference  
- Sola, Joan. "Quaternion kinematics for the error-state Kalman filter." arXiv preprint arXiv:1711.02508 (2017).  
- Tricks to do annotation : https://github.com/isl-org/Open3D/issues/2
