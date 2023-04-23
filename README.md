# Rotation operator toy example

This simple project implements rotating a bunny point cloud by randomly generating a rotation angle and unit vector.  
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


### Quaternion result
