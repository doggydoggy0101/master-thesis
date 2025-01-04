# :croissant: C++ Example 

This example shows how to use FracGM and QGM in C++.

```shell
cd examples/cpp
```

## :gear: Build

```shell
mkdir build
cd build
cmake .. 
make
```

## :checkered_flag: Run

```shell
./registration_example
```

This main function will read two point clouds from the [`data`](../data) folder and solve the registration problem.

```shell
Ground Truth:
  0.996927  0.0668736 -0.0406664  -0.115577
 -0.066129   0.997618  0.0194009 -0.0387705
 0.0418676 -0.0166518   0.998978   0.114875
         0          0          0          1

FracGM:
  0.997735  0.0553886 -0.0381708  -0.111112
-0.0544668   0.998208  0.0247814 -0.0367663
  0.039475 -0.0226463   0.998964   0.129335
         0          0          0          1

QGM:
  0.997735  0.0553883 -0.0381704  -0.111111
-0.0544665   0.998208  0.0247813 -0.0367662
 0.0394746 -0.0226462   0.998964   0.129335
         0          0          0          1

```