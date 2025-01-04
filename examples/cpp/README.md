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
  0.997806  0.0551264 -0.0366526  -0.114988
-0.0542926   0.998252  0.0233696 -0.0368429
 0.0378768 -0.0213283   0.999055   0.122853
         0          0          0          1

QGM:
  0.997806  0.0551263 -0.0366525  -0.114988
-0.0542925   0.998252  0.0233693 -0.0368427
 0.0378767 -0.0213281   0.999055   0.122852
         0          0          0          1

```