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

IRLS-TLS:
  0.997703   0.054569 -0.0401353  -0.103475
-0.0537937   0.998349   0.020152 -0.0292205
 0.0411687 -0.0179467   0.998991   0.119473
         0          0          0          1

IRLS-GM:
  0.997806  0.0551263 -0.0366525  -0.114988
-0.0542925   0.998252  0.0233693 -0.0368427
 0.0378767 -0.0213281   0.999055   0.122852
         0          0          0          1

GNC-TLS:
  0.997722  0.0544789 -0.0397885  -0.103705
-0.0537178   0.998357   0.019954 -0.0289391
 0.0408102 -0.0177712   0.999009   0.118865
         0          0          0          1

GNC-GM:
  0.997845  0.0546486 -0.0363091  -0.114217
-0.0538252   0.998279  0.0232826 -0.0365992
  0.037519 -0.0212781   0.999069   0.122891
         0          0          0          1

MS-GNC-L0:
   0.99793  0.0535857 -0.0355476  -0.108713
-0.0528424   0.998371  0.0215304 -0.0294398
 0.0366434 -0.0196074   0.999136   0.125892
         0          0          0          1

MS-GNC-TLS:
  0.997756   0.054013 -0.0395729  -0.102999
-0.0532017   0.998357  0.0212761 -0.0300278
 0.0406571  -0.019123    0.99899   0.120328
         0          0          0          1

FracGM:
  0.997806  0.0551264 -0.0366526  -0.114988
-0.0542926   0.998252  0.0233696 -0.0368429
 0.0378768 -0.0213283   0.999055   0.122853
         0          0          0          1

```