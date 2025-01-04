# :snake: Python Example

This example shows how to use FracGM and QGM in Python. Make sure the project is built with python binding.

```shell
cd examples/python
```

## :checkered_flag: Run

```shell
python3 ./registration_example.py
```

This main function will read two point clouds from the [`data`](../data) folder and solve the registration problem.

```shell
Ground truth:
[[ 0.99692656  0.06687358 -0.04066644 -0.11557694]
 [-0.06612899  0.99761788  0.01940087 -0.03877054]
 [ 0.04186755 -0.01665178  0.99897777  0.11487489]
 [ 0.          0.          0.          1.        ]]

FracGM:
[[ 0.99780643  0.05512641 -0.03665262 -0.1149879 ]
 [-0.05429257  0.99825156  0.02336957 -0.03684285]
 [ 0.03787682 -0.02132834  0.99905478  0.12285266]
 [ 0.          0.          0.          1.        ]]

QGM:
[[ 0.99780643  0.05512631 -0.03665255 -0.11498784]
 [-0.05429248  0.99825157  0.02336934 -0.03684266]
 [ 0.03787673 -0.02132812  0.99905479  0.12285238]
 [ 0.          0.          0.          1.        ]]

```