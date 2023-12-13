# Coming soon!
# Evaluation of Camera Pose Accuracy

This repository contains the code for evaluating the accuracy of estimated camera poses using the TUM [^1] and Bonn [^2] RGB-D dynamic datasets.

## Datasets

- TUM: We selected 6 different indoor dynamic sequences with moving people and violent camera shaking for evaluation.
- Bonn: We selected 20 sequences of more complex dynamic motion in indoor scenes.

## Evaluation Metrics

We used two metrics to measure the accuracy between the estimated camera poses and the ground truth:

1. Absolute Trajectory Error (ATE): This metric measures the absolute error in trajectory estimation, expressed in meters.
2. Relative Pose Error (RPE): This metric measures the relative error in pose estimation, expressed in meters per second.

For detailed definitions of these metrics, please refer to the TUM dataset documentation [^1].

## System Configuration

All experiments were performed on a desktop computer with the following specifications:

- CPU: 3.6 GHz Intel Core i9-9900K
- RAM: 16 GB
- GPU: No GPU acceleration was used.

## Usage

Please refer to the instructions provided in the respective dataset directories for running the evaluation code.

## References

[^1]: TUM Dataset. Website: [https://www.tum.de](https://www.tum.de)
[^2]: Bonn Dataset. Website: [https://www.bonn.de](https://www.bonn.de)
