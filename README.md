# ESP32 Drone

The aim of this project is to build a with as many self build and/or cheap components. It shall be used to explain the theory behind drones, therefore also measuring the angle, filtering the sensor data and how to control the drones motors. This project will concentrate on the software side of drones, but can be later extended to implement the hardware parts such as buck converter, motorcontroller and so on.

## Sensor Setup

So far the project only consists of an ESP32 as brain and a MPU9250 Sensor to measure the total angle of the drone. In the following table I want to show the pin configurations used so far:

|   ESP 32 - GPIO Pin    |        Funktion        |       Kommentar        |
|------------------------|------------------------|------------------------|
|          32            |       MPU SDA_PIN      |                        |
|          33            |       MPU SCL_PIN      |                        |

Please make sure to also connect GND and 3.3V to all the components.

## Kalman Filter

### Theory

In order to conroll the drone we need to measure the absolut angle of it constantly. For that the MPU9250 is being used. The Sensor consists of a accelerometer to measure the acceleration in all three dimension, a gyroscope to get the angular acceleration and a magnetometer to measure the magnetic field of the earth.

#### The problem

- The gyroscope measures angular velocity and is smooth in the short term, but it drifts over time, especially because we have to integrate the sensor value in order to get the angle, which makes the error even bigger.
- The accelerometer can estimate orientation relative to gravity, but it is noisy and sensitive to vibrations and movement.
- The magnetometer just measures the horizontal direction of the drone.

#### The solution: Sensor fusion with a Kalman filter

The Kalman filter combines both sensors to balance their weaknesses by fusing those values. Therefore the filter needs a model which is in our case the integrated values of the gyroscope. In order to integrate them, the value is the new reading is added to the old sum *angleRollGyro* and then multiplied with the time between two measurement *dt* (See function *calcAngles*). The sensor is then used to 'guess' the next angle of the drone.

To improve this prediction, the accelerometer is used as a reference measurement.  
While noisy, it provides an absolute estimate of the angle based on gravity. The Kalman filter uses these accelerometer values to correct the prediction and compensate for the gyroscope’s drift over time.

#### Adjust the filter

In order to adjust the filter we have two matrices which set the model uncertainty and the expected measurement noise.

##### Q_cov – Process Noise Covariance (Model Uncertainty)

The *Q_cov* matrix is being implented in the definition part of the code as shown:

BLA::Matrix<2, 2> Q_cov = {
    0.008*0.008, 0,
    0,           0.0002*0.0002
};

This matrix represents how uncertain your model (gyroscope-based prediction) is.

- The diagonal values are the variances (σ²) of your state variables.
- In a typical angle estimation, your state might be:
  - angle
  - gyro bias
  
So:

- Q_cov[1, 1] --> uncertainty in the angle prediction
- Q_cov[2, 2] --> uncertainty in the gyro bias

The off-diagonal elements are 0, meaning you assume the uncertainties are independent.

Interpretation:

- Larger values:
  As seen in the following picture will the filter follow the measurements really closely, which outputs a really noisy signal.
  ![alt text](High_Q.png)
- Smaller values --> more trust in the model
