"""
Adaption of work by:
Michael Fonder and Marc Van Droogenbroeck
 Mid-Air: A multi-modal dataset for extremely low altitude drone flights
 Conference on Computer Vision and Pattern Recognition Workshop (CVPRW)

BRIEF:  Since the IMU data in the midair dataset is simulated, the data
        must be synthesised using the 'ground truth' data. This file
        does this and allows for easy customisability fo the sensor noise
        parameters, which allows for accurate representation in VI-SLAM 
        configuration
        
TODO:
    ensure portability and readability 
    assess random walk - is this currently just like more gaussian noise
        and not a cumulative thing? needs more research
"""
import h5py
import numpy as np
from pyquaternion import Quaternion
import os
import sys

if __name__ == '__main__':
    environment = 'VO_test'   # the environment of the simulation
    condition = 'sunny'         # the weather we're interested in
    trajectory = '0000'         # the particular trajectory number
    camera = 'color_left'       # the camera that we're using

    accStdDev = 0.08        # [m/s^2] standard deviation of accelerometer noise (gaussian)
    gyrStdDev = 0.004       # [rad/s] standard deviation of gyroscope noise (gaussian)
    accRWStdDev = 0.00004   # [m/s^2] accelerometer bias random walk noise standard deviation 
    gyrRWStdDev = 2.0e-6    # [rad/s] gyroscope bias random walk noise standard deviation 
    
    # define the path to the folder containing our sensor records
    sensorRecords = os.getcwd() + '/MidAir/' + environment + '/' + condition + '/sensor_records.hdf5'
    
    answer = str(input("Warning: this script will overwrite IMU measurements stored in the given hdf5 dataset. \n"+ \
                       "Do you want to proceed? (y/n): "))
    if not(answer=="y" or answer=="Y"):
        sys.exit(0)

    database = h5py.File(sensorRecords, "a")
    db_path = os.path.dirname(sensorRecords)

    # IMU noise parameters chosen randomly in a range of values encountered in real devices
    #noise_acc = 2 * np.power(10., -np.random.uniform(low=1., high=3., size=(1, 3))) 
    noise_acc = np.array([[accStdDev]*3])#2 * np.power(10., [[-1*np.random.randint(10000, 30000) / 10000] * 3])
    
    #noise_gyr = np.power(10., -np.random.uniform(low=1., high=3., size=(1, 3)))
    noise_gyr = np.array([[gyrStdDev]*3])#np.power(10., [[-1*np.random.randint(10000, 30000) / 10000] * 3])
    
    #imu_bias_acc_rw = 2 * np.power(10., -np.random.uniform(low=3., high=6., size=(1, 3)))
    imu_bias_acc_rw = np.array([[accRWStdDev]*3])#2 * np.power(10., [[-1*np.random.randint(30000, 60000) / 10000] * 3])
    
    #imu_bias_gyr_rw = np.power(10., -np.random.uniform(low=4., high=6., size=(1, 3)))
    imu_bias_gyr_rw = np.array([[gyrRWStdDev]*3]) #np.power(10., [[-1*np.random.randint(40000, 60000) / 10000] * 3])
    
    for dataset in database:
        print("Currently processing : %s" % dataset)
        gt_group = database[dataset]["groundtruth"]
        gt_attitude = gt_group["attitude"]
        gt_angular_vel = gt_group["angular_velocity"]
        gt_accelerations = gt_group["acceleration"]

        imu_group = database[dataset]["imu"]

        # Set init parameters
        imu_accelerometer = np.zeros(gt_attitude.shape, dtype=float)
        imu_gyroscope = np.zeros(gt_attitude.shape, dtype=float)

        imu_bias_acc = np.array([[0.,0.,0.]])#np.random.normal([0., 0., 0.], imu_bias_acc_rw)
        imu_bias_gyr = np.array([[0.,0.,0.]])#np.random.normal([0., 0., 0.], imu_bias_gyr_rw)

        init_bias_est_acc = imu_bias_acc + np.random.normal([0., 0., 0.], noise_acc)
        init_bias_est_gyr = imu_bias_gyr + np.random.normal([0., 0., 0.], noise_gyr)
        
        imu_group["accelerometer"].attrs["init_bias_est"] = init_bias_est_acc
        imu_group["gyroscope"].attrs["init_bias_est"] = init_bias_est_gyr
        
        
        # Pass over trajectory to generate simulated sensor measurements
        for i in range(gt_attitude.shape[0]):
            attitude = Quaternion(gt_attitude[i, :])
            
            accNoiseInstant = np.random.normal([0., 0., 0.], noise_acc)
            gyrNoiseInstant = np.random.normal([0., 0., 0.], noise_gyr)
            
            imu_accelerometer = attitude.conjugate.rotate(gt_accelerations[i, :] + np.array([0., 0., -9.81])) \
                                                            + imu_bias_acc + accNoiseInstant
            imu_gyroscope = gt_angular_vel[i, :] + imu_bias_gyr + gyrNoiseInstant
            
            accRWComponent = np.random.normal([0., 0., 0.], imu_bias_acc_rw)
            gyrRWComponent = np.random.normal([0., 0., 0.], imu_bias_gyr_rw)
            
            imu_bias_acc += accRWComponent
            imu_bias_gyr += gyrRWComponent
            
            imu_group["accelerometer"][i] = imu_accelerometer
            imu_group["gyroscope"][i] = imu_gyroscope
            
            
database.close()
