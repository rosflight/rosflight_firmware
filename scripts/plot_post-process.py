import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import norm

est = np.fromfile("../test/build/estimate.bin", dtype=np.float64)
est = np.reshape(est, (-1, 8))

truth = np.fromfile("../test/build/truth.bin", dtype=np.float64)
truth = np.reshape(truth, (-1, 5))

imu_filt = np.fromfile("../test/build/imu_filt.bin", dtype=np.float64)
imu_filt = np.reshape(imu_filt, (-1, 7))

imu = np.fromfile("../test/build/imu.bin", dtype=np.float64)
imu = np.reshape(imu, (-1, 7))

est_labels=['t','qw','qx','qy','qz','bx','by','bz']
truth_labels=['t','qw','qx','qy','qz']
imu_filt_labels=['t','accx','accy','accz','gyrox','gyroy','gyroz']

plt.figure(1)
for i in range(1,5,1):
    plt.subplot(4,1,i)
    plt.ylabel(est_labels[i])
    plt.plot(est[:,0], est[:,i], label="est")
    plt.plot(truth[:,0], truth[:,i], label="truth")
    plt.legend()

plt.figure(2)
for i in range(5,8,1):
    plt.subplot(3,1,i-4)
    plt.ylabel(est_labels[i])
    plt.plot(est[:,0], est[:,i])


acc_max = 9.80665 * 1.1
acc_min = 9.80665 * 0.9
acc_mag = norm(imu_filt[:,1:4], axis=1)
acc_within_bounds_idx = (acc_mag > acc_min) & (acc_mag < acc_max)
good_acc = acc_mag.copy()
bad_acc = acc_mag.copy()
good_acc[~acc_within_bounds_idx]=np.nan
bad_acc[acc_within_bounds_idx]=np.nan
plt.figure(3)
plt.title("accelerometer magnitude check")
plt.plot(imu_filt[:,0], good_acc, '.', label="in bounds")
plt.plot(imu_filt[:,0], bad_acc, '.', label="out of bounds")
plt.plot([imu_filt[0,0], imu_filt[-1,0]], [acc_max, acc_max], ':', color=[0.2, 0.2, 0.2])
plt.plot([imu_filt[0,0], imu_filt[-1,0]], [acc_min, acc_min], ':', color=[0.2, 0.2, 0.2])
plt.legend()

plt.figure(4)
for i in range(1,4,1):
    plt.subplot(3,1,i)
    plt.plot(imu[:,0], imu[:,i], label="unfiltered")
    plt.plot(imu_filt[:,0], imu_filt[:,i], label="filtered")
    plt.legend()

plt.show()