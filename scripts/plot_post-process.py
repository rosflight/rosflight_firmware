import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import norm


def quat2euler(q):
    w = q[0, :]
    x = q[1, :]
    y = q[2, :]
    z = q[3, :]
    return np.array([np.arctan2(2.0 * (w * x + y * z), 1. - 2. * (x * x + y * y)),
                     np.arcsin(2.0 * (w * y - z * x)),
                     np.arctan2(2.0 * (w * z + x * y), 1. - 2. * (y * y + z * z))])

est = np.fromfile("../test/build/estimate.bin", dtype=np.float64)
est = np.reshape(est, (-1, 8))

cmd = np.fromfile("../test/build/cmd.bin", dtype=np.float64)
cmd = np.reshape(cmd, (-1, 5))

truth = np.fromfile("../test/build/truth.bin", dtype=np.float64)
truth = np.reshape(truth, (-1, 5))
truth[:,0] -= 2.25

# rotate quaternion to account for mocap stupidity
R = np.array([[0, -1, 0],
              [0, 0, -1],
              [1, 0, 0]])
truth[:,2:] = truth[:,2:].dot(R)

imu_filt = np.fromfile("../test/build/imu_filt.bin", dtype=np.float64)
imu_filt = np.reshape(imu_filt, (-1, 7))

imu = np.fromfile("../test/build/imu.bin", dtype=np.float64)
imu = np.reshape(imu, (-1, 7))

est_labels=['t','qw','qx','qy','qz','bx','by','bz']
truth_labels=['t','qw','qx','qy','qz']
imu_filt_labels=['t','accx','accy','accz','gyrox','gyroy','gyroz']

true_euler = quat2euler(truth[:,1:].T)
true_euler[2,:] -= true_euler[2,0]
est_euler = quat2euler(est[:,1:].T)



plt.figure(1)
for i in range(1,5,1):
    plt.suptitle('quaternion')
    plt.subplot(4,1,i)
    plt.ylabel(est_labels[i])
    plt.plot(est[:,0], est[:,i], label="est")
    plt.plot(truth[:,0], truth[:,i], label="truth")
    plt.legend()

plt.figure(2)
for i in range(5,8,1):
    plt.suptitle('biases')
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
# plt.figure(3)
# plt.title("accelerometer magnitude check")
# plt.plot(imu_filt[:,0], good_acc, '.', label="in bounds")
# plt.plot(imu_filt[:,0], bad_acc, '.', label="out of bounds")
# plt.plot([imu_filt[0,0], imu_filt[-1,0]], [acc_max, acc_max], ':', color=[0.2, 0.2, 0.2])
# plt.plot([imu_filt[0,0], imu_filt[-1,0]], [acc_min, acc_min], ':', color=[0.2, 0.2, 0.2])
# plt.legend()

# plt.figure(4)
# for i in range(1,4,1):
#     plt.subplot(3,1,i)
#     plt.plot(imu[:,0], imu[:,i], label="unfiltered")
#     plt.plot(imu_filt[:,0], imu_filt[:,i], label="filtered")
#     plt.legend()

plt.figure(5)
ax = None
for i in range(3):
    plt.suptitle('euler')
    ax = plt.subplot(3,1,i+1, sharex = ax)
    plt.plot(truth[:,0], true_euler[i,:], label="truth")
    plt.plot(est[:,0], est_euler[i,:], label="est")
    plt.plot(cmd[:,0], cmd[:,i+1], label="cmd")
    plt.legend()




plt.show()