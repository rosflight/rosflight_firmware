import matplotlib.pyplot as plt
import numpy as np

est = np.fromfile("../test/build/estimate.bin", dtype=np.float64)
est = np.reshape(est, (-1, 8))

truth = np.fromfile("../test/build/truth.bin", dtype=np.float64)
truth = np.reshape(truth, (-1, 5))

est_labels=['t','qw','qx','qy','qz','bx','by','bz']
truth_labels=['t','qw','qx','qy','qz']

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

plt.show()