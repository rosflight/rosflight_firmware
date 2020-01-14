import numpy as np
import matplotlib.pyplot as plt

stateType = np.dtype([
	('t', np.float64),
	('q', (np.float64,4)),
	('qhat', (np.float64,4)),
	('err', (np.float64,3)),
	('eulerErr', (np.float64,3)),
	('bias', (np.float32,3))
])

def plotResults(filename):
	data = np.fromfile("../test/build/"+filename, dtype=stateType)

	plt.figure(figsize=[12,9])
	plt.suptitle(filename)
	for i in range(4):
		plt.subplot(4, 3, 3*i+1)
		plt.plot(data['t'], data['q'][:,i], label="q")
		plt.plot(data['t'], data['qhat'][:,i], label="qhat")
		if i == 0:
			plt.legend()

	for i in range(3):
		plt.subplot(4,3,3*i+2)
		plt.plot(data['t'], data['err'][:,i])
	plt.subplot(4,3,11)
	err_norm = np.sqrt(np.sum(np.square(data['err']),axis=1))
	plt.plot(data['t'], err_norm)

	for i in range(3):
		plt.subplot(4,3,3*i+3)
		plt.plot(data['t'], data['bias'][:,i])


	print("{} max error: {}".format(filename, np.max(err_norm)))


if __name__ == '__main__':
	plotResults("linearGyro.bin")
	plotResults("quadGyro.bin")
	plotResults("expInt.bin")
	plotResults("expQuadInt.bin")
	plotResults("acc.bin")
	plotResults("estState.bin")
	plotResults("estBias.bin")
	plotResults("estStateExtAtt.bin")
	plotResults("movingExtAtt.bin")

	plt.show()
