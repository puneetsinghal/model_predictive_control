import pickle
import numpy as np
import pickle
from IPython import embed
class DB_Processor(object):
	def __init__(self):
		pass

	@staticmethod
	def loadRawData(fileName, input_state_size, output_state_size):
		data=pickle.load(open(fileName,'rb'))

		#Uploading the data into xk_data and uk_data

		# X_and_U = np.zeros([int(len(data)/1000), 1000, input_state_size])

		# for row in range(len(data)):
		# 	firstIndex = row//1000
		# 	secondIndex = row%1000
		# 	X_and_U[firstIndex, secondIndex, :output_state_size] = data[row][0]
		# 	X_and_U[firstIndex, secondIndex, output_state_size:] = data[row][1]

		X_and_U = np.array(data)

		return X_and_U
	@staticmethod
	def gen_epoch(data_file, batch_size, time_steps, input_state_size, output_state_size):
		rawData = DB_Processor.loadRawData(data_file, input_state_size, output_state_size) # X and U concatenated
		rawInputData 	= rawData[:, :-1, :]
		rawOutputData 	= rawData[:, 1:, :]

		numDataSamples = rawData.shape[0]
		numSamples_list = [rawData.shape[1]-1 for i in range(rawData.shape[0])]
		numBatches_List = [(samples - time_steps + 1) for samples in numSamples_list]

		num_batch = sum(numBatches_List)
		input_epoch = np.zeros([num_batch, time_steps, input_state_size])
		output_epoch = np.zeros([num_batch, time_steps, output_state_size])
		firstIndex = 0
		for i in range(numDataSamples):
			for batchNum in range(numBatches_List[i]):
				for t in range(time_steps):
					# embed()
					try:
						input_epoch[firstIndex, t, :] = \
											np.array((rawInputData[i, batchNum+t][0] + rawInputData[i, batchNum+t][1]))
						output_epoch[firstIndex, t, :] = np.array(rawOutputData[i, batchNum+t][0])
					except:
						print(i, numDataSamples, batchNum)
						embed()
				firstIndex += 1
		return input_epoch, output_epoch, num_batch