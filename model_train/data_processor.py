import pickle
import numpy as np
import pickle

class DB_Processor(object):
	def __init__(self):
		pass

	@staticmethod
	def loadRawData(fileName):
		data=pickle.load(open(fileName,'rb'))

		#Uploading the data into xk_data and uk_data

		X_and_U = np.zeros([int(len(data)/1000), 1000, 5])

		for row in range(len(data)):
			firstIndex = row//1000
			secondIndex = row%1000
			X_and_U[firstIndex, secondIndex, :4] = data[row][0]
			X_and_U[firstIndex, secondIndex, 4:] = data[row][1]

		return X_and_U
	@staticmethod
	def gen_epoch(data_file, batch_size, time_steps, input_state_size, output_state_size):
		rawData = DB_Processor.loadRawData(data_file) # X and U concatenated
		rawInputData 	= rawData[:, :-1, :]
		rawOutputData 	= rawData[:, 1:, :]

		numDataSamples = rawData.shape[0]
		numSamples_list = [rawData.shape[1]-1 for i in range(rawData.shape[0])]
		numBatches_List = [(samples - time_steps + 1) for samples in numSamples_list]

		num_batch = sum(numBatches_List)
		input_epoch = np.zeros([num_batch, time_steps, input_state_size])
		output_epoch = np.zeros([num_batch, time_steps, output_state_size])

		for i in range(numDataSamples):
			for batchNum in range(numBatches_List[i]):
				input_epoch[i*numDataSamples + batchNum, :, :] = rawInputData[i, batchNum:batchNum+time_steps, :]
				output_epoch[i*numDataSamples + batchNum, :, :] = rawOutputData[i, batchNum:batchNum+time_steps, :output_state_size]

		return input_epoch, output_epoch, num_batch

