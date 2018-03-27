import numpy as np
import scipy.io as sio

def load_matfile(matfile):

    data = sio.loadmat(matfile)
    input_data = np.transpose(data['Input_data'])
    output_data = np.transpose(data['Output_data'])
    print(output_data)
