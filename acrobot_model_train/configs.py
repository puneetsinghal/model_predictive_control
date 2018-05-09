import numpy as np

def Purturb(x, percent):
    x_p = x * (1 + percent * 2*(np.random.rand(1) - 0.5))
    return x_p 

def PurturbParams(purturbed_params, percentage):
    purturbed_params = {}
    purturbed_params['m1'] = Purturb(params['m1'], percentage)
    purturbed_params['m2'] = Purturb(params['m2'], percentage)
    purturbed_params['l1'] = Purturb(params['l1'], percentage)
    purturbed_params['l2'] = Purturb(params['l2'], percentage) 

    purturbed_params['I1'] = Purturb(params['I1'], percentage)
    purturbed_params['I2'] = Purturb(params['I2'], percentage)

    purturbed_params['x0'] = params['x0'] # current state space
    purturbed_params['Ts'] = params['Ts']             # time iteration
    purturbed_params['g'] = params['g']
    purturbed_params['N'] = params['N']             # Event horizon = 10
    purturbed_params['u0'] = params['u0']# initial force
    return purturbed_params


class Params:
    def __init__(self):
        params = {}
        params['m1'] = 1.
        params['m2'] = 1.
        params['l1'] = 0.5
        params['l2'] = 0.5 
        params['g'] = -9.81
        params['I1'] = 0.1
        params['I2'] = 0.1

        params['x0'] = np.array([1, 0, 0, 0]).reshape((4,1)) # current state space
        params['Ts'] = 0.01             # time iteration
        params['N'] = 10              # Event horizon = 10
        params['u0'] = np.array(0).reshape(1,1)               # initial force
        
        self.params = params
    def get_params(self):
        return self.params

    def get_purturbed_params(self, percentage):
        
        # Set up uncertainty real model
        purturbed_params = PurturbParams(self.params, percentage)
        return purturbed_params 



