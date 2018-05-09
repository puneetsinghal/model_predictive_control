# Generate training data 

from robot import Acrobot, IntegrationEstimation, Energy
import configs
import numpy as np

def random_usage():
    # return random control usage in reasonable range
    u_min = -20
    u_max = 20
    u_rand = u_min + (u_max - u_min) * np.random.rand()
    return u_rand

if __name__ == '__main__':
    params = configs.Params()
    params = params.get_params()
    robot = Acrobot(params)

    # results = ode45(@(t,x)acrobotDynamicsCT(t, x, u, params), linspace(0,4,4/params.Ts), x0);

    # N = size(results.x,2);
    energy = np.zeros((int(10.0/params['Ts']),1))
    xk = params['x0']
    uk = 0
    xHistory = [xk[:,0].tolist()]
    
    data_points = 1e8

    for i in range(int(data_points)):
        uk = random_usage()
        xk = IntegrationEstimation(xk, uk, params['Ts'], robot, 30)
        energy[i] = Energy(params, xk)
        xHistory += [xk[:,0].tolist()]
    
