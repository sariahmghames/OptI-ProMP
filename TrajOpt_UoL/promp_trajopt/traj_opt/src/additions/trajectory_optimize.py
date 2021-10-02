import numpy as np
from cost_function import CostFunction    # import Class CostFunction
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import scipy.ndimage  # n dim image 
# from matplotlib.animation import FuncAnimation
from franka_kinematics import FrankaKinematics  # import Class of FrankaKinematics
import scipy.optimize as opt   # Scipy is python library for optimization, integration, linear alg, image and sig processing...


class trajectory_optimization():

    def __init__(self):
        self.franka_kin = FrankaKinematics()
        self.cost_fun = CostFunction() # instance of the class
        # self.object_list = np.array([[0.25, 0.21, 0.71, 0.1], [0.25, -0.21, 0.51, 0.1]])  # for two spheres
        self.object_list = np.array([[0.25, 0.21, 0.71, 0.1]])  # for one sphere (x,y,z, radius)
        self.initial_joint_values = np.zeros(7)  # + 0.01 * np.random.random(7)
        self.ang_deg = 60
        self.desired_joint_values = np.array(
            [np.pi * self.ang_deg / 180, np.pi / 3, 0.0, np.pi / 6, np.pi / 6, np.pi / 6, np.pi / 6])
        self.optCurve = []
        self.step_size = 0.02

    def discretize(self, start, goal, step_size=0.02):
        w = goal - start
        step = step_size * w / np.linalg.norm(w)
        n = int(np.linalg.norm(w) / np.linalg.norm(step)) + 1  # get the number of discretized points, +1 is for the 0th sample
        discretized = np.outer(np.arange(1, n), step) + start  # arange() returns evenly spaced values and outer() returns the outer prod btw the 2 inp arguments
        return discretized

    def sphere(self, ax, radius, centre):
        u = np.linspace(0, 2 * np.pi, 13)  # start , goal , num of pts
        v = np.linspace(0, np.pi, 7)
        x = centre[0] + radius * np.outer(np.cos(u), np.sin(v))
        y = centre[1] + radius * np.outer(np.sin(u), np.sin(v))
        z = centre[2] + radius * np.outer(np.ones(np.size(u)), np.cos(v))
        xdata = scipy.ndimage.zoom(x, 3) # 3 dim image
        ydata = scipy.ndimage.zoom(y, 3)
        zdata = scipy.ndimage.zoom(z, 3)
        ax.plot_surface(xdata, ydata, zdata, rstride=3, cstride=3, color='w', shade=0) # rstride: array row stride (step size), default = 1

    def finite_diff_matrix(self, trajectory):
        rows, columns = trajectory.shape  # columns = nDoF
        A = 2 * np.eye(rows) # initialized for 1 DoF
        A[0, 1] = -1
        A[rows-1, rows-2] = -1
        for ik in range(0, rows-2):   # range(start, stop) or range(nb) and nb will be dim
            A[ik + 1, ik] = -1
            A[ik + 1, ik + 2] = -1

        dim = rows*columns  # the row dim of A matrix (differentiation matrix) for multiple DoFs
        fd_matrix = np.zeros((dim, dim)) # the finite difference matrix for the n DoFs
        b = np.zeros((dim, 1))
        i, j = 0, 0  # i is for the len(A) and j is the nb of DoFs
        while i < dim:
            fd_matrix[i:i+len(A), i:i+len(A)] = A   # len(A) gives nb of rows of A  ; fd_matrix is diag block
            b[i] = -2 * self.initial_joint_values[j]
            b[i+len(A)-1] = -2 * self.desired_joint_values[j]
            i = i + len(A)  # shift because the other elements of b vector will remain 0 in finite differentiation of a traj (PDE), and bcz we are storing block elements of A in fd_matrix for each dof per iteration
            j = j + 1
        return fd_matrix, b

    def smoothness_objective(self, trajectory):
        trajectory = np.squeeze(trajectory) 
	# remove 1-dim entries from the shape of an array , e.g shape of ([[[0], [1], [2]]]) is (1,3,1) start by external [ toward the most 		internal one and see how many sub [] it includes, by squeezing this array the resulting shape is (3)
        trajectory = np.transpose(trajectory.reshape((7, len(trajectory) / 7)))   # ? , len() is the number of elements along the largest dimension
	# reshape into 7 rows and len()/7 columns (or the number of elements in each matrix forming a row is len()/7) we got ([[7xn1],[7xn1],[7xn1]...) originally the traj is 7xn (n is the sampling nb of 	the traj of 1 DoF), by transpose we got sub arrrays of n1x7 in col
        rows, columns = trajectory.shape
        dim = rows * columns
        fd_matrix, b = self.finite_diff_matrix(trajectory)  # traj has nb of col = nb of DoFs
        trajectory = trajectory.T.reshape((dim, 1))  # T transpose, get 7xn then dimx1 
        F_smooth = 0.5*(np.dot(trajectory.T, np.dot(fd_matrix, trajectory)) + np.dot(trajectory.T, b) + 0.25*np.dot(b.T, b)) 
	# 0.25*np.dot(b.T, b)  ?? check 0.25
        return F_smooth

    def calculate_obstacle_cost(self, trajectory):  # traj is 7xn
        obstacle_cost = 0
        trajectory = np.squeeze(trajectory) 
        trajectory = np.transpose(trajectory.reshape((7, len(trajectory) / 7))) # len(traj) is along the highest btw rows and col, after transpose it becomes nx7
        vel_normalised, vel_mag, vel = self.calculate_normalised_workspace_velocity(
            trajectory)  # vel_normalised = vel/vel_mag
        for jvs in range(len(trajectory)):
            robot_discretised_points = np.array(
                self.cost_fun.get_robot_discretised_points(trajectory[jvs], self.step_size))  #trajectory[jvs] is joint_values 1x7 at a time
            dist = self.cost_fun.compute_minimum_distance_to_objects(robot_discretised_points, self.object_list)
            obsacle_cost_potential = np.array(self.cost_fun.cost_potential(dist)) # c(x,u) in CHOMP paper
            obstacle_cost += np.sum(np.multiply(obsacle_cost_potential, vel_mag[jvs, :]))#Eq (3) in CHOMP papern discritized formalization, += is over time and sum() is among body points (vel_mag and obstacle cost potential are vectors with nb of body pts dependency)
            # obstacle_cost += np.sum(obsacle_cost_potential)
        return obstacle_cost

    def calculate_total_cost(self, trajectory):
        F_smooth = self.smoothness_objective(trajectory)  # smoothness of trajectory is captured here
        obstacle_cost = self.calculate_obstacle_cost(trajectory)
        return 10 * F_smooth + 1.5 * obstacle_cost  # Eq (1) in CHOMP paper

    def calculate_smoothness_gradient(self, trajectory):
        trajectory = np.squeeze(trajectory)
        trajectory = np.transpose(trajectory.reshape((7, len(trajectory) / 7)))
        rows, columns = trajectory.shape
        dim = rows * columns
        fd_matrix, b = self.finite_diff_matrix(trajectory)
        trajectory = trajectory.T.reshape((dim, 1))  #traj is a vector among all DoFs, traj is 7xn
        smoothness_gradient = 0.5*b + fd_matrix.dot(trajectory)  # sariah: Eq (11) in CHOMP paper ??
        return smoothness_gradient #/np.linalg.norm(smoothness_gradient)

    def calculate_normalised_workspace_velocity(self, trajectory): # points on robot body during its movement to desired conf (dx/dt)
        # We have not divided by  time as this has been indexed and is thus not available
        trajectory = np.insert(trajectory, len(trajectory), self.desired_joint_values, axis=0)
        robot_body_points = np.array([self.cost_fun.get_robot_discretised_points(joint_values, self.step_size) for joint_values in trajectory])
        velocity = np.diff(robot_body_points, axis=0)
        vel_magnitude = np.linalg.norm(velocity, axis=2)
        vel_normalised = np.divide(velocity, vel_magnitude[:, :, None], out=np.zeros_like(velocity), where=vel_magnitude[:, :, None] != 0) # out zero normalized vel where magnitude is 0
        return vel_normalised, vel_magnitude, velocity

    def calculate_curvature(self, vel_normalised, vel_magnitude, velocity): # refer Eq (12) in CHOMP paper
        time_instants, body_points, n = velocity.shape[0], velocity.shape[1], velocity.shape[2] # shape[0] is the dim of time, shape[1] is dim of body pts
        acceleration = np.gradient(velocity, axis=0)
        curvature, orthogonal_projector = np.zeros((time_instants, body_points, n)), np.zeros((time_instants, body_points, n, n)) #initialization
        for tm in range(time_instants):
            for pts in range(body_points):
                ttm = np.dot(vel_normalised[tm, pts, :].reshape(3, 1), vel_normalised[tm, pts, :].reshape(1, 3)) # x(xi, u) * x(xi, u)^T
                temp = np.eye(3) - ttm
                orthogonal_projector[tm, pts] = temp
                if vel_magnitude[tm, pts]:
                    curvature[tm, pts] = np.dot(temp, acceleration[tm, pts, :])/vel_magnitude[tm, pts]**2
                else:
                    # curv.append(np.array([0, 0, 0]))
                    curvature[tm, pts] = np.array([0, 0, 0])
        return curvature, orthogonal_projector

    def fun(self, points): # related to Eq (21)
        dist = self.cost_fun.compute_minimum_distance_to_objects(points, self.object_list)
        return np.array(self.cost_fun.cost_potential(dist))

    def gradient_cost_potential(self, robot_discretised_points): # for Eq (12) ... grad of c
        gradient_cost_potential = list() # list variable is necessary for append(), appending element at the end of the list
        for points in robot_discretised_points:
            grad = opt.approx_fprime(points, self.fun, [1e-06, 1e-06, 1e-06]) # Finite-difference approximation of the gradient of a scalar function, (xk, f, epsilon), gradient of f with respect to xk = workspace body pts, epsilon in eq (21)
            gradient_cost_potential.append(grad)
        return np.array(gradient_cost_potential)

    def calculate_mt(self, trajectory): # trajectory is nx7 for 7dof
        n_time, ndof = trajectory.shape  #
        M_t =np.zeros((n_time, ndof, n_time*ndof))
        for t in range(n_time):
            k = 0
            for d in range(ndof):
                M_t[t, d, k + t] = 1  # ???
                k = k + n_time
        return M_t

    def calculate_obstacle_cost_gradient(self, trajectory):
        trajectory = np.squeeze(trajectory)
        trajectory = np.transpose(trajectory.reshape((7, len(trajectory) / 7)))
        vel_normalised, vel_magnitude, velocity = self.calculate_normalised_workspace_velocity(trajectory)
        curvature, orthogonal_projector = self.calculate_curvature(vel_normalised, vel_magnitude, velocity)
        obstacle_gradient = list()
        for jvs in range(len(trajectory)):
            obst_grad = np.zeros(trajectory.shape[1]) # Initialization, shape[1] is nDoF and at each time step there is an obst grad for each DoF 
            robot_discretised_points, joint_index = np.array(self.cost_fun.get_robot_discretised_points(trajectory[jvs], self.step_size, with_joint_index=True))
            dist = self.cost_fun.compute_minimum_distance_to_objects(robot_discretised_points, self.object_list,)
            obstacle_cost_potential = np.array(self.cost_fun.cost_potential(dist))  ## c
            gradient_cost_potential = self.gradient_cost_potential(robot_discretised_points)  # grad of c
            jacobian = self.cost_fun.calculate_jacobian(robot_discretised_points, joint_index, trajectory[jvs])
            for num_points in range(robot_discretised_points.shape[0]):
                temp1 = orthogonal_projector[jvs, num_points].dot(gradient_cost_potential[num_points, :])
                temp2 = obstacle_cost_potential[num_points] * curvature[jvs, num_points, :]
                temp3 = vel_magnitude[jvs, num_points] * (temp1 - temp2)
                temp4 = jacobian[num_points, :].T.dot(temp3)
                obst_grad += temp4   #/np.linalg.norm(temp4)
                # obst_grad += jacobian[num_points, :].T.dot(gradient_cost_potential[num_points, :])
            obstacle_gradient.append(obst_grad)
        temp5 = np.transpose(np.array(obstacle_gradient))  # Eq (12)
        return temp5

    def cost_gradient_analytic(self, trajectory):  # calculate grad(cost) = grad(smoothness_cost) + grad(obstacle_cost)
        smoothness_gradient = self.calculate_smoothness_gradient(trajectory)
        obstacle_gradient = self.calculate_obstacle_cost_gradient(trajectory)
        trajectory = np.squeeze(trajectory)
        cost_gradient = obstacle_gradient.reshape((len(trajectory), 1)) + smoothness_gradient
        return np.squeeze(cost_gradient)

    def cost_gradient_numeric(self, trajectory):
        trajectory = np.squeeze(trajectory)
        obst_cost_grad_numeric = opt.approx_fprime(trajectory, traj_opt.calculate_obstacle_cost,
                                              1e-08 * np.ones(len(trajectory)))
        smoothness_gradient = np.squeeze(self.calculate_smoothness_gradient(trajectory))
        return np.squeeze(obst_cost_grad_numeric + smoothness_gradient)

    def animation(self, optimized_trajectory, initial_trajectory):  # initial_trajectory is from ProMP
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        # ax.axis('off')
        plt.show(block=False)
        while True:
            for i in range(len(optimized_trajectory)):
                _, T_joint_optim = self.franka_kin.fwd_kin(optimized_trajectory[i])  # T is traj ?
                _, T_joint_init = self.franka_kin.fwd_kin(initial_trajectory[i])
                ax.clear()
                self.sphere(ax, 0.05, [0.25, 0.21, 0.71])  # axis, radius , center
                self.franka_kin.plotter(ax, T_joint_optim, 'optim', color='blue')
                self.franka_kin.plotter(ax, T_joint_init, 'init', color='red')
                # for x, y, z in self.cost_fun.get_robot_discretised_points(trajectory[i],step_size=0.2):
                #     plt.grid()
                #     ax.scatter(x, y, z, 'gray')
                fig.canvas.draw()
                fig.canvas.flush_events()
                plt.pause(0.01)
            plt.pause(1)
        # plt.show(block=True)

    def optim_callback(self, xk):   # xk is the workspace body points, here we calculate total costs based on traj of body pts and not only joints
        # xk = xk.reshape(len(xk) / 7, 7)
        print("xk size: {}\n".format(len(xk)))
        costs = self.calculate_total_cost(xk) # xk as traj wll be nxlen(body pts)
        self.optCurve.append(xk)
        print("Iteration {}: {}\n".format(len(self.optCurve), costs))  # first {} refers to first element of format() and second {} refers to second element


# time index = 0; robot_discretised points index = 1, vector index = 2


if __name__ == '__main__':

    traj_opt = trajectory_optimization()

    initial_joint_values = traj_opt.initial_joint_values
    desired_joint_values = traj_opt.desired_joint_values
    trajectory = traj_opt.discretize(initial_joint_values, desired_joint_values, step_size=0.3) # waypoints parametrization as initial traj

    initial_trajectory = trajectory # at time 0
    initial_trajectory = np.insert(initial_trajectory, len(initial_trajectory), desired_joint_values, axis=0)  # inserting goal state at the end of discretized
    initial_trajectory = np.insert(initial_trajectory, 0, initial_joint_values, axis=0)  # inserting start state at the start of discretized

    trajectoryFlat = trajectory.T.reshape((-1)) # by transpose: we got 7 x n

    smoothness_grad_analytic = np.squeeze(traj_opt.calculate_smoothness_gradient(trajectoryFlat)) # analytic == as in CHOMP paper
    smoothness_grad_numeric = opt.approx_fprime(trajectoryFlat, traj_opt.smoothness_objective, 1e-08*np.ones(len(trajectoryFlat)))  # numeric is grad calculation based on Finite difference approx

    obst_cost_grad_analytic = np.reshape(traj_opt.calculate_obstacle_cost_gradient(trajectoryFlat), -1)
    obst_cost_grad_numeric = opt.approx_fprime(trajectoryFlat, traj_opt.calculate_obstacle_cost,
                                                1e-08 * np.ones(len(trajectoryFlat)))

    print('obst_cost_grad_analytic {} \n'.format(obst_cost_grad_analytic))  # gradient cost of initialized traj 

    print('obst_cost_grad_numeric {} \n'.format(obst_cost_grad_numeric)) # gradient cost of initialized traj

    tmp_var = input('You can terminate this run here!!') # without optimization of initialized traj, enter a number  

    # smoothness_grad_numeric1 = np.zeros(trajectoryFlat.shape)
    # for i in range(trajectoryFlat.shape[0]):
    #     trajectoryTemp = trajectoryFlat.copy()
    #     trajectoryTemp[i] = trajectoryTemp[i] + 10**-8
    #     val1 = traj_opt.smoothness_objective(trajectoryTemp)
    #     trajectoryTemp[i] = trajectoryTemp[i] - 2 * 10 ** -8
    #     val2 = traj_opt.smoothness_objective(trajectoryTemp)
    #     smoothness_grad_numeric1[i] = (val1 - val2) / (2 * 10 ** -8)



    ################### With Gradient ############################
    # minimize total cost
    # scipy.minimize(fun, x0, args=(), method='BFGS', jac=None, tol=None, callback=None, options={'gtol': 1e-05, 'norm': inf, 'eps': 1.4901161193847656e-08, 'maxiter': None, 'disp': False, 'return_all': False})
    # BFGS solver uses a limited computer memory and it is a popular algorithm for parameter estimation in machine learning.
    # disp : Set to True to print convergence messages.
    # mixter :max nb of iterations
    # gtol : Gradient norm must be less than gtol before successful termination.

    optimized_trajectory = opt.minimize(traj_opt.calculate_total_cost, trajectoryFlat, method='BFGS', jac=traj_opt.cost_gradient_analytic, options={'maxiter': 15, 'disp': True}, callback=traj_opt.optim_callback)

    optimized_trajectory = np.transpose(optimized_trajectory.x.reshape((7, len(optimized_trajectory.x) / 7)))
    optimized_trajectory = np.insert(optimized_trajectory, 0, initial_joint_values, axis=0)
    optimized_trajectory = np.insert(optimized_trajectory, len(optimized_trajectory), desired_joint_values, axis=0)
    print('result of optimizaed trajectory {}\n'.format(optimized_trajectory))
    traj_opt.animation(optimized_trajectory, initial_trajectory)

    print('Finished plotting the optimized trajectory with Gradient passed to the minimization Algo')

    ############ Without Gradient ##########################
    optimized_trajectory = opt.minimize(traj_opt.calculate_total_cost, trajectoryFlat, method='BFGS',
                 options={'maxiter': 1, 'disp': True}, callback=traj_opt.optim_callback)
    jac = optimized_trajectory.jac
    # total_cost_gradient = traj_opt.cost_gradient_numeric(optimized_trajectory.x)
    smoothness_gradient = np.squeeze(traj_opt.calculate_smoothness_gradient(optimized_trajectory.x))
    obstacl_cost_grad_numeric = opt.approx_fprime(optimized_trajectory.x, traj_opt.calculate_obstacle_cost,
                                               1e-08 * np.ones(len(optimized_trajectory.x)))
    print(jac - (smoothness_gradient + obstacl_cost_grad_numeric))  # jac (i.e numeric gradient of resulted optimized traj) - total numeric cost gradient of the optimized traj

    optimized_trajectory = np.transpose(optimized_trajectory.x.reshape((7, len(optimized_trajectory.x) / 7)))
    optimized_trajectory = np.insert(optimized_trajectory, 0, initial_joint_values, axis=0)
    optimized_trajectory = np.insert(optimized_trajectory, len(optimized_trajectory), desired_joint_values, axis=0)
    print('result \n', optimized_trajectory)
    traj_opt.animation(optimized_trajectory, initial_trajectory)

    print('finished')
