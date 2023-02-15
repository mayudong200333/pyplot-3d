from basic import Sphere, Line, Arrow

import numpy as np

def rotx(theta):
    mx = np.array([[1,0,0],
                   [0,np.cos(theta),-np.sin(theta)],
                   [0,np.sin(theta),np.cos(theta)]])
    return mx

def roty(theta):
    my = np.array([[np.cos(theta),0,np.sin(theta)],
                   [0,1,0],
                   [-np.sin(theta),0,np.cos(theta)]])
    return my

def rotz(theta):
    mz = np.array([[np.cos(theta),-np.sin(theta),0],
                   [np.sin(theta),np.cos(theta),0],
                   [0,0,1]])
    return mz


class Uav:
    '''
    Draws a quadrotor at a given position, with a given attitude.
    '''

    def __init__(self, ax, arm_length):
        '''
        Initialize the quadrotr plotting parameters.

        Params:
            ax: (matplotlib axis) the axis where the sphere should be drawn
            arm_length: (float) length of the quadrotor arm

        Returns:
            None
        '''

        self.ax = ax
        self.arm_length = arm_length

        self.b1 = np.array([-1.0, 0.0, 0.0]).T
        self.b2 = np.array([0.0, 1.0, 0.0]).T
        self.b3 = np.array([0.0, 0.0, -1.0]).T
        
        self.c1 = np.array([0.0,0.0,-0.25]).T
        self.c2 = np.array([0.0,0.0,-0.25]).T
        self.c3 = np.array([0.0,0.0,-0.25]).T
        self.c4 = np.array([0.0,0.0,-0.25]).T
        

        # Center of the quadrotor
        self.body = Sphere(self.ax, 0.01, 'y')

        # Each motor
        self.motor1 = Sphere(self.ax, 0.01, 'b')
        self.motor2 = Sphere(self.ax, 0.01, 'b')
        self.motor3 = Sphere(self.ax, 0.01, 'b')
        self.motor4 = Sphere(self.ax, 0.01, 'b')

        # Arrows for the each body axis
        self.arrow_b1 = Arrow(ax, self.b1, 'r')
        self.arrow_b2 = Arrow(ax, self.b2, 'g')
        self.arrow_b3 = Arrow(ax, self.b3, 'b')
        
        self.arrow_c1 = Arrow(ax,self.c1,'b',x0=np.array([0.0,self.arm_length,0.0]).T)
        self.arrow_c2 = Arrow(ax,self.c2,'r',x0=np.array([self.arm_length,0.0,0.0]).T)
        self.arrow_c3 = Arrow(ax,self.c3,'purple',x0=np.array([0.0,-self.arm_length,0.0]).T)
        self.arrow_c4 = Arrow(ax,self.c4,'g',x0=np.array([-self.arm_length,0.0,0.0]).T)

        # Quadrotor arms
        self.arm_b1 = Line(ax)
        self.arm_b2 = Line(ax)
    

    def draw_at(self, x=np.array([0.0, 0.0, 0.0]).T, R=np.eye(3),tiltingAngels=np.array([0.0,0.0,0.0,0.0])):
        '''
        Draw the quadrotor at a given position, with a given direction

        Args:
            x: (3x1 numpy.ndarray) position of the center of the quadrotor, 
                default = [0.0, 0.0, 0.0]
            R: (3x3 numpy.ndarray) attitude of the quadrotor in SO(3)
                default = eye(3)
        
        Returns:
            None
        '''

        # First, clear the axis of all the previous plots
        self.ax.clear()

        # Center of the quadrotor
        self.body.draw_at(x)
        #
        # Each motor
        self.motor1.draw_at(x + R.dot(self.b1) * self.arm_length)
        self.motor2.draw_at(x + R.dot(self.b2) * self.arm_length)
        self.motor3.draw_at(x + R.dot(-self.b1) * self.arm_length)
        self.motor4.draw_at(x + R.dot(-self.b2) * self.arm_length)

        # Arrows for the each body axis
        # self.arrow_b1.draw_from_to(x, R.dot(self.b1) * self.arm_length * 1.8)
        # self.arrow_b2.draw_from_to(x, R.dot(self.b2) * self.arm_length * 1.8)
        # self.arrow_b3.draw_from_to(x, R.dot(self.b3) * self.arm_length * 1.8)

        # Quadrotor arms
        self.arm_b1.draw_from_to(np.array([-self.arm_length, 0.0, 0.0]), x + R.dot(-self.b1) * self.arm_length)
        self.arm_b2.draw_from_to(np.array([0.0, self.arm_length, 0.0]), x + R.dot(-self.b2) * self.arm_length)
        
        
        self.cd1 = roty(tiltingAngels[0]).dot(self.c1)
        self.cd2 = rotx(tiltingAngels[1]).dot(self.c2)
        self.cd3 = roty(-tiltingAngels[2]).dot(self.c3)
        self.cd4 = rotx(-tiltingAngels[3]).dot(self.c4)
        self.arrow_c1.draw_from_to(np.array([0.0,self.arm_length,0.0]).T,self.cd1)
        self.arrow_c2.draw_from_to(np.array([self.arm_length,0.0,0.0]).T,self.cd2)
        self.arrow_c3.draw_from_to(np.array([0.0,-self.arm_length,0.0]).T,self.cd3)
        self.arrow_c4.draw_from_to(np.array([-self.arm_length,0.0,0.0]).T,self.cd4)
    



if __name__ == '__main__':
    from utils import ypr_to_R

    from matplotlib import animation
    from mpl_toolkits.mplot3d import Axes3D

    import matplotlib.pyplot as plt
    
    angles = np.loadtxt('./FL6DoF.csv', delimiter=',')
    allsteps = len(angles)
    angles = angles.T
    
    def update_plot(i, x, R, tiltingAngels):
        uav_plot.draw_at(x[:, i], R[:, :, i],tiltingAngels[:,i])
        
        # These limits must be set manually since we use
        # a different axis frame configuration than the
        # one matplotlib uses.
        xmin, xmax = -0.5, 0.5
        ymin, ymax = -0.5, 0.5
        zmin, zmax = -0.5, 0.5

        ax.set_xlim([xmin, xmax])
        ax.set_ylim([ymax, ymin])
        ax.set_zlim([zmax, zmin])

    # Initiate the plot
    plt.style.use('seaborn')

    fig = plt.figure()
    ax = fig.gca(projection='3d')

    arm_length = 0.24  # in meters
    uav_plot = Uav(ax, arm_length=0.1785)


    # Create some fake simulation data
    t_end = 1
    steps = 1000

    x = np.zeros((3, steps))
    x[0, :] = np.arange(0, t_end, t_end / steps) * 0
    x[1, :] = np.arange(0, t_end, t_end / steps) * 0

    R = np.zeros((3, 3, steps))
    for i in range(steps):
        ypr = np.array([i*0, 0.1 * i*0, 0.0])
        R[:, :, i] = ypr_to_R(ypr, degrees=True)
        
    tiltingAngles = np.zeros((4,steps))
    interval = allsteps // steps
    initial = 0
    for i in range(steps):
        tiltingAngles[:,i] = angles[:,initial]
        initial += interval
    
    # Run the simulation
    ani = animation.FuncAnimation(fig, update_plot, frames=300, repeat=False, interval=1, \
        fargs=(x, R,tiltingAngles))
    
    ani.save('shabi.gif',writer='pillow',fps=30)