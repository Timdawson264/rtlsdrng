"""
A simple example of an animated plot
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

fig, ax = plt.subplots()

x_axis = np.arange(0, 2*np.pi, 0.01)        # x-array
line, = ax.plot(x_axis, np.sin(x_axis))

def animate(i):
    
    #adds i/10 to every element
    line.set_ydata(np.sin(x_axis+i/10))  # update the data

ani = animation.FuncAnimation(fig, animate, np.arange(1, 200),  interval=25)
    
plt.show()
