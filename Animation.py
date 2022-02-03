import numpy as np 
from matplotlib import pyplot as plt 

x_data = []
y_data = []
x_y_color = []


fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111)
plt.ion()    
ax.set_xlim(0,200)
ax.set_ylim(0,200)
ticks = np.arange(0, 200, 10)
ax.set_xticks(ticks)
ax.set_yticks(ticks)
ax.grid()
plt.autoscale(False)
plt.show()
    
def animate_scan(x, y,cl, mr='s'):
     x_data.append(x)
     y_data.append(y)
     x_y_color.append(cl)
     plt.scatter(x_data,y_data,c=x_y_color,marker=mr)
     fig.canvas.draw()
#      plt.ioff()
#      plt.show()
    
if __name__ == '__main__':
    for i in range(0, 20, 1):
        animate_scan(i, i+10, cl='black', mr='d')
    plt.ioff()
    plt.show()
