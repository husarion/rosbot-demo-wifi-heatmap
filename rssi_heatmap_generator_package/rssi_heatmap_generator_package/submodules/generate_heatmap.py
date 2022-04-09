from collections import namedtuple
import cv2
from matplotlib import pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
import numpy as np
import scipy.interpolate

RssiWaypoint = namedtuple('RssiWaypoint','x y rssi')

sample_data = [
    RssiWaypoint(0,0,-10),
    RssiWaypoint(0,4,-20),
    RssiWaypoint(0,8,-30),
    RssiWaypoint(0,12,-40),
    RssiWaypoint(0,16,-60),
    RssiWaypoint(9,4,-60),
    RssiWaypoint(9,8,-40),
    RssiWaypoint(9,12,-30),
    RssiWaypoint(9,16,-20),
    RssiWaypoint(5,0,-10),
    RssiWaypoint(5,4,20),
    RssiWaypoint(5,8,-20),
    RssiWaypoint(5,12,-20),
    RssiWaypoint(5,16,-20),
    RssiWaypoint(5,19,-20),
]

def generate_heatmap(data,x_image_size,y_image_size,resolution_coeff):
    blank_image = np.zeros((x_image_size,y_image_size,1)) #Set empty image

    for waypoint in sample_data:
        blank_image[waypoint.x][waypoint.y] = waypoint.rssi #DEBUG mark datapoints
    
    cmapGR = LinearSegmentedColormap( #Colour map 
    'GreenRed',
    {
        'red':  ((0.0, 1.0, 1.0),
                (0.5, 1.0, 1.0),
                (1.0, 0.0, 0.0)),

        'green':((0.0, 0.0, 0.0),
                (0.5, 1.0, 1.0),
                ( 1.0, 1.0, 1.0)),
                
        'blue': ((0.0, 0.0, 0.0),
                (0.5, 0.0, 0.0),
                (1.0, 0.0, 0.0))
    },)

    xdata,ydata,valdata = [],[],[] #Data arrays for interpolation
    for datapoint in data: #Fill data arrays with measurement data
        xdata.append(datapoint.x)
        ydata.append(datapoint.y)
        valdata.append(datapoint.rssi)

    f = scipy.interpolate.interp2d(xdata,ydata,valdata,kind = 'linear') #Interpolation result function
    xpoints = np.arange(0,x_image_size,1) #Set data arrays for interpolation according to image size 
    ypoints = np.arange(0,y_image_size,1)
    interp_data = f(xpoints,ypoints) #Interpolate over data arrays using f function
    interp_data = np.transpose(interp_data)
    resized_interp_data = cv2.resize(interp_data,(resolution_coeff*y_image_size,resolution_coeff*x_image_size),interpolation=cv2.INTER_AREA)
#DEBUG Show results
    fig,axs = plt.subplots(2,2)
    axs[0][0].imshow(blank_image,vmin = -60,vmax = 0,cmap = cmapGR) #Set colouring limits using vmax,vmin
    axs[0][1].set_title('original')
    axs[0][1].imshow(interp_data,vmin = -60,vmax = 0,cmap = cmapGR) #Set colouring limits using vmax,vmin
    axs[1][0].set_title('interpolation result')
    axs[1][0].imshow(resized_interp_data,vmin = -60,vmax = 0,cmap = cmapGR) #Set colouring limits using vmax,vmin
    axs[0][0].set_title('resized')
    axs[1][1].imshow(cv2.GaussianBlur(resized_interp_data,(31,31),0),vmin = -60,vmax = 0,cmap = cmapGR) #Set colouring limits using vmax,vmin
    axs[1][1].set_title('smoothed out')
    plt.show()
#DEBUG
    return cv2.GaussianBlur(resized_interp_data,(31,31),0) # Add arg to set mask size ???


generate_heatmap(sample_data,10,20,5)