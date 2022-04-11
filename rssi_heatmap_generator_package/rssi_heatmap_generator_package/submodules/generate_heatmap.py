from collections import namedtuple
import cv2
from matplotlib import pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
import numpy as np
import scipy.interpolate

RssiWaypoint = namedtuple('RssiWaypoint','x y rssi')

sample_data = [(RssiWaypoint(x,y,-(x*y*2)))for x in range(0,10,2) for y in range(0,20,4)]

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

    f = scipy.interpolate.interp2d(xdata,ydata,valdata,kind = 'cubic') #Interpolation result function
    xpoints = np.arange(0,x_image_size,1) #Set data arrays for interpolation according to image size 
    ypoints = np.arange(0,y_image_size,1)
    interp_data = f(xpoints,ypoints) #Interpolate over data arrays using f function
#Ensure that none of interpolated values are out of bounds (-100,0)
    for i,_ in enumerate(interp_data):
        for j,_ in enumerate(interp_data[i]):
            if interp_data[i][j] > 0: interp_data[i][j] = 0
            if interp_data[i][j] < -100: interp_data[i][j] = -100

    interp_data = np.transpose(interp_data)
    resized_interp_data = cv2.resize(interp_data,(resolution_coeff*y_image_size,resolution_coeff*x_image_size),interpolation=cv2.INTER_AREA)
#DEBUG Show results
    fig,axs = plt.subplots(2,2)
    axs[0][0].imshow(blank_image,cmap = cmapGR) #Set colouring limits using vmax,vmin
    axs[0][0].set_title('original')
    axs[0][0].axis("off")
    axs[0][1].imshow(interp_data,cmap = cmapGR) #Set colouring limits using vmax,vmin
    axs[0][1].set_title('interpolation result')
    axs[0][1].axis("off")   
    axs[1][0].imshow(resized_interp_data,cmap = cmapGR) #Set colouring limits using vmax,vmin
    axs[1][0].set_title('resized')
    axs[1][0].axis("off")
    axs[1][1].imshow(cv2.blur(resized_interp_data,(31,31)),cmap = cmapGR) #Set colouring limits using vmax,vmin
    axs[1][1].set_title('smoothed out')
    axs[1][1].axis("off")
    plt.show()
#DEBUG
    return cv2.GaussianBlur(resized_interp_data,(31,31),0) # Add arg to set mask size ???


generate_heatmap(sample_data,10,20,5)