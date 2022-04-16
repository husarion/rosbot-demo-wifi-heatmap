from collections import namedtuple
import cv2
from matplotlib import pyplot as plt
import matplotlib
from matplotlib.colors import LinearSegmentedColormap
import matplotlib.cm as cm
import numpy as np
import scipy.interpolate

RssiWaypoint = namedtuple('RssiWaypoint','x y rssi')

# sample_data = [(RssiWaypoint(x,y,-(x*y)))for x in range(0,10,2) for y in range(0,20,4)]

#Custom red to green colormap
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

#Function generating rgb image based on rssi data array
def rssi2rgb(data,cmap):
    xsize,ysize = np.shape(data)[0], np.shape(data)[1]
    norm = matplotlib.colors.Normalize(vmin=-100,vmax=0,clip = True) #Normalize data to (-100,0)
    mapper = cm.ScalarMappable(norm = norm,cmap = cmap)
    rgb_image = np.zeros((xsize,ysize,3))    
    for i,_ in enumerate(data):
        for j,point in enumerate(data[i]):
            rgb_image[i][j] =  mapper.to_rgba(point)[: -1]
    return rgb_image

#Main function generating continous heatmap from discrete rssi data
def generate_heatmap(data,x_image_size,y_image_size,resolution_coeff):
    blank_image = np.zeros((x_image_size,y_image_size)) #Set empty image
    for waypoint in data:
        blank_image[waypoint.x][waypoint.y] = waypoint.rssi #DEBUG mark datapoints
    
    # rgb_blank_image = rssi2rgb(blank_image,cmapGR)

    xdata,ydata,valdata = [],[],[] #Data arrays for interpolation
    for datapoint in data: #Fill data arrays with measurement data
        xdata.append(datapoint.x)
        ydata.append(datapoint.y)
        valdata.append(datapoint.rssi)

    f = scipy.interpolate.interp2d(xdata,ydata,valdata,kind = 'cubic') #Interpolation result function
    xpoints = np.arange(0,x_image_size,1) #Set data arrays for interpolation according to image size 
    ypoints = np.arange(0,y_image_size,1)
    interp_data = f(xpoints,ypoints) #Interpolate over data arrays using f function
    interp_data = np.transpose(interp_data)
    # rgb_interp_data = rssi2rgb(interp_data,cmapGR)
#Resize and smoothe out image
    resized_interp_data = cv2.resize(interp_data,(resolution_coeff*y_image_size,resolution_coeff*x_image_size),interpolation=cv2.INTER_CUBIC)
    rgb_resized_interp_data = rssi2rgb(resized_interp_data,cmapGR)
                                                            
# DEBUG Show results
    # fig,axs = plt.subplots(2,2)
    # axs[0][0].imshow(interp_data,cmap = cmapGR) #Set colouring limits using vmax,vmin
    # axs[0][0].set_title('original')
    # axs[0][0].axis("off")
    # axs[0][1].imshow(rgb_interp_data) #Set colouring limits using vmax,vmin
    # axs[0][1].set_title('interpolation result')
    # axs[0][1].axis("off")   
    # axs[1][0].imshow(resized_interp_data,cmap = cmapGR) #Set colouring limits using vmax,vmin
    # axs[1][0].set_title('resized')
    # axs[1][0].axis("off")
    # axs[1][1].imshow(rgb_resized_interp_data) #Set colouring limits using vmax,vmin
    # axs[1][1].set_title('smoothed out')
    # axs[1][1].axis("off")
    # plt.show()
# DEBUG
    return rgb_resized_interp_data

# generate_heatmap(sample_data,10,20,5)





