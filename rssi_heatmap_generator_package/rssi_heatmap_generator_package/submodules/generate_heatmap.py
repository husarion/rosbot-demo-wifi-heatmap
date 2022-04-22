from collections import namedtuple
from copy import copy
import cv2
from matplotlib import pyplot as plt
import matplotlib
from matplotlib.colors import LinearSegmentedColormap
import matplotlib.cm as cm
import numpy as np
import scipy.interpolate
from cmath import inf

RssiWaypoint = namedtuple('RssiWaypoint','x y rssi')

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
def rssi2rgb(data,cmap,minval = -100,maxval = 0):
    xsize,ysize = np.shape(data)[0], np.shape(data)[1]
    norm = matplotlib.colors.Normalize(vmin=minval,vmax=maxval,clip = True) #Normalize data to (-100,0)
    mapper = cm.ScalarMappable(norm = norm,cmap = cmap)
    rgb_image = np.zeros((xsize,ysize,3),dtype='uint8')    
    for i,_ in enumerate(data):
        for j,point in enumerate(data[i]):
            rgb_image[i][j] =  mapper.to_rgba(point,bytes=True)[: -1]
    return rgb_image

#Main function generating continous heatmap from discrete rssi data
def generate_heatmap(data,x_image_size,y_image_size,resize_coeff,filtered:bool = False,relative = False):

    minrssi = inf
    maxrssi = -100
    xdata,ydata,valdata = [],[],[] #Data arrays for interpolation
    for datapoint in data: #Fill data arrays with measurement data
        if datapoint.rssi > maxrssi : maxrssi = datapoint.rssi
        if datapoint.rssi < minrssi : minrssi = datapoint.rssi
        xdata.append(datapoint.x)
        ydata.append(datapoint.y)
        valdata.append(datapoint.rssi)
# Stack data into [N,2] dimension for RBFInterpolator 
    RBFdata = np.stack((np.array(xdata).ravel(),np.array(ydata).ravel()),-1)
# Get interpolant
    f = scipy.interpolate.RBFInterpolator(RBFdata,np.array(valdata).ravel())
# Prepare data arrays to pass to f
    xpoints,ypoints = np.meshgrid(np.arange(0,x_image_size,1),np.arange(0,y_image_size,1))
    RBFpoints = np.stack((np.array(xpoints).ravel(),np.array(ypoints).ravel()),-1)
# Get result from function
    interp_data = f(RBFpoints).reshape(np.array(xpoints).shape)
# Change rssi vals to RGB
# If absolute get rgb vals in range (minrssi,maxrssi), else get vals from range(-100,0)
    if relative:
        rgb_interp_data = rssi2rgb(interp_data,cmapGR,minrssi,maxrssi)
    else:
        rgb_interp_data = rssi2rgb(interp_data,cmapGR,minrssi,maxrssi)
# Add median filter
    if filtered:
        rgb_interp_data = cv2.medianBlur(rgb_interp_data,7)
# Resize image
    rgb_interp_data = cv2.resize(rgb_interp_data,(resize_coeff*y_image_size,resize_coeff*x_image_size),interpolation=cv2.INTER_CUBIC)

    return rgb_interp_data

#Fuction for adding obsctacles from saved map to generated heatmap
def add_heatmap(map,heatmap):
    newmap = copy(map)
    if np.shape(map) != np.shape(heatmap):
        if np.shape(map)[0] / np.shape(heatmap)[0] == np.shape(map)[1] / np.shape(heatmap)[1]:
            print("Heatmap resized, rsizing map...")
            resize_coeff = np.shape(heatmap)[0] / np.shape(map)[0]
            newmap = cv2.resize(newmap,(int(resize_coeff*len(newmap[0])),int(resize_coeff*len(newmap))),interpolation=cv2.INTER_AREA)
        elif np.shape(map) != np.shape(heatmap):
            raise Exception("Map sizes do not match, map size is {msize} and heatmap size is {hsize}".format(
                msize = np.shape(map),hsize = np.shape(heatmap)))
    for i,_ in enumerate(newmap):
        for j,_ in enumerate(newmap[0]):
            if newmap[i][j][0] == 254:
                newmap[i][j] = heatmap[i][j]
    return newmap

#Function for diplaying measured data on map
def add_waypoints(map,waypoints):
    newmap = copy(map)
    norm = matplotlib.colors.Normalize(vmin=-100,vmax=0,clip = True) #Normalize data to (-100,0)
    mapper = cm.ScalarMappable(norm = norm,cmap = cmapGR)
    for waypoint in waypoints:
        newmap[waypoint.y,waypoint.x] = mapper.to_rgba(waypoint.rssi,bytes=True)[: -1]
    return newmap

        





