# rosbot-demo-wifi-heatmap
Using ROSbot for automatically creation of a Wi-Fi signal strength (RSSI) heatmap of the given area. Entire software is based on [rosbot-pro-demo](https://github.com/DominikN/rosbot-pro-demo.git)

## Setup
1. Create husarnet network according to [this tutorial](https://husarnet.com/docs/begin-linux)
2. Clone contents of this repository to pc and ROSbot:
~~~ bash
git clone https://github.com/husarion/rosbot-demo-wifi-heatmap.git
~~~

3. Create '.env' file inside 'docker-compose' folder and paste your husarnet joincode

4.  On ROSbot:
    Run bash script exposing /proc/net/wireless data to container:
    ~~~ bash
    chmod +x rosbot-demo-wifi-heatmap/net_expose.sh
    rosbot-demo-wifi-heatmap/net_expose.sh &
    ~~~
5. On pc:
   Run script enabling rviz:
   ~~~ bash
   xhost local:root
   ~~~
   
## Launch 
On pc:
~~~ bash
cd rosbot-demo-wifi-heatmap/docker-compose
docker compose -f compose.pc.yaml -f compose.pc.husarnet.yaml up
~~~
On rosbot:
~~~ bash
cd rosbot-demo-wifi-heatmap/docker-compose
docker compose -f compose.rosbot.yaml -f compose.rosbot.husarnet.yaml up
~~~
## Usage
1. Mapping:
At first an appropriate map of dseired area needs to be created using rviz goal pose, which allows user to pass waypoints to robot. While reaching those waypoints ROSbot will map area around it thanks to SLAM toolbox. Several imortant things must be considered while mapping and choosing environment to map:
    - Points too close to obstacles or unknown areas will not be marked as waypoints
    - The less obstacles, the better because ROSbot will be able to measure RSSI in more waypoints, resulting in more accurate heatmap at the end
    - Remember that some obstacles are invisible to ROSbot's lidar for exemple transparent surfaces, or obstacles located under lidar's view
2. Measuring:
After desired area is mapped execute following command on your pc will start the process of autonomous measurement:
    
~~~ bash
 docker exec docker-compose-mappers-1 /run.sh
~~~

After running this command a map will be displayed with waypoints marked. Green ones are actual waypoints marked for measurement, while the red ones were considered too close to obstacles or unknown areas. Remember that in the areas with less waypoints, data will also be less acurate (less data for interpolation). ROSbot will start going through all green waypoints, measuring RSSI for ten seconds in each one and sending data to a ROS2 node on pc.

3. After all waypoints have been achieved, two heatmaps will be generated based on collected data. Raw images will be saved in '/heatmaps' repistory. For full figures with heatbars use SAVE button in matplotlib GUI and save images in '/heatmaps' directory, which at default is mounted on folder with the same name in your home directory.
## Results
Here are results of rssi measurement in our office:
![result1](/sample-images/Figure_1.png)
![result2](/sample-images/Figure_2.png)

