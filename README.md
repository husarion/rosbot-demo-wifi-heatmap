# rosbot-demo-wifi-heatmap
This project allows to measure and display RSSI (Received Signal Strength Indication) of a given Wi-Fi network in the desired area, using ROSbot 2 PRO and PC, connected over a peer-to-peer [Husarnet VPN network](https://husarnet.com/). The entire software is based on [rosbot-pro-demo](https://github.com/DominikN/rosbot-pro-demo.git)

## Setup
1. Create Husarnet network according to [this tutorial](https://husarnet.com/docs/begin-linux)
2. Clone contents of this repository to pc and ROSbot:
~~~ bash
git clone https://github.com/husarion/rosbot-demo-wifi-heatmap.git
~~~

3. Create '.env' file inside 'docker-compose' folder, paste your Husarnet joincode and generate DDS config files:
    ~~~
    ./generate-vpn-config.sh
    ~~~
This will create `secrets` file, which you need to copy to the same directory on another device.

4.  On ROSbot:
    Run bash script exposing /proc/net/wireless data to the container:
    ~~~ bash
    chmod +x rosbot-demo-wifi-heatmap/net_expose.sh
    rosbot-demo-wifi-heatmap/net_expose.sh &
    ~~~
5. On pc:
   Run script enabling RViz:
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
At first an appropriate map of the desired area needs to be created using RViz goal pose, which allows the user to pass waypoints to the robot. While reaching those waypoints ROSbot will map the area around it thanks to the SLAM toolbox. Several important things must be considered while mapping and choosing environment to map:
    - Points too close to obstacles or unknown areas will not be marked as waypoints
    - The fewer obstacles, the better because ROSbot will be able to measure RSSI in a more waypoints, resulting in more accurate heatmap at the end
    - Remember that some obstacles are invisible to ROSbot's lidar for example transparent surfaces or obstacles located under the lidar's view
2. Measuring:
After the desired area is mapped, execute the following command on your pc to start the process of autonomous measurement:
    
~~~ bash
 docker exec docker-compose-mappers-1 /run.sh
~~~

After running this command a map will be displayed with waypoints marked. Green ones are actual waypoints marked for measurement, while the red ones were considered too close to obstacles or unknown areas. Remember that in the areas with fewer waypoints, data will also be less accurate (less data for interpolation). ROSbot will start going through all green waypoints, measuring RSSI for ten seconds in each one and sending data to a ROS2 node on pc.

3. After all the waypoints have been achieved, two heatmaps will be generated based on the collected data. Raw images will be saved in '/heatmaps' repository. For full figures with heatbars use the SAVE button in matplotlib GUI and save images in '/heatmaps' directory, which at default is mounted on the folder with the same name in your home directory.
## Results
Here are the results of RSSI measurement in our office:
![result1](/sample-images/Figure_1.png)
![result2](/sample-images/Figure_2.png)

