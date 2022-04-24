# rosbot-demo-wifi-heatmap
Using ROSbot for automatically creation of a Wi-Fi signal strength (RSSI) heatmap of the given area.
## Setup
1. Create husarnet network according to [this tutorial](https://husarnet.com/docs/begin-linux)\
2. Clone contents of this repository to pc and ROSbot:
~~~ bash
git clone https://github.com/husarion/rosbot-demo-wifi-heatmap.git
~~~

3. Create '.env' file inside 'docker-compose' folder and paste your husarnet joincode
On pc:
    - create rssi mappers image:
    ~~~ bash
    docker build -t rssi_mappers:latest rosbot-demo-wifi-heatmap/mapper-packages
    ~~~
    - enabale running rviz inside a container:
    ~~~ bash
    xhost local:root
    ~~~
 On ROSbot:
    - create custom nav2 image:
    ~~~ bash
    docker build -t rssi_map_nav2:latest rosbot-demo-wifi-heatmap/nav2-wifi-heatmap
    ~~~
    - run bash script exposing /proc/net/wireless data to container:
    ~~~
    chmod +x rosbot-demo-wifi-heatmap/fakenet.sh
    rosbot-demo-wifi-heatmap/fakenet.sh &
    ~~~
    
## Launch 
On pc:
~~~ bash
cd rosbot-demo-wifi-heatmap
docker compose -f compose.pc.yaml -f compose.pc.husarnet.yaml up
~~~
On rosbot:
~~~ bash
cd rosbot-demo-wifi-heatmap
docker compose -f compose.rosbot.yaml -f compose.rosbot.husarnet.yaml up
~~~
## Usage
1. Map desired area using rviz goal pose
2. Begin autonomous data collection and heatmap generation using:
~~~ bash
 docker exec docker-compose-mappers-1 /run.sh
~~~
3. After process is finished, heatmaps will be saved in folder selected as bind mounts in 'compose.pc.yaml' file from '/heatmaps' directory inside the container
