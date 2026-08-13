# Isaac sim robot navigáció

isaac sim 5.1: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html

g1 robot: https://github.com/unitreerobotics/unitree_sim_isaaclab

WSL Ubuntu 22.04 kell ehhez a projekthez.

## ros2, nav2, slam telepítés

wsl-ben:
```
sudo apt install software-properties-common curl -y
sudo add-apt-repository universe -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```
```
sudo apt install ros-humble-desktop -y
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup -y
sudo apt install ros-humble-slam-toolbox -y
sudo apt install ros-dev-tools -y
```
Automatikus betöltés beállítása:
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
source ~/.bashrc
```

## Mirrored networking:

Új fájl ide (szóköz nélkül):

C:\Users\username\ .wslconfig

tartalma legyen:
```
[wsl2]
networkingMode=mirrored
```
Ezután wsl ugyanazt az IP-t kell mutassa mint a windows. Így ellenőrizhető:
```
hostname -I
```

## Fast DDS beállítás

wsl-ben:
```
cat > ~/fastdds.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="participant_profile" is_default_profile="true">
        <rtps>
            <builtin>
                <metatrafficUnicastLocatorList>
                    <locator>
                        <udpv4>
                            <address>ip address</address>
                        </udpv4>
                    </locator>
                </metatrafficUnicastLocatorList>
                <initialPeersList> 
                    <locator>
                        <udpv4>
                            <address>ip address</address>
                        </udpv4>
                    </locator>
                </initialPeersList>
            </builtin>
            <defaultUnicastLocatorList>
                <locator>
                    <udpv4>
                        <address>ip address</address>
                    </udpv4>
                </locator>
            </defaultUnicastLocatorList>
        </rtps>
    </participant>
</profiles>
EOF
echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=/home/username/fastdds.xml' >> ~/.bashrc
source ~/.bashrc
```
Windowson is egy ugyanilyen tartalmú fájlt, ide: C:\Users\username\fastdds.xml

## Környezeti változók

1
- név: ROS_DOMAIN_ID
- érték: 0

2
- név: FASTRTPS_DEFAULT_PROFILES_FILE
- érték: C:\Users\username\fastdds.xml

3
- RMW_IMPLEMENTATION
- rmw_fastrtps_cpp

## Isaac sim indítás

isaac-sim.selector.bat

- ROS Bridge Extension: isaacsim.ros2.bridge
- Use Internal ROS2 Libraries: humble
- Start

Miután elindult, Window -> extensions -> ROS2 BRIDGE enabled, akkor jó.

## Navigation stage

A g1_navigation_stage.usd jelenetben van egy g1 robot, amin van egy lidar szenzor. Van 2 Action Graph, az egyik a /clock, a másik pedig a /tf, és /scan topic-okat publikálja. Miközben megy a szimuláció, wsl-en meg lehet nézni a ros2 topic-okat.
```
ros2 daemon start
```
Pár mp várakozás
```
ros2 topic list
```
Erre ezt kell kapni:
```
/clock
/parameter_events
/rosout
/scan
/tf
```
/scan: A LiDAR által érzékelt környezet pontfelhőjét tartalmazó ROS 2 topic.

/tf: A robot különböző részeinek egymáshoz és a világhoz viszonyított pozícióját és orientációját tartalmazó ROS 2 topic.

/clock: A szimulációs időt tartalmazó ROS 2 topic.

## Point cloud to laser scan

A /scan topic point cloud-ot publikál, de a SLAM-hez laser scan kell.

Ezt telepítettem:
```
sudo apt install ros-humble-pointcloud-to-laserscan -y
```
Egyik terminálon ez futott:
```
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
  --ros-args \
  -r cloud_in:=/scan \
  -r scan:=/scan_2d \
  -p target_frame:=sensor \
  -p transform_tolerance:=1.0 \
  -p min_height:=-10.0 \
  -p max_height:=10.0 \
  -p range_min:=0.05 \
  -p range_max:=100.0 \
  -p use_sim_time:=true
```
Másikon:
```
ros2 topic echo /scan_2d --once
```
De nem jött rá válasz, szóval ez még nem jó.

Idáig jutottamm.