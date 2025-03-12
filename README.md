### Run planning

1. To start the servo demo
    
    ```bash
    cd ~/ws_humble/
    source install/setup.bash
    ros2 launch dual_arm_panda_moveit_config demo_servo.launch.py 
    ```
    
2. start dlo model
    
    ```bash
    ros2 launch mtc_tutorial load_dlo.launch.py
    ```
    
3. start planning in MTC to enable `tf:buffer`
    
    ```bash
    ros2 launch mtc_tutorial pick_place_demo_dual.launch.py exe:=dual_mtc_routing
    ```
    
    Currently, the follower will wait for the master to reach an initial position and send a start signal to start tracking.
    
4. To load scene
    
    ```bash
    source install/setup.bash
    ros2 launch mtc_tutorial load_scene.launch.py scene_file:=/home/tp2/ws_humble/scene/mongodb_8.scene
    ```
    
    Remember to kill load_scene manually after loaded.