YOLO
==============
Train an object identification model with a pretrained YOLO v11 model.

The trained YOLO model should be able to identify traffic signs at a T-junction.

# 1&emsp;View the Camera Feed

1. Start VSCode and connect to the robot (See [02_pc.md](02_pc.md)).

2. Bringup the robot sensors, motor controller and camera with the robot's `bringup.sh` script, on a VSCode terminal `RobotTerminalA`.
    1. On the VSCode window that is connected to the robot, type `` CTRL+` `` to open a new terminal.
    
    2. Let this VSCode terminal be `RobotTerminalA`.

    3. In the terminal, change directory to the `tb3_yolo` directory, and start up the sensors, motor controller and camera, using `./bringup.sh`:

    ```bash
    cd ~/tb3_yolo
    ./bringup.sh
    ```
    Note that `./bringup.sh` can be run directly without `cd` as long as the terminal is in the `tb3_yolo` directory.
    

4. View the camera feed with the PC's `view_cam` script, on a terminal `PCTerminalA`.
    1. Press `CTRL+ALT+T` to open a new terminal on the PC.

    2. Let this terminal be `PCTerminalA`.

    3. Suppose your robot's username is `ece83` and your team number is `83`. Then the value after `ROS_DOMAIN_ID` in the following command should be `83`. Replace the value with your team number accordingly.
    
    4. In the terminal, run the following command by replacing the `ROS_DOMAIN_ID` value with your team number:
        ```bash
        cd ~/tb3_yolo_pc
        ROS_DOMAIN_ID=83 ./view_cam.sh
        ```
        Ensure that there are **no spaces** before and after `=`. Note that `ROS_DOMAIN_ID=83 ./view_cam.sh` can be run directly without `cd` as long as the terminal is in the `tb3_yolo_pc` directory.

    5. Wait for a few seconds for the feed to show up.

5. When complete, press `CTRL+C` on **both** terminals to stop the programs.

# 2&emsp;Capture Images
1. Run `bringup.sh` in `RobotTerminalA`. Open a new terminal on VSCode with `` CTRL+` `` if needed.

2. Run `view_cam.sh` in `PCTerminalA`. Open a new terminal with `CTRL+ALT+T` if needed.

3. Run the `capture.sh` script to start collecting images on a terminal `PCTerminalB`.
    1. Open a new terminal with `CTRL+ALT+T`.

    2. Let this terminal be `PCTerminalB`.

    3. In the terminal, run the `capture.sh` script. Replace the value in `ROS_DOMAIN_ID` with the team number, just like the previous section.
    ```bash
    cd ~/tb3_yolo_pc
    ROS_DOMAIN_ID=83 ./capture.sh
    ```

4. Try to take multiple images by pressing `SPACE` in `PCTerminalB`.

5. When complete, press `CTRL+C` on all three terminals.

6. The pictures can be found in the folder `~/tb3_yolo_pc/train_yolo/datasets/traffic_signs/images/`. Open `Files` in Ubuntu and type the path from the previous sentence into the address bar to navigate to the folder.


# 3&emsp;Collect Images for Training YOLO Model

1. Design a method to collect 40 images of the same traffic sign (one image class) so that it can be robustly identified by the trained YOLO model:
    - The robot will have to identify the traffic sign at a T-junction, and the robot is facing the junction from the vertical line of the 'T'.
    - The traffic sign is placed at the junction on the wall in front of the robot.
    - The traffic sign should be identified anywhere between 20cm to 40cm from the robot.
    - The robot may be facing slightly off-angle from the sign as it approaches the junction.
    - There may be colored paper on the ground at the junction.
    - All QR codes, regardless of the text encoded, should be presented as one image class.

2. The methodology should be repeated for all traffic signs (all image classes).

3. When ready to collect the images, begin running the programs:
    1. Run `bringup.sh` in `RobotTerminalA`. Open a new terminal on VSCode with `` CTRL+` `` if needed.
    2. Run `view_cam.sh` in `PCTerminalA`. Open a new terminal with `CTRL+ALT+T` if needed.
    3. Run `capture.sh` in `PCTerminalB`. Open a new terminal with `CTRL+ALT+T` if needed.

4. On `PCTerminalB` where `capture.sh` is running, collect the images by pressing `SPACE`.

5. Once all images are gathered, press `CTRL+C` in all three terminals.

6. If the images collected are not satisfactory, you may restart the process to overwrite the images, or overwrite a certain picture by running `capture.sh` differently:
    - To replace `img005.jpg` and onwards, run the following by replacing the value of `ROS_DOMAIN_ID` with the team number.
        ```bash
        ROS_DOMAIN_ID=83 ./capture.sh img 4
        ```
    - To replace `img102.jpg` and onwards, run the following by replacing `ROS_DOMAIN_ID`.
        ```bash
        ROS_DOMAIN_ID=83 ./capture.sh img 101
        ```
    - To replace `img005.jpg` to `img007.jpg`, run the following by replacing `ROS_DOMAIN_ID` and pressing `SPACE` only **three times** before exiting with `CTRL+C`:
        ```bash
        ROS_DOMAIN_ID=83 ./capture.sh img 4
        ```

# 4&emsp;Label Images for Training YOLO Model

1. Open a web browser and go to https://www.makesense.ai/. 

2. Click `Get Started` on the bottom right.

3. Upload all images from the `img` folder.

4. Click `Object Detection`.

5. Create a label for each of the image class. The first class is assigned the integer `0`, the second is `1`, etc.

6. Click `Start project`.

7. Begin to spend about an hour annotating, by drawing bounding boxes across the object in every image. Ensure that the correct label (image class) is chosen.

8. Once the labelling is complete, click `Export and Save Annotations`.

9. Select `A .zip package containing files in YOLO format`.

10. Click `Export`.

11. Download the `.zip` file, and extract all the `.txt` annotation files into the `~/train_yolo/datasets/traffic_signs/labels/` folder.

# 5&emsp;Begin Training YOLO Model

1. With the images and labels in place, configure the training.

    1. Open the `~/train_yolo/datasets.yaml` file from `PCTerminalA`:
        ```bash
        code ~/tb3_yolo_pc/train_yolo/datasets.yaml
        ```

    2. Look for `names` in the `yaml` file. The integer should correspond to image class chosen earlier in the labelling process. The name must correspond to the image class represented by the integer. The name must be a single word, there must be one space between the name and `:`.

    ```yaml
    # ...

    # Classes
    names:
        0: left
        1: right
        2: qr
    ```

    3. Save and close with `CTRL+S` and `CTRL+SHIFT+W`.

2. in `PCTerminalA`, run the following script to train the model. Wait for about an hour.

    ```bash
    cd ~/tb3_yolo_pc/train_yolo
    python3 train_yolo.py
    ```
    

# 6&emsp;Transfer Trained YOLO Model to Robot
1. Locate the best model using `Files`. The best model will appear in the directory
    `~/tb3_yolo_pc/train_yolo/runs/detect/train/weights/`, as `best.pt`. 

2. Copy the `best.pt` into the robot. 
    1. On the VSCode that is connected to the robot, open the Explorer pane by using `CTRL+SHIFT+E`.
    2. If the folder is not open on the explorer pane, press `Open Folder`, and navigate to the `tb3_yolo` folder. For `ece83`, the path is `/home/ece83/tb3_yolo`. Trust the authors if prompted. This may close some terminals, but simply open them again later.
    3. Drag the `best.pt` from `Files` into the empty space in VSCode's explorer pane.

# 7&emsp;Test the YOLO Model
    1. Run `bringup.sh` in `RobotTerminalA`. Open a new terminal on VSCode with `` CTRL+` `` if needed.
    
    2. Run `run.sh` in `RobotTerminalB`. Open a new terminal on VSCode with `` CTRL+` `` if needed.
        ```bash
        cd ~/tb3_yolo
        ./run.sh
        ```

    3. Run `view_yolo.sh` in `PCTerminalA`. Open a new terminal with `CTRL+ALT+T` if needed. Replace the value of `ROS_DOMAIN_ID` with the team number:
        ```bash
        cd ~/tb3_yolo_pc
        ROS_DOMAIN_ID=83 ./view_yolo.sh
        ```

    4. Wait for a few seconds. Present the images in front of the camera, and the image classes and confidence values should appear on the camera feed.