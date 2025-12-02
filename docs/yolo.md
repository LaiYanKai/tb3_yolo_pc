YOLO
==============
Training an object identification model with a pretrained YOLO v11 model.

# Robot Setup

1. On VSCode, on an available terminal, start the robot in the `tb3_yolo` folder with:
    ```bash
    cd ~/tb3_yolo
    ./bringup.sh
    ```

2. On the remote PC, on an available terminal, start the view_cam program with:
    ```bash
    cd ~/tb3_yolo_pc
    ./view_cam.sh
    ```

3. On the remote PC, on an available terminal, start the capture program with:
    ```bash
    cd ~/tb3_yolo_pc
    ./capture.sh
    ```

# Collect Images

1. On the remote PC terminal where `capture.sh` is running, press `SPACE` to capture about 20 images of the same traffic sign (one image class) from a different perspective. 

2. Repeat the previous step for all image classes (i.e. all traffic signs)

3. The images will be stored in the `train_yolo/datasets/traffic_signs/images/` folder in the same `tb3_yolo_pc` directory. You can use `Files` to find it.

# Label Images

1. Open a web browser and go to https://www.makesense.ai/. 

1. Click `Get Started` on the bottom right.
2. Upload all images from the `img` folder.
3. Click `Object Detection`.
4. Create a label for each of the image class. The first class is assigned the integer `0`, the second is `1`, etc.
5. Click `Start project`.
6. Begin to spend about 30 minutes annotating, by drawing bounding boxes across the obejct in every image. Ensure that th ecorrect label (image class) is chosen.
7. Once complete, click `Export and Save Annotations`.
8. Select `A .zip package containing files in YOLO format`.
9. Click `Export`.
10. Download the `.zip` file, and extract all the `.txt` annotation files into the `train_yolo/datasets/traffic_signs/labels/` folder.

# Train Images

1. With the images and labels in place, open the `train_yolo/datasets.yaml` file and make sure that the class labels correspond to the images used.

2. Run the script to train the model. Wait for about an hour.

    ```bash
    cd ~/tb3_yolo_pc/train_yolo
    python3 train_yolo.py
    ```
    
3. Locate the best model. The best model will appear as in the relative directory
    `runs/detect/train/weights/`, as the file `best.pt`.

4. Copy the `best.pt` into the robot's `tb3_yolo` directory.