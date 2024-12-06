# TP3py_realtime
This is a system to stream, plot, and save Tobii Pro Glasses 3 IMU and gaze data in real time
You can download it or use git clone
Currently, I have only tested on macos but ubuntu should also works

## Step by step (For macos)
1. Open terminal, go inside the folder, and run the following code
```
python3 -m venv .venv
source .venv/bin/activate
```

2. Then you can execute the install script
```
chmod +x install_mac.sh
./install_mac.sh 
```
4. Then create a data folder
```
mkdir data
```
3. If all the dependencies are installed properly, execute the following script
4. Here make sure that the Tobii Pro Glasses 3 is connected to your os via wifi
```
python tobii_plot.py
```
4. A CV window will show up and you can see the plots
5. Calibration is not included so you have to manually calibrate using API. 
```access:
http://192.168.75.51
```
6. Sometimes data is not streaming. You should press 'q' to exit wait for a few seconds and try executing again
7. You can ask me (Akmal) anytime if you encounter troubles
8. When you have finished deactivate the virtual environment
```
deactivate
```
