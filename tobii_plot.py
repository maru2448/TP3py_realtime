import numpy as np
import pandas as pd
import time
import os
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
import cv2
import pickle
from datetime  import datetime
from PyQt5.QtCore import QThread
from TP3py_Gstream import TP3py_Gstream

class tobii_rt:
    def __init__(self):
        # Initialize
        buffer_size = 500
        path = 'TP3py_realtime/' # Insert your own path
        self.parent_dir = os.path.join(os.path.expanduser('~'), path)
        self.data = {'timestamp':np.zeros(buffer_size),  # prepare dummy data
                     'x':np.zeros(buffer_size),
                     'y':np.zeros(buffer_size),
                     'z':np.zeros(buffer_size)
                     }
        self.DF_imu = pd.DataFrame([]) # saving dataframes
        self.DF_Lgze = pd.DataFrame([])
        self.DF_Rgze = pd.DataFrame([])

        # Stream data from TP3py_Gstream
        self.GstreamWorker = TP3py_Gstream(path=path)
        self.thread1 = QThread()
        self.GstreamWorker.moveToThread(self.thread1)
        self.thread1.started.connect(self.GstreamWorker.do_work)
        self.GstreamWorker.Gstreamfinished.connect(self.thread1.quit)
        self.GstreamWorker.Gstreamfinished.connect(self.GstreamWorker.deleteLater)
        self.thread1.finished.connect(self.thread1.deleteLater)
        self.thread1.start()
        time.sleep(10)
    
    def get_data(self, name):
        try:
        # Try opening buffer data
            with open(f'{self.parent_dir}{name}.pkl', 'rb') as file:
                data = pickle.load(file)
        except:
            # If data has no buffer, pass dummy data
            try:
                if 'imu' in name:
                    data = self.imu
                elif 'Lgze' in name:
                    data = self.Lgze
                elif 'Rgze' in name:
                    data = self.Rgze
            except:
                data = self.data
        return data

    def create_plot(self):
        # Create subplots
        fig, axes = plt.subplots(3, 3, figsize=(19, 10))
        canvas = FigureCanvas(fig)
        axes = axes.flatten()

        # Get data from buffer dataset
        imu = self.get_data('imu_buffer')
        self.imu = imu
        self.Lgze = self.get_data('Lgze_buffer') # left gaze
        Lgze = self.angular(self.Lgze)
        self.Rgze = self.get_data('Rgze_buffer') # right gaze
        Rgze = self.angular(self.Rgze)

        # Pile data
        self.pile_data(pd.DataFrame(imu), name='imu')
        self.pile_data(Lgze, name='Lgze')
        self.pile_data(Rgze, name='Rgze')

        # Plot om axes
        axes[0].plot(imu['timestamp'], imu['x'])
        axes[0].set(xlabel='time [s]', ylabel='GYRO_X [deg/s]')
        axes[1].plot(imu['timestamp'], imu['y'])
        axes[1].set(xlabel='time [s]', ylabel='GYRO_Y [deg/s]')
        axes[2].plot(imu['timestamp'], imu['z'])
        axes[2].set(xlabel='time [s]', ylabel='GYRO_Z [deg/s]')
        axes[3].plot(imu['timestamp'], Lgze['x'])
        axes[3].set(xlabel='time [s]', ylabel='(L)EYE_X [deg/s]')
        axes[4].plot(imu['timestamp'], Lgze['y'])
        axes[4].set(xlabel='time [s]', ylabel='(L)EYE_Y [deg/s]')
        axes[5].plot(imu['timestamp'], Lgze['z'])
        axes[5].set(xlabel='time [s]', ylabel='(L)EYE_Z [deg/s]')
        axes[6].plot(imu['timestamp'], Rgze['x'])
        axes[6].set(xlabel='time [s]', ylabel='(R)EYE_X [deg/s]')
        axes[7].plot(imu['timestamp'], Rgze['y'])
        axes[7].set(xlabel='time [s]', ylabel='(R)EYE_Y [deg/s]')
        axes[8].plot(imu['timestamp'], Rgze['z'])
        axes[8].set(xlabel='time [s]', ylabel='(R)EYE_Z [deg/s]')

        for ax in axes:
            ax.grid(alpha=0.3)

        plt.tight_layout()
        canvas.draw()

        img = np.frombuffer(canvas.buffer_rgba(), dtype=np.uint8)  # Use buffer_rgba
        img = img.reshape(canvas.get_width_height()[::-1] + (4,))  # For RGBA
        plt.close(fig)  # Close the figure to free memory
        return img
    
    def pile_data(self, data, name):
        if name == "imu":
            self.DF_imu = pd.concat([self.DF_imu, data], ignore_index=True).drop_duplicates().reset_index(drop=True)
        elif name == "Lgze":
            self.DF_Lgze = pd.concat([self.DF_Lgze, data], ignore_index=True).drop_duplicates().reset_index(drop=True)
        elif name == "Rgze":
            self.DF_Rgze = pd.concat([self.DF_Rgze, data], ignore_index=True).drop_duplicates().reset_index(drop=True)
            
    def angular(self, gzedir):
        df = pd.DataFrame(gzedir)
        # Calculate the rolling mean of gaze direction over 3 consecutive rows
        gaze_dir_rolling = df.iloc[:, 1:].rolling(window=3, min_periods=1).mean()
        # Normalize the gaze direction
        gaze_norm = np.linalg.norm(gaze_dir_rolling, axis=1, keepdims=True)
        FilterGazeDir = gaze_dir_rolling / gaze_norm
        # Calculate the eye angle velocity
        dt = df['timestamp'].diff()
        FilterEyeAngVelo = np.zeros_like(FilterGazeDir)
        # Vectorized calculation of eye angle velocity
        # Use numpy's diff to get the difference between adjacent rows
        # differential = np.diff(FilterGazeDir, axis=0)
        # FilterEyeAngVelo[1:-1] = differential[1:] / dt[1:, np.newaxis]
        neighbors_sum = (FilterGazeDir[:-2] + FilterGazeDir[2:]) / 0.05
       
        FilterEyeAngVelo[1:-1] = np.degrees(np.cross(FilterGazeDir[1:-1], neighbors_sum[:-2]))
        FilterEyeAngVelodf = pd.concat([df['timestamp'], pd.DataFrame(FilterEyeAngVelo, columns=df.columns[1:])], axis=1)
        return FilterEyeAngVelodf
    
    def stop(self):
        # Stop streaming from Gstreamer
        self.GstreamWorker.stop()
        # Stop Gstreamer thread
        self.thread1.quit()
        self.thread1.wait()
        # Delete buffer data files
        for filename in os.listdir(self.parent_dir):
            if filename.endswith(".pkl"):
                file_path = os.path.join(self.parent_dir, filename)
                os.remove(file_path)
        # Save dataframe
        date = datetime.now().strftime("%Y%m%d_%H%M%S")
        if len(self.DF_imu)>0:
            self.DF_imu.to_csv(f'{self.parent_dir}/data/imu_{date}.csv', index=False)
        if len(self.DF_Lgze)>0:   
            self.DF_Lgze.to_csv(f'{self.parent_dir}/data/Lgze{date}.csv', index=False) 
        if len(self.DF_Rgze)>0:   
            self.DF_Rgze.to_csv(f'{self.parent_dir}/data/Rgze{date}.csv', index=False) 
        print('Files are saved...')
                
def main():
    # initiate tobii realtime
    wd = tobii_rt()
    # Plot on cv2
    while True:
        plot_image = wd.create_plot()
        # Convert RGBA to BGR for OpenCV
        plot_image_bgr = cv2.cvtColor(plot_image, cv2.COLOR_RGBA2BGR)
        # Display in OpenCV window
        cv2.imshow("Realtime 3x3 Subplots", plot_image_bgr)
        # Break on pressing 'q' update at every 100ms
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows() # destroy window
    # stop the streaming
    wd.stop()
    print('Streaming safely stopped')    

if __name__ == "__main__":
    main()
