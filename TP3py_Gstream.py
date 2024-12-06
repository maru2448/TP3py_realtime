import gi
gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
import json
import os
import numpy, pickle
import pandas as pd
from collections import deque
from datetime  import datetime
from gi.repository import Gst
from PyQt5.QtCore import QObject, pyqtSignal
import time
Gst.init(None)

class TP3py_Gstream(QObject):
    buffer_size = 500  # Set buffer size
    # Initialize buffer dicts
    imutime = {'timestamp': deque(maxlen=buffer_size), 
               'x': deque(maxlen=buffer_size),
               'y': deque(maxlen=buffer_size),
               'z': deque(maxlen=buffer_size)
               }
    Lgazetime = {'timestamp': deque(maxlen=buffer_size),
                 'x': deque(maxlen=buffer_size),
                 'y': deque(maxlen=buffer_size),
                 'z': deque(maxlen=buffer_size)
                 }
    Rgazetime = {'timestamp': deque(maxlen=buffer_size),
                 'x': deque(maxlen=buffer_size),
                 'y': deque(maxlen=buffer_size),
                 'z': deque(maxlen=buffer_size)
                 }

    """Set of signals for outputting the data being streamed to other threads"""   
    # give worker class a finished signal
    Gstreamfinished = pyqtSignal()
    # Signal containing eye video frame
    EyeVideo_signal = pyqtSignal(numpy.ndarray)
    # Signal containing scene video frame
    SceneVideo_signal = pyqtSignal(numpy.ndarray)
    # Signal containing IMU data
    IMU_signal = pyqtSignal(str)
    # Signal containing Gaze data
    Gaze_signal = pyqtSignal(str)
    # Signal containing Timestamps - PTS
    TS_signal = pyqtSignal(str)
    # Signal containing TTL data
    TTL_signal = pyqtSignal(str)

    def __init__(self, parent=None, path=None):
        # set saving path for videos
        self.path = path
        QObject.__init__(self, parent=parent)
        #super().__init__()
        self.continue_run = True  # provide a bool run condition for the class
        self.elapsedTime = 0
        # Default values of experiment names and Initials
        self.ExpNamestring = ""
        self.InitialNamestring = "Test"
        # Parent directory
        self.parent_dir = os.path.join(os.path.expanduser('~'), self.path)

        ################ Guidelines for chaning the Gstreamer's pipeline specifications ####################
        # Change the video buffer size by changing the width and height variables in video decoder branches
        # If gaze marker is needed to be overlaid on scene video set gaze-overlay=true          
        # Change the length of the saving videos by changing   max-size-time=60000000000 (micro seconds)            
        self.pipeline = Gst.parse_launch('rtspsrc location=rtsp://192.168.75.51:8554/live/all?gaze-overlay=false \
        latency=100 name=src protocols=GST_RTSP_LOWER_TRANS_TCP buffer-mode = 0 drop-on-latency=true \
        src. ! application/x-rtp,payload=96 ! rtpjitterbuffer max-misorder-time = 0  ! rtph264depay ! h264parse  ! tee name=tscene \
        tscene. ! queue ! avdec_h264 ! videoscale ! video/x-raw,width=1920, height=1080! appsink name=Appscenesink \
        tscene. ! queue ! splitmuxsink  max-size-time=60000000000  name=scenesink \
        src. ! application/x-rtp,payload=98 ! rtpjitterbuffer max-misorder-time = 0  ! rtph264depay ! h264parse  ! tee name=teye \
        teye. ! queue ! avdec_h264 ! videoscale ! video/x-raw,width=512, height=128! appsink name=Appeyesink \
        teye. ! queue ! splitmuxsink  max-size-time=60000000000  name=eyesink \
        src. ! application/x-rtp,payload=99 ! rtpjitterbuffer max-misorder-time = 0  ! appsink name=Gazesink \
        src. ! application/x-rtp,payload=100 ! rtpjitterbuffer max-misorder-time = 0 ! appsink name=TTLsink \
        src. ! application/x-rtp,payload=101 ! rtpjitterbuffer max-misorder-time = 0 ! appsink name=IMUsink ')

        self.appEyeMsink = self.pipeline.get_by_name("eyesink")
        self.appIMUsink = self.pipeline.get_by_name("IMUsink")
        self.appGazesink = self.pipeline.get_by_name("Gazesink")
        self.appEyeMsinkGUI = self.pipeline.get_by_name("Appeyesink")
        self.appScenesinkGUI = self.pipeline.get_by_name("Appscenesink")
        self.appScenesink = self.pipeline.get_by_name("scenesink")
        self.appTTLsink = self.pipeline.get_by_name("TTLsink")

        print("Gstream init. done!")

    # """ Handles for accessing the first frame PTS in the video saving branch of Gstreamer"""
    def format_location_full_callback_eye (self, splitmux, fragment_id, firstsample):
        buf =firstsample.get_buffer()
        if self.continue_run:
            self.TS_signal.emit('{"eyeMSTS":'+ str(buf.pts/1e9)+'}'+'\n')

    def format_location_full_callback_scene (self, splitmux, fragment_id, firstsample):
        buf =firstsample.get_buffer()
        if self.continue_run:
            self.TS_signal.emit('{"sceneMSTS":'+ str(buf.pts/1e9)+'}'+'\n')

    """ Scene movie buffer management """
    #Emitting the video buffer in SceneVideo_signal
    def gst_to_sceneimage(self, sample):
        buf = sample.get_buffer()
        recievedTime = (buf.pts)/1e9 
        caps = sample.get_caps()

        temph = caps.get_structure(0).get_value('height')//2
        tempw = caps.get_structure(0).get_value('width')//2
        size = temph*tempw

        h = caps.get_structure(0).get_value('height')
        w = caps.get_structure(0).get_value('width')

        YUV_arr = numpy.ndarray(
            (h*3//2, w),
            buffer=buf.extract_dup(0, h*w*3//2),
            dtype=numpy.uint8)

        if self.continue_run:
            self.SceneVideo_signal.emit(YUV_arr)

        return YUV_arr, recievedTime

    def rebin(self, arr, new_shape):
        shape = (new_shape[0], arr.shape[0] // new_shape[0],
             new_shape[1], arr.shape[1] // new_shape[1])
        return arr.reshape(shape).max(-1).max(1)

    def SceneM_new_buffer(self, sink, data):
        sample = sink.emit("pull-sample")
        EyeImageBuffer, IMUtime = self.gst_to_sceneimage(sample)
        self.extractTS_scene(IMUtime)
        return Gst.FlowReturn.OK

    """ Eye movie buffer management """
    #Emitting the video buffer in EyeVideo_signal
    def gst_to_eyeimage(self, sample):
        buf = sample.get_buffer()
        recievedTime = (buf.pts)/1e9 
        caps = sample.get_caps()

        arr = numpy.ndarray(
            (caps.get_structure(0).get_value('height'),
             caps.get_structure(0).get_value('width'),
             1),
            buffer=buf.extract_dup(0, buf.get_size()),
            dtype=numpy.uint8)
       
        if self.continue_run:
            self.EyeVideo_signal.emit(arr)
        return arr, recievedTime
    
    def EyeM_new_buffer(self, sink, data):
        sample = sink.emit("pull-sample")
        EyeImageBuffer, Eyetime = self.gst_to_eyeimage(sample)
        self.extractTS_eye(Eyetime)
        return Gst.FlowReturn.OK

    """ IMU buffer management """
    # Returns the raw output of IMU Payloads
    def gst_to_IMU(self, sample):
        buf = sample.get_buffer()
        caps = sample.get_caps()
        recievedTime = (buf.pts)/1e9
        arr = numpy.ndarray(
            (1,buf.get_size()),
            buffer=buf.extract_dup(0, buf.get_size()),
            dtype=numpy.uint8)

        return bytes(arr), recievedTime

    def IMU_new_buffer(self, sink, data):
        sample = sink.emit("pull-sample")
        streamString, IMUtime = self.gst_to_IMU(sample)
        self.extractIMU(str(streamString), IMUtime)
        return Gst.FlowReturn.OK

    """ Gaze data buffer management """
    # Returns the raw output of Gaze and TTL Payloads
    def gst_to_Gaze(self, sample):
        buf = sample.get_buffer()
        recievedTime = (buf.pts)/1e9 
        arr = numpy.ndarray(
            (1,buf.get_size()),
            buffer=buf.extract_dup(0, buf.get_size()),
            dtype=numpy.uint8)
        
        return bytes(arr), recievedTime

    def Gaze_new_buffer(self, sink, data):
        sample = sink.emit("pull-sample")
        streamString, IMUtime = self.gst_to_Gaze(sample)
        self.extractGazeD(str(streamString), IMUtime)
        return Gst.FlowReturn.OK

    """ TTL data buffer management """
    def TTL_new_buffer(self, sink, data):
        sample = sink.emit("pull-sample")
        streamString, IMUtime = self.gst_to_Gaze(sample)
        self.extractTTL(str(streamString), IMUtime)
        return Gst.FlowReturn.OK

    def do_work(self):

        # Initialize the path were files are being saved   datetime.today().strftime('%Y%m%d%H%M')
        self.path = os.path.join(self.parent_dir, self.InitialNamestring)
        # Create the subject directory
        try:
            os.mkdir(self.path)
        except OSError as error:
            print(error)
        self.path = os.path.join(self.path , datetime.today().strftime('%Y%m%d%H%M'))
        self.timestr = datetime.today().strftime('-%H-%M')
        # Create the experiment directory as a subdirectory of the subject folder
        try:
            os.mkdir(self.path)
        except OSError as error:
            print(error)


        """
        Saving the scene and eye movies right away in a segmented manner while keeping the first frame timestamps
        """
        self.appEyeMsink.set_property('location',  self.path+'/eye-'+self.ExpNamestring+self.timestr+"-%d.mov")
        self.appEyeMsink.connect("format-location-full", self.format_location_full_callback_eye)
        self.appScenesink.set_property('location',  self.path+'/scene-'+self.ExpNamestring+self.timestr+"-%d.mov")
        self.appScenesink.connect("format-location-full", self.format_location_full_callback_scene)

        #Setting up the IMU, Gaze data and scene/eyes videos for processing and demonstration
        ########IMU#######################################
        self.appIMUsink.set_property("emit-signals", True)
        self.appIMUsink.connect("new-sample", self.IMU_new_buffer, self.appIMUsink)
        ########Gaze Data#################################
        self.appGazesink.set_property("emit-signals", True)
        self.appGazesink.connect("new-sample", self.Gaze_new_buffer, self.appGazesink)
        ########Eye Movie#################################
        self.appEyeMsinkGUI.set_property("emit-signals", True)
        self.appEyeMsinkGUI.connect("new-sample", self.EyeM_new_buffer, self.appEyeMsinkGUI)
        ########Scene Movie###############################
        self.appScenesinkGUI.set_property("emit-signals", True)
        self.appScenesinkGUI.connect("new-sample", self.SceneM_new_buffer, self.appScenesinkGUI)
        ########TTLs#######################################
        self.appTTLsink.set_property("emit-signals", True)
        self.appTTLsink.connect("new-sample", self.TTL_new_buffer, self.appTTLsink)

        # Start playing
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            print("Unable to set the pextractIMUipeline to the playing state.")
            exit(-1)

        # Wait until error or EOS
        bus = self.pipeline.get_bus()

        # Parse the pipeline message
        while self.continue_run:
            #message = bus.timed_pop_filtered(10000, Gst.MessageType.ANY)
            message = bus.timed_pop(10000)
            if message:
                if message.type == Gst.MessageType.ERROR:
                    err, debug = message.parse_error()
                    print(("Error received from element %s: %s" % (
                        message.src.get_name(), err)))
                    print(("Debugging information: %s" % debug))
                    break
                elif message.type == Gst.MessageType.EOS:
                    print("End-Of-Stream reached.")
                    break
                elif message.type == Gst.MessageType.STATE_CHANGED:
                    if isinstance(message.src, Gst.Pipeline):
                        old_state, new_state, pending_state = message.parse_state_changed()
                        print(("Pipeline state changed from %s to %s." %
                            (old_state.value_nick, new_state.value_nick)))
                elif message.type == Gst.MessageType.APPLICATION:
                    print(message.get_structure().get_name())

        print('End of streaming')
        self.Gstreamfinished.emit()  # emit the finished signal when the loop is done

    def stop(self):
        self.continue_run = False  # set the run condition to false on stop
        self.pipeline.send_event(Gst.Event.new_eos())
        # Wait until the eos being processed
        time.sleep(5)
        # Set the pipeline state to NULL
        self.pipeline.set_state(Gst.State.NULL)

    # For extracting IMU data
    def extractIMU(self, streamString, IMUtime):
        Acc_ind = streamString.find("accelerometer")
        Mag_ind = streamString.find("magnetometer")

        if Acc_ind > 0:
            if self.continue_run:
                # Get imu signal and save buffer
                self.IMU_signal.emit('{"tacc":'+ str(IMUtime) + ',' +str(streamString[Acc_ind - 1:len(streamString) - 1])+'\n')
                data = json.loads(streamString[Acc_ind - 2:len(streamString) - 1])['gyroscope']
                self.imutime['timestamp'].append(IMUtime)
                self.imutime['x'].append(data[0])
                self.imutime['y'].append(data[1])
                self.imutime['z'].append(data[2])
                if len(self.imutime['timestamp']) == self.buffer_size:
                    with open('imu_buffer.pkl', 'wb') as file:
                        pickle.dump(self.imutime, file)
                else:
                    pass
                    
            #--- For extracting the variables use the following (i.e. AccD.acceletometer[0] -> x axis acceleration)
            # AccD = json.loads(streamString[Acc_ind - 2:len(streamString) - 1], object_hook=
            # lambda d: namedtuple('X', d.keys())
            # (*d.values()))
            # print(json.loads(streamString[Acc_ind - 2:len(streamString) - 1])['accelerometer'][0]) 

        if Mag_ind > 0 :
            if self.continue_run:
                # print('g', self.continue_run)
                self.IMU_signal.emit('{"tmag":'+ str(IMUtime) + ',' +str(streamString[Mag_ind - 1:len(streamString) - 1])+'\n')
            # --- For extracting the variables use the following (i.e. AccD.acceletometer[0] -> x axis acceleration)
            #MagD = json.loads(streamString[Mag_ind-2:len(streamString)-1], object_hook =
            #      lambda d : namedtuple('X', d.keys())
            #      (*d.values()))

    # For extracting Gaze data
    def extractGazeD(self, streamString, IMUtime):
        gaze2Dind = streamString.find("gaze2d")
        # Check if the JSON has gaze data
        if gaze2Dind > 0:
            # --- For extracting the gaze variables use the following (i.e. GazeD.gaze2d[0] -> gaze horizontal position in scene frame)
            #GazeD = json.loads(streamString[gaze2Dind - 2:len(streamString) - 1], object_hook=
            #lambda d: namedtuple('X', d.keys())
            #(*d.values()))

            if self.continue_run:
                # Get gazes signal and save buffer
                self.Gaze_signal.emit('{"tgz":'+ str(IMUtime) + ',' +str(streamString[gaze2Dind - 1:len(streamString) - 1])+'\n')
                # Left gaze direction
                try:
                    dataL = json.loads(streamString[gaze2Dind - 2:len(streamString) - 1])['eyeleft']
                    dataL_gd = dataL['gazedirection']
                except:
                    dataL_gd = numpy.full(3, numpy.nan)
                self.Lgazetime['timestamp'].append(IMUtime)
                self.Lgazetime['x'].append(dataL_gd[0])
                self.Lgazetime['y'].append(dataL_gd[1])
                self.Lgazetime['z'].append(dataL_gd[2])
                if len(self.Lgazetime['timestamp']) == self.buffer_size:
                    with open('Lgze_buffer.pkl', 'wb') as file:
                        pickle.dump(self.Lgazetime, file)
                else:
                    pass
                # Right gaze direction
                try:
                    dataR = json.loads(streamString[gaze2Dind - 2:len(streamString) - 1])['eyeright']
                    dataR_gd = dataR['gazedirection']
                except:
                    dataR_gd = numpy.full(3, numpy.nan)
                self.Rgazetime['timestamp'].append(IMUtime)
                self.Rgazetime['x'].append(dataR_gd[0])
                self.Rgazetime['y'].append(dataR_gd[1])
                self.Rgazetime['z'].append(dataR_gd[2])
                if len(self.Rgazetime['timestamp']) == self.buffer_size:
                    with open('Rgze_buffer.pkl', 'wb') as file:
                        pickle.dump(self.Rgazetime, file)
                else:
                    pass

    # For sending eye video timestamps
    def extractTS_eye(self, time):
        if self.continue_run:
            self.TS_signal.emit('{"ttseye":'+ str(time) + '}'+'\n')
            self.elapsedTime = time

    # For sending scene video timestamps
    def extractTS_scene(self, time):
        if self.continue_run:
            self.TS_signal.emit('{"ttsscene":'+ str(time) + '}'+'\n')

    # For sending scene video timestamps
    def extractTTL(self, streamString, IMUtime):
        TTLind = streamString.find("direction")
        if TTLind > 0:
            if self.continue_run:
                print('T')
                self.TTL_signal.emit('{"tttl":'+ str(IMUtime) + ',' +str(streamString[TTLind - 1:len(streamString) - 1])+'\n')

    def EstablishKeyworkEvents(self, events):
        if self.continue_run:
            self.TTL_signal.emit('{"tevent":' + str(self.elapsedTime) + ',"KeyEvents":' + '"'+ str(events) + '"' + '}\n')