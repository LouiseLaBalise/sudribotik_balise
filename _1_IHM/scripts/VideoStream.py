import io, time, threading
#To install picamera whereas your are not on a raspberry pi config do : 1-export READTHEDOCS=True 2-pip install picamera
#import picamera #it won't work but you will have the module -----------------------------------------------------------------



class VideoStream(object):

    #Constructor
    def __init__(self):
        self.frame = None #image is stored here
        self.streaming = False #not streaming yet
        #Create a new thread which will execute captureFrames() method when started
        self.thread = threading.Thread(target=self.captureFrames())
        #This is a daemon thread : It closes automatically when the main thread ends
        self.thread.daemon = True

    def start(self):
        #If camera is not currently streaming
        if not self.streaming: 
            self.thread.start() #start thread
            self.streaming = True

    def stop(self):
        self.streaming = False
    
    #Stream video capture function
    def captureFrames(self):
        #Use python context manager to ensure camera is properly closed after use.
        """with picamera.PiCamera() as camera : -------------------------------------------------------------------------------
            camera.resolution = (640, 480)
            camera.framerate = 24
            time.sleep(0.5) #Let camera time to initialise

            stream = io.BytesIO() #stream init

            #Capture each frame into 'stream'
            for k in camera.capture_continuous(stream, "jpg", use_video_port=True):
                if not self.streaming : break

                stream.seek(0) #cursor placed at the start to read all the datas
                self.frame = stream.read() #store the data inputed

                stream.seek(0) #cursor at the beginning
                stream.truncate() #truncate stream to ensure taht everything which will be write after that position is deleted
                #it optimize ram memory
                time.sleep(0.1)#regulate capture frame, can be modified"""


    def getFrame(self):
        return self.frame

