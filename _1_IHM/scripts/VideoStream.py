import io, time, threading, subprocess
from PIL import Image


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
        '''video_width, video_height = 640, 480
        command = [
            "raspivid",
            "-t", "0",                  #capture indefinitely
            "-w", str(video_width),     #set width
            "-h", str(video_height),    #set height
            "-fps", "20",               #set framerate
            "-o", "-"                   #set output on stdout
        ]

        #Run the above command on a new processus but redirect the output stream so python can get acces to it
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        try :
            while self.streaming:
                #Get the raw frame on the output to get every bytes we multipy img size by 3 (channels of rgb colors)
                raw_frame = process.stdout.read(video_width * video_height * 3)

                #Convert raw image into real image using pillow
                image = Image.frombytes('RGB', (video_width, video_height), raw_frame, 'raw', "RGB")

                #Store image on a stream bytes variable, 
                # its quicker since image is save in buffer and not in hard disk, its easier to transmitt 
                stream = io.BytesIO()
                image.save(stream, format='jpg')

                #Then store it into a class variable 
                self.frame = stream.getValue()

                time.sleep(0.1) #little sleep to re-syncronize with current frame

        except Exception as e :
            print(f"Erreur lors de la prise de video : {e}")
        finally:
            process.terminate() #kill process whatever happend to free all ressources #'''

    def getFrame(self):
        return self.frame

