#===========================================
#For RTSP
import gi
import cv2
import argparse

# import required library like Gstreamer and GstreamerRtspServer
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GObject
#===========================================

import threading

import zmq
import base64
import numpy as np

context = zmq.Context()
footage_socket = context.socket(zmq.SUB)
footage_socket.bind('tcp://*:5555')
footage_socket.setsockopt_string(zmq.SUBSCRIBE, np.unicode_(''))


#======================================================================================
# Local RTSP server and streamer 

# Sensor Factory class which inherits the GstRtspServer base class and add
# properties to it.
class SensorFactory(GstRtspServer.RTSPMediaFactory):
  def __init__(self, **properties):
    super(SensorFactory, self).__init__(**properties)
    #self.cap = cv2.VideoCapture(opt.device_id)
    self.number_frames = 0
    self.fps = opt.fps
    self.duration = 1 / self.fps * Gst.SECOND  # duration of a frame in nanoseconds
    self.launch_string = 'appsrc name=source is-live=true block=true format=GST_FORMAT_TIME ' \
                         'caps=video/x-raw,format=BGR,width={},height={},framerate={}/1 ' \
                         '! videoconvert ! video/x-raw,format=I420 ' \
                         '! x264enc speed-preset=ultrafast tune=zerolatency ' \
                         '! rtph264pay config-interval=1 name=pay0 pt=96' \
                         .format(opt.image_width, opt.image_height, self.fps)
  # method to capture the video feed from the camera and push it to the
  # streaming buffer.
  def on_need_data(self, src, length):
  #============================================
    frame = footage_socket.recv_string()
    img = base64.b64decode(frame)
    npimg = np.fromstring(img, dtype=np.uint8)
    source = cv2.imdecode(npimg, 1)
  #============================================  
    data = source.tostring()
    buf = Gst.Buffer.new_allocate(None, len(data), None)
    buf.fill(0, data)
    buf.duration = self.duration
    timestamp = self.number_frames * self.duration
    buf.pts = buf.dts = int(timestamp)
    buf.offset = timestamp
    self.number_frames += 1
    retval = src.emit('push-buffer', buf)
    print('pushed buffer, frame {}, duration {} ns, durations {} s'.format(self.number_frames,
                                                                           self.duration,
                                                                           self.duration / Gst.SECOND))
    if retval != Gst.FlowReturn.OK:
      print(retval)
  # attach the launch string to the override method
  def do_create_element(self, url):
    return Gst.parse_launch(self.launch_string)
    
  # attaching the source element to the rtsp media
  def do_configure(self, rtsp_media):
    self.number_frames = 0
    appsrc = rtsp_media.get_element().get_child_by_name('source')
    appsrc.connect('need-data', self.on_need_data)

# Rtsp server implementation where we attach the factory sensor with the stream uri
class GstServer(GstRtspServer.RTSPServer):
  def __init__(self, **properties):
    print(44)
    super(GstServer, self).__init__(**properties)
    print(255)
    self.factory = SensorFactory()
    self.factory.set_shared(True)
    self.set_service(str(opt.port))
    self.get_mount_points().add_factory(opt.stream_uri, self.factory)
    self.attach(None)
        
        
# getting the required information from the user 
parser = argparse.ArgumentParser()
parser.add_argument("--device_id", default="0" ,required=False, help="device id for the \
                video device or video file location")
parser.add_argument("--fps", default=30, required=False, help="fps of the camera", type = int)
parser.add_argument("--image_width", default=320, required=False, help="video frame width", type = int)
parser.add_argument("--image_height", default=240, required=False, help="video frame height", type = int)
parser.add_argument("--port", default=1234, help="port to stream video", type = int)
parser.add_argument("--stream_uri", default = "/video_stream", help="rtsp video stream uri")
opt = parser.parse_args()

try:
    opt.device_id = int(opt.device_id)
except ValueError:
    pass

def main():
  GObject.threads_init()
  Gst.init(None)
  server = GstServer()
  loop = GObject.MainLoop()
  loop.run()


main()

