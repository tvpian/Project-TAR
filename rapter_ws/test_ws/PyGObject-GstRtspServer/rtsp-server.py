#  Copyright (C) 2020 Matteo Benedetto <me at enne2.net>
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GLib, GObject, GstRtspServer

Gst.init(None)

port = "8554"
mount_point = "/test"

server = GstRtspServer.RTSPServer.new()
server.set_service(port)
mounts = server.get_mount_points()
factory = GstRtspServer.RTSPMediaFactory.new()
factory.set_launch("videotestsrc ! videoconvert ! theoraenc ! queue ! rtptheorapay name=pay0")
mounts.add_factory(mount_point, factory)
server.attach()

#  start serving
print ("stream ready at rtsp://127.0.0.1:" + port + "/test");

loop = GLib.MainLoop()
loop.run()
