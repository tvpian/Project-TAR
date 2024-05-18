# PyGObject-GstRtspServer
#### Simple implementation of GstRtspServer using Python GObject Introspection
I've found no examples of Gstreamer's RTSP server implementation using GObject Python introspection, so this is a working script deducted from the C examples available at https://gitlab.freedesktop.org/gstreamer/gst-rtsp-server.
It could be useful for Rasperry Pi camera stream projects.

Install dependencies
------------

```sh
sudo apt install python3 python3-gst-1.0 \
    gstreamer1.0-plugins-base \
    gir1.2-gst-rtsp-server-1.0
```
