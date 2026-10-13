"""Read only an X11 app's compositor backing pixmap, never the desktop."""
import ctypes as C
import ctypes.util
import numpy as np


class XImage(C.Structure):
    _fields_=[('width',C.c_int),('height',C.c_int),('xoffset',C.c_int),('format',C.c_int),
        ('data',C.c_void_p),('byte_order',C.c_int),('bitmap_unit',C.c_int),('bitmap_bit_order',C.c_int),
        ('bitmap_pad',C.c_int),('depth',C.c_int),('bytes_per_line',C.c_int),('bits_per_pixel',C.c_int),
        ('red_mask',C.c_ulong),('green_mask',C.c_ulong),('blue_mask',C.c_ulong)]


class Visual(C.Structure):
    _fields_=[('ext_data',C.c_void_p),('visualid',C.c_ulong),('visual_class',C.c_int),
        ('red_mask',C.c_ulong),('green_mask',C.c_ulong),('blue_mask',C.c_ulong),
        ('bits_per_rgb',C.c_int),('map_entries',C.c_int)]


class AttributePrefix(C.Structure):
    _fields_=[('x',C.c_int),('y',C.c_int),('width',C.c_int),('height',C.c_int),
        ('border_width',C.c_int),('depth',C.c_int),('visual',C.POINTER(Visual))]


class AppWindowCapture:
    def __init__(self):
        self.x=C.CDLL(ctypes.util.find_library('X11'))
        self.composite=C.CDLL(ctypes.util.find_library('Xcomposite'))
        self.x.XOpenDisplay.argtypes=[C.c_char_p];self.x.XOpenDisplay.restype=C.c_void_p
        self.display=self.x.XOpenDisplay(None)
        if not self.display:raise RuntimeError('X11 display unavailable')
        self.composite.XCompositeNameWindowPixmap.argtypes=[C.c_void_p,C.c_ulong]
        self.composite.XCompositeNameWindowPixmap.restype=C.c_ulong
        self.x.XGetGeometry.argtypes=[C.c_void_p,C.c_ulong,C.POINTER(C.c_ulong),C.POINTER(C.c_int),C.POINTER(C.c_int),C.POINTER(C.c_uint),C.POINTER(C.c_uint),C.POINTER(C.c_uint),C.POINTER(C.c_uint)]
        self.x.XGetImage.argtypes=[C.c_void_p,C.c_ulong,C.c_int,C.c_int,C.c_uint,C.c_uint,C.c_ulong,C.c_int]
        self.x.XGetImage.restype=C.POINTER(XImage)
        self.x.XGetWindowAttributes.argtypes=[C.c_void_p,C.c_ulong,C.c_void_p]
        self.x.XDestroyImage.argtypes=[C.POINTER(XImage)]
        self.x.XFreePixmap.argtypes=[C.c_void_p,C.c_ulong]
        self.x.XCloseDisplay.argtypes=[C.c_void_p]

    def grab(self,window):
        pixmap=self.composite.XCompositeNameWindowPixmap(self.display,window)
        root=C.c_ulong();x=C.c_int();y=C.c_int();width=C.c_uint();height=C.c_uint();border=C.c_uint();depth=C.c_uint()
        try:
            self.x.XGetGeometry(self.display,pixmap,C.byref(root),C.byref(x),C.byref(y),C.byref(width),C.byref(height),C.byref(border),C.byref(depth))
            image=self.x.XGetImage(self.display,pixmap,0,0,width.value,height.value,C.c_ulong(-1).value,2)
            if not image:raise RuntimeError('App backing image unavailable')
            try:
                im=image.contents
                attributes=C.create_string_buffer(512)
                self.x.XGetWindowAttributes(self.display,window,attributes)
                visual=C.cast(attributes,C.POINTER(AttributePrefix)).contents.visual.contents
                masks=(visual.red_mask,visual.green_mask,visual.blue_mask)
                # XGetImage of a pixmap has no Visual and reports zero masks;
                # use the owning app window's actual Visual instead.
                if im.bits_per_pixel!=32 or im.byte_order!=0 or masks!=(0xff0000,0xff00,0xff):
                    raise RuntimeError(f'Unsupported X11 pixel format: bits={im.bits_per_pixel}, order={im.byte_order}, masks={(im.red_mask,im.green_mask,im.blue_mask)}, depth={im.depth}')
                raw=C.string_at(im.data,im.bytes_per_line*im.height)
                return np.frombuffer(raw,np.uint8).reshape(im.height,im.bytes_per_line//4,4)[:,:im.width,:3].copy()
            finally:self.x.XDestroyImage(image)
        finally:self.x.XFreePixmap(self.display,pixmap)

    def close(self):
        if self.display:self.x.XCloseDisplay(self.display);self.display=None
