from pathlib import Path
import ctypes,subprocess,re,time,sys
from PIL import ImageGrab
P=Path(__file__).resolve().parents[1]
x=ctypes.CDLL('libX11.so.6');x.XOpenDisplay.restype=ctypes.c_void_p;x.XOpenDisplay.argtypes=[ctypes.c_char_p];d=x.XOpenDisplay(None)
for f in ['XRaiseWindow','XMoveResizeWindow','XFlush','XSetInputFocus']:getattr(x,f).restype=ctypes.c_int
x.XRaiseWindow.argtypes=[ctypes.c_void_p,ctypes.c_ulong];x.XMoveResizeWindow.argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.c_int,ctypes.c_int,ctypes.c_uint,ctypes.c_uint];x.XFlush.argtypes=[ctypes.c_void_p]
s=subprocess.check_output(['xwininfo','-root','-tree'],text=True)
for pattern,coords in [(' - RViz',(430,50,1400,960)),('"Joint State Publisher"',(20,50,390,960))]:
 lines=[l for l in s.splitlines() if pattern in l]
 if lines:
  wid=int(re.search('0x[0-9a-f]+',lines[0])[0],16);x.XMoveResizeWindow(d,wid,*coords);x.XRaiseWindow(d,wid)
x.XFlush(d);time.sleep(.7)
out=Path(sys.argv[1]) if len(sys.argv)>1 else P/'screenshots/desktop.png';ImageGrab.grab(xdisplay=':1').save(out)
