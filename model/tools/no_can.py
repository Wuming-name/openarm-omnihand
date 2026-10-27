"""Linux x86_64 inherited seccomp barrier: prohibit AF_CAN sockets before ROS/Kit loads."""
import ctypes, os, platform
def install():
    assert platform.machine() == 'x86_64'
    class Filter(ctypes.Structure):
        _fields_=[('code',ctypes.c_ushort),('jt',ctypes.c_ubyte),('jf',ctypes.c_ubyte),('k',ctypes.c_uint)]
    class Program(ctypes.Structure):
        _fields_=[('len',ctypes.c_ushort),('filter',ctypes.POINTER(Filter))]
    # Validate architecture, then deny socket(AF_CAN, ...); all other syscalls allowed.
    rules=[(0x20,0,0,4),(0x15,1,0,0xc000003e),(0x06,0,0,0x80000000),
           (0x20,0,0,0),(0x15,0,3,41),(0x20,0,0,16),(0x15,0,1,29),
           (0x06,0,0,0x00050001),(0x06,0,0,0x7fff0000)]
    arr=(Filter*len(rules))(*(Filter(*r) for r in rules)); prog=Program(len(rules),arr)
    libc=ctypes.CDLL(None,use_errno=True)
    if libc.prctl(38,1,0,0,0) or libc.prctl(22,2,ctypes.byref(prog),0,0):
        raise OSError(ctypes.get_errno(),'Failed AF_CAN seccomp barrier')
    print('S5A AF_CAN seccomp barrier installed',flush=True)
if __name__=='__main__':
    import sys
    install();os.execvp(sys.argv[1],sys.argv[1:])
