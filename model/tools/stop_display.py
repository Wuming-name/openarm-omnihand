"""Stop only this model's current ROS domain display launch, using normal SIGINT."""
from pathlib import Path
import os,signal,time
count=0;domain=os.environ.get('ROS_DOMAIN_ID','188')
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  cmd=(p/'cmdline').read_bytes().split(b'\0');env=(p/'environ').read_bytes()
  if any(c.endswith(b'/ros2') for c in cmd) and b'launch' in cmd and b'openarm_omnihand_description' in cmd and ('ROS_DOMAIN_ID='+domain+'\0').encode() in env:os.kill(int(p.name),signal.SIGINT);print('SIGINT',p.name);count+=1
 except (FileNotFoundError,PermissionError,ProcessLookupError):pass
print('requested',count)
