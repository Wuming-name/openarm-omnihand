"""Rebuild the portable URDF from included source evidence, without original workspace access."""
from pathlib import Path, PurePosixPath
import xml.etree.ElementTree as E
import copy
P=Path(__file__).resolve().parents[1];root=E.parse(P/'sources/accepted_closed.urdf').getroot();off={j.get('name'):j for j in E.parse(P/'sources/omnihand_sdk_0_8_0_right.urdf').findall('joint')}
for j in list(root.findall('joint')):
 n=j.get('name')
 if n.startswith('omni_') and n[5:] in off and off[n[5:]].get('type')!='fixed':
  k=copy.deepcopy(off[n[5:]]);k.set('name',n)
  for tag in ['parent','child']:k.find(tag).set('link','omni_'+k.find(tag).get('link'))
  if k.find('mimic') is not None:k.find('mimic').set('joint','omni_'+k.find('mimic').get('joint'))
  root.remove(j);root.append(k)
for mesh in root.findall('.//mesh'):
 pkg,rel=mesh.get('filename')[10:].split('/',1);rel=PurePosixPath(rel).relative_to('meshes').as_posix();mesh.set('filename','package://openarm_omnihand_description/meshes/'+pkg+'/'+rel)
root.set('name','openarm_omnihand');E.indent(root);out=P/'ros2_description/urdf/openarm_omnihand.urdf';data=E.tostring(root,encoding='unicode',xml_declaration=True)
if out.read_text()!=data:raise RuntimeError('Reconstruction differs: refuse automatic overwrite')
print('Included-source reconstruction byte-identical')
