#!/usr/bin/env python3
from pathlib import Path
import sys,json,xml.etree.ElementTree as E
import numpy as np
from model_math import Robot,origin
P=Path(__file__).resolve().parents[1];U=P/'ros2_description/urdf/openarm_omnihand.urdf';r=Robot(U);rows=json.loads((P/'manifest/joint_manifest.json').read_text());poses=json.loads((P/'ros2_description/config/poses.json').read_text())
assert len(r.links)==len(r.root.findall('link'));assert len({j.get('name') for j in r.joints})==len(r.joints)
assert len({j.find('child').get('link') for j in r.joints})==len(r.joints)
assert all(j.find('parent').get('link') in r.links and j.find('child').get('link') in r.links for j in r.joints)
for mesh in r.root.findall('.//mesh'):
 uri=mesh.get('filename');assert uri.startswith('package://openarm_omnihand_description/');assert (P/'ros2_description'/uri.split('/',3)[3]).is_file()
active={j.get('name'):j for j in r.joints if j.get('type')!='fixed'}
for n,j in active.items():
 assert j.get('type')=='revolute';axis=np.fromstring(j.find('axis').get('xyz'),sep=' ');assert np.isfinite(axis).all() and abs(np.linalg.norm(axis)-1)<1e-8
 lim=j.find('limit');assert float(lim.get('lower'))<float(lim.get('upper'));assert float(lim.get('effort'))>0 and float(lim.get('velocity'))>0
 if j.find('mimic') is not None:assert j.find('mimic').get('joint') in rows['independent']
 for p,q in poses.items():
  assert float(lim.get('lower'))-1e-12<=q[n]<=float(lim.get('upper'))+1e-12
  m=j.find('mimic')
  if m is not None:assert abs(q[n]-(q[m.get('joint')]*float(m.get('multiplier'))+float(m.get('offset','0'))))<1e-12
refs=Robot(P/'sources/accepted_closed.urdf');q=poses['closed_hand'];oldfk=refs.fk(q)[0];newfk=r.fk(q)[0];closed_error=max(np.max(np.abs(oldfk[n]-newfk[n])) for n in r.links);assert closed_error<1e-12
for j in refs.joints:
 if not j.get('name').startswith('omni_'):
  other=next(x for x in r.joints if x.get('name')==j.get('name'))
  assert E.tostring(j).strip()==E.tostring(other).strip()
changes={}
for n in rows['independent']:
 j=active[n];q2=q.copy();lo=float(j.find('limit').get('lower'));hi=float(j.find('limit').get('upper'));q2[n]=q[n]+(.01 if q[n]+.01<hi else -.01);T=r.fk(q2)[0];delta=float(np.max(np.abs(T[j.find('child').get('link')]-newfk[j.find('child').get('link')])));assert delta>1e-6;changes[n]=delta
for n,l in r.links.items():
 a=refs.links[n];b=E.fromstring(E.tostring(l))
 # URI relocation is the only link-element change.
 for mesh,old in zip(b.findall('.//mesh'),a.findall('.//mesh')):mesh.set('filename',old.get('filename'))
 assert E.tostring(b).strip()==E.tostring(a).strip(),n
result={'passed':True,'links':len(r.links),'joints':len(r.joints),'revolute':len(active),'independent':len(rows['independent']),'closed_FK_max_element_error':closed_error,'independent_joint_transform_changes':changes,'FK':{p:{n:T.tolist() for n,T in r.fk(v)[0].items()} for p,v in poses.items()},'mass_total_kg':sum(x[0] for x in r.bodies.values()),'mass_inertia_preserved':True,'all_link_geometry_preserved':True}
(P/'reports/structural_validation.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k not in ['FK','independent_joint_transform_changes']}))
