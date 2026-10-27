"""Offline URDF, rigid-body and mesh math. No robot interfaces."""
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation


def vec(value, default='0 0 0'):
    return np.fromstring(value if value is not None else default, sep=' ')


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    out = np.eye(4)
    out[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    out[:3, 3] = xyz
    return out


def origin(element):
    item = element.find('origin')
    return np.eye(4) if item is None else transform(vec(item.get('xyz')), vec(item.get('rpy')))


def points(T, p):
    return np.asarray(p) @ T[:3, :3].T + T[:3, 3]


def fmt(a):
    return ' '.join(f'{float(x):.15g}' for x in np.asarray(a).flat)


def put_origin(parent, T):
    old = parent.find('origin')
    if old is not None:
        parent.remove(old)
    return ET.SubElement(parent, 'origin', xyz=fmt(T[:3, 3]),
                         rpy=fmt(Rotation.from_matrix(T[:3, :3]).as_euler('xyz')))


def read_inertial(link):
    e = link.find('inertial')
    if e is None:
        return 0., np.zeros(3), np.zeros((3, 3))
    T = origin(e)
    d = {k: float(v) for k, v in e.find('inertia').attrib.items()}
    I = np.array([[d['ixx'], d['ixy'], d['ixz']], [d['ixy'], d['iyy'], d['iyz']],
                  [d['ixz'], d['iyz'], d['izz']]])
    return float(e.find('mass').get('value')), T[:3, 3], T[:3, :3] @ I @ T[:3, :3].T


def put_inertial(link, mass, com, inertia):
    for old in link.findall('inertial'):
        link.remove(old)
    e = ET.SubElement(link, 'inertial')
    put_origin(e, transform(com))
    ET.SubElement(e, 'mass', value=str(float(mass)))
    keys = [('ixx',0,0),('ixy',0,1),('ixz',0,2),('iyy',1,1),('iyz',1,2),('izz',2,2)]
    ET.SubElement(e, 'inertia', **{k: f'{inertia[i,j]:.15g}' for k,i,j in keys})


def aggregate(bodies):
    mass = sum(m for m, c, I in bodies)
    if mass <= 0:
        raise ValueError('No positive mass to aggregate')
    com = sum(m*c for m,c,I in bodies) / mass
    inertia = np.zeros((3,3))
    for m,c,I in bodies:
        r=c-com
        inertia += I + m*(r@r*np.eye(3)-np.outer(r,r))
    return mass, com, inertia


def stl_triangles(path):
    data = Path(path).read_bytes()
    n = struct.unpack('<I', data[80:84])[0] if len(data)>=84 else 0
    if len(data)==84+50*n:
        dtype=np.dtype([('normal','<f4',3),('v','<f4',(3,3)),('a','<u2')])
        return np.frombuffer(data, offset=84, dtype=dtype)['v'].astype(float)
    return np.array([line.split()[1:] for line in data.decode().splitlines()
                     if line.strip().startswith('vertex ')], float).reshape(-1,3,3)


def closed_surface_properties(triangles, density):
    """Tetrahedral integration with a nearby origin; fail on open/nonmanifold meshes."""
    _, inv = np.unique(np.round(triangles.reshape(-1,3), 8), axis=0, return_inverse=True)
    f=inv.reshape(-1,3)
    edges=np.sort(np.concatenate([f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]]),axis=1)
    _, counts=np.unique(edges,axis=0,return_counts=True)
    if np.any(counts!=2):
        raise ValueError('Surface is not closed/manifold')
    ref=triangles.mean(axis=(0,1))
    a,b,c=(triangles-ref).transpose(1,0,2)
    v=np.einsum('ij,ij->i',a,np.cross(b,c))/6
    if v.sum()<0:
        v=-v
    volume=v.sum()
    if volume<=1e-15:
        raise ValueError('Surface volume is zero')
    first=np.einsum('i,ij->j',v,(a+b+c)/4)
    s=a+b+c
    second=np.einsum('i,ij,ik->jk',v,s,s)
    for z in [a,b,c]:
        second+=np.einsum('i,ij,ik->jk',v,z,z)
    second/=20
    center=first/volume
    central=second-volume*np.outer(center,center)
    I=density*(np.trace(central)*np.eye(3)-central)
    return density*volume, center+ref, I, volume


class Robot:
    def __init__(self, root):
        self.root = ET.parse(root).getroot() if isinstance(root,(str,Path)) else root
        self.links={x.get('name'):x for x in self.root.findall('link')}
        self.joints=self.root.findall('joint')
        children={j.find('child').get('link') for j in self.joints}
        roots=set(self.links)-children
        if len(roots)!=1:
            raise ValueError(f'Expected one root, got {roots}')
        self.base=roots.pop()
        self.joint_names=[j.get('name') for j in self.joints if j.get('type')!='fixed']
        self.bodies={n:read_inertial(l) for n,l in self.links.items()}

    def fk(self, q=None):
        q={} if q is None else q
        if not isinstance(q,dict):
            q=dict(zip(self.joint_names,q))
        poses={self.base:np.eye(4)}
        axes={}
        parents={}
        left=list(self.joints)
        while left:
            progressed=False
            for j in left[:]:
                parent=j.find('parent').get('link'); child=j.find('child').get('link')
                if parent not in poses:
                    continue
                T=poses[parent]@origin(j)
                parents[child]=(parent,j.get('name'))
                if j.get('type')!='fixed':
                    axis=vec(j.find('axis').get('xyz'));axis/=np.linalg.norm(axis)
                    name=j.get('name');value=q.get(name,0.)
                    mimic=j.find('mimic')
                    if mimic is not None:
                        value=q.get(mimic.get('joint'),0)*float(mimic.get('multiplier','1'))+float(mimic.get('offset','0'))
                    axes[name]=(T[:3,3].copy(),T[:3,:3]@axis)
                    M=np.eye(4);M[:3,:3]=Rotation.from_rotvec(axis*value).as_matrix()
                    T=T@M
                poses[child]=T
                left.remove(j);progressed=True
            if not progressed:
                raise ValueError('Disconnected or cyclic kinematic graph')
        return poses,axes,parents

    def gravity(self,q,gravity=np.array([0.,0.,-9.81])):
        poses,axes,parents=self.fk(q)
        tau={name:0. for name in self.joint_names}
        for name,(mass,com,I) in self.bodies.items():
            if not mass:
                continue
            p=points(poses[name],com)
            cursor=name
            while cursor in parents:
                parent,joint=parents[cursor]
                if joint in axes:
                    o,a=axes[joint]
                    # Compensation is opposite the physical gravitational moment.
                    tau[joint]-=a@np.cross(p-o,mass*gravity)
                cursor=parent
        return np.array([tau[n] for n in self.joint_names])

    def potential(self,q,gravity=np.array([0.,0.,-9.81])):
        poses,_,_=self.fk(q)
        return sum(-m*(gravity@points(poses[n],c)) for n,(m,c,I) in self.bodies.items())

    def aggregate_in_base(self,q=None):
        poses,_,_=self.fk(q)
        bodies=[]
        for n,(m,c,I) in self.bodies.items():
            if not m:
                continue
            T=poses[n]
            bodies.append((m,points(T,c),T[:3,:3]@I@T[:3,:3].T))
        return aggregate(bodies)
