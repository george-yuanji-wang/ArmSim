"""Balanced edge-only prisms and saddles, with no face diagonals or apexes."""

from itertools import permutations
import math
import numpy as np

from armsim.model_primitives import cylinder

EDGE_RADIUS=0.0018


def edge(body,name,start,end,material='arm_blue',radius=EDGE_RADIUS,group=0):
    start,end=np.asarray(start),np.asarray(end)
    length=np.linalg.norm(end-start)
    if length<1e-8:raise ValueError(f'Zero-length structural edge: {name}')
    cylinder(body,name,(start+end)/2,(end-start)/length,radius,length,
             2700*math.pi*radius**2*length,material,group)
    return name


def box_frame(body,name,low,high,width,depth,material='arm_blue'):
    """Eight vertices and twelve perimeter edges; no rectangular-face diagonals."""
    corners=np.array([[x,y,z] for z in (low,high)
                      for x,y in ((-width,-depth),(width,-depth),(width,depth),(-width,depth))])
    pairs=[(i,(i+1)%4) for i in range(4)]+[(i+4,(i+1)%4+4) for i in range(4)]
    pairs += [(i,i+4) for i in range(4)]
    names=[edge(body,f'{name}_{index}',corners[a],corners[b],material)
           for index,(a,b) in enumerate(pairs)]
    return {'low':low,'high':high,'width':width,'depth':depth,'corners':corners,'geoms':names}


def case_mount(body,name,center,axis,radius,frame):
    """Four equal stays connect four distinct mid-rim ports to a quad base.

    Roll cases use symmetric frustums. Pitch cases use trapezoidal saddles.
    Offset motors use a rectangular inset whose contact angles are chosen
    analytically for equal stay lengths. No support converges to one point.
    """
    center,axis=np.asarray(center),np.asarray(axis)
    lo,hi,w,d=frame['low'],frame['high'],frame['width'],frame['depth']
    names=[]
    if abs(axis[2])>0.9:
        diagonal=radius/np.sqrt(2)
        contacts=center+np.array([[-diagonal,-diagonal,0],[diagonal,-diagonal,0],
                                   [diagonal,diagonal,0],[-diagonal,diagonal,0]])
        base=frame['corners'][4:].copy()
        face_axis=None
    elif abs(axis[0])>0.9 and abs(center[1])>1e-8:
        distance=center[1]+d
        tangent_half=min(0.024,0.75*min(center[2]-lo,hi-center[2]))
        if tangent_half<=0:raise ValueError(f'Offset motor outside its supporting frame: {name}')
        mean=np.arctan2(tangent_half,distance)
        spread=min(mean*0.45,np.radians(10))
        first,second=mean-spread,mean+spread
        axial_half=min(0.018,0.75*min(w-center[0],w+center[0]))
        contacts=[]
        for angle,sign in ((first,-1),(second,-1),(second,1),(first,1)):
            contacts.append(center+[0,-radius*np.cos(angle),sign*radius*np.sin(angle)])
        contacts=np.array(contacts)
        base=np.array([[center[0]-axial_half,-d,center[2]-tangent_half],
                       [center[0]+axial_half,-d,center[2]-tangent_half],
                       [center[0]+axial_half,-d,center[2]+tangent_half],
                       [center[0]-axial_half,-d,center[2]+tangent_half]])
        face_axis,tangent_axis,tangent_bound=1,0,w
    else:
        face_axis=0 if abs(axis[0])>0.9 else 1
        tangent_axis=1-face_axis
        face_bound=w if face_axis==0 else d
        tangent_bound=d if face_axis==0 else w
        half=d if face_axis==0 else min(w*0.45,radius*0.48)
        # The same trapezoidal saddle can use a narrower inset for a small
        # motor rim. Its four stays remain equal and its base remains attached
        # to the original cage rails; widening the cage or adding a fallback
        # bracket would be an unrelated topology change. Preserve every
        # already-fitting footprint exactly.
        if half*1.5>=radius:
            half=radius*0.64
        available=hi-lo
        if available<=0:raise ValueError(f'Case saddle does not fit: {name}')
        rise=np.sqrt(radius**2-(half/2)**2)-np.sqrt(radius**2-(half*1.5)**2)
        if rise>available:
            # Solve sqrt(r^2-h^2/4)-sqrt(r^2-9h^2/4)=s for h.
            # Reserve a small vertical margin within the existing frame.
            span=available*0.9
            half=min(half,np.sqrt(span/2*(np.sqrt(4*radius**2+9*span**2/16)-5*span/4)))
        face=min((-face_bound,face_bound),key=lambda value:abs(value-center[face_axis]))
        lower=center[2]-np.sqrt(radius**2-(half/2)**2)
        upper=center[2]-np.sqrt(radius**2-(half*1.5)**2)
        base_low=hi-(upper-lower)
        if base_low<lo-1e-10:raise ValueError(f'Case saddle does not fit: {name}')
        contacts=[];base=[]
        for tangent,zport,zbase in ((-half/2,lower,base_low),(half/2,lower,base_low),
                                    (half*1.5,upper,hi),(-half*1.5,upper,hi)):
            contact=center.copy();contact[tangent_axis]+=tangent;contact[2]=zport
            anchor=center.copy();anchor[face_axis]=face
            anchor[tangent_axis]=np.sign(tangent)*half;anchor[2]=zbase
            contacts.append(contact);base.append(anchor)
        contacts,base=np.array(contacts),np.array(base)
    for i,(contact,anchor) in enumerate(zip(contacts,base)):
        names.append(edge(body,f'{name}_case_contact_{i}',contact,anchor,group=4))
    if np.ptp(np.linalg.norm(base-contacts,axis=1))>1e-10:
        raise ValueError(f'Unbalanced case stays: {name}')
    if face_axis is not None:
        for i,(a,b) in enumerate(zip(base,np.roll(base,-1,axis=0))):
            on_end=abs(a[2]-b[2])<1e-9 and min(abs(a[2]-lo),abs(a[2]-hi))<1e-9
            on_rail=abs(a[tangent_axis]-b[tangent_axis])<1e-9 and abs(abs(a[tangent_axis])-tangent_bound)<1e-9
            if not(on_end or on_rail):names.append(edge(body,f'{name}_base_{i}',a,b,group=4))
        for i,anchor in enumerate(base):
            if min(abs(anchor[2]-lo),abs(anchor[2]-hi))<1e-9:continue
            target=anchor.copy();target[tangent_axis]=(-1 if i in (0,3) else 1)*tangent_bound
            if np.linalg.norm(target-anchor)>1e-9:
                names.append(edge(body,f'{name}_to_rail_{i}',anchor,target,group=4))
    return names,contacts,np.vstack([contacts,base])


def output_frame(body,name,center,axis,radius,link_frame):
    """A compact rim prism with four distinct, balanced outgoing connectors."""
    center,axis=np.asarray(center),np.asarray(axis)
    first=np.array([0.,0.,1.]) if abs(axis[2])<0.9 else np.array([1.,0.,0.])
    second=np.cross(axis,first)
    half=radius+0.003
    corners=np.array([center+half*(a*first+b*second) for a,b in ((-1,-1),(1,-1),(1,1),(-1,1))])
    back=corners-axis*0.006
    names=[]
    for layer,points in enumerate((corners,back)):
        for i in range(4):
            names.append(edge(body,f'{name}_rim_{layer}_{i}',points[i],points[(i+1)%4],radius=0.0015,group=4))
    for i in range(4):
        names.append(edge(body,f'{name}_post_{i}',corners[i],back[i],radius=0.0015,group=4))
    contacts=[]
    for i,direction in enumerate((first,-first,second,-second)):
        contact=center+direction*radius
        contacts.append(contact)
        names.append(edge(body,f'{name}_disk_contact_{i}',contact,center+direction*half,radius=0.0015,group=4))
    eligible=corners if abs(axis[2])>0.9 else np.array([p for p in np.vstack((corners,back)) if p[2]>center[2]])
    anchors=link_frame['corners'][:4]
    order=min(permutations(range(4)),key=lambda order:sum(np.linalg.norm(eligible[k]-anchor)**2 for k,anchor in zip(order,anchors)))
    for i,(k,anchor) in enumerate(zip(order,anchors)):
        names.append(edge(body,f'{name}_to_link_{i}',eligible[k],anchor,radius=0.0017,group=4))
    return names,np.array(contacts),np.vstack((corners,back))
