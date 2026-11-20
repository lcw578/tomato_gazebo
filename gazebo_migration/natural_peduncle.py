"""C1 Hermite tubes with Bishop frames and exact FCL surface clearance.

Only generated migration instances are changed. Source UV/face records survive
byte-for-byte. Fruit geometry is always placed with a proper rigid transform.
"""
from functools import lru_cache
from pathlib import Path
import math
import shutil
import numpy as np
import trimesh
import fcl
from scipy.optimize import minimize_scalar


def unit(v):
    return np.asarray(v) / max(np.linalg.norm(v), 1e-14)


class HermiteTube:
    def __init__(self, a, b, ma, mb, source_path, r_a, r_b):
        self.a, self.b, self.ma, self.mb = map(np.asarray, (a, b, ma, mb))
        self.source_path = np.asarray(source_path)
        self.source_arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(self.source_path, axis=0), axis=1))]
        self.r_a, self.r_b = r_a, r_b
        self.grid = np.linspace(0., 1., 2001)
        xyz = self.position(self.grid)
        self.arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xyz, axis=0), axis=1))]
        self.ts = self.derivative(self.grid)
        self.ts /= np.linalg.norm(self.ts, axis=1)[:, None]
        n = unit(np.cross(self.ts[0], [0., 0., 1.]))
        if np.linalg.norm(n) < .9: n = unit(np.cross(self.ts[0], [0., 1., 0.]))
        frames = [n]
        for t0, t1 in zip(self.ts, self.ts[1:]):
            axis = np.cross(t0, t1)
            # Minimal rotation; no Frenet normal or curvature division.
            n = n + np.cross(axis, n) + np.cross(axis, np.cross(axis, n))/(1+np.dot(t0,t1))
            n = unit(n-t1*np.dot(t1,n))
            frames.append(n)
        self.ns = np.asarray(frames)

    def position(self, s):
        s = np.asarray(s)[...,None]
        return (2*s**3-3*s*s+1)*self.a+(s**3-2*s*s+s)*self.ma+(-2*s**3+3*s*s)*self.b+(s**3-s*s)*self.mb

    def derivative(self, s):
        s = np.asarray(s)[...,None]
        return (6*s*s-6*s)*self.a+(3*s*s-4*s+1)*self.ma+(-6*s*s+6*s)*self.b+(3*s*s-2*s)*self.mb

    def curvature(self, s):
        s0 = np.asarray(s)[...,None]
        second = (12*s0-6)*self.a+(6*s0-4)*self.ma+(-12*s0+6)*self.b+(6*s0-2)*self.mb
        d = self.derivative(s)
        return np.linalg.norm(np.cross(d, second), axis=-1)/np.linalg.norm(d, axis=-1)**3

    def source_fraction(self, point):
        segments = np.diff(self.source_path, axis=0)
        lengths = np.diff(self.source_arc)
        u = np.clip(np.sum((point-self.source_path[:-1])*segments, axis=1)/lengths**2,0,1)
        centers = self.source_path[:-1]+segments*u[:,None]
        i = np.argmin(np.linalg.norm(centers-point,axis=1))
        return (self.source_arc[i]+u[i]*lengths[i])/self.source_arc[-1]

    def parameter(self, fraction):
        return np.interp(fraction*self.arc[-1], self.arc, self.grid)

    def map_center(self, point):
        return self.position(self.parameter(self.source_fraction(point)))

    def radius(self, point):
        s = self.source_fraction(point)
        return self.r_a+(self.r_b-self.r_a)*(s*s*(3-2*s))

    def ring(self, fraction, u):
        s = self.parameter(fraction)
        t = unit(self.derivative(s))
        n = np.array([np.interp(s,self.grid,self.ns[:,i]) for i in range(3)])
        n = unit(n-t*np.dot(t,n))
        b = np.cross(t,n)
        radius = self.r_a+(self.r_b-self.r_a)*(fraction*fraction*(3-2*fraction))
        return self.position(s)+radius*(math.cos(2*math.pi*u)*n+math.sin(2*math.pi*u)*b)

    def record(self):
        maxima = [float(self.curvature(s)) for s in (0.,1.)]
        for lo,hi in zip(np.linspace(0,1,21)[:-1],np.linspace(0,1,21)[1:]):
            maxima.append(float(-minimize_scalar(lambda s:-self.curvature(s),bounds=(lo,hi),method='bounded').fun))
        crown=float(max(0.,self.position(self.grid)[:,2].max()-max(self.a[2],self.b[2])))
        return dict(type='cubic_hermite_bishop', A=self.a.tolist(), B=self.b.tolist(),
                    tangent_A=self.ma.tolist(), tangent_B=self.mb.tolist(),
                    source_centerline=self.source_path.tolist(), radius_A_m=self.r_a, radius_B_m=self.r_b,
                    chord_length_m=float(np.linalg.norm(self.b-self.a)),
                    arc_length_m=float(self.arc[-1]), crown_height_m=crown,
                    max_curvature_m_inv=max(maxima),
                    min_speed=float(np.linalg.norm(self.derivative(self.grid),axis=1).min()))

    @classmethod
    def from_record(cls, r):
        return cls(r['A'],r['B'],r['tangent_A'],r['tangent_B'],r['source_centerline'],r['radius_A_m'],r['radius_B_m'])


def tube_obj_vertices(source, tube):
    """Remap the known closed, face-expanded source tube, without re-topology."""
    lines = Path(source).read_text().splitlines(keepends=True)
    vertices = np.array([[float(x) for x in l.split()[1:4]] for l in lines if l.startswith('v ')])
    tex = np.array([[float(x) for x in l.split()[1:3]] for l in lines if l.startswith('vt ')])
    assert len(vertices) == len(tex) and (len(vertices)-96)%96 == 0
    count = len(vertices)-96  # Two 16-triangle end caps, three vertices per triangle.
    side_tex = tex[:count]
    length_uv = side_tex[:,1].max()
    new = np.empty_like(vertices)
    # Cache repeated seam/corner vertices to preserve bitwise closure on export.
    cache = {}
    for i in range(count):
        key = tuple(side_tex[i])
        if key not in cache: cache[key] = tube.ring(side_tex[i,1]/length_uv,side_tex[i,0])
        new[i] = cache[key]
    for end in range(2):
        for j in range(16):
            start = count+end*48+j*3
            new[start] = tube.position(float(end))
            angles = [(j+1)/16,j/16] if end == 0 else [j/16,(j+1)/16]
            for q,u in enumerate(angles): new[start+q+1] = tube.ring(float(end),u)
    return lines, new


def write_tube_obj(source,target,tube,origin):
    lines, vertices = tube_obj_vertices(source,tube)
    target.parent.mkdir(parents=True,exist_ok=True)
    index=0
    with target.open('w') as dst:
        for line in lines:
            if line.startswith('v '):
                dst.write('v '+' '.join(f'{x:.12g}' for x in vertices[index]-origin)+'\n'); index+=1
            elif line.startswith('vn '):
                raise ValueError('Source tube contains unexpected normal records; explicit normals must be rebuilt')
            elif line.startswith('mtllib '): dst.write(f'mtllib {target.with_suffix(".mtl").name}\n')
            else: dst.write(line)
    if source.with_suffix('.mtl').exists(): shutil.copy2(source.with_suffix('.mtl'),target.with_suffix('.mtl'))


@lru_cache(maxsize=256)
def obj_mesh(path):
    from randomize_corridor_instances import raw_obj
    mesh=raw_obj(Path(path))
    mesh.update_faces(mesh.nondegenerate_faces(height=1e-10))
    mesh.remove_unreferenced_vertices()
    return mesh


@lru_cache(maxsize=256)
def bvh(path):
    mesh=obj_mesh(path)
    result=fcl.BVHModel(); result.beginModel(len(mesh.vertices),len(mesh.faces))
    result.addSubModel(np.asarray(mesh.vertices,dtype=np.float64),np.asarray(mesh.faces,dtype=np.int32)); result.endModel()
    return result


def group(objects):
    manager=fcl.DynamicAABBTreeCollisionManager(); manager.registerObjects(objects); manager.setup()
    return (manager,objects)


def distance(left,right):
    data=fcl.DistanceData(fcl.DistanceRequest(enable_nearest_points=True))
    left[0].distance(right[0],data,fcl.defaultDistanceCallback)
    value=float(data.result.min_distance)
    return 0. if value<1e-12 else value


def geometry_group(nodes, linear=np.eye(3), offset=np.zeros(3), common=None):
    from randomize_corridor_instances import parse_pose, rotation_between
    objects=[]
    for node in nodes:
        center,rotation=parse_pose(node); shape=next(iter(node.find('geometry')))
        if shape.tag=='mesh':
            uri=shape.findtext('uri'); path=common/uri.split('tomato_v2_common_assets/',1)[1]
            scale=np.fromstring(shape.findtext('scale','1 1 1'),sep=' ')
            if np.allclose(linear.T@linear,np.eye(3),atol=1e-10) and np.allclose(scale,1):
                geometry=bvh(str(path)); transform=fcl.Transform(linear@rotation,linear@center+offset)
            else:
                mesh=obj_mesh(str(path)); verts=((mesh.vertices*scale)@rotation.T+center)@linear.T+offset
                geometry=fcl.BVHModel(); geometry.beginModel(len(verts),len(mesh.faces)); geometry.addSubModel(verts,np.asarray(mesh.faces,dtype=np.int32));geometry.endModel();transform=fcl.Transform()
        elif shape.tag=='sphere':
            geometry=fcl.Sphere(float(shape.findtext('radius')));transform=fcl.Transform(linear@center+offset)
        elif shape.tag=='cylinder':
            half=float(shape.findtext('length'))/2
            a=linear@(center-rotation[:,2]*half)+offset;b=linear@(center+rotation[:,2]*half)+offset
            geometry=fcl.Cylinder(float(shape.findtext('radius')),float(np.linalg.norm(b-a)))
            transform=fcl.Transform(rotation_between(np.array([0.,0.,1.]),b-a),(a+b)/2)
        else: raise ValueError(shape.tag)
        objects.append(fcl.CollisionObject(geometry,transform))
    return group(objects)


def stem_nodes(link,plant):
    from randomize_corridor_instances import stem_attachment_geom
    return [n for n in list(link.findall('collision'))+list(link.findall('visual'))
            if stem_attachment_geom(n.get('name',''),plant) is None and
            (n.tag=='collision' or '_vine_' in n.get('name','') or 'Branch1' in n.get('name',''))]


def source_chain(root,plant,k):
    import re
    from randomize_corridor_instances import cylinder_ends,parse_pose
    prefix='contact_stalk_plant_' if (plant,k)==(9,1) else f'plant_peduncle_contact_{plant}_{k}_'
    nodes=sorted([n for n in root.findall('collision') if re.fullmatch(re.escape(prefix)+r'\d+',n.get('name',''))],key=lambda n:n.get('name'))
    first=list(cylinder_ends(nodes[0]));next_center=parse_pose(nodes[1])[0]
    start=max(first,key=lambda p:np.linalg.norm(p-next_center)); result=[start]
    for n in nodes:
        ends=cylinder_ends(n);result.append(max(ends,key=lambda p:np.linalg.norm(p-result[-1])))
    return np.asarray(result)


@lru_cache(maxsize=40)
def rachis_interface(path):
    mesh=obj_mesh(path)
    shaft=max(mesh.split(only_watertight=False),key=lambda m:m.extents[2])
    top=shaft.bounds[1,2]
    centers=[]
    radii=[]
    for dz in (.0004,.0008,.0012,.0018,.0025,.003):
        section=shaft.section(plane_origin=[0,0,top-dz],plane_normal=[0,0,1])
        if section is None: continue
        loops=section.discrete
        if not loops: continue
        loop=max(loops,key=len)[:-1]
        # Midpoint of section bounds is unbiased by polygon edge tessellation.
        center=(loop.min(0)+loop.max(0))/2
        centers.append(center);radii.append(np.min(np.linalg.norm(loop[:,:2]-center[:2],axis=1)))
        if len(centers)==4: break
    if len(centers)<3: raise ValueError(f"No usable top shaft cross sections: {path}")
    centers=np.array(centers)
    coefficients=np.array([np.polyfit(centers[:,2],centers[:,i],1) for i in (0,1)])
    z=top-.002
    point=np.array([np.polyval(coefficients[0],z),np.polyval(coefficients[1],z),z])
    tangent=unit(np.r_[-coefficients[:,0],-1.])
    return point,tangent,float(np.mean(radii)),dict(shaft_component_faces=len(shaft.faces),shaft_top_z=float(top), interface_overlap_m=.002)


def place_truss(cfg,side,k,root,link,plant,linear,stem_group,path,radii,arc,params,previous):
    from randomize_corridor_instances import (COMMON,parse_pose,path_at,rotation_between,rot,rot_axis,peduncle_endpoints,stem_attachment_geom,shape_linear)
    import copy
    old_origin=parse_pose(link)[0]
    old_a,old_b,r_a,_=peduncle_endpoints(root,plant,k)
    chain=source_chain(root,plant,k)
    rachis_node=next(n for n in link.findall('visual') if 'rachis' in n.get('name',''))
    rachis_path=COMMON/rachis_node.findtext('geometry/mesh/uri').split('tomato_v2_common_assets/',1)[1]
    b_local,terminal,r_b,shaft_info=rachis_interface(str(rachis_path))
    source_b=b_local+old_origin
    mesh=obj_mesh(str(rachis_path))
    down=unit(np.linalg.svd(mesh.vertices-mesh.vertices.mean(0))[2][0])
    if down[2]>0: down=-down
    fc=params['fruit'][k];tc=params['truss'][k]
    center,tangent,index=path_at(path,arc+tc['slide_m'])
    aisle=np.array([0.,1. if side==0 else -1.,0.])
    options=cfg.get('randomization',{}).get('peduncle_curve',{})
    required_self=float(options.get('min_self_stem_clearance_m',.015))
    required_intra=float(options.get('min_intra_plant_clearance_m',.050))
    # Desired random azimuth survives, subject to the hard geometric constraints.
    requested=tc['azimuth_rad']+fc['yaw_rad']*.35
    nodes=list(link.findall('visual'))+list(link.findall('collision'))
    alignment=rotation_between(down,np.array([0.,0.,-1.]))
    swing=rot('x',fc['gravity_roll_rad'])@rot('y',fc['gravity_pitch_rad'])
    gravity=swing@alignment
    start_degrees=float(np.clip(22+math.degrees(fc['pitch_rad'])*.65,10,35))
    chord_min,chord_max=map(float,options.get('chord_length_range_m',[.052,.088]))
    arc_min,arc_max=map(float,options.get('arc_length_range_m',[.058,.098]))
    crown_max=float(options.get('max_crown_height_m',.012))
    chords=options.get('chord_length_candidates_m',np.linspace(chord_min,chord_max,10).tolist())
    down_xy=unit(np.r_[-tangent[:2],0.]) if np.linalg.norm(tangent[:2])>1e-8 else np.zeros(3)
    # Prefer the aisle-facing projection away from the stem's downhill direction.
    preferred=aisle-down_xy*max(0.,float(np.dot(aisle,down_xy)))
    preferred=unit(preferred) if np.linalg.norm(preferred)>1e-8 else aisle
    preferred=rot('z',requested)@preferred
    if np.dot(preferred,aisle)<float(options.get('aisle_direction_min_dot',0.)) or np.dot(preferred,down_xy)>float(options.get('stem_down_direction_max_dot',.20)):
        preferred=unit(aisle-down_xy*max(0.,float(np.dot(aisle,down_xy))))
    offsets=np.deg2rad([0.,15.,-15.,30.,-30.,45.,-45.,60.,-60.,75.,-75.,90.,-90.,105.,-105.,120.,-120.])
    spin_degrees=[0.]
    for angle in range(10,180,10): spin_degrees.extend([float(angle),float(-angle)])
    spin_degrees.append(180.)
    spin_offsets=np.deg2rad(spin_degrees)
    debug=[]
    dz=-.004
    for chord in chords:
        chord=float(chord)
        if not chord_min<=chord<=chord_max: continue
        horizontal=math.sqrt(max(0.,chord*chord-dz*dz))
        for extra in offsets:
            direction=rot('z',extra)@preferred
            if np.dot(direction,aisle)<float(options.get('aisle_direction_min_dot',0.)): continue
            if float(np.dot(direction,down_xy))>float(options.get('stem_down_direction_max_dot',.20)): continue
            radial=unit(direction-tangent*np.dot(direction,tangent))
            anchor=center+radial*(radii[index]-.0015)
            # Align the actual top shaft tangent outward, keeping its whole axis
            # gravity aligned. Cluster yaw is a small seeded residual about this
            # safe azimuth, rather than an unconstrained inward turn.
            terminal_down=gravity@terminal
            base_yaw=math.atan2(direction[1],direction[0])-math.atan2(terminal_down[1],terminal_down[0])
            new_b=anchor+horizontal*direction+np.array([0.,0.,dz])
            length=np.linalg.norm(new_b-anchor)
            if not chord_min<=length<=chord_max: continue
            ta=math.cos(math.radians(start_degrees))*direction+np.array([0.,0.,math.sin(math.radians(start_degrees))])
            chosen=None
            for spin in spin_offsets:
                yaw=base_yaw+fc['fruit_yaw_rad']*.30+spin
                fruit_linear=rot('z',yaw)@gravity
                tip_direction=unit(fruit_linear@terminal)
                offset=new_b-fruit_linear@source_b
                body=geometry_group(nodes,fruit_linear,offset+fruit_linear@old_origin,COMMON)
                self_gap=distance(stem_group,body)
                if self_gap<required_self+.0002:
                    debug.append(("self",chord,float(extra),float(spin),self_gap)); continue
                intra=min((distance(body,other) for other in previous),default=100.)
                if intra<required_intra+.0002:
                    debug.append(("intra",chord,float(extra),float(spin),intra)); continue
                tube=HermiteTube(anchor,new_b,ta*length,tip_direction*length,chain,r_a,max(r_b,.001))
                if not arc_min<=tube.arc[-1]<=arc_max:
                    debug.append(('arc',chord,float(extra),float(spin),float(tube.arc[-1])));continue
                crown=float(max(0.,tube.position(tube.grid)[:,2].max()-max(anchor[2],new_b[2])))
                if crown>crown_max:
                    debug.append(('crown',chord,float(extra),float(spin),crown));continue
                chosen=(yaw,fruit_linear,tip_direction,offset,body,self_gap,intra,tube)
                break
            if chosen is None: continue
            yaw,fruit_linear,tip_direction,offset,body,self_gap,intra,tube=chosen
            # A monotone outward projection proves no handle/backtracking or
            # centerline self intersection. Regularity bounds curvature too.
            if np.min(tube.derivative(tube.grid)@direction)<1e-5:
                debug.append(('backtrack',chord,float(extra)));continue
            record=tube.record()
            if record['max_curvature_m_inv']*max(r_a,r_b)>.65:
                debug.append(('curvature',chord,float(extra),record['max_curvature_m_inv']*max(r_a,r_b)));continue
            ped_nodes=[]
            for original in list(root.findall('collision'))+list(root.findall('visual')):
                if stem_attachment_geom(original.get('name',''),plant)!=k or original.find('geometry/mesh') is not None: continue
                n=copy.deepcopy(original)
                n.find('geometry/*/radius').text=str(tube.radius(parse_pose(n)[0]))
                shape_linear(n,np.eye(3),tube.map_center,np.zeros(3));ped_nodes.append(n)
            ped_group=geometry_group(ped_nodes,common=COMMON)
            full=group(body[1]+ped_group[1])
            full_intra=min((distance(full,other) for other in previous),default=100.)
            if full_intra<required_intra+.0002:
                debug.append(('full_intra',chord,float(extra),full_intra)); continue
            a_cap=group([fcl.CollisionObject(fcl.Sphere(r_a),fcl.Transform(anchor))])
            b_cap=group([fcl.CollisionObject(fcl.Sphere(max(r_b,.001)),fcl.Transform(new_b))])
            stem_gap=distance(stem_group,a_cap);tip_gap=distance(body,b_cap)
            if stem_gap>1e-8 or tip_gap>1e-8:
                debug.append(('interface',chord,float(extra),stem_gap,tip_gap));continue
            record.update(rachis_source_interface=shaft_info, start_elevation_deg=start_degrees,
                          outward_xy_unit=direction.tolist(),outreach_m=float(horizontal),
                          stem_down_xy_unit=down_xy.tolist(),
                          stem_down_dot=float(np.dot(direction,down_xy)),
                          azimuth_offset_deg=float(math.degrees(extra)),
                          fruit_spin_offset_deg=float(math.degrees(spin)),
                          minimum_outward_derivative_m=float(np.min(tube.derivative(tube.grid)@direction)))
            return dict(tube=tube,fruit_linear=fruit_linear,source_b=source_b,new_b=new_b,
                        anchor=anchor,body=body,full=full,down=down,terminal=terminal,
                        self_gap=self_gap,intra_gap=intra,curve_record=record,
                        stem_gap=stem_gap,tip_gap=tip_gap,tangent=tangent,center=center,
                        fruit_yaw=yaw,old_a=old_a,old_b=old_b)
    from collections import Counter
    reasons=Counter(row[0] for row in debug)
    examples={reason:next(row for row in debug if row[0]==reason) for reason in reasons}
    raise ValueError(f'No safe compact Hermite peduncle for plant {plant}, truss {k}, side {side}: '
                     f'{dict(reasons)}; examples={examples}')
