"""Constrained panel meshing, explicit seam welding, and inspectable previews."""
from pathlib import Path
import json
import html
import numpy as np
from .quality import mesh_metrics, self_intersections, rest_matrices, strain_metrics
from .schema import write_json

def point_key(p):
    return tuple(np.round(p,9))

def segment_distance(points, a, b):
    a,b=np.asarray(a),np.asarray(b)
    t=np.clip(((points-a)*(b-a)).sum(axis=1)/np.dot(b-a,b-a),0,1)
    return np.linalg.norm(points-(a+t[:,None]*(b-a)),axis=1)

def triangulate_panel(polygon, step, seam_segments):
    from CGAL.CGAL_Kernel import Point_2
    from CGAL.CGAL_Mesh_2 import Mesh_2_Constrained_Delaunay_triangulation_2
    from matplotlib.path import Path as PolygonPath
    from pygarment.meshgen.triangulation_utils import mark_domain
    polygon=np.asarray(polygon,dtype=float)
    cdt=Mesh_2_Constrained_Delaunay_triangulation_2()
    boundary=[]
    for a,b in zip(polygon,np.roll(polygon,-1,axis=0)):
        n=max(1,int(np.ceil(np.linalg.norm(b-a)/step)))
        boundary.extend(a+(b-a)*t/n for t in range(n))
    handles=[cdt.insert(Point_2(float(p[0]),float(p[1]))) for p in boundary]
    for a,b in zip(handles,handles[1:]+handles[:1]): cdt.insert_constraint(a,b)
    path=PolygonPath(polygon)
    low,high=polygon.min(axis=0),polygon.max(axis=0)
    grid=np.array([(x,z) for x in np.arange(low[0]+step*.5,high[0],step)
                           for z in np.arange(low[1]+step*.5,high[1],step)])
    if len(grid):
        grid=grid[path.contains_points(grid)]
        # Avoid nearly coincident interior/boundary points.
        dist=np.min(np.stack([segment_distance(grid,a,b) for a,b in zip(polygon,np.roll(polygon,-1,axis=0))]),axis=0)
        for p in grid[dist>step*.15]: cdt.insert(Point_2(float(p[0]),float(p[1])))
    pts=sorted({(v.point().x(),v.point().y()) for v in cdt.finite_vertices()})
    lookup={point_key(p):i for i,p in enumerate(pts)}
    info=mark_domain(cdt)
    faces=[]
    for f in cdt.finite_faces():
        if info[f].in_domain():
            face=[lookup[point_key((f.vertex(i).point().x(),f.vertex(i).point().y()))] for i in range(3)]
            a,b,c=np.array(pts)[face]
            if np.cross(b-a,c-a)<0: face=[face[0],face[2],face[1]]
            faces.append(face)
    pts=np.asarray(pts)
    if not len(faces): raise ValueError("panel triangulation is empty")
    # A corner triangle with all vertices on sewn boundaries would coincide
    # with its opposite panel after welding. Add one interior vertex.
    sewn=np.min(np.stack([segment_distance(pts,a,b) for a,b in seam_segments]),axis=0)<1e-8 if seam_segments else np.zeros(len(pts),dtype=bool)
    expanded=[]
    for face in sorted(faces):
        if sewn[face].all():
            center=pts[face].mean(axis=0);idx=len(pts);pts=np.vstack([pts,center])
            expanded.extend([[face[0],face[1],idx],[face[1],face[2],idx],[face[2],face[0],idx]])
        else: expanded.append(face)
    return pts,np.asarray(expanded,dtype=np.int32)

def build_mesh(pattern,step,depth):
    seam_segments={p["name"]:[] for p in pattern["panels"]}
    panels={p["name"]:p for p in pattern["panels"]}
    for seam in pattern["seams"]:
        for ref in seam:
            p=panels[ref["panel"]]["vertices_m"];i=ref["edge"]
            seam_segments[ref["panel"]].append((p[i],p[(i+1)%len(p)]))
    vertices=[]; faces=[]; rest=[]; face_panels=[]; welded={}; mapping=[]; seam_edges=set()
    for panel_index,panel in enumerate(pattern["panels"]):
        seg=seam_segments[panel["name"]]
        pts,local_faces=triangulate_panel(panel["vertices_m"],step,seg)
        dist=np.min(np.stack([segment_distance(pts,a,b) for a,b in seg]),axis=0) if seg else np.full(len(pts),np.inf)
        local_map=[]
        for i,(x,z) in enumerate(pts):
            sewn=dist[i]<1e-8
            key=point_key((x,z))
            if sewn and key in welded: idx=welded[key]
            else:
                y=panel["side"]*depth*(1-np.exp(-dist[i]/(step*1.5))) if seg else 0.0
                idx=len(vertices);vertices.append([float(x),float(y),float(z)])
                if sewn:welded[key]=idx
            local_map.append(idx)
            mapping.append({"panel":panel["name"],"local_vertex":i,"vertex_id":idx,"rest_uv_m":[float(x),float(z)],"sewn":bool(sewn)})
        local_map=np.asarray(local_map)
        if panel["side"]>0: local_faces=local_faces[:,[0,2,1]]
        for f in local_faces:
            faces.append(local_map[f]); rest.append(np.column_stack([pts[f],np.zeros(3)]));face_panels.append(panel_index)
        for a,b in seg:
            ids=np.flatnonzero(segment_distance(pts,np.asarray(a),np.asarray(b))<1e-8)
            direction=np.asarray(b)-np.asarray(a)
            ids=ids[np.argsort((pts[ids]-a)@direction)]
            for i,j in zip(ids[:-1],ids[1:]):seam_edges.add(tuple(sorted([int(local_map[i]),int(local_map[j])])))
    vertices=np.asarray(vertices,dtype=float);faces=np.asarray(faces,dtype=np.int32)
    rest=np.asarray(rest,dtype=float)
    metrics=mesh_metrics(vertices,faces)
    if metrics["degenerate_triangles"] or metrics["nonmanifold_edges"] or metrics["duplicate_faces"]:
        raise ValueError("invalid sewn mesh: "+str(metrics))
    metrics["initial_self_intersection_pairs"]=self_intersections(vertices,faces)
    inv,_=rest_matrices(rest);metrics.update(strain_metrics(vertices,faces,inv))
    if metrics["initial_self_intersection_pairs"]:raise ValueError("intersecting initial garment")
    from .quality import edge_table
    edges,counts=edge_table(faces);boundary=edges[counts==1]
    graph={}
    for i,j in boundary:
        graph.setdefault(int(i),set()).add(int(j));graph.setdefault(int(j),set()).add(int(i))
    if any(len(v)!=2 for v in graph.values()):raise ValueError("boundary is not a collection of loops")
    unseen=set(graph);loops=0
    while unseen:
        pending=[unseen.pop()];loops+=1
        while pending:
            for v in graph[pending.pop()]:
                if v in unseen:unseen.remove(v);pending.append(v)
    expected={"tshirt":4,"trousers":3,"open_coat":3,"towel":1,"napkin":1,"long_sleeve":4,"shorts":3,"skirt":2}[pattern["category"]]
    if loops!=expected:raise ValueError(f"garment openings: expected {expected}, got {loops}")
    metrics["boundary_loops"]=loops
    return vertices,faces,rest,np.asarray(face_panels),np.asarray(sorted(seam_edges),dtype=np.int32).reshape(-1,2),mapping,metrics

def save_preview(path,vertices,faces,title,face_panels=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    fig=plt.figure(figsize=(10,7),facecolor="#f1f5f9")
    ax=fig.add_subplot(111,projection="3d",facecolor="#f1f5f9")
    palette=np.array([[.18,.45,.68,1],[.26,.62,.76,1],[.87,.49,.30,1]])
    colors=palette[np.zeros(len(faces),dtype=int) if face_panels is None else np.asarray(face_panels)%3]
    ax.add_collection3d(Poly3DCollection(vertices[faces],facecolors=colors,edgecolors=(.10,.19,.28,.18),linewidths=.18))
    mid=(vertices.min(axis=0)+vertices.max(axis=0))/2
    extent=max(np.ptp(vertices,axis=0).max()*.57,.1)
    for setter,c in zip([ax.set_xlim,ax.set_ylim,ax.set_zlim],mid):setter(c-extent,c+extent)
    ax.view_init(elev=10,azim=-67);ax.set_box_aspect((1,1,1))
    ax.set_xlabel("x (m)");ax.set_ylabel("y (m)");ax.set_zlabel("z (m)")
    ax.set_title(title,fontsize=14,pad=18)
    fig.tight_layout();fig.savefig(path,dpi=140);plt.close(fig)

def save_glb(path,vertices,faces,face_panels=None):
    import trimesh
    mesh=trimesh.Trimesh(vertices=vertices,faces=faces,process=False)
    palette=np.array([[46,115,173,255],[66,158,194,255],[222,125,77,255]],dtype=np.uint8)
    mesh.visual.face_colors=palette[np.zeros(len(faces),dtype=int) if face_panels is None else np.asarray(face_panels)%3]
    # GLTF is Y-up; canonical scientific data remains Z-up.
    transform=np.array([[1,0,0,0],[0,0,1,0],[0,-1,0,0],[0,0,0,1]],dtype=float)
    mesh.apply_transform(transform)
    mesh.export(str(path))

def save_viewer(path,positions,faces,title,fps=30):
    data=json.dumps({"positions":np.asarray(positions).round(6).tolist(),"faces":np.asarray(faces).tolist(),"fps":fps},separators=(",",":"))
    page="""<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>TITLE</title>
<style>body{margin:0;background:#0c1625;color:#dfeaf6;font:15px system-ui}header{padding:22px 5%;display:flex;gap:20px;align-items:center}h1{font-size:20px;margin:0}button{background:#52b9b1;border:0;border-radius:8px;padding:10px 20px;cursor:pointer}canvas{width:100%;height:72vh;display:block}footer{padding:18px 5%;display:flex;gap:20px;align-items:center}input{flex:1}small{color:#9baebe}</style>
<header><h1>TITLE</h1><small>拖动旋转 · 滚轮缩放 · 真实网格轨迹</small></header><canvas></canvas><footer><button id="play">播放</button><input id="frame" type="range" value="0"><span id="time"></span></footer>
<script>const D=DATA,c=document.querySelector('canvas'),ctx=c.getContext('2d'),slider=document.querySelector('#frame');slider.max=D.positions.length-1;let yaw=.35,pitch=.12,zoom=1,playing=false,last=0;
const flat=D.positions.flat(),mins=[0,1,2].map(i=>flat.reduce((m,v)=>Math.min(m,v[i]),Infinity)),maxs=[0,1,2].map(i=>flat.reduce((m,v)=>Math.max(m,v[i]),-Infinity)),mid=mins.map((v,i)=>(v+maxs[i])/2),size=Math.max(...maxs.map((v,i)=>v-mins[i]));
function draw(){c.width=c.clientWidth*devicePixelRatio;c.height=c.clientHeight*devicePixelRatio;const W=c.width,H=c.height,scale=Math.min(W,H)*.72/size*zoom,p=D.positions[+slider.value];ctx.clearRect(0,0,W,H);const verts=p.map(v=>{let x=v[0]-mid[0],y=v[1]-mid[1],z=v[2]-mid[2],u=x*Math.cos(yaw)+y*Math.sin(yaw),depth=-x*Math.sin(yaw)+y*Math.cos(yaw);return [W/2+u*scale,H/2-(z*Math.cos(pitch)-depth*Math.sin(pitch))*scale,depth*Math.cos(pitch)+z*Math.sin(pitch)]});const fs=D.faces.map(f=>({f,z:f.reduce((s,i)=>s+verts[i][2],0)/3})).sort((a,b)=>b.z-a.z);for(const {f} of fs){ctx.beginPath();f.forEach((i,j)=>j?ctx.lineTo(...verts[i].slice(0,2)):ctx.moveTo(...verts[i].slice(0,2)));ctx.closePath();let a=p[f[0]],b=p[f[1]],v=p[f[2]],e=b.map((x,i)=>x-a[i]),g=v.map((x,i)=>x-a[i]),n=[e[1]*g[2]-e[2]*g[1],e[2]*g[0]-e[0]*g[2],e[0]*g[1]-e[1]*g[0]],len=Math.hypot(...n);const light=.5+.5*Math.abs((n[0]*.3-n[1]*.8+n[2]*.5)/len);ctx.fillStyle='hsl(192 48% '+(30+light*30)+'%)';ctx.fill();ctx.strokeStyle='#10283744';ctx.lineWidth=.35*devicePixelRatio;ctx.stroke()}document.querySelector('#time').textContent=(+slider.value/D.fps).toFixed(2)+' s / '+((D.positions.length-1)/D.fps).toFixed(2)+' s'}
c.onpointerdown=e=>{c.setPointerCapture(e.pointerId);c.onpointermove=x=>{yaw+=x.movementX*.01;pitch+=x.movementY*.01;draw()}};c.onpointerup=()=>c.onpointermove=null;c.onwheel=e=>{e.preventDefault();zoom*=Math.exp(-e.deltaY*.001);draw()};slider.oninput=draw;document.querySelector('#play').onclick=()=>{playing=!playing;document.querySelector('#play').textContent=playing?'暂停':'播放'};function tick(t){if(playing&&t-last>1000/D.fps){slider.value=(+slider.value+1)%D.positions.length;last=t;draw()}requestAnimationFrame(tick)}window.onresize=draw;draw();requestAnimationFrame(tick);</script></html>"""
    Path(path).write_text(page.replace("TITLE",html.escape(title)).replace("DATA",data),encoding="utf-8")
