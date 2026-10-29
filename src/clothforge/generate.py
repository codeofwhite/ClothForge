"""Parametric single-layer clothes and flat textiles using GarmentCode."""
from pathlib import Path
import json
import subprocess
import numpy as np
from .schema import CONTRACT,canonical_hash,digest,write_json,read_json,validate_manifest,validate_material
from .mesh import point_key,build_mesh,save_preview,save_glb,save_viewer

def clip_half(polygon,bound,keep_left):
    result=[]
    for a,b in zip(polygon,polygon[1:]+polygon[:1]):
        inside_a=(a[0]<=bound) if keep_left else (a[0]>=bound)
        inside_b=(b[0]<=bound) if keep_left else (b[0]>=bound)
        if inside_a:result.append(a)
        if inside_a!=inside_b:
            t=(bound-a[0])/(b[0]-a[0]);result.append([bound,a[1]+t*(b[1]-a[1])])
    return result

def make_pattern(category,params):
    import pygarment as pyg
    if category in ["towel", "napkin"]:
        w,h=params["width_m"],params["length_m"]
        poly=[[-w/2,0],[w/2,0],[w/2,h],[-w/2,h]]
        openings=set(range(4))
    elif category=="skirt":
        w,h,b=params["waist_width_m"],params["length_m"],params["hem_width_m"]
        poly=[[-b/2,0],[b/2,0],[w/2,h],[-w/2,h]]
        openings={0,2}
        if params.get("skirt_style"):
            from assets.garment_programs.skirt_paneled import SkirtPanel
            panel=SkirtPanel("clothforge_skirt",waist_length=w*100,length=h*100,
                             ruffles=1,flare=params["hem_flare_m"]*100)
            points=np.asarray([edge.start for edge in panel.edges],dtype=float)/100
            points-=np.array([(points[:,0].min()+points[:,0].max())/2,points[:,1].min()])
            poly=points.tolist()
            openings={1,3}
    elif category in ["tshirt","long_sleeve","open_coat"]:
        w,h,s,n,a=params["width_m"],params["length_m"],params["sleeve_m"],params["neck_width_m"],params["arm_width_m"]
        neck_depth=params["neck_depth_m"]
        poly=[[-w/2,0],[w/2,0],[w/2,h-a],[w/2+s,h-a-.04],[w/2+s+.02,h-.04],[w/2,h],[n/2,h],[n/2,h-neck_depth],[0,h-neck_depth-.015],[-n/2,h-neck_depth],[-n/2,h],[-w/2,h],[-w/2-s-.02,h-.04],[-w/2-s,h-a-.04],[-w/2,h-a]]
        openings={0,3,6,7,8,9,12}
        if category=="tshirt" and params.get("neck_style"):
            from assets.garment_programs.collars import CircleNeckHalf,VNeckHalf
            factory={"crew":CircleNeckHalf,"vneck":VNeckHalf}[params["neck_style"]]
            edges=factory(neck_depth*100,n*100)
            linear=[]
            for edge in edges:
                linear.extend(edge.linearize(n_verts_inside=9 if params["neck_style"]=="crew" else 0))
            points=[edge.start for edge in linear]+[linear[-1].end]
            half=[[n/2-float(x)/100,h+float(y)/100] for x,y in points]
            neck=half+[[-x,z] for x,z in half[-2::-1]]
            poly=poly[:6]+neck+poly[11:]
            openings={0,3,7+len(neck),*range(6,6+len(neck)-1)}
    else:
        w,h,hip,rise,gap=params["waist_width_m"],params["length_m"],params["hip_width_m"],params["rise_m"],params["leg_gap_m"]
        hem=gap/2+params["hem_width_m"] if "hem_width_m" in params else hip*.425
        poly=[[-w/2,h],[w/2,h],[hip/2,h-rise*.8],[hem,0],[gap/2,0],[0,h-rise],[-gap/2,0],[-hem,0],[-hip/2,h-rise*.8]]
        openings={0,3,6}
    panels=[{"name":"back","side":1,"vertices_m":poly}]
    if category=="open_coat":
        panels += [{"name":"front_left","side":-1,"vertices_m":clip_half(poly,-.012,True)},
                   {"name":"front_right","side":-1,"vertices_m":clip_half(poly,.012,False)}]
    elif category not in {"towel", "napkin"}:panels.append({"name":"front","side":-1,"vertices_m":poly})
    edge_key=lambda a,b:tuple(sorted([point_key(a),point_key(b)]))
    sewing_keys={edge_key(poly[i],poly[(i+1)%len(poly)]) for i in range(len(poly)) if i not in openings}
    refs={}
    for panel in panels:
        p=panel["vertices_m"]
        for i in range(len(p)):
            key=edge_key(p[i],p[(i+1)%len(p)])
            if key in sewing_keys:refs.setdefault(key,[]).append({"panel":panel["name"],"edge":i})
    if any(len(v)!=2 for v in refs.values()):raise ValueError("sewing edges are not paired")
    seams=[refs[k] for k in sorted(refs)]
    # Use upstream Panel/Edge/Interface/Stitches to emit its native sewing schema.
    component=pyg.Component("clothforge_"+category);objects={}
    for item in panels:
        points=np.asarray(item["vertices_m"])*100
        origin=points[0].copy();points-=origin
        panel=pyg.Panel(item["name"])
        panel.edges=pyg.EdgeSeqFactory.from_verts(*points.tolist(),loop=True)
        panel.translation=np.array([origin[0],origin[1],item["side"]*1.2])
        objects[item["name"]]=panel;component.subs.append(panel)
    for a,b in seams:
        component.stitching_rules.append((pyg.Interface(objects[a["panel"]],objects[a["panel"]].edges[a["edge"]]),
                                          pyg.Interface(objects[b["panel"]],objects[b["panel"]].edges[b["edge"]])))
    raw=component.assembly()
    return {"contract":"clothforge_pattern_v1","category":category,"units":"m","panels":panels,"seams":seams,
            "garmentcode":{"pattern":raw.pattern,"properties":{"units_in_meter":100}},
            "construction":"flat-panel parametric sewing; no seam allowances, lining, fasteners or body fitting",
            "canonical_axes":"right-handed Z-up; native GarmentCode pattern uses centimeters and Y-up",
            "initialization":"small geometric separation away from sewn edges; per-triangle rest shape stays planar"}

def sample_parameters(category,rng):
    if category in {"towel", "napkin"}:
        return {"width_m":float(rng.uniform(.35,.55) if category=="towel" else rng.uniform(.25,.40)),
                "length_m":float(rng.uniform(.65,.95) if category=="towel" else rng.uniform(.25,.40))}
    if category=="skirt":
        return {"waist_width_m":float(rng.uniform(.32,.44)),"length_m":float(rng.uniform(.42,.65)),
                "hem_width_m":float(rng.uniform(.58,.82))}
    if category in {"trousers", "shorts"}:
        return {"waist_width_m":float(rng.uniform(.32,.48)),"length_m":float(rng.uniform(.80,1.06) if category=="trousers" else rng.uniform(.40,.52)),
                "hip_width_m":float(rng.uniform(.46,.58)),"rise_m":float(rng.uniform(.25,.34)),"leg_gap_m":float(rng.uniform(.07,.14))}
    coat=category=="open_coat"
    return {"width_m":float(rng.uniform(.50,.64) if coat else rng.uniform(.42,.58)),
            "length_m":float(rng.uniform(.76,.96) if coat else rng.uniform(.55,.74)),
            "sleeve_m":float(rng.uniform(.35,.48) if coat or category=="long_sleeve" else rng.uniform(.14,.24)),
            "neck_width_m":float(rng.uniform(.16,.21)),"neck_depth_m":float(rng.uniform(.055,.095)),
            "arm_width_m":float(rng.uniform(.17,.23))}

def write_garment(category,params,name,seed,variant,dest,config,revision,style=None):
    import trimesh
    root=Path(__file__).resolve().parents[2];config_hash=canonical_hash(config)
    pattern=make_pattern(category,params)
    write_json(dest/"pattern.json",pattern)
    v,f,rest,panel_ids,seam_edges,mapping,metrics=build_mesh(pattern,config["mesh_edge_m"],config["initial_half_depth_m"])
    with (dest/"mesh.npz").open("xb") as stream:
        np.savez_compressed(stream,vertices=v,faces=f,rest_triangles=rest,face_panels=panel_ids,seam_edges=seam_edges)
    trimesh.Trimesh(vertices=v,faces=f,process=False).export(str(dest/"model.obj"))
    save_glb(dest/"model.glb",v,f,panel_ids)
    save_preview(dest/"preview.png",v,f,name+" | initial sewn mesh",panel_ids)
    save_viewer(dest/"preview.html",v[None],f,name)
    write_json(dest/"seams.json",{"seams":pattern["seams"],"vertex_mapping":mapping,"seam_edges":seam_edges.tolist()})
    manifest={"contract":CONTRACT,"id":name,"category":category,"variant":variant,"seed":seed,"units":"m",
              "parameters":params,"pattern_sha256":digest(dest/"pattern.json"),"mesh_sha256":digest(dest/"mesh.npz"),
              "upstream_revision":revision,"generator_config_sha256":config_hash,
              "generator_sources":{p.name:digest(p) for p in [root/"src/clothforge"/n for n in ["schema.py","quality.py","mesh.py","generate.py"]]},
              "geometry_metrics":metrics,"physical_validation":"not_run"}
    if style:manifest.update(style)
    write_json(dest/"manifest.json",validate_manifest(manifest))
    for material in config["materials"]:
        write_json(dest/("config-"+material["id"]+".json"),{"contract":"clothforge_run_config_v1","garment":name,"mesh_sha256":manifest["mesh_sha256"],"material":material,"simulation":config["simulation"],"gates":config["gates"],"config_sha256":config_hash})
    return manifest

def generate(config_path,output,limit=None):
    import trimesh
    config=read_json(config_path);root=Path(__file__).resolve().parents[2];output=Path(output)
    revision=subprocess.check_output(["git","-C",str(root/"third_party/GarmentCode"),"rev-parse","HEAD"],text=True).strip()
    for mat in config["materials"]:validate_material(mat)
    config_hash=canonical_hash(config)
    write_json(output/"generation_config.json",config)
    records=[];failures=[]
    for category_index,category in enumerate(config["categories"]):
        count=config["variants_per_category"] if limit is None else min(limit,config["variants_per_category"])
        for variant in range(count):
            seed=config["seed"]+category_index*1000+variant
            rng=np.random.default_rng(seed);params=sample_parameters(category,rng)
            name=f"{category}-{variant:03d}"
            dest=output/"garments"/name;dest.mkdir(parents=True,exist_ok=False)
            try:
                manifest=write_garment(category,params,name,seed,variant,dest,config,revision);records.append(manifest)
                with np.load(dest/"mesh.npz") as mesh:v,f=mesh["vertices"],mesh["faces"]
                print("GENERATED",name,len(v),len(f),flush=True)
            except Exception as error:
                failure={"id":name,"category":category,"variant":variant,"seed":seed,"error_type":type(error).__name__,"error":str(error)}
                write_json(dest/"generation_failure.json",failure);failures.append(failure)
                print("GENERATION_FAILED",name,str(error),flush=True)
    index={"contract":"clothforge_generation_index_v1","config_sha256":config_hash,"garments":records,"failures":failures,
           "geometry_count":len(records),"material_configuration_count":len(records)*len(config["materials"]),"upstream_revision":revision}
    write_json(output/"index.json",index)
    return index


def generate_one(request_path,output):
    request=read_json(request_path);root=Path(__file__).resolve().parents[2]
    dest=Path(output);dest.mkdir(parents=True,exist_ok=False)
    revision=subprocess.check_output(["git","-C",str(root/"third_party/GarmentCode"),"rev-parse","HEAD"],text=True).strip()
    return write_garment(request.get("category","trousers"),request["parameters"],request["id"],request["seed"],0,dest,request["config"],revision,
                         {"style_id":request["style_id"],"style_name":request["style_name"]})
