"""Append-only Studio registration through its existing asset contract."""
from pathlib import Path
from .schema import read_json,digest

SOURCE_ID="clothforge-pilot-v1"
def export_studio(generation,runs,registry_root,sources_file,dry_run=False,source_id=SOURCE_ID):
    from robot_data_contracts.asset import AssetArtifact,AssetDraft
    from robot_data_studio.services.asset_service import AssetRegistry,fingerprint
    generation=Path(generation).resolve();runs=Path(runs).resolve()
    source_root=generation.parent
    registry=AssetRegistry(root=Path(registry_root),sources_file=Path(sources_file))
    configured=Path(registry.sources()[source_id]["root"]).resolve()
    if configured!=source_root:raise ValueError("Studio source root differs from pilot")
    index=read_json(generation/"index.json");config=read_json(generation/"generation_config.json")
    drafts=[]
    def artifact(path,role):
        path=Path(path).resolve()
        if source_root not in path.parents:raise ValueError("artifact outside pilot")
        return AssetArtifact(source_id=source_id,path=path.relative_to(source_root).as_posix(),role=role)
    def add(asset_id,kind,name,data,artifacts,refs=None,tags=None,engines=None,description=""):
        drafts.append(AssetDraft(asset_id=asset_id,kind=kind,name=name,data=data,artifacts=artifacts,
                                references=refs or [],tags=["clothforge","pilot-v1" if source_id==SOURCE_ID else "library-v1",*(tags or [])],
                                engines=engines or [],description=description,
                                change_note="ClothForge pilot v1: source-preserving first registration"))
    def ref(asset_id,role):return {"asset_id":asset_id,"version":1,"role":role}
    for material in config["materials"]:
        if source_id!=SOURCE_ID:
            registry.get("MAT-clothforge-"+material["id"],1)
            continue
        add("MAT-clothforge-"+material["id"],"material",material["name"],{"parameters":material,"calibration_status":"synthetic_unmeasured"},
            [artifact(generation/"generation_config.json","parameters")],engines=["newton_vbd"],
            description="合成有效参数，尚未对应任何实测布料。")
    titles={"tshirt":"短袖上衣","trousers":"长裤","open_coat":"单层开襟外套","towel":"毛巾","napkin":"餐巾","long_sleeve":"长袖上衣","shorts":"短裤","skirt":"半身裙"}
    for garment in index["garments"]:
        folder=generation/"garments"/garment["id"]
        if digest(folder/"mesh.npz")!=garment["mesh_sha256"] or digest(folder/"pattern.json")!=garment["pattern_sha256"]:
            raise ValueError("source changed since generation: "+garment["id"])
        mid="MDL-clothforge-"+garment["id"]
        add(mid,"model",titles[garment["category"]]+" "+str(garment["variant"]+1).zfill(2),
            {"parameters":garment["parameters"],"category":garment["category"],"geometry_metrics":garment["geometry_metrics"],
             "physical_validation":"see_exact_configuration_validation","units":"m","seed":garment["seed"],
             "upstream_revision":garment["upstream_revision"]},
            [artifact(folder/name,role) for name,role in [
                ("preview.png","preview"),("model.glb","model"),("model.obj","model_obj"),("mesh.npz","simulation_mesh"),
                ("pattern.json","sewing_pattern"),("seams.json","seams"),("manifest.json","provenance"),("preview.html","interactive_preview")]],
            tags=[garment["category"]],engines=["newton_vbd"],
            description="参数化单层衣物；保留版片、缝合和静止形状。几何生成与动力学验证分开记录。")
        for material in config["materials"]:
            cid="CFG-clothforge-"+garment["id"]+"-"+material["id"]
            native=read_json(folder/("config-"+material["id"]+".json"))
            add(cid,"config",titles[garment["category"]]+" "+str(garment["variant"]+1).zfill(2)+" / "+material["name"],
                {"parameters":native,"generation_config_sha256":index["config_sha256"]},
                [artifact(folder/("config-"+material["id"]+".json"),"configuration")],
                refs=[ref(mid,"geometry"),ref("MAT-clothforge-"+material["id"],"material")],
                tags=[garment["category"]],engines=["newton_vbd"])
    for path in sorted((runs/"runs").glob("*/validation.json")):
        result=read_json(path);directory=path.parent
        run_id=result["garment_id"]+"-"+result["material_id"]+"-"+result["scenario"]
        cid="CFG-clothforge-"+result["garment_id"]+"-"+result["material_id"]
        artifacts=[artifact(path,"metrics")]
        for name,role in [("request.json","configuration"),("initialization.json","initialization"),
                          ("trajectory.npz","trajectory"),("replay.html","interactive_replay"),
                          ("preview.png","preview"),("final.glb","final_mesh")]:
            if (directory/name).is_file():artifacts.append(artifact(directory/name,role))
        log=directory.parent/(directory.name+".log")
        if log.is_file():artifacts.append(artifact(log,"log"))
        add("VAL-clothforge-"+run_id,"validation",run_id,result,artifacts,
            refs=[ref(cid,"tested_config")],tags=[result["scenario"]],engines=["newton_vbd"],
            description="仅验证合成布料的固定悬挂/落下短轨迹；通过不代表实物标定、连续无穿透或机器人任务成功。")
    # Preflight every file and collision before the first publication.
    prepared=[]
    ids={draft.asset_id for draft in drafts}
    if len(ids)!=len(drafts):raise ValueError("duplicate import ID")
    for draft in drafts:
        inspected=[registry.inspect_artifact(a) for a in draft.artifacts]
        draft=AssetDraft.model_validate(draft.model_copy(update={"artifacts":inspected}).model_dump())
        history=registry.versions(draft.asset_id)
        sha=fingerprint(draft.model_dump(mode="json"))
        if history and history[0].content_sha256!=sha:raise ValueError("existing v1 differs: "+draft.asset_id)
        for r in draft.references:
            if r.asset_id not in ids:registry.get(r.asset_id,r.version)
        prepared.append(draft)
    counts={kind:sum(d.kind==kind for d in prepared) for kind in ["material","model","config","validation"]}
    if dry_run:return {"dry_run":True,"count":len(prepared),"counts":counts}
    created=unchanged=0
    for draft in prepared:
        record,is_new=registry.publish(draft,expected_version=0,idempotent=True)
        if record.version!=1:raise ValueError("unexpected source version")
        created+=int(is_new);unchanged+=int(not is_new)
    integrity=[]
    for draft in prepared:
        result=registry.integrity(registry.get(draft.asset_id,1))
        if result["status"]!="current":integrity.append({"asset_id":draft.asset_id,**result})
    return {"dry_run":False,"count":len(prepared),"counts":counts,"created":created,"unchanged":unchanged,"integrity_errors":integrity}
