"""Regenerate upstream sewing topology; keep rendered PLY indices out of physics."""
from pathlib import Path
import json
import subprocess
import time

import numpy as np

from .schema import CONTRACT, canonical_hash, digest, read_json, write_json
from .quality import mesh_metrics, rest_matrices, strain_metrics, self_intersections, edge_table
from .mesh import save_glb, save_preview, save_viewer


def material_geometry(box):
    faces = np.asarray(box.faces, dtype=np.int32)
    texture_faces = np.asarray(box.faces_with_texture, dtype=np.int32)[:, [1, 3, 5]]
    local_uv = np.asarray(box.vertex_texture)[texture_faces]
    lengths = np.stack([np.linalg.norm(local_uv[:, i] - local_uv[:, j], axis=1)
                        for i, j in [(0, 1), (0, 2), (1, 2)]], axis=1)
    corrected = 0
    for n, face in enumerate(faces):
        for k, (i, j) in enumerate([(0, 1), (0, 2), (1, 2)]):
            key = tuple(sorted((int(face[i]), int(face[j]))))
            if key in box.orig_lens:
                lengths[n, k] = box.orig_lens[key]
                corrected += 1
    a, b, c = lengths.T
    x = (a*a + b*b - c*c) / (2*a)
    height2 = b*b - x*x
    if np.any(a <= 0) or np.any(height2 <= 0):
        raise ValueError('upstream rest lengths do not define positive triangles')
    rest = np.zeros((len(faces), 3, 3))
    rest[:, 1, 0] = a
    rest[:, 2, 0], rest[:, 2, 1] = x, np.sqrt(height2)
    offsets = np.cumsum([len(box.panels[n].panel_vertices) for n in box.panelNames])
    panels = np.searchsorted(offsets, texture_faces[:, 0], side='right').astype(np.int32)
    from pygarment.meshgen.render.texture_utils import unwarp_UV, normalize_UVs
    packed, _ = unwarp_UV(np.asarray(box.vertex_texture), texture_faces)
    uv, _ = normalize_UVs(packed)
    edges, _ = edge_table(faces)
    labels = box.stitch_segmentation
    seam_edges = [edge for edge in edges if isinstance(labels[edge[0]], list)
                  and isinstance(labels[edge[1]], list) and set(labels[edge[0]]) & set(labels[edge[1]])]
    return faces, rest/100, panels, np.asarray(seam_edges, dtype=np.int32).reshape(-1, 2), np.asarray(uv)[texture_faces], corrected


def draped_pose(box, folder, sid, candidates):
    import trimesh
    from scipy.spatial import cKDTree
    initial = trimesh.load(folder/(sid+'_boxmesh.ply'), process=False)
    draped = trimesh.load(folder/(sid+'_sim.ply'), process=False)
    if not np.array_equal(initial.faces, draped.faces):
        raise ValueError('source initial and draped face order differs')
    points = np.asarray(box.vertices)
    _, nearby = cKDTree(initial.triangles_center).query(points, k=candidates)
    triangles = initial.triangles[nearby].reshape(-1, 3, 3)
    closest = trimesh.triangles.closest_point(triangles, np.repeat(points, candidates, axis=0)).reshape(-1, candidates, 3)
    distance = np.linalg.norm(closest-points[:, None], axis=2)
    best = distance.argmin(axis=1)
    ids = nearby[np.arange(len(points)), best]
    bary = trimesh.triangles.points_to_barycentric(initial.triangles[ids], closest[np.arange(len(points)), best])
    vertices = (draped.triangles[ids]*bary[:, :, None]).sum(axis=1)
    # right-handed Y-up -> Z-up, centimeters -> meters
    vertices = vertices[:, [0, 2, 1]] * np.array([.01, -.01, .01])
    vertices[:, :2] -= (vertices[:, :2].min(axis=0)+vertices[:, :2].max(axis=0))/2
    vertices[:, 2] -= vertices[:, 2].min()
    return vertices, {'method': 'nearest_initial_surface_barycentric_transfer',
                      'maximum_distance_m': float(distance.min(axis=1).max()/100),
                      'p95_distance_m': float(np.quantile(distance.min(axis=1), .95)/100)}


def hanging_pins(vertices, faces, labels, category):
    if category in {'trousers', 'skirt'}:
        candidates = np.asarray(labels['lower_interface'], dtype=int)
        semantic = 'upstream_lower_interface'
    else:
        edges, count = edge_table(faces)
        boundary = np.unique(edges[count == 1])
        candidates = boundary[vertices[boundary, 2] >= vertices[boundary, 2].max()-.04]
        semantic = 'highest_open_boundary'
    left = candidates[np.argmin(vertices[candidates, 0])]
    right = candidates[np.argmax(vertices[candidates, 0])]
    if left == right:
        raise ValueError('hanging opening has no distinct left/right vertices')
    return {'hanging_pins': [int(left), int(right)], 'source': semantic}


def convert_one(sid, config, output):
    import trimesh
    from pygarment.meshgen.boxmeshgen import BoxMesh
    source = Path(config['source_root'])/'samples'/sid
    rows = {r['sample_id']: r for r in read_json(Path(config['source_root'])/'sample_summary.json')}
    source_row = rows[sid]
    category = ('long_sleeve' if 'long_sleeve' in source_row['styles'] else 'tshirt') if source_row['upstream_category']=='upper_garment' else config['category_mapping'][source_row['upstream_category']]
    folder = Path(output)/'garments'/sid
    folder.mkdir(parents=True, exist_ok=False)
    spec_path = source/(sid+'_specification.json')
    box = BoxMesh(spec_path, res=config['mesh_resolution_cm'])
    box.load()
    faces, rest, panels, seams, uv, corrected = material_geometry(box)
    vertices, mapping = draped_pose(box, source, sid, config['pose_mapping_candidates'])
    metrics = mesh_metrics(vertices, faces)
    if metrics['degenerate_triangles'] or metrics['nonmanifold_edges'] or metrics['duplicate_faces']:
        raise ValueError('invalid regenerated topology: '+str(metrics))
    inv, _ = rest_matrices(rest)
    metrics.update(strain_metrics(vertices, faces, inv))
    metrics['initial_self_intersection_pairs'] = self_intersections(vertices, faces)
    pins = hanging_pins(vertices, faces, box.vertex_labels, category) if category in config['runnable_categories'] else None
    root = Path(__file__).resolve().parents[2]
    material_config = read_json(root/config['material_config'])
    revision = subprocess.check_output(['git','-C',str(root/'third_party/GarmentCode'),'rev-parse','HEAD'],text=True).strip()
    np.savez_compressed(folder/'mesh.npz', vertices=vertices, faces=faces, rest_triangles=rest,
                        face_panels=panels, seam_edges=seams, face_uv=uv)
    (folder/'pattern.json').write_bytes(spec_path.read_bytes())
    write_json(folder/'seams.json', {'representation':'upstream_shared_vertex_topology',
                                   'seam_edges':seams.tolist(),'vertex_labels':{k:[int(i) for i in ids] for k,ids in box.vertex_labels.items()}})
    title = {'tshirt':'上衣','long_sleeve':'长袖上衣','trousers':'裤子','skirt':'半身裙','dress':'连衣裙','jumpsuit':'连体服'}[category]+' · GarmentCodeData '+sid
    save_glb(folder/'model.glb',vertices,faces,panels)
    trimesh.Trimesh(vertices=vertices,faces=faces,process=False).export(folder/'model.obj')
    save_preview(folder/'preview.png',vertices,faces,'GarmentCodeData '+sid,panels)
    save_viewer(folder/'preview.html',vertices[None],faces,title)
    manifest = {'contract':CONTRACT if category in config['runnable_categories'] else 'garmentcodedata_geometry_v1',
                'id':'gcd-v2-'+sid.lower(),'category':category,'name':title,'variant':0,'seed':1708445391,'units':'m',
                'parameters':{'length_m':float(np.ptp(vertices[:,2])),'width_m':float(np.ptp(vertices[:,0]))},
                'parameter_semantics':'draped_axis_aligned_extents',
                'pattern_sha256':digest(folder/'pattern.json'),'mesh_sha256':digest(folder/'mesh.npz'),
                'upstream_revision':revision,'generator_config_sha256':canonical_hash(config),
                'geometry_metrics':metrics,'physical_validation':'not_run','fixtures':pins,
                'provenance':{'dataset':'GarmentCodeData','version':'v2','batch':'garments_5000_0',
                  'body_group':'default_body','sample_id':sid,'original_category':source_row['upstream_category'],
                  'styles':source_row['styles'],'official_split':source_row['official_split'],
                  'license':'CC-BY-4.0','url':'https://doi.org/10.3929/ethz-b-000690432',
                  'construction':'regenerated_upstream_BoxMesh; source PLY remains visual reference',
                  'mesh_resolution_cm':config['mesh_resolution_cm'],'pose_transfer':mapping,
                  'rest_reference':'upstream_planar_panel_edges_with_regenerated_seam_corrections',
                  'corrected_face_edges':corrected,'converter_sha256':digest(Path(__file__)),
                  'source_hashes':{p.name:digest(p) for p in sorted(source.iterdir()) if p.is_file()}}}
    write_json(folder/'manifest.json',manifest)
    for material in material_config['materials']:
        write_json(folder/f"config-{material['id']}.json",{'contract':'clothforge_run_config_v1','garment':manifest['id'],
                   'mesh_sha256':manifest['mesh_sha256'],'material':material,'simulation':material_config['simulation'],
                   'gates':material_config['gates'],'config_sha256':canonical_hash(config)})
    return {'sample_id':sid,'status':'converted','category':category,'runnable':category in config['runnable_categories'],
            'manifest':str(folder/'manifest.json'),'metrics':metrics,'pose_transfer':mapping}


def convert(config_path, output=None):
    config = read_json(config_path)
    output = Path(output or config['output_root'])
    output.mkdir(parents=True, exist_ok=True)
    if (output/'conversion_report.json').exists():
        raise FileExistsError('conversion output already exists; choose a new --output directory')
    results = []
    for sid in config['samples']:
        start = time.monotonic()
        try:
            row = convert_one(sid,config,output)
        except BaseException as error:
            if isinstance(error,(KeyboardInterrupt,SystemExit)): raise
            # Upstream geometry errors inherit BaseException. Preserve each failure.
            row = {'sample_id':sid,'status':'failed','error_type':type(error).__name__,'error':str(error)}
        row['wall_seconds'] = round(time.monotonic()-start,3)
        results.append(row)
        (output/'conversion_report.json').write_text(json.dumps({'config':config,'results':results},ensure_ascii=False,indent=2,allow_nan=False))
        print(json.dumps(row,ensure_ascii=False),flush=True)
    return {'converted':sum(r['status']=='converted' for r in results),'failed':sum(r['status']=='failed' for r in results),
            'report':str(output/'conversion_report.json')}
