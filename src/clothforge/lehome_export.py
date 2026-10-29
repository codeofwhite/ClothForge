"""Source-preserving ClothForge mesh to LeHome particle-cloth asset export.

Run in the existing LeHome Isaac Sim runtime. Native stiffness is intentionally
not inferred from another solver's parameters; only areal mass and friction map.
"""
from __future__ import annotations

import json
from pathlib import Path
import numpy as np
from .schema import digest, read_json, write_json

CATEGORIES = {'shorts': 'Pant_Short', 'trousers': 'Pant_Long',
              'tshirt': 'Top_Short', 'long_sleeve': 'Top_Long'}


def subdivide_midpoints(vertices, faces, levels):
    """Split triangles into four using shared indexed-edge midpoints, without smoothing."""
    history = {}
    original = vertices.copy()
    for level in range(1, levels + 1):
        count = len(faces)
        edges = np.concatenate((faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]))
        unique, inverse = np.unique(np.sort(edges, axis=1), axis=0, return_inverse=True)
        offset = len(vertices)
        midpoints = (vertices[unique[:, 0]] + vertices[unique[:, 1]]) / 2
        ab, bc, ca = inverse.reshape(3, count) + offset
        a, b, c = faces.T
        faces = np.concatenate((np.stack((a, ab, ca), axis=1),
                                np.stack((ab, b, bc), axis=1),
                                np.stack((ca, bc, c), axis=1),
                                np.stack((ab, bc, ca), axis=1)))
        vertices = np.concatenate((vertices, midpoints))
        history[f"level_{level}_parent_edges"] = unique
        history[f"level_{level}_vertex_offset"] = np.asarray(offset)
    np.testing.assert_array_equal(vertices[:len(original)], original)
    return vertices, faces, history


def export_lehome(garment, material, baseline, output, runtime_output, source_label=None, subdivisions=0, geometry_scale=1.0):
    from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf, Vt
    import yaml
    garment, baseline, output = Path(garment), Path(baseline), Path(output)
    runtime_output = Path(runtime_output)
    if not np.isfinite(geometry_scale) or geometry_scale <= 0:
        raise ValueError('geometry_scale must be positive and finite')
    if not 0 <= subdivisions <= 3:
        raise ValueError('subdivisions must be between 0 and 3')
    if runtime_output.is_absolute() or '..' in runtime_output.parts:
        raise ValueError('runtime_output must be relative to the native LeHome working directory')
    manifest = read_json(garment / 'manifest.json')
    if manifest['category'] not in CATEGORIES:
        raise ValueError('This LeHome challenge adapter supports shorts/trousers/tshirt/long_sleeve only')
    material_path = garment / f'config-{material}.json'
    recipe = read_json(material_path)
    if digest(garment / 'mesh.npz') != manifest['mesh_sha256'] or recipe['mesh_sha256'] != manifest['mesh_sha256']:
        raise ValueError('Source mesh fingerprint mismatch')
    source_files = ['mesh.npz', 'manifest.json', 'pattern.json', 'seams.json', material_path.name]
    before = {name: digest(garment / name) for name in source_files}
    with np.load(garment / 'mesh.npz', allow_pickle=False) as data:
        vertices, faces, rest = data['vertices'].copy(), data['faces'].copy(), data['rest_triangles'].copy()
    if not np.isfinite(vertices).all() or faces.ndim != 2 or faces.shape[1] != 3:
        raise ValueError('Expected finite vertices and triangular mesh')
    if faces.min() < 0 or faces.max() >= len(vertices):
        raise ValueError('Invalid triangle index')
    source_vertex_count, source_triangle_count = len(vertices), len(faces)
    source_bounds = np.stack((vertices.min(axis=0), vertices.max(axis=0)))
    source_triangle_points = vertices[faces]
    source_surface_area = np.linalg.norm(np.cross(source_triangle_points[:, 1] - source_triangle_points[:, 0],
                                                 source_triangle_points[:, 2] - source_triangle_points[:, 0]), axis=1).sum() / 2
    vertices, faces, refinement = subdivide_midpoints(vertices, faces, subdivisions)
    np.testing.assert_allclose(np.stack((vertices.min(axis=0), vertices.max(axis=0))), source_bounds, atol=1e-12)
    points = vertices[faces]
    refined_area = np.linalg.norm(np.cross(points[:, 1] - points[:, 0], points[:, 2] - points[:, 0]), axis=1).sum() / 2
    np.testing.assert_allclose(refined_area, source_surface_area, rtol=1e-12)
    # Proper rotation: (x,y,z) -> (x,z,-y), then center in the tabletop plane.
    transformed = vertices[:, [0, 2, 1]].copy()
    transformed[:, 2] *= -1
    center_xy = (transformed[:, :2].min(axis=0) + transformed[:, :2].max(axis=0)) / 2
    transformed[:, :2] -= center_xy
    transformed *= geometry_scale
    areas = np.linalg.norm(np.cross(rest[:, 1] - rest[:, 0], rest[:, 2] - rest[:, 0]), axis=1) / 2
    source_rest_area = float(areas.sum())
    area = source_rest_area * geometry_scale ** 2
    source_material = recipe['material']
    if source_material['mass_per_area_unit'] != 'kg/m2':
        raise ValueError('Explicit kg/m2 areal density required')
    total_mass = area * source_material['mass_per_area']
    if not total_mass > 0:
        raise ValueError('Positive total mass required')
    native = yaml.safe_load(baseline.read_text())
    native['objects']['garment_config']['particle_mass'] = total_mass / len(vertices)
    native['objects']['particle_material']['friction'] = source_material['friction']
    output.mkdir(parents=True, exist_ok=False)
    if subdivisions:
        np.savez_compressed(output / 'refinement.npz', **refinement)
    usd_path = output / 'garment.usda'
    stage = Usd.Stage.CreateNew(str(usd_path))
    root = UsdGeom.Xform.Define(stage, '/Garment')
    stage.SetDefaultPrim(root.GetPrim())
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    mesh = UsdGeom.Mesh.Define(stage, '/Garment/mesh')
    mesh.CreatePointsAttr(Vt.Vec3fArray(transformed.tolist()))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(faces)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(faces.astype(int).reshape(-1).tolist()))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateExtentAttr(Vt.Vec3fArray([transformed.min(axis=0).tolist(), transformed.max(axis=0).tolist()]))
    visual = UsdShade.Material.Define(stage, '/Garment/Looks/Fabric')
    shader = UsdShade.Shader.Define(stage, '/Garment/Looks/Fabric/Shader')
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.22, .40, .62))
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.8)
    shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(0.)
    visual.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(visual)
    stage.GetRootLayer().Save()
    # Verify USD serialization keeps vertex numbering and triangular connectivity.
    # Retain stage ownership while accessing USD attributes.
    check_stage = Usd.Stage.Open(str(usd_path))
    loaded = UsdGeom.Mesh(check_stage.GetPrimAtPath('/Garment/mesh'))
    np.testing.assert_array_equal(np.asarray(loaded.GetFaceVertexIndicesAttr().Get()), faces.reshape(-1))
    np.testing.assert_allclose(np.asarray(loaded.GetPointsAttr().Get()), transformed, atol=3e-8, rtol=1e-6)
    category = CATEGORIES[manifest['category']]
    name = category + '_ClothForge_' + manifest['id'].replace('-', '_')
    folder = output / 'Challenge_Garment' / 'Release' / category / name
    folder.mkdir(parents=True)
    config = {'id': 0, 'asset_path': str(runtime_output / 'garment.usda'),
              'visual_usd_paths': [], 'scale': [1., 1., 1.], 'check_point': [], 'success_distance': [],
              'initial_pos_range': [-.04, -.02, .67, -.04, -.02, .67],
              'initial_rot_range': [0, 0, 0, 0, 0, 0],
              'soft_reset_pos_range': [-.04, -.02, .67, -.04, -.02, .67],
              'soft_reset_rot_range': [0, 0, 0, 0, 0, 0],
              'evaluation_status': 'unconfigured', 'clothforge_source_mesh_sha256': manifest['mesh_sha256']}
    write_json(folder / (name + '.json'), config)
    (output / 'Challenge_Garment/Release/Release_test_list.txt').write_text(name + '\n')
    (output / 'particle_config.yaml').write_text(yaml.safe_dump(native, sort_keys=False))
    provenance = {'contract': 'clothforge_lehome_export_v1', 'source_garment': source_label or str(garment),
        'source_id': manifest['id'], 'category': manifest['category'], 'garment_name': name,
        'source_hashes': before, 'material_id': material, 'source_material': source_material,
        'baseline_particle_config_sha256': digest(baseline), 'exporter_sha256': digest(Path(__file__)),
        'vertices': len(vertices), 'triangles': len(faces), 'rest_area_m2': area,
        'source_rest_area_m2': source_rest_area, 'geometry_scale': geometry_scale,
        'scaled_flat_bounds_m': (transformed.max(axis=0)-transformed.min(axis=0)).tolist(),
        'expected_total_mass_kg': total_mass, 'uniform_particle_mass_kg': total_mass / len(vertices),
        'coordinate_transform': '(x,y,z)->(x,z,-y), centered in XY; meters, proper rotation',
        'subdivision_levels': subdivisions, 'subdivision_method': 'shared indexed-edge midpoint; no smoothing',
        'source_vertices': source_vertex_count, 'source_triangles': source_triangle_count,
        'original_vertex_prefix_preserved': True, 'welded_seam_connectivity_preserved': True,
        'surface_shape_and_area_preserved': geometry_scale == 1.0,
        'refinement_preserves_source_shape_and_area': True, 'uniform_proportions_preserved': True,
        'source_surface_area_m2': float(source_surface_area),
        'refined_surface_area_m2': float(refined_area) * geometry_scale ** 2,
        'refinement_provenance': 'refinement.npz' if subdivisions else None,
        'source_mesh_topology_preserved': subdivisions == 0, 'seams': 'existing welded vertices preserved; no new constraints',
        'native_particle_config': native, 'physics_fields_mapped': ['areal_mass_to_uniform_particle_mass', 'friction'],
        'stiffness_mapping': 'not attempted; native LeHome baseline values retained',
        'visual_material': {'diffuse_color': [.22,.40,.62], 'roughness': .8, 'same_for_all_physics_profiles': True},
        'calibration_status': 'synthetic_unmeasured', 'task_evaluation_available': False,
        'limitations': ['no physical equivalence between solvers claimed', 'no real-cloth calibration',
                        'uniform particle masses approximate areal distribution', 'no success criterion for imported topology'],
        'usd_sha256': digest(usd_path)}
    assert all(digest(garment / name) == value for name, value in before.items())
    write_json(output / 'provenance.json', provenance)
    return provenance
