"""PhysX surface solver in the existing isolated Isaac Sim runtime."""
from pathlib import Path
import os,sys,subprocess,time
import numpy as np
from .schema import read_json,write_json

def material_parameters(material):
    """Convert 2-D nominal membrane scales to PhysX 3-D material units."""
    h=.0006;mu=float(material['tri_ke']);lam=float(material['tri_ka'])
    nu=lam/(2*mu+lam)
    return {'thickness_m':h,'young_pa':2*mu*(1+nu)/h,'poisson':nu,
            'density_kg_m3':material['mass_per_area']/h,'bend_pa':material['edge_ke']/h**3}

def motion_verdict(result,diagnostics,scenario,tolerance):
    """Supplement the unchanged surface gates with an independent motion check."""
    check=diagnostics.get('freefall',{})
    error=check.get('maximum_com_error_m')
    result['motion_check']={'maximum_com_error_m':error,'tolerance_m':tolerance,
                            'scope':'first 0.2 seconds before ground contact' if scenario=='drop' else 'not applicable to pinned cloth'}
    if scenario=='drop' and (error is None or not np.isfinite(error) or error>tolerance or (check.get('time_end_s') or 0)<.19):
        result['failed_gates'].append('freefall_motion_mismatch')
        result['outcome']='failed'
    result['evaluator_version']='clothforge_sampled_surface_v2_motion'
    return result

def simulate(garment_dir,material_id,scenario,output,run_config=None):
    from .native_io import load,finish
    manifest,config,*_=load(garment_dir,material_id,scenario,output,run_config)
    runtime=os.environ.get('CLOTHFORGE_ISAAC_PYTHON')
    if not runtime:
        raise RuntimeError('Set CLOTHFORGE_ISAAC_PYTHON to your Isaac Sim python.sh launcher')
    subprocess.run([runtime,'-u','-B','-m','clothforge.physx_worker',str(output)],check=True)
    if (Path(output)/'native-error.json').exists():raise RuntimeError(read_json(Path(output)/'native-error.json')['error'])
    result=finish(output,manifest,config,material_id,scenario,'physx_surface')
    tolerance=config['simulation']['physx_parameters'].get('motion_tolerance_m')
    if tolerance is not None:
        before=result['outcome']
        result=motion_verdict(result,read_json(Path(output)/'diagnostics.json'),scenario,tolerance)
        revised=Path(output)/'validation-motion.json'
        write_json(revised,result)
        revised.replace(Path(output)/'validation.json')
        if before!=result['outcome']:
            from .mesh import save_preview
            with np.load(Path(output)/'trajectory.npz',allow_pickle=False) as trajectory, np.load(Path(output)/'native-input.npz',allow_pickle=False) as source:
                save_preview(Path(output)/'preview.png',trajectory['positions'][-1],trajectory['faces'],manifest['id']+' | '+scenario+' | failed',source['face_panels'])
        import json
        print('RESULT',json.dumps(result),flush=True)
    return result

def native(output):
    output=Path(output);config=read_json(output/'request.json')['config'];mat=config['material'];settings=config['simulation']
    with np.load(output/'native-input.npz',allow_pickle=False) as d:v,flat,f,rest,pins=[d[k].copy() for k in ['initial_positions','flat_positions','faces','rest_triangles','pins']]
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'hide_ui':True,'multi_gpu':False,'active_gpu':0,'physics_gpu':0})
    try:
        import omni.usd
        from isaacsim.core.api import SimulationContext
        from isaacsim.core.experimental.materials import SurfaceDeformableMaterial
        from isaacsim.core.experimental.prims import DeformablePrim
        from omni.physx.scripts import deformableUtils
        from pxr import UsdGeom,Vt,Sdf,PhysicsSchemaTools,UsdPhysics,PhysxSchema
        from .quality import rest_matrices
        stage=omni.usd.get_context().get_stage();UsdGeom.Xform.Define(stage,'/World')
        mesh=UsdGeom.Mesh.Define(stage,'/World/Cloth')
        mesh.GetPointsAttr().Set(Vt.Vec3fArray(v.tolist()));mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray([3]*len(f)))
        mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray(f.reshape(-1).tolist()));mesh.GetSubdivisionSchemeAttr().Set('none')
        if not deformableUtils.set_physics_surface_deformable_body(stage,'/World/Cloth'):raise ValueError('PhysX surface creation failed')
        prim=mesh.GetPrim();prim.ApplyAPI('PhysxSurfaceDeformableBodyAPI')
        # PhysX rejects exactly coincident reference vertices and currently
        # requires shared simulation/rest topology. Preserve the planar metric
        # to float32 precision while making opposite panels distinguishable.
        native_reference=flat.copy();native_reference[:,1]=v[:,1]*1e-4
        expected_lengths=np.linalg.norm(rest[:,:,None]-rest[:,None,:],axis=-1)
        q=native_reference.astype(np.float32)[f]
        metric_error=float(np.max(abs(np.linalg.norm(q[:,:,None]-q[:,None,:],axis=-1)-expected_lengths)))
        if metric_error>1e-6:raise ValueError('PhysX reference representation exceeds metric tolerance')
        prim.GetAttribute('omniphysics:restShapePoints').Set(Vt.Vec3fArray(native_reference.tolist()))
        prim.GetAttribute('omniphysics:restTriVtxIndices').Set(Vt.Vec3iArray(f.tolist()))
        prim.GetAttribute('physxDeformableBody:selfCollision').Set(True)
        # Default centimetre-scale offsets would filter opposite garment panels.
        collision=PhysxSchema.PhysxCollisionAPI.Apply(prim)
        contact=settings['physx_parameters']
        collision.CreateRestOffsetAttr(contact['rest_offset_m']);collision.CreateContactOffsetAttr(contact['contact_offset_m'])
        prim.GetAttribute('physxDeformableBody:selfCollisionFilterDistance').Set(contact['self_collision_filter_distance_m'])
        prim.GetAttribute('physxDeformableBody:enableSpeculativeCCD').Set(True)
        prim.GetAttribute('physxDeformableBody:collisionPairUpdateFrequency').Set(contact.get('pair_updates',1))
        prim.GetAttribute('physxDeformableBody:collisionIterationMultiplier').Set(contact.get('collision_iterations',1))
        prim.GetAttribute('physxDeformableBody:solverPositionIterationCount').Set(settings['iterations'])
        prim.GetAttribute('physxDeformableBody:linearDamping').Set(0.)
        if contact.get('disable_sleep',False):
            for name in ['sleepThreshold','settlingThreshold','settlingDamping']:
                prim.GetAttribute('physxDeformableBody:'+name).Set(0.)
        cloth=DeformablePrim('/World/Cloth',deformable_type='surface')
        # PhysX consumes volumetric Young's modulus and density. Surface
        # stretch/shear override attributes are unsupported in this runtime.
        mapped=material_parameters(mat)
        thickness=mapped['thickness_m'];nu=mapped['poisson'];young=mapped['young_pa']
        material=SurfaceDeformableMaterial('/World/Material',static_frictions=[mat['friction']],dynamic_frictions=[mat['friction']],
            youngs_moduli=[young],poissons_ratios=[nu],densities=[mat['mass_per_area']/thickness])
        material.set_surface_stiffnesses(bend_stiffnesses=[mat['edge_ke']/thickness**3])
        material.set_surface_thicknesses([thickness]);cloth.apply_physics_materials(material)
        if contact.get('explicit_zero_material_damping',False):
            material_prim=stage.GetPrimAtPath('/World/Material')
            if not material_prim.ApplyAPI('PhysxSurfaceDeformableMaterialAPI'):
                raise ValueError('PhysX material damping API unavailable')
            material_prim.GetAttribute('physxDeformableMaterial:elasticityDamping').Set(0.)
            material_prim.GetAttribute('physxDeformableMaterial:bendDamping').Set(0.)
        _,reference_areas=rest_matrices(rest)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(reference_areas.sum()*mat['mass_per_area']))
        # Native vertex-to-world constraints, never per-step position overwrites.
        if len(pins):
            UsdGeom.Xform.Define(stage,'/World/Fixture')
            attachment=stage.DefinePrim('/World/Pins','OmniPhysicsVtxXformAttachment')
            attachment.GetRelationship('omniphysics:src0').SetTargets(['/World/Cloth'])
            attachment.GetRelationship('omniphysics:src1').SetTargets(['/World/Fixture'])
            attachment.GetAttribute('omniphysics:vtxIndicesSrc0').Set(Vt.IntArray(pins.tolist()))
            attachment.GetAttribute('omniphysics:localPositionsSrc1').Set(Vt.Vec3fArray(v[pins].tolist()))
        PhysicsSchemaTools.addGroundPlane(stage,'/World/Ground','Z',3.,(0,0,0),(.5,.5,.5))
        dt=1/settings['fps']/settings['substeps']
        context=SimulationContext(physics_dt=dt,rendering_dt=1/settings['fps'],stage_units_in_meters=1.,device='cuda:0')
        stage.GetRootLayer().Export(str(output/'authored-scene.usda'))
        attrs={str(a.GetName()):str(a.Get()) for a in prim.GetAttributes() if 'physx' in str(a.GetName())}
        write_json(output/'collision-properties.json',{'schemas':list(prim.GetAppliedSchemas()),'attributes':attrs})
        context.initialize_physics();context.play()
        cloth.set_nodal_positions(v[None,...].astype(np.float32));cloth.set_nodal_velocities(np.zeros((1,len(v),3),dtype=np.float32))
        nf,_,rf=cloth.get_element_indices();np0,_,rp=cloth.get_nodal_positions()
        if not np.array_equal(nf.numpy()[0],f):raise ValueError('PhysX changed topology')
        native_rest=rp.numpy()[0][rf.numpy()[0]]
        native_lengths=np.linalg.norm(native_rest[:,:,None]-native_rest[:,None,:],axis=-1)
        expected_lengths=np.linalg.norm(rest[:,:,None]-rest[:,None,:],axis=-1)
        if not np.allclose(native_lengths,expected_lengths,atol=1e-6):raise ValueError('PhysX rest metric mismatch')
        _,areas=rest_matrices(rest)
        mass=float(np.linalg.norm(np.cross(native_rest[:,1]-native_rest[:,0],native_rest[:,2]-native_rest[:,0]),axis=1).sum()*.5*mat['mass_per_area'])
        if not np.isclose(mass,areas.sum()*mat['mass_per_area'],rtol=1e-5):raise ValueError('PhysX native rest mass mismatch')
        initial_native_time=context.current_time
        ps=[np0.numpy()[0].copy()];vs=[np.zeros_like(v)];frames=round(settings['duration_s']*settings['fps']);started=time.monotonic()
        if not np.allclose(ps[0],v,atol=1e-6):raise ValueError('PhysX initial pose mismatch')
        for frame in range(frames):
            for _ in range(settings['substeps']):context.step(render=False)
            ps.append(cloth.get_nodal_positions()[0].numpy()[0].copy());vs.append(cloth.get_nodal_velocities().numpy()[0].copy())
            if not np.isfinite(ps[-1]).all():break
            if time.monotonic()-started>settings['max_gpu_wall_seconds']:raise TimeoutError('GPU time budget exhausted')
            if (frame+1)%15==0:print('FRAME',frame+1,'/',frames,flush=True)
        np.savez_compressed(output/'trajectory.npz',positions=np.array(ps),velocities=np.array(vs),faces=f,rest_triangles=rest,pins=pins,times=np.arange(len(ps))/settings['fps'])
        write_json(output/'initialization.json',{'pins':pins.tolist(),'unconstrained_total_mass_kg':mass,
            'mass_source':'explicit PhysicsMassAPI total mass equals reference area times areal density; volumetric material density corrected; runtime nodal masses not exposed',
            'material_rest_reference':'planar pattern metric within 1e-6 m edge tolerance; separated native reference vertices','reference_embedding_max_offset_m':float(np.max(abs(native_reference-flat))),'rest_metric_max_error_m':metric_error,'material_area_m2':float(areas.sum()),
            'solver':'PhysX Surface Deformable','native_versions':{'isaac_sim':'6.0.1'},'physical_gpu':os.environ.get('CUDA_VISIBLE_DEVICES'),
            'material_mapping':{'young_pa':young,'poisson':nu,'thickness_m':thickness,'density_kg_m3':mat['mass_per_area']/thickness,'bend_pa':mat['edge_ke']/thickness**3,'note':'nominal membrane Lame and bending scale mapping; uniform seam bending; damping unmapped; not real calibrated'},
            'fixture':'native OmniPhysicsVtxXformAttachment','gravity_m_s2':[0,0,-9.81],'ground_z_m':0,
            'native_elapsed_simulation_s':context.current_time-initial_native_time,'resolved_simulation':settings,'physical_wall_seconds':time.monotonic()-started})
        context.stop()
    except BaseException as error:
        import traceback
        traceback.print_exc()
        write_json(output/'native-error.json',{'error':str(error),'traceback':traceback.format_exc()})
        raise
    finally:app.close()

if __name__=='__main__':native(sys.argv[1])
