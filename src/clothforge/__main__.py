"""Explicit entry points; generation and simulation use isolated interpreters."""
import argparse
import json
from pathlib import Path

def main():
    parser=argparse.ArgumentParser(prog="clothforge")
    sub=parser.add_subparsers(dest="command",required=True)
    gen=sub.add_parser("generate");gen.add_argument("--config",required=True);gen.add_argument("--output",required=True);gen.add_argument("--limit",type=int)
    one=sub.add_parser("generate-one");one.add_argument("--request",required=True);one.add_argument("--output",required=True)
    gcd=sub.add_parser("convert-garmentcodedata");gcd.add_argument("--config",required=True);gcd.add_argument("--output")
    sim=sub.add_parser("simulate");sim.add_argument("--garment",required=True);sim.add_argument("--material",required=True)
    sim.add_argument("--scenario",choices=["hanging","drop"],required=True);sim.add_argument("--output",required=True)
    sim.add_argument("--run-config");sim.add_argument("--engine",choices=["newton_vbd","genesis_ipc","newton_xpbd","physx_surface","ppf"],default="newton_vbd")
    pilot=sub.add_parser("pilot");pilot.add_argument("--generation",required=True);pilot.add_argument("--config",required=True);pilot.add_argument("--output",required=True)
    repair=sub.add_parser("repair");repair.add_argument("--generation",required=True);repair.add_argument("--config",required=True);repair.add_argument("--output",required=True)
    repair.add_argument("--stage",choices=["diagnose","confirm","holdout"],default="diagnose");repair.add_argument("--profile")
    export=sub.add_parser("export-studio");export.add_argument("--generation",required=True);export.add_argument("--runs",required=True)
    export.add_argument("--registry-root",required=True);export.add_argument("--sources-file",required=True);export.add_argument("--dry-run",action="store_true")
    export.add_argument("--source-id",default="clothforge-pilot-v1")
    lehome=sub.add_parser("export-lehome")
    lehome.add_argument("--garment",required=True)
    lehome.add_argument("--material",nargs="+",default=["medium"])
    lehome.add_argument("--baseline",required=True)
    lehome.add_argument("--output",required=True)
    lehome.add_argument("--runtime-prefix",required=True,help="Output prefix relative to native LeHome cwd")
    lehome.add_argument("--source-label")
    lehome.add_argument("--scale",type=float,default=1.0)
    lehome.add_argument("--subdivisions",type=int,choices=range(4),default=0)
    args=parser.parse_args()
    if args.command=="export-lehome":
        # USD lives inside Isaac Sim's initialized runtime, not ordinary Python.
        from isaacsim import SimulationApp
        app=SimulationApp({"headless":True})
        try:
            from .lehome_export import export_lehome
            for material in args.material:
                result=export_lehome(args.garment,material,args.baseline,
                    Path(args.output)/material,Path(args.runtime_prefix)/material,args.source_label,args.subdivisions,args.scale)
                print(json.dumps({k:result[k] for k in ["source_id","material_id","vertices","triangles","expected_total_mass_kg"]}),flush=True)
        finally:
            app.close()
    elif args.command=="generate":
        from .generate import generate
        result=generate(args.config,args.output,args.limit)
        print(json.dumps({k:result[k] for k in ["geometry_count","material_configuration_count","failures"]},ensure_ascii=False))
        if result["failures"]:raise SystemExit(1)
    elif args.command=="generate-one":
        from .generate import generate_one
        print(json.dumps(generate_one(args.request,args.output),ensure_ascii=False))
    elif args.command=="convert-garmentcodedata":
        from .garmentcodedata import convert
        print(json.dumps(convert(args.config,args.output),ensure_ascii=False))
    elif args.command=="simulate":
        if args.engine=="genesis_ipc":
            from .genesis_worker import simulate
        elif args.engine=="physx_surface":
            from .physx_worker import simulate
        elif args.engine=="ppf":
            from .ppf_worker import simulate
        else:
            from .newton_worker import simulate
        options={"engine_id":args.engine} if args.engine.startswith("newton_") else {}
        simulate(args.garment,args.material,args.scenario,args.output,args.run_config,**options)
    elif args.command=="pilot":
        from .newton_worker import run_pilot
        result=run_pilot(args.generation,args.config,args.output)
        print(json.dumps({k:v for k,v in result.items() if k!="results"},ensure_ascii=False))
    elif args.command=="repair":
        from .repair import run_repair
        result=run_repair(args.generation,args.config,args.output,args.stage,args.profile)
        print(json.dumps({k:v for k,v in result.items() if k!="results"},ensure_ascii=False))
    else:
        from .studio_export import export_studio
        print(json.dumps(export_studio(args.generation,args.runs,args.registry_root,args.sources_file,args.dry_run,args.source_id),ensure_ascii=False))
if __name__=="__main__":main()
