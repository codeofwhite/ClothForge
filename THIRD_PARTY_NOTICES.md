# Third-party notices

ClothForge source code and the included synthetic demo trajectories are released under the root MIT license. Solver runtimes, datasets and third-party source code are not bundled and are not relicensed by ClothForge.

## GarmentCode

The parametric garment pipeline imports GarmentCode. The included demo uses ClothForge's simplified parametric T-shirt geometry, not a redistributed GarmentCodeData dataset sample. GarmentCode's notice is reproduced below because generation uses its APIs.

Upstream: https://github.com/maria-korosteleva/GarmentCode

Generation revision: `d449629979028123a5c4dc9e732a2ec19b7fce31`.

```text
MIT License

Copyright (c) 2024 Maria Korosteleva

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## Optional runtimes

- Newton: https://github.com/newton-physics/newton
- NVIDIA Warp: https://github.com/NVIDIA/warp
- Genesis: https://github.com/Genesis-Embodied-AI/Genesis
- libuipc: https://github.com/spiriMirror/libuipc
- Isaac Sim / PhysX: https://github.com/isaac-sim/IsaacSim
- PPF contact solver: https://github.com/st-tech/ppf-contact-solver

Each runtime and container remains subject to its own upstream license and installation terms. The PPF adapter references a historical local Docker image ID, not a pullable registry URL; obtain the official runtime separately. No solver binaries, Docker layers, dataset samples, or private Studio services are included.

Other optional dependencies include NumPy, SciPy, matplotlib, trimesh and CGAL Python bindings. Consult their distributions for notices and terms, especially CGAL before distributing a combined executable.
