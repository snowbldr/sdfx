
[![Go Report Card](https://goreportcard.com/badge/github.com/deadsy/sdfx)](https://goreportcard.com/report/github.com/deadsy/sdfx)
[![GoDoc](https://godoc.org/github.com/deadsy/sdfx?status.svg)](https://godoc.org/github.com/deadsy/sdfx/sdf)

# sdfx

A simple CAD package written in Go (https://golang.org/)

 * Objects are modelled with 2d and 3d signed distance functions (SDFs).
 * Objects are defined with Go code.
 * Objects are rendered to an STL/3MF file to be viewed and/or 3d printed.

## Performance Fork Notice

This is a **performance-focused fork** of [deadsy/sdfx](https://github.com/deadsy/sdfx).

 * **The public API is unchanged.** Existing code written against upstream compiles and runs against this fork without modification. Renders are bit-exact equivalent where the algorithm is unchanged (verified per-commit via a material STL-diff harness).
 * **One additive API:** a new parallel marching-cubes octree renderer, `render.NewMarchingCubesOctreeParallel`, which is a drop-in replacement for `render.NewMarchingCubesOctree`.
 * **Aggregate speedup across `examples/`: ~3.2× total wall-clock, ~3.8× geomean per-example** vs the upstream baseline (`deadsy@c4b81ba`). Details below.
 * **The plan is to upstream these changes** back to `deadsy/sdfx`. Everything here is structured as a series of focused, reviewable commits with no API breakage. Issues and PRs against the upstream repo are the intended destination.

## How To

 1. See the `examples/`.
 2. Write some Go code to define your own object.
 3. Build and run the Go code.
 4. Preview the output in a 3D file viewer (e.g. http://www.meshlab.net/).
 5. Print the STL/3MF file if you like it enough.

[SDF Viewer Go](https://github.com/Yeicor/sdf-viewer-go) or [SDFX-UI](https://github.com/Yeicor/sdfx-ui) allow faster development iteration, replacing steps 3 and 4 until the final build.

## Why?

 * SDFs make CSG easy.
 * As a language, Go > OpenSCAD.
 * SDFs can easily do filleting and chamfering (hard to do with OpenSCAD).
 * SDFs are hackable to try out oddball ideas.

## Quick Example

```go
package main

import (
    "log"

    "github.com/deadsy/sdfx/render"
    "github.com/deadsy/sdfx/sdf"
    v3 "github.com/deadsy/sdfx/vec/v3"
)

func main() {
    box, err := sdf.Box3D(v3.Vec{40, 40, 20}, 2)
    if err != nil { log.Fatal(err) }
    cyl, err := sdf.Cylinder3D(30, 10, 1)
    if err != nil { log.Fatal(err) }
    s := sdf.Difference3D(box, cyl)

    // Parallel octree renderer — ~3-10× faster on realistic models.
    render.ToSTL(s, "out.stl", render.NewMarchingCubesOctreeParallel(200))
}
```

## Framework Reference

All APIs live in `github.com/deadsy/sdfx/sdf` (geometry and operators) and `github.com/deadsy/sdfx/render` (rendering and file I/O). Constructors that can fail return `(SDFn, error)`; pure algebraic operators return the SDF directly.

### 2D primitives

| Constructor | Purpose |
| --- | --- |
| `Circle2D(radius)` | Circle. |
| `Box2D(size, round)` | Axis-aligned rectangle with optional corner rounding. |
| `Line2D(length, round)` | Rounded line segment. |
| `NewPolygon()` / `Polygon2D(verts)` / `FlatPolygon2D(verts)` | Polygonal shapes from chained paths or vertex lists. `FlatPolygon2D` uses a flat in-memory layout for tight hot-loops. |
| `Nagon(n, radius)` | Vertex set for a regular N-gon. |
| `Mesh2D(lines)` / `Mesh2DSlow(lines)` | SDF from a set of line segments. |
| `NewBezierSpline(pts)` / `NewBezier()` | Bézier curve construction for use with `Polygon2D`. |
| `CubicSpline2D(knots)` | Cubic spline SDF through knot points. |
| `ArcSpiral2D(...)` | Archimedean / logarithmic arc spiral. |
| `GearRack2D(p)` | Involute gear rack. |
| `FlatFlankCam2D(...)` / `ThreeArcCam2D(...)` | Flat-flank and three-arc cam profiles. |
| `NewText(s)` / `LoadFont(path)` / `Text2D(font, text, height)` | TrueType text as a 2D SDF. |

### 3D primitives

| Constructor | Purpose |
| --- | --- |
| `Box3D(size, round)` | Axis-aligned rounded box. |
| `Sphere3D(radius)` | Sphere. |
| `Cylinder3D(height, radius, round)` | Rounded cylinder. |
| `Capsule3D(height, radius)` | Capsule (rounded cylinder with spherical caps). |
| `Cone3D(height, r0, r1, round)` | Frustum / truncated cone. |
| `Gyroid3D(scale)` | Triply-periodic gyroid surface. |
| `Mesh3D(triangles)` / `Mesh3DSlow(triangles)` | SDF from triangle soup. |
| `ThreadLookup(name)` + `Screw3D(...)` | Named threads (`ThreadLookup`) feeding a helical screw body. |
| `AcmeThread`, `ISOThread`, `ANSIButtressThread`, `PlasticButtressThread` | Thread-profile builders. |
| `NewFlange1(...)` | Flange construction. |

### Extrusion, revolution, loft

| Operator | Purpose |
| --- | --- |
| `Extrude3D(sdf2, h)` | Straight extrusion. |
| `TwistExtrude3D(sdf2, h, twist)` | Linearly twisted extrusion. |
| `ScaleExtrude3D(sdf2, h, scale)` | Tapered extrusion. |
| `ScaleTwistExtrude3D(sdf2, h, twist, scale)` | Combined taper + twist. |
| `ExtrudeRounded3D(sdf2, h, round)` | Extrude with rounded top/bottom edges. |
| `Revolve3D(sdf2)` / `RevolveTheta3D(sdf2, theta)` | Full or partial revolve around the Z axis. |
| `Loft3D(s0, s1, h, round)` | Interpolating loft between two 2D profiles. |

### Boolean, offset, and shell

| Operator | Purpose |
| --- | --- |
| `Union2D(...)`, `Union3D(...)` | N-ary union (with rotational variants below). |
| `Difference2D(a, b)`, `Difference3D(a, b)` | Subtraction. |
| `Intersect2D(a, b)`, `Intersect3D(a, b)` | Intersection. |
| `Cut2D(sdf, p, v)`, `Cut3D(sdf, p, n)` | Half-space cut. |
| `Offset2D(sdf, k)`, `Offset3D(sdf, k)` | Offset the surface by a signed distance. |
| `Shell3D(sdf, thickness)` | Hollow-shell the volume. |
| `Elongate2D(sdf, h)`, `Elongate3D(sdf, h)` | Stretch an SDF by a vector without distorting the profile. |

Custom blend functions live in `sdf/utils.go`: `RoundMin`, `ChamferMin`, `ExpMin`, `PowMin`, `PolyMin`, `PolyMax`.

### Transformation and repetition

| Operator | Purpose |
| --- | --- |
| `Transform2D(sdf, M33)` / `Transform3D(sdf, M44)` | Apply a matrix (including translation/rotation/scale). |
| `ScaleUniform2D(sdf, k)` / `ScaleUniform3D(sdf, k)` | Uniform scale without per-axis shearing. |
| `Center2D(sdf)` / `CenterAndScale2D(sdf, k)` | Recenter (and optionally rescale) by bounding box. |
| `Array2D(sdf, n, step)` / `Array3D(sdf, n, step)` | Grid-array a shape. |
| `RotateUnion2D(sdf, n, step)` / `RotateUnion3D(sdf, n, step)` | Radial array (union of rotated copies). |
| `RotateCopy2D(sdf, n)` | Radial copy without union (fast rotational symmetry). |
| `LineOf2D(sdf, p0, p1, pat)` / `LineOf3D(sdf, p0, p1, pat)` | Place copies along a line with a pattern string. |
| `Multi2D(sdf, pts)` / `Multi3D(sdf, pts)` / `Orient3D(sdf, base, dirs)` | Place many copies at explicit positions / orientations. |
| `AddConnector(sdf, connectors...)` | Attach named connector frames for assembly tooling. |

Matrix helpers in `sdf/matrix.go`: `Identity2d`, `Identity3d`, `Translate2d`, `Translate3d`, `Scale2d`, `Scale3d`, `Rotate2d`, `Rotate3d`, `RotateX/Y/Z`, `MirrorX/Y/XY/XZ/YZ/XeqY`, `RotateToVector`.

### Caching and voxelization

| Helper | Purpose |
| --- | --- |
| `Cache2D(sdf)` | LRU-backed memoization wrapper for expensive 2D SDFs. |
| `NewVoxelSDF3(sdf, meshCells, progress)` | Precompute an SDF into a voxel grid for fast repeated evaluation. |

### Bounding-box and map utilities

`Box2` / `Box3` support `Extend`, `Include`, `Translate`, `Size`, `Center`, `ScaleAboutCenter`, `Enlarge`, `Square` / `Cube`, `Contains`, `Vertices`, `Snap`, `Equals`, `MinDist2`, `MinDist2GT`, `MinMaxDist2`, `Random`, `RandomSet`. `NewMap2` maps a 2D region to an integer grid (used internally by raster renderers).

### Renderers

| Constructor | Output | Notes |
| --- | --- | --- |
| `render.NewMarchingCubesOctreeParallel(cells)` | 3D triangle mesh | **New in this fork.** Drop-in replacement for the serial octree renderer; parallel across GOMAXPROCS. |
| `render.NewMarchingCubesOctree(cells)` | 3D triangle mesh | Adaptive octree; good speed/quality trade-off. |
| `render.NewMarchingCubesUniform(cells)` | 3D triangle mesh | Fixed-resolution marching cubes. |
| `render.NewMarchingSquaresQuadtree(cells)` | 2D line mesh | Adaptive quadtree. |
| `render.NewMarchingSquaresUniform(cells)` | 2D line mesh | Fixed-resolution marching squares. |
| `render.NewDualContouring2D(cells)` | 2D line mesh | Dual contouring with sharp-feature preservation. |
| `render.dc.NewDualContouringV1(...)` / `NewDualContouringV2(...)` / `NewDualContouringDefault(cells)` | 3D triangle mesh | Experimental 3D dual contouring. |

### Output formats

| Function | Purpose |
| --- | --- |
| `render.ToSTL(sdf, path, renderer)` | Write 3D mesh to binary STL. |
| `render.To3MF(sdf, path, renderer)` | Write 3D mesh to 3MF. |
| `render.ToDXF(sdf, path, renderer)` | Write 2D polylines to DXF. |
| `render.ToSVG(sdf, path, renderer)` | Write 2D polylines to SVG. |
| `render.ToTriangles(sdf, renderer)` | Return triangles in-memory. |
| `render.CollectTriangles(sdf, renderer)` | Same as above; returns a value slice. |
| `render.NewPNG(name, bbox, pixels)` | Rasterize an SDF2 to PNG (grayscale distance field). |
| `render.NewDXF(name)` / `render.SaveDXF(path, lines)` | Low-level DXF writer. |
| `render.NewSVG(name, style)` / `render.SaveSVG(path, style, lines)` | Low-level SVG writer. |
| `render.LoadSTL(path)` / `render.SaveSTL(path, tris)` | Round-trip STL for mesh-based workflows. |
| `render.CountBoundaryEdges`, `render.IsWatertight`, `render.MaxZ` | Mesh QA helpers. |

Setting the environment variable `SDFX_CPUPROF=/tmp/prof` when running any renderer will write pprof CPU profiles tagged by output filename — no source changes needed.

## Performance

Wall-clock render-time comparison across all 75 runnable examples in `examples/`, measured on a single machine against upstream `deadsy@c4b81ba` (the pre-perf baseline) with `tools/benchcmp/run.sh`:

```
total base (deadsy):  239.55 s
total head (fork):     75.17 s
total speedup:           3.19x
geomean per-example:     3.83x
```

Highlights (largest wins):

| Example | deadsy (s) | fork (s) | Speedup |
| --- | ---: | ---: | ---: |
| picorx | 25.68 | 0.77 | **33.35×** |
| axochord | 14.30 | 0.50 | **28.60×** |
| gridfinity | 14.71 | 0.54 | **27.24×** |
| servo | 3.51 | 0.22 | **15.95×** |
| cylinder_head | 3.55 | 0.28 | **12.68×** |
| draincover | 5.46 | 0.53 | **10.30×** |
| ringnut_tool | 2.41 | 0.24 | **10.04×** |
| msquare | 4.41 | 0.44 | **10.02×** |
| axoloti | 2.87 | 0.30 | **9.57×** |
| pico_cnc | 3.22 | 0.34 | **9.47×** |
| nutsandbolts | 10.07 | 2.00 | **5.04×** |

Where speedups are modest (or absent):

| Example | deadsy (s) | fork (s) | Speedup | Why |
| --- | ---: | ---: | ---: | --- |
| gyroid | 16.60 | 15.73 | 1.06× | Uniform implicit surface — surface touches almost every evaluated cell, so octree and bbox pruning cannot skip work. |
| hollowing_stl | 20.94 | 20.57 | 1.02× | STL-load-bound; renderer is not the hot path. |
| mesh_test, monkey_hat | ~6 / ~1 | ~6 / ~1 | ~1.0× | Mesh-SDF dominated; already linear in triangle count. |

### What changed

The gains come from a stack of small, individually verified changes rather than one big rewrite:

 * **Parallel marching-cubes octree renderer** (`render/march3p.go`) — fans octree subtrees across workers with a per-worker direct-mapped SDF cache (Fibonacci-hashed) and an atomic work counter for lock-free dispatch.
 * **Bounding-box pruning in Union/Difference** — `Box2.MinDist2GT` / `Box3.MinDist2GT` short-circuit children whose bbox cannot improve the current best distance.
 * **Hot-path inlining** — several SDF `Evaluate` methods were flattened or slimmed to fit the Go inliner's 80-cost budget (e.g. `CylinderSDF3`, `TransformSDF3`, `ExtrudeSDF3`).
 * **Allocation-free hot paths** — per-worker triangle buffers, pooled direct caches, consolidated Phase-3 allocations, pre-sliced scratch space, cube-by-value recursion.
 * **Flat-array `FlatPolygon2D` / `flatMeshSDF2`** — AoS layout with precomputed `1/length²`, measurably friendlier to the branch predictor and L1.
 * **Direct-byte STL triangle encoding** — bypasses the `encoding/binary` reflection path.
 * **Misc. numerical cleanups** — e.g. dropping redundant `math.Abs` on values already known to be non-negative; hoisting loop-invariant slice bounds.

All changes preserve bit-exact float semantics where the algorithm is unchanged — verified per commit via `tools/stldiff/run.sh`, which SHA-256-compares output STLs between BASE and HEAD for every example.

### Reproducing the benchmark

```
bash tools/benchcmp/run.sh c4b81ba HEAD 3
```

Arguments: `BASE_REF HEAD_REF N_REPEATS` (median-of-N). Environment: `SDFX_BENCHCMP_TIMEOUT` (per-run wall-clock cap, default 240 s), `SDFX_BENCHCMP_SKIP`, `SDFX_BENCHCMP_ONLY`.

## Development

 * [Roadmap](docs/ROADMAP.md)

## Gallery

![wheel](docs/gallery/wheel.png "Pottery Wheel Casting Pattern")
![core_box](docs/gallery/core_box.png "Pottery Wheel Core Box")
![cylinder_head](docs/gallery/head.png "Cylinder Head")
![msquare](docs/gallery/msquare.png "M-Square Casting Pattern")
![axoloti](docs/gallery/axoloti.png "Axoloti Mount Kit")
![text](docs/gallery/text.png "TrueType font rendering")
![gyroid](docs/gallery/gyroid.png "Gyroid Surface")
![icosahedron](docs/gallery/icosahedron.png "Icosahedron")
![cc16a](docs/gallery/cc16a.png "Reddit CAD Challenge 16A")
![cc16b](docs/gallery/cc16b_0.png "Reddit CAD Challenge 16B")
![cc18b](docs/gallery/cc18b.png "Reddit CAD Challenge 18B")
![cc18c](docs/gallery/cc18c.png "Reddit CAD Challenge 18C")
![gear](docs/gallery/gear.png "Involute Gear")
![camshaft](docs/gallery/camshaft.png "Wallaby Camshaft")
![geneva](docs/gallery/geneva1.png "Geneva Mechanism")
![nutsandbolts](docs/gallery/nutsandbolts.png "Nuts and Bolts")
![extrude1](docs/gallery/extrude1.png "Twisted Extrusions")
![extrude2](docs/gallery/extrude2.png "Scaled and Twisted Extrusions")
![bezier1](docs/gallery/bezier_bowl.png "Bowl made with Bezier Curves")
![bezier2](docs/gallery/bezier_shape.png "Extruded Bezier Curves")
![voronoi](docs/gallery/voronoi.png "2D Points Distance Field")
