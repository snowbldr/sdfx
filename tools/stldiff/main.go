// stldiff: compare two binary STL files.
//
// Reports IDENTICAL / MINOR / MATERIAL plus the metrics that drove the
// classification. The metrics are layered so cheap checks rule out the
// common cases and only real diffs pay for the expensive ones:
//
//  1. Canonical hash    — exact byte equality of the sorted triangle list.
//     Hits IDENTICAL and exits.
//  2. Triangle count, bbox — original metrics, kept for context.
//  3. Boundary-edge count — edges shared by exactly one triangle. Holes
//     (e.g. the buttress-thread bug where the SDF
//     discontinuity at the SawTooth wrap left gaps in
//     the screw threads) leave boundary edges, so any
//     increase from 0 is a strong "geometry broke"
//     signal that count/bbox alone misses.
//  4. Surface area, volume — sum of triangle areas, signed-tetrahedron
//     volume sum. Holes punch the area up while
//     leaving volume nearly unchanged, exactly the
//     buttress signature; tessellation drift moves
//     neither.
//  5. Symmetric Hausdorff — p99 (and max) of point-to-nearest-triangle
//     distance, A→B and B→A. Catches the "subset of
//     triangles drifted" class — the kind of
//     regression where features deformed slightly
//     without changing topology, which the integral
//     metrics can't see.
//
// MATERIAL is reported when ANY metric crosses its threshold. MINOR is
// the residual: hash differs but everything looks within float-drift
// tolerance.
package main

import (
	"crypto/sha1"
	"encoding/binary"
	"fmt"
	"io"
	"math"
	"os"
	"sort"
)

type vec3 struct{ x, y, z float32 }

type tri struct{ a, b, c vec3 }

func loadSTL(path string) ([]tri, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()
	var header [80]byte
	if _, err := io.ReadFull(f, header[:]); err != nil {
		return nil, err
	}
	var count uint32
	if err := binary.Read(f, binary.LittleEndian, &count); err != nil {
		return nil, err
	}
	tris := make([]tri, count)
	for i := uint32(0); i < count; i++ {
		var buf [12]float32
		if err := binary.Read(f, binary.LittleEndian, &buf); err != nil {
			return nil, err
		}
		var attr uint16
		if err := binary.Read(f, binary.LittleEndian, &attr); err != nil {
			return nil, err
		}
		tris[i] = tri{
			a: vec3{buf[3], buf[4], buf[5]},
			b: vec3{buf[6], buf[7], buf[8]},
			c: vec3{buf[9], buf[10], buf[11]},
		}
	}
	return tris, nil
}

func lessVec(a, b vec3) bool {
	if a.x != b.x {
		return a.x < b.x
	}
	if a.y != b.y {
		return a.y < b.y
	}
	return a.z < b.z
}

func sortTriVerts(t tri) tri {
	vs := [3]vec3{t.a, t.b, t.c}
	sort.Slice(vs[:], func(i, j int) bool { return lessVec(vs[i], vs[j]) })
	return tri{vs[0], vs[1], vs[2]}
}

func canonicalHash(tris []tri) string {
	sorted := make([]tri, len(tris))
	for i, t := range tris {
		sorted[i] = sortTriVerts(t)
	}
	sort.Slice(sorted, func(i, j int) bool {
		a, b := sorted[i], sorted[j]
		if a.a != b.a {
			return lessVec(a.a, b.a)
		}
		if a.b != b.b {
			return lessVec(a.b, b.b)
		}
		return lessVec(a.c, b.c)
	})
	h := sha1.New()
	buf := make([]byte, 4)
	for _, t := range sorted {
		for _, v := range [3]vec3{t.a, t.b, t.c} {
			for _, f := range [3]float32{v.x, v.y, v.z} {
				binary.LittleEndian.PutUint32(buf, math.Float32bits(f))
				h.Write(buf)
			}
		}
	}
	return fmt.Sprintf("%x", h.Sum(nil))[:12]
}

//-----------------------------------------------------------------------------
// Boundary-edge count.
//
// An edge shared by exactly one triangle is a hole boundary. For a
// watertight closed mesh this should be 0. Marching cubes shares vertex
// coordinates exactly across adjacent triangles (it caches edge-vertex
// positions per cube edge), so float-bit equality is a reliable
// canonicalization key — no tolerance needed.

func canonEdge(a, b vec3) [2]vec3 {
	if lessVec(a, b) {
		return [2]vec3{a, b}
	}
	return [2]vec3{b, a}
}

func boundaryEdges(tris []tri) int {
	counts := make(map[[2]vec3]int, len(tris)*3)
	for _, t := range tris {
		counts[canonEdge(t.a, t.b)]++
		counts[canonEdge(t.b, t.c)]++
		counts[canonEdge(t.c, t.a)]++
	}
	bnd := 0
	for _, c := range counts {
		if c == 1 {
			bnd++
		}
	}
	return bnd
}

//-----------------------------------------------------------------------------
// Surface area + signed volume.
//
// Surface area: sum of |½·(b−a)×(c−a)| over triangles.
// Volume: sum of ⅙·(a · (b×c)) over triangles, then |·|. The signed
// tetrahedron-from-origin formula gives the exact enclosed volume for any
// closed orientable mesh regardless of where the origin sits, since the
// signs from outside-pointing tets cancel.

func surfaceArea(tris []tri) float64 {
	var total float64
	for _, t := range tris {
		ax, ay, az := float64(t.a.x), float64(t.a.y), float64(t.a.z)
		bx, by, bz := float64(t.b.x), float64(t.b.y), float64(t.b.z)
		cx, cy, cz := float64(t.c.x), float64(t.c.y), float64(t.c.z)
		ux, uy, uz := bx-ax, by-ay, bz-az
		vx, vy, vz := cx-ax, cy-ay, cz-az
		nx := uy*vz - uz*vy
		ny := uz*vx - ux*vz
		nz := ux*vy - uy*vx
		total += 0.5 * math.Sqrt(nx*nx+ny*ny+nz*nz)
	}
	return total
}

func volumeOf(tris []tri) float64 {
	var total float64
	for _, t := range tris {
		ax, ay, az := float64(t.a.x), float64(t.a.y), float64(t.a.z)
		bx, by, bz := float64(t.b.x), float64(t.b.y), float64(t.b.z)
		cx, cy, cz := float64(t.c.x), float64(t.c.y), float64(t.c.z)
		// a · (b × c)
		cross_x := by*cz - bz*cy
		cross_y := bz*cx - bx*cz
		cross_z := bx*cy - by*cx
		total += (ax*cross_x + ay*cross_y + az*cross_z) / 6.0
	}
	return math.Abs(total)
}

//-----------------------------------------------------------------------------
// Point-to-triangle distance (Ericson, "Real-Time Collision Detection").
//
// Classifies p into the seven Voronoi regions of the triangle (3 vertex,
// 3 edge, 1 face) and returns the distance to the closest point in that
// region. We keep the algebra in float64 since STL coordinates are float32
// and squared lengths compound the precision loss.

func pointTriDist2(px, py, pz float64, t tri) float64 {
	ax, ay, az := float64(t.a.x), float64(t.a.y), float64(t.a.z)
	bx, by, bz := float64(t.b.x), float64(t.b.y), float64(t.b.z)
	cx, cy, cz := float64(t.c.x), float64(t.c.y), float64(t.c.z)

	abx, aby, abz := bx-ax, by-ay, bz-az
	acx, acy, acz := cx-ax, cy-ay, cz-az
	apx, apy, apz := px-ax, py-ay, pz-az

	d1 := abx*apx + aby*apy + abz*apz
	d2 := acx*apx + acy*apy + acz*apz
	if d1 <= 0 && d2 <= 0 {
		return apx*apx + apy*apy + apz*apz
	}

	bpx, bpy, bpz := px-bx, py-by, pz-bz
	d3 := abx*bpx + aby*bpy + abz*bpz
	d4 := acx*bpx + acy*bpy + acz*bpz
	if d3 >= 0 && d4 <= d3 {
		return bpx*bpx + bpy*bpy + bpz*bpz
	}

	vc := d1*d4 - d3*d2
	if vc <= 0 && d1 >= 0 && d3 <= 0 {
		v := d1 / (d1 - d3)
		qx := ax + abx*v - px
		qy := ay + aby*v - py
		qz := az + abz*v - pz
		return qx*qx + qy*qy + qz*qz
	}

	cpx, cpy, cpz := px-cx, py-cy, pz-cz
	d5 := abx*cpx + aby*cpy + abz*cpz
	d6 := acx*cpx + acy*cpy + acz*cpz
	if d6 >= 0 && d5 <= d6 {
		return cpx*cpx + cpy*cpy + cpz*cpz
	}

	vb := d5*d2 - d1*d6
	if vb <= 0 && d2 >= 0 && d6 <= 0 {
		w := d2 / (d2 - d6)
		qx := ax + acx*w - px
		qy := ay + acy*w - py
		qz := az + acz*w - pz
		return qx*qx + qy*qy + qz*qz
	}

	va := d3*d6 - d5*d4
	if va <= 0 && (d4-d3) >= 0 && (d5-d6) >= 0 {
		w := (d4 - d3) / ((d4 - d3) + (d5 - d6))
		qx := bx + (cx-bx)*w - px
		qy := by + (cy-by)*w - py
		qz := bz + (cz-bz)*w - pz
		return qx*qx + qy*qy + qz*qz
	}

	// Inside the face — project p onto the plane.
	denom := 1.0 / (va + vb + vc)
	v := vb * denom
	w := vc * denom
	qx := ax + abx*v + acx*w - px
	qy := ay + aby*v + acy*w - py
	qz := az + abz*v + acz*w - pz
	return qx*qx + qy*qy + qz*qz
}

//-----------------------------------------------------------------------------
// Uniform grid over triangle bboxes.
//
// Cell size is chosen as ~3× the average edge length so each cell holds a
// handful of triangles. Each triangle is inserted into every cell its bbox
// touches. Queries spiral outward from the query point's cell until the
// shell distance exceeds the best distance found, at which point no closer
// triangle can exist in any unvisited cell.

type triGrid struct {
	cells    map[[3]int][]int
	cellSize float64
	origin   vec3
	tris     []tri
}

func newTriGrid(tris []tri) *triGrid {
	g := &triGrid{tris: tris}
	if len(tris) == 0 {
		return g
	}
	bmin := tris[0].a
	bmax := tris[0].a
	var totalEdge float64
	for _, t := range tris {
		for _, v := range [3]vec3{t.a, t.b, t.c} {
			if v.x < bmin.x {
				bmin.x = v.x
			}
			if v.y < bmin.y {
				bmin.y = v.y
			}
			if v.z < bmin.z {
				bmin.z = v.z
			}
			if v.x > bmax.x {
				bmax.x = v.x
			}
			if v.y > bmax.y {
				bmax.y = v.y
			}
			if v.z > bmax.z {
				bmax.z = v.z
			}
		}
		ab := vec3{t.b.x - t.a.x, t.b.y - t.a.y, t.b.z - t.a.z}
		bc := vec3{t.c.x - t.b.x, t.c.y - t.b.y, t.c.z - t.b.z}
		ca := vec3{t.a.x - t.c.x, t.a.y - t.c.y, t.a.z - t.c.z}
		totalEdge += math.Sqrt(float64(ab.x*ab.x + ab.y*ab.y + ab.z*ab.z))
		totalEdge += math.Sqrt(float64(bc.x*bc.x + bc.y*bc.y + bc.z*bc.z))
		totalEdge += math.Sqrt(float64(ca.x*ca.x + ca.y*ca.y + ca.z*ca.z))
	}
	avgEdge := totalEdge / float64(3*len(tris))
	g.cellSize = avgEdge * 3
	if g.cellSize <= 0 {
		// Degenerate input (all triangles zero-length): fall back to
		// the bbox-derived size.
		dx := float64(bmax.x - bmin.x)
		dy := float64(bmax.y - bmin.y)
		dz := float64(bmax.z - bmin.z)
		g.cellSize = math.Max(math.Max(dx, dy), dz) / 32
		if g.cellSize <= 0 {
			g.cellSize = 1
		}
	}
	g.origin = bmin
	g.cells = make(map[[3]int][]int)
	for i, t := range tris {
		tbmin := vec3{
			minF32(t.a.x, t.b.x, t.c.x),
			minF32(t.a.y, t.b.y, t.c.y),
			minF32(t.a.z, t.b.z, t.c.z),
		}
		tbmax := vec3{
			maxF32(t.a.x, t.b.x, t.c.x),
			maxF32(t.a.y, t.b.y, t.c.y),
			maxF32(t.a.z, t.b.z, t.c.z),
		}
		i0 := g.cellIdx(tbmin)
		i1 := g.cellIdx(tbmax)
		for x := i0[0]; x <= i1[0]; x++ {
			for y := i0[1]; y <= i1[1]; y++ {
				for z := i0[2]; z <= i1[2]; z++ {
					key := [3]int{x, y, z}
					g.cells[key] = append(g.cells[key], i)
				}
			}
		}
	}
	return g
}

func (g *triGrid) cellIdx(p vec3) [3]int {
	return [3]int{
		int(math.Floor(float64(p.x-g.origin.x) / g.cellSize)),
		int(math.Floor(float64(p.y-g.origin.y) / g.cellSize)),
		int(math.Floor(float64(p.z-g.origin.z) / g.cellSize)),
	}
}

// nearestDist returns the Euclidean distance from p to the closest
// triangle in the grid. seen is a scratch map reused across queries to
// avoid per-call allocation; pass an empty map and clear() between calls.
func (g *triGrid) nearestDist(p vec3, seen map[int]struct{}) float64 {
	if len(g.tris) == 0 {
		return math.Inf(1)
	}
	cp := g.cellIdx(p)
	bestSq := math.Inf(1)
	px, py, pz := float64(p.x), float64(p.y), float64(p.z)
	for r := 0; ; r++ {
		// Scan the cube shell of half-width r centered on cp.
		for dx := -r; dx <= r; dx++ {
			for dy := -r; dy <= r; dy++ {
				for dz := -r; dz <= r; dz++ {
					if absInt(dx) != r && absInt(dy) != r && absInt(dz) != r {
						// Interior of the cube — already covered at smaller r.
						continue
					}
					key := [3]int{cp[0] + dx, cp[1] + dy, cp[2] + dz}
					for _, ti := range g.cells[key] {
						if _, ok := seen[ti]; ok {
							continue
						}
						seen[ti] = struct{}{}
						d2 := pointTriDist2(px, py, pz, g.tris[ti])
						if d2 < bestSq {
							bestSq = d2
						}
					}
				}
			}
		}
		// After scanning shell r, any triangle we haven't seen lies in
		// some cell at radius ≥ r+1. The closest point in such a cell
		// is at distance ≥ r·cellSize from p (one cell away from the
		// already-scanned shell). When that lower bound exceeds the
		// best distance found, we can stop.
		if !math.IsInf(bestSq, 1) {
			lower := float64(r) * g.cellSize
			if lower*lower > bestSq {
				break
			}
		}
		// Hard stop in pathological cases (sparse grid). The diameter
		// of the world bbox in cells is an upper bound.
		if r > 4096 {
			break
		}
	}
	return math.Sqrt(bestSq)
}

//-----------------------------------------------------------------------------
// Hausdorff: vertex of A → nearest triangle in B (and vice versa). We
// deduplicate vertices first since STL stores each triangle's three
// vertices independently and sharing inflates the work by ~6×.

func uniqueVerts(tris []tri) []vec3 {
	set := make(map[vec3]struct{}, len(tris)*3)
	for _, t := range tris {
		set[t.a] = struct{}{}
		set[t.b] = struct{}{}
		set[t.c] = struct{}{}
	}
	out := make([]vec3, 0, len(set))
	for v := range set {
		out = append(out, v)
	}
	return out
}

type hausStats struct {
	max, p99, mean float64
}

func hausdorffOneWay(verts []vec3, target *triGrid) hausStats {
	if len(verts) == 0 || len(target.tris) == 0 {
		return hausStats{}
	}
	dists := make([]float64, len(verts))
	seen := make(map[int]struct{}, 64)
	var sum float64
	for i, v := range verts {
		clear(seen)
		d := target.nearestDist(v, seen)
		dists[i] = d
		sum += d
	}
	sort.Float64s(dists)
	max := dists[len(dists)-1]
	p99idx := int(math.Floor(0.99 * float64(len(dists)-1)))
	p99 := dists[p99idx]
	return hausStats{max: max, p99: p99, mean: sum / float64(len(dists))}
}

//-----------------------------------------------------------------------------

type metrics struct {
	count       int
	min, max    vec3
	hash        string
	bndEdges    int
	surfaceArea float64
	volume      float64
}

func computeMetrics(tris []tri) metrics {
	m := metrics{count: len(tris)}
	if len(tris) == 0 {
		return m
	}
	m.min = tris[0].a
	m.max = tris[0].a
	for _, t := range tris {
		for _, v := range [3]vec3{t.a, t.b, t.c} {
			if v.x < m.min.x {
				m.min.x = v.x
			}
			if v.y < m.min.y {
				m.min.y = v.y
			}
			if v.z < m.min.z {
				m.min.z = v.z
			}
			if v.x > m.max.x {
				m.max.x = v.x
			}
			if v.y > m.max.y {
				m.max.y = v.y
			}
			if v.z > m.max.z {
				m.max.z = v.z
			}
		}
	}
	m.hash = canonicalHash(tris)
	m.bndEdges = boundaryEdges(tris)
	m.surfaceArea = surfaceArea(tris)
	m.volume = volumeOf(tris)
	return m
}

func bboxDelta(a, b metrics) float64 {
	ax, ay, az := float64(a.max.x-a.min.x), float64(a.max.y-a.min.y), float64(a.max.z-a.min.z)
	bx, by, bz := float64(b.max.x-b.min.x), float64(b.max.y-b.min.y), float64(b.max.z-b.min.z)
	return math.Abs(ax-bx) + math.Abs(ay-by) + math.Abs(az-bz)
}

func bboxSize(m metrics) float64 {
	return float64(m.max.x-m.min.x) + float64(m.max.y-m.min.y) + float64(m.max.z-m.min.z)
}

func absInt(x int) int {
	if x < 0 {
		return -x
	}
	return x
}

func minF32(a, b, c float32) float32 {
	if b < a {
		a = b
	}
	if c < a {
		a = c
	}
	return a
}

func maxF32(a, b, c float32) float32 {
	if b > a {
		a = b
	}
	if c > a {
		a = c
	}
	return a
}

func relDelta(a, b float64) float64 {
	denom := math.Max(math.Abs(a), math.Abs(b))
	if denom == 0 {
		return 0
	}
	return math.Abs(a-b) / denom
}

//-----------------------------------------------------------------------------

// Thresholds for promoting MINOR → MATERIAL. Tuned conservatively so that
// only changes large enough to be visible in a print (or to indicate a
// structural defect) trip the wire. The Hausdorff thresholds are relative
// to the bbox sum so they scale across part sizes.
const (
	thrBboxRel    = 1e-4 // bbox edge sum drift > 0.01% of size
	thrTriRel     = 0.01 // tri-count change > 1%
	thrAreaRel    = 5e-3 // surface area drift > 0.5%
	thrVolRel     = 5e-3 // volume drift > 0.5%
	thrHausP99Rel = 1e-3 // p99 of point-to-tri drift > 0.1% of bbox sum
	thrHausMaxRel = 5e-3 // max of point-to-tri drift > 0.5% of bbox sum
)

func main() {
	if len(os.Args) != 3 {
		fmt.Fprintln(os.Stderr, "usage: stldiff <a.stl> <b.stl>")
		os.Exit(2)
	}
	a, err := loadSTL(os.Args[1])
	if err != nil {
		fmt.Fprintln(os.Stderr, "A:", err)
		os.Exit(2)
	}
	b, err := loadSTL(os.Args[2])
	if err != nil {
		fmt.Fprintln(os.Stderr, "B:", err)
		os.Exit(2)
	}
	ma, mb := computeMetrics(a), computeMetrics(b)
	if ma.hash == mb.hash {
		fmt.Printf("IDENTICAL  tris=%d hash=%s bnd=%d\n", ma.count, ma.hash, ma.bndEdges)
		return
	}

	// Hausdorff is the expensive metric — only compute it once we know the
	// hashes differ. Both directions because asymmetric Hausdorff misses
	// features that exist in only one of the meshes.
	gridA := newTriGrid(a)
	gridB := newTriGrid(b)
	vertsA := uniqueVerts(a)
	vertsB := uniqueVerts(b)
	hausAB := hausdorffOneWay(vertsA, gridB)
	hausBA := hausdorffOneWay(vertsB, gridA)
	hausP99 := math.Max(hausAB.p99, hausBA.p99)
	hausMax := math.Max(hausAB.max, hausBA.max)

	bb := bboxDelta(ma, mb)
	size := bboxSize(ma)
	relBbox := bb / size
	triDelta := mb.count - ma.count
	triRel := float64(absInt(triDelta)) / float64(ma.count+1)
	areaRel := relDelta(ma.surfaceArea, mb.surfaceArea)
	volRel := relDelta(ma.volume, mb.volume)
	hausP99Rel := hausP99 / size
	hausMaxRel := hausMax / size
	bndDelta := mb.bndEdges - ma.bndEdges

	material := relBbox >= thrBboxRel ||
		triRel >= thrTriRel ||
		bndDelta != 0 ||
		areaRel >= thrAreaRel ||
		volRel >= thrVolRel ||
		hausP99Rel >= thrHausP99Rel ||
		hausMaxRel >= thrHausMaxRel
	status := "MINOR   "
	if material {
		status = "MATERIAL"
	}

	fmt.Printf("%s  tris=%d→%d (Δ%+d, %.2f%%)  bnd=%d→%d (Δ%+d)  area=Δ%.2f%%  vol=Δ%.2f%%  haus.p99=%.3e (%.3f%%)  haus.max=%.3e (%.3f%%)  bbox-Δ=%.3e (%.3f%%)  hashA=%s hashB=%s\n",
		status,
		ma.count, mb.count, triDelta, triRel*100,
		ma.bndEdges, mb.bndEdges, bndDelta,
		areaRel*100,
		volRel*100,
		hausP99, hausP99Rel*100,
		hausMax, hausMaxRel*100,
		bb, relBbox*100,
		ma.hash, mb.hash,
	)
}
