//-----------------------------------------------------------------------------
/*

Parallel Marching Cubes Octree

Convert an SDF3 to a triangle mesh.
Uses octree space subdivision with parallel processing across CPU cores.

The single-threaded MarchingCubesOctree leaves most CPU idle on modern
machines. This implementation splits the octree into independent subtrees
and processes them in parallel, one goroutine per CPU core. Each goroutine
gets its own SDF evaluation cache so there is no lock contention.

Architecture:
  Phase 1 (sequential): A scout worker walks the octree down to a fan-out
    depth and collects non-empty subcubes as work items. This is cheap
    because it only evaluates cube centers (isEmpty checks).
  Phase 2 (parallel): N goroutines pull subcubes from a channel and
    recursively process them, each with an independent cache.
  Phase 3 (sequential): Triangles are written in subcube index order.

*/
//-----------------------------------------------------------------------------

package render

import (
	"fmt"
	"math"
	"runtime"
	"sync"

	"github.com/deadsy/sdfx/sdf"
	"github.com/deadsy/sdfx/vec/conv"
	v3 "github.com/deadsy/sdfx/vec/v3"
	"github.com/deadsy/sdfx/vec/v3i"
)

//-----------------------------------------------------------------------------
// Direct-mapped SDF evaluation cache.
//
// The standard Go map used by dcache3 (march3x.go) spends ~40% of CPU time
// in runtime.mapaccess2, runtime.madvise, and runtime.memclrNoHeapPointers.
// This direct-mapped cache replaces it, reducing cache overhead to <1%.
//
// Design: fixed-size array indexed by fibonacci hash of packed coordinates.
// On collision the old entry is evicted — the value is simply re-evaluated
// from the SDF, so there is no correctness issue, only a minor perf cost.
// Experimentally ~2/3 of lookups hit, similar to the Go map, but without
// the overhead of hashing, bucket chains, and GC scanning.

type cacheEntry struct {
	key uint64  // packed v3i.Vec; 0 = empty sentinel
	val float64
}

type directCache struct {
	entries []cacheEntry
	mask    int // size - 1, for fast modulo via bitwise AND
}

// directCacheBits controls the cache size: 1<<bits entries (16 bytes each).
// 18 → 256K entries, 4MB per worker. Large enough for good hit rates on
// complex models, small enough to stay in L3 cache per core.
const directCacheBits = 18

// directCachePool recycles directCache backing arrays across renders and
// workers. Each render allocates one cache per CPU (~4MB each); without
// pooling a multi-render pipeline like examples/picorx churns ~200MB of
// transient allocations that show up as runtime.madvise in pprof.
var directCachePool = sync.Pool{
	New: func() any {
		size := 1 << directCacheBits
		return &directCache{
			entries: make([]cacheEntry, size),
			mask:    size - 1,
		}
	},
}

// acquireDirectCache returns a zeroed directCache from the pool. Zeroing
// is required for correctness: if a stale entry's packed key happens to
// match a new lookup, get() would return a wrong distance. 4MB memclr is
// ~0.13ms per worker and is paid once per render.
func acquireDirectCache() *directCache {
	c := directCachePool.Get().(*directCache)
	clear(c.entries)
	return c
}

func releaseDirectCache(c *directCache) {
	directCachePool.Put(c)
}

// packVec packs 3 grid coordinates into a single uint64 key.
// 21 bits per axis supports coordinates up to 2^21 (~2M), which at typical
// resolutions (0.01–1.0mm) covers bounding boxes up to ~2km.
// +1 reserves key=0 as the empty sentinel so we never confuse an empty
// slot with the origin coordinate (0,0,0).
func packVec(vi v3i.Vec) uint64 {
	return (uint64(vi.X)&0x1FFFFF | (uint64(vi.Y)&0x1FFFFF)<<21 | (uint64(vi.Z)&0x1FFFFF)<<42) + 1
}

// index uses fibonacci hashing (multiply by golden ratio constant, take
// high bits) to distribute keys evenly across the table. This gives better
// distribution than simple modulo for the spatially clustered grid coords
// we see in octree traversal.
func (c *directCache) index(packed uint64) int {
	return int(packed*0x9E3779B97F4A7C15>>32) & c.mask
}

func (c *directCache) get(packed uint64) (float64, bool) {
	e := &c.entries[c.index(packed)]
	if e.key == packed {
		return e.val, true
	}
	return 0, false
}

func (c *directCache) set(packed uint64, val float64) {
	idx := c.index(packed)
	c.entries[idx] = cacheEntry{key: packed, val: val}
}

//-----------------------------------------------------------------------------
// Per-goroutine worker.
//
// Each goroutine gets its own worker with an independent cache. Adjacent
// subcubes processed by different goroutines will re-evaluate shared
// boundary vertices, but this redundancy is cheaper than any sharing
// mechanism (sync.Map, sharded locks, etc.) — profiling confirmed this.

type mcWorker struct {
	origin     v3.Vec       // corner of the bounding cube in world space
	resolution float64      // size of the smallest octree cube (half the requested mesh resolution)
	hdiag      []float64    // precomputed half-diagonal length per octree level, for isEmpty checks
	s          sdf.SDF3     // the SDF being rendered
	cache      *directCache // per-worker evaluation cache (no sharing, no locks)
}

func newMCWorker(s sdf.SDF3, origin v3.Vec, resolution float64, levels uint) *mcWorker {
	w := &mcWorker{
		origin:     origin,
		resolution: resolution,
		hdiag:      make([]float64, levels),
		s:          s,
		cache:      acquireDirectCache(),
	}
	// Precompute the half-diagonal distance for cubes at each octree level.
	// Used by isEmpty to determine if a cube can possibly contain the surface.
	for i := 0; i < len(w.hdiag); i++ {
		side := float64(int(1)<<uint(i)) * resolution
		w.hdiag[i] = 0.5 * math.Sqrt(3.0*side*side)
	}
	return w
}

// evaluate returns the world-space position and SDF distance for a grid point,
// using the cache to avoid redundant SDF evaluations. Vertices shared between
// adjacent cubes (which is most of them) typically hit the cache.
func (w *mcWorker) evaluate(vi v3i.Vec) (v3.Vec, float64) {
	v := w.origin.Add(conv.V3iToV3(vi).MulScalar(w.resolution))
	packed := packVec(vi)
	if d, ok := w.cache.get(packed); ok {
		return v, d
	}
	d := w.s.Evaluate(v)
	w.cache.set(packed, d)
	return v, d
}

// evaluateD is evaluate without the v3.Vec position. isEmpty discards the
// position on every call — skipping it saves the 3-float world-space
// computation per octree-cell prune test, which happens many times more
// often than leaf-corner evaluations.
func (w *mcWorker) evaluateD(vi v3i.Vec) float64 {
	packed := packVec(vi)
	if d, ok := w.cache.get(packed); ok {
		return d
	}
	v := w.origin.Add(conv.V3iToV3(vi).MulScalar(w.resolution))
	d := w.s.Evaluate(v)
	w.cache.set(packed, d)
	return d
}

// isEmpty tests whether a cube can possibly contain any part of the surface.
// It evaluates the SDF at the cube center: if the absolute distance exceeds
// the half-diagonal (the farthest any corner can be from the center), the
// surface cannot intersect this cube and we can skip it entirely.
func (w *mcWorker) isEmpty(c cube) bool {
	s := 1 << (c.n - 1)
	d := w.evaluateD(c.v.AddScalar(s))
	if d < 0 {
		d = -d
	}
	return d >= w.hdiag[c.n]
}

// processCube recursively subdivides the octree. At the leaf level (n==1),
// it evaluates the 8 cube corners and runs marching cubes to emit triangles.
// Triangles are appended to a flat slice (value types, not pointers) to
// avoid per-triangle heap allocation — the pointer slice required by
// Triangle3Writer is created once when writing results.
//
// Cube is passed by value (32 bytes). Passing a *cube to the recursive
// call caused &cube{...} literals to escape to the heap, costing ~12% CPU
// in runtime.newobject/mallocgc (pprof picorx rhs). Value recursion keeps
// everything on the stack.
func (w *mcWorker) processCube(c cube, tris []sdf.Triangle3) []sdf.Triangle3 {
	if w.isEmpty(c) {
		return tris
	}
	if c.n == 1 {
		// Leaf cube: evaluate all 8 corners and generate triangles.
		// Corner offsets are 0 and 2 (not 0 and 1) because the level-0
		// cube is at half resolution — see the resolution *= 0.5 below.
		c0, d0 := w.evaluate(c.v.Add(v3i.Vec{0, 0, 0}))
		c1, d1 := w.evaluate(c.v.Add(v3i.Vec{2, 0, 0}))
		c2, d2 := w.evaluate(c.v.Add(v3i.Vec{2, 2, 0}))
		c3, d3 := w.evaluate(c.v.Add(v3i.Vec{0, 2, 0}))
		c4, d4 := w.evaluate(c.v.Add(v3i.Vec{0, 0, 2}))
		c5, d5 := w.evaluate(c.v.Add(v3i.Vec{2, 0, 2}))
		c6, d6 := w.evaluate(c.v.Add(v3i.Vec{2, 2, 2}))
		c7, d7 := w.evaluate(c.v.Add(v3i.Vec{0, 2, 2}))
		corners := [8]v3.Vec{c0, c1, c2, c3, c4, c5, c6, c7}
		values := [8]float64{d0, d1, d2, d3, d4, d5, d6, d7}
		return mcAppendTriangles(tris, corners, values, 0)
	}
	// Subdivide into 8 child cubes and recurse.
	n := c.n - 1
	s := 1 << n
	for _, off := range mcOctreeOffsets(s) {
		tris = w.processCube(cube{c.v.Add(off), n}, tris)
	}
	return tris
}

//-----------------------------------------------------------------------------

func mcOctreeOffsets(s int) [8]v3i.Vec {
	return [8]v3i.Vec{
		{0, 0, 0}, {s, 0, 0}, {s, s, 0}, {0, s, 0},
		{0, 0, s}, {s, 0, s}, {s, s, s}, {0, s, s},
	}
}

// mcAppendTriangles is an append-based variant of mcToTriangles (march3.go).
// mcToTriangles allocates a new slice per cube, which means a heap allocation
// for every non-empty leaf. This version appends to a caller-owned slice that
// grows across the entire subtree, amortizing allocation.
func mcAppendTriangles(tris []sdf.Triangle3, p [8]v3.Vec, v [8]float64, x float64) []sdf.Triangle3 {
	// Build the case index from the 8 corner signs.
	index := 0
	for i := 0; i < 8; i++ {
		if v[i] < x {
			index |= 1 << uint(i)
		}
	}
	// No edges crossed — cube is entirely inside or outside the surface.
	if mcEdgeTable[index] == 0 {
		return tris
	}
	// Interpolate surface crossing points along each active edge.
	var points [12]v3.Vec
	for i := 0; i < 12; i++ {
		if mcEdgeTable[index]&(1<<uint(i)) != 0 {
			a := mcPairTable[i][0]
			b := mcPairTable[i][1]
			points[i] = mcInterpolate(p[a], p[b], v[a], v[b], x)
		}
	}
	// Emit triangles from the lookup table.
	//
	// Triangle3 is 72 bytes, so an append of a zero value costs one 72-byte
	// zero-copy even when capacity is already present (profiled at 26% CPU
	// on picorx rhs). Extending len without the zero copy requires bypassing
	// append: if cap allows, we reslice and write fields directly; only the
	// grow path still uses append. With a pre-sized worker buffer the grow
	// path is hit on the order of 1-3 times per render.
	table := mcTriangleTable[index]
	count := len(table) / 3
	for i := 0; i < count; i++ {
		// Check degeneracy before writing. Triangle3.Degenerate(0) compares
		// vertices via math.Abs(x-y) <= 0, which reduces to x == y for normal
		// floats. Direct struct equality skips the 6 math.Abs calls per
		// triangle and, on degenerate cases, avoids three 24-byte vertex
		// writes followed by a truncate.
		p0 := points[table[i*3+0]]
		p1 := points[table[i*3+1]]
		p2 := points[table[i*3+2]]
		if p0 == p1 || p1 == p2 || p2 == p0 {
			continue
		}
		n := len(tris)
		if n < cap(tris) {
			tris = tris[:n+1]
		} else {
			tris = append(tris, sdf.Triangle3{})
		}
		dst := &tris[n]
		dst[2] = p0
		dst[1] = p1
		dst[0] = p2
	}
	return tris
}

//-----------------------------------------------------------------------------

// collectSubcubes walks the octree down to targetLevel and returns all
// non-empty subcubes at that depth. These become the work items for
// parallel processing. The scout worker's cache warms up during this
// traversal, but we don't share it with the parallel workers (each gets
// its own cache to avoid lock contention).
func collectSubcubes(w *mcWorker, c cube, targetLevel uint) []cube {
	if w.isEmpty(c) {
		return nil
	}
	if c.n <= targetLevel || c.n == 1 {
		return []cube{c}
	}
	n := c.n - 1
	s := 1 << n
	var cubes []cube
	for _, off := range mcOctreeOffsets(s) {
		cubes = append(cubes, collectSubcubes(w, cube{c.v.Add(off), n}, targetLevel)...)
	}
	return cubes
}

func parallelMarchingCubesOctree(s sdf.SDF3, resolution float64, output sdf.Triangle3Writer) {
	// Pad the bounding box by 1% so the surface doesn't land exactly on
	// the boundary, which can cause marching cubes to miss edge triangles.
	bb := s.BoundingBox()
	bb = bb.ScaleAboutCenter(1.01)
	longAxis := bb.Size().MaxComponent()

	// The smallest octree cube (level 1) spans 2 grid units, so we halve
	// the resolution to make level-1 cubes match the requested mesh cell size.
	resolution = 0.5 * resolution

	// Number of octree levels needed to cover the bounding box.
	levels := uint(math.Ceil(math.Log2(longAxis/resolution))) + 1

	// Phase 1: a single scout worker traverses the top of the octree to
	// collect non-empty subcubes as parallel work items. This is sequential
	// but cheap — it only evaluates cube centers for isEmpty checks.
	scout := newMCWorker(s, bb.Min, resolution, levels)
	topCube := cube{v3i.Vec{0, 0, 0}, levels - 1}

	// Choose a fan-out depth that produces roughly nCPU*8 work items.
	// Too few items → poor load balancing (some cores idle while others
	// finish large subtrees). Too many → excessive overhead from goroutine
	// scheduling. 8x oversubscription is a good balance.
	nCPU := runtime.NumCPU()
	fanoutLevel := levels - 1
	for fanoutLevel > 2 {
		fanoutLevel--
		items := 1
		for i := uint(0); i < levels-1-fanoutLevel; i++ {
			items *= 8
			if items >= nCPU*8 {
				break
			}
		}
		if items >= nCPU*8 {
			break
		}
	}

	subcubes := collectSubcubes(scout, topCube, fanoutLevel)
	releaseDirectCache(scout.cache)

	// Phase 2: distribute subcubes across goroutines. Each goroutine gets
	// its own worker (and therefore its own cache) — no shared mutable state.
	//
	// Each worker appends into one ever-growing buffer across all its subcubes
	// instead of starting from nil per subcube. A nil-per-subcube approach
	// runs the slice through ~14 doubling growths each (picorx rhs has ~19k
	// triangles per subcube), which profiled at 11% CPU in memmove+memclr
	// from growslice. Sharing the buffer lets allocation amortize across the
	// worker's entire workload, leaving at most one realloc per worker.
	// A (workerID, start, end) triplet per subcube preserves subcube-order
	// output, so STLs stay deterministic.
	type span struct {
		worker, start, end int
	}
	spans := make([]span, len(subcubes))
	workerBufs := make([][]sdf.Triangle3, nCPU)

	var wg sync.WaitGroup
	work := make(chan int, len(subcubes))

	// Pre-size each worker's triangle buffer. Starting from nil incurs
	// ~7 growslice doublings per worker at picorx rhs scale (150k tris /
	// worker), each doing memclr + memmove on an ever-larger backing
	// array. 16k is large enough for most small subcubes and small enough
	// that idle workers don't waste significant memory.
	const initialTriCap = 16384
	for i := 0; i < nCPU; i++ {
		wg.Add(1)
		go func(wid int) {
			defer wg.Done()
			w := newMCWorker(s, bb.Min, resolution, levels)
			buf := make([]sdf.Triangle3, 0, initialTriCap)
			for idx := range work {
				start := len(buf)
				buf = w.processCube(subcubes[idx], buf)
				spans[idx] = span{worker: wid, start: start, end: len(buf)}
			}
			workerBufs[wid] = buf
			releaseDirectCache(w.cache)
		}(i)
	}

	for i := range subcubes {
		work <- i
	}
	close(work)
	wg.Wait()

	// Phase 3: write triangles in subcube index order. This produces
	// deterministic output regardless of goroutine scheduling.
	// Convert from flat value slice to pointer slice as required by
	// the Triangle3Writer interface.
	for _, sp := range spans {
		if sp.end == sp.start {
			continue
		}
		tris := workerBufs[sp.worker][sp.start:sp.end]
		ptrs := make([]*sdf.Triangle3, len(tris))
		for i := range tris {
			ptrs[i] = &tris[i]
		}
		output.Write(ptrs)
	}
	output.Close()
}

//-----------------------------------------------------------------------------

// MarchingCubesOctreeParallel renders using marching cubes with octree space
// sampling and parallel processing across all available CPU cores.
type MarchingCubesOctreeParallel struct {
	meshCells int // number of cells on the longest axis of bounding box
}

// NewMarchingCubesOctreeParallel returns a Render3 object.
func NewMarchingCubesOctreeParallel(meshCells int) *MarchingCubesOctreeParallel {
	return &MarchingCubesOctreeParallel{
		meshCells: meshCells,
	}
}

// Info returns a string describing the rendered volume.
func (r *MarchingCubesOctreeParallel) Info(s sdf.SDF3) string {
	bbSize := s.BoundingBox().Size()
	resolution := bbSize.MaxComponent() / float64(r.meshCells)
	cells := conv.V3ToV3i(bbSize.MulScalar(1 / resolution))
	return fmt.Sprintf("%dx%dx%d, resolution %.2f", cells.X, cells.Y, cells.Z, resolution)
}

// Render produces a 3d triangle mesh over the bounding volume of an sdf3.
func (r *MarchingCubesOctreeParallel) Render(s sdf.SDF3, output sdf.Triangle3Writer) {
	bbSize := s.BoundingBox().Size()
	resolution := bbSize.MaxComponent() / float64(r.meshCells)
	parallelMarchingCubesOctree(s, resolution, output)
}

//-----------------------------------------------------------------------------
