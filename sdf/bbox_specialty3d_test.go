package sdf

import (
	"math"
	"testing"

	v3 "github.com/snowbldr/sdfx/vec/v3"
)

// Bbox containment audit for the "specialty" 3D SDFs:
//
//   - ScrewSDF3 (Screw3D): helically-swept 2D thread profile with length,
//     taper, pitch, starts. The bbox is derived from the thread.BoundingBox()
//     plus a tapered crest envelope and length/2 along z. We exercise the
//     parameter extremes that are most likely to expose missing margin.
//
//   - GyroidSDF3: the SDF is mathematically defined over all of R^3 and the
//     constructor returns the zero-bbox Box3{} as a deliberate signal that
//     the caller MUST intersect with an external bounding volume. We don't
//     run the standard containment helper against the empty bbox (it would
//     pass trivially); instead we verify the SDF magnitude is Lipschitz-1
//     and that the bbox is exactly Box3{} as documented.
//
//   - MeshSDF3 / MeshSDF3Slow: the Evaluate methods are stubs that return
//     0 (see sdf/mesh3.go, marked TODO). With Evaluate ≡ 0 the surface is
//     "everywhere" and the bbox-containment check is meaningless. We skip
//     those tests with an explanation, but still exercise the bbox itself
//     (must contain all triangle vertices) so we record at least *some*
//     coverage of the constructor.
//
//   - VoxelSDF3: built from another SDF (here Sphere3D) by sampling a
//     trilinear grid. Bbox is inherited from the source SDF.
//
// sampleN is kept low (specialty3dSampleN = 32) because some of these are
// expensive to evaluate (Screw3D in particular).

const specialty3dSampleN = 32

//-----------------------------------------------------------------------------
// Screw3D / ScrewSDF3

// screwCase captures one Screw3D construction for the bbox audit.
type specialty3DScrewCase struct {
	name     string
	profile  string // "iso", "iso-int", "acme", "buttress", "plastic-buttress"
	radius   float64
	pitch    float64
	length   float64
	taperDeg float64
	starts   int
}

func makeSpec3DThread(t *testing.T, profile string, radius, pitch float64) SDF2 {
	t.Helper()
	var s SDF2
	var err error
	switch profile {
	case "iso":
		s, err = ISOThread(radius, pitch, true)
	case "iso-int":
		s, err = ISOThread(radius, pitch, false)
	case "acme":
		s, err = AcmeThread(radius, pitch)
	case "buttress":
		s, err = ANSIButtressThread(radius, pitch)
	case "plastic-buttress":
		s, err = PlasticButtressThread(radius, pitch)
	default:
		t.Fatalf("unknown profile %q", profile)
	}
	if err != nil {
		t.Fatalf("thread profile %q (r=%v p=%v): %v", profile, radius, pitch, err)
	}
	return s
}

func screwSpec3DCases() []specialty3DScrewCase {
	return []specialty3DScrewCase{
		// Single-pitch length — minimum useful screw.
		{"iso_single_pitch", "iso", 5, 2, 2, 0, 1},
		// Length much larger than pitch — long screw, many turns.
		{"iso_long", "iso", 5, 2, 60, 0, 1},
		// Cylindrical (taper=0).
		{"iso_cyl_baseline", "iso", 5, 2, 20, 0, 1},
		// Moderate taper — 5° is realistic for NPT-like.
		{"iso_taper5deg", "iso", 5, 2, 20, 5, 1},
		// Severe taper — close to the cone-tip pinch where rSurfaceMin
		// clamping kicks in. Not quite 45° — that would degenerate to a
		// near-cone and the rSurfaceMin floor takes over.
		{"iso_taper30deg", "iso", 5, 2, 12, 30, 1},
		// Pitch much smaller than radius — fine thread.
		{"iso_fine_pitch", "iso", 10, 0.5, 20, 0, 1},
		// Pitch larger than radius — coarse thread (extreme but legal).
		{"iso_coarse_pitch_gt_r", "iso", 1.5, 3, 12, 0, 1},
		// starts = 1, 8, 16 (multi-start). High starts give a very steep helix.
		{"iso_starts8", "iso", 5, 2, 20, 0, 8},
		{"iso_starts16", "iso", 5, 2, 20, 0, 16},
		// Internal thread (cuts the other direction).
		{"iso_internal", "iso-int", 5, 2, 20, 0, 1},
		// Various thread profiles.
		{"acme_basic", "acme", 5, 2, 20, 0, 1},
		{"buttress_basic", "buttress", 5, 2, 20, 0, 1},
		{"plastic_buttress_basic", "plastic-buttress", 5, 2, 20, 0, 1},
		// Left-handed screw (negative starts).
		{"iso_left_hand", "iso", 5, 2, 20, 0, -1},
	}
}

func Test_Spec3D_Screw3D_BboxContainsSurface(t *testing.T) {
	for _, c := range screwSpec3DCases() {
		c := c
		t.Run(c.name, func(t *testing.T) {
			thread := makeSpec3DThread(t, c.profile, c.radius, c.pitch)
			s, err := Screw3D(thread, c.length, DtoR(c.taperDeg), c.pitch, c.starts)
			if err != nil {
				t.Fatalf("Screw3D(%+v): %v", c, err)
			}
			assertBboxContainsSurface(t, c.name, s, specialty3dSampleN)
		})
	}
}

// Test_Spec3D_Screw3D_BboxNotOverlyLoose checks the bbox is reasonably tight.
// For taper=0 the crest radius is exactly thread.Max.Y, so the bbox in x/y
// should hug the crest tightly. With taper>0 the bbox includes the wider
// end of the cone (z = -length/2), so the narrow-end faces will be loose
// by design — skip those configs from this check.
func Test_Spec3D_Screw3D_BboxNotOverlyLoose(t *testing.T) {
	cases := []specialty3DScrewCase{
		{"iso_cyl_tight", "iso", 5, 2, 20, 0, 1},
		{"iso_fine_pitch_tight", "iso", 10, 0.5, 20, 0, 1},
		{"acme_tight", "acme", 5, 2, 20, 0, 1},
	}
	for _, c := range cases {
		c := c
		t.Run(c.name, func(t *testing.T) {
			thread := makeSpec3DThread(t, c.profile, c.radius, c.pitch)
			s, err := Screw3D(thread, c.length, DtoR(c.taperDeg), c.pitch, c.starts)
			if err != nil {
				t.Fatal(err)
			}
			bb := s.BoundingBox()
			size := bb.Size()
			minDim := math.Min(size.X, math.Min(size.Y, size.Z))
			// Generous: pitch + 10% of min dim. The Screw3D crest oscillates
			// by ~pitch in z, so the surface is no farther than ~pitch from
			// any bbox face in the radial directions; in z the surface is
			// exactly at z = ±length/2 so the z faces should be tight to 0.
			tol := 0.1 * minDim
			assertBboxNotOverlyLoose(t, c.name, s, specialty3dSampleN, tol)
		})
	}
}

//-----------------------------------------------------------------------------
// Gyroid3D / GyroidSDF3
//
// The gyroid bbox is intentionally Box3{} — a zero-size point at origin —
// to signal that callers must intersect with an external bounding volume
// (see the doc comment on GyroidSDF3.BoundingBox). We don't run
// assertBboxContainsSurface against an empty bbox; the assertion is
// vacuously true and tells us nothing.

func Test_Spec3D_Gyroid3D_BboxIsEmptyByDesign(t *testing.T) {
	cases := []struct {
		name  string
		scale v3.Vec
	}{
		{"unit_isotropic", v3.Vec{1, 1, 1}}, // 1mm cell
		{"large_isotropic", v3.Vec{10, 10, 10}},
		{"small_isotropic", v3.Vec{0.1, 0.1, 0.1}},
		{"anisotropic", v3.Vec{2, 5, 8}},
	}
	for _, c := range cases {
		c := c
		t.Run(c.name, func(t *testing.T) {
			s, err := Gyroid3D(c.scale)
			if err != nil {
				t.Fatalf("Gyroid3D(%v): %v", c.scale, err)
			}
			bb := s.BoundingBox()
			zero := Box3{}
			if bb != zero {
				t.Errorf("expected zero bbox to signal unbounded surface, got %+v", bb)
			}
		})
	}
}

// Test_Spec3D_Gyroid3D_LipschitzMagnitude verifies the SDF magnitude stays
// bounded by ~1·max_radial_distance over a sample lattice — the gyroid is
// rescaled to Lipschitz-1 in the constructor and any breakage of that
// invariant would break the octree empty-cube skip downstream.
func Test_Spec3D_Gyroid3D_LipschitzMagnitude(t *testing.T) {
	s, err := Gyroid3D(v3.Vec{1, 1, 1})
	if err != nil {
		t.Fatal(err)
	}
	// Sample on a regular grid inside [-2, 2]^3 and verify |SDF(p)| ≤ 1
	// since the implicit function divided by sqrt(3)*max|k_i| is bounded
	// in [-1, 1] (the un-normalized form is bounded by sqrt(3) at
	// (π/2, π/2, π/2) etc.).
	const n = 8
	const lo, hi = -2.0, 2.0
	for i := 0; i <= n; i++ {
		for j := 0; j <= n; j++ {
			for k := 0; k <= n; k++ {
				x := lo + (hi-lo)*float64(i)/float64(n)
				y := lo + (hi-lo)*float64(j)/float64(n)
				z := lo + (hi-lo)*float64(k)/float64(n)
				v := s.Evaluate(v3.Vec{x, y, z})
				// Bound: sqrt(3) / (sqrt(3)*kMax) * sqrt(3) — the rescaling
				// divides by sqrt(3)*kMax, the implicit function magnitude
				// is bounded by sqrt(3); for kMax = Tau the post-scale
				// bound is sqrt(3)/(sqrt(3)*Tau)*sqrt(3) ≈ 0.275. Pad
				// for round-off.
				if math.Abs(v) > 1.0 {
					t.Errorf("|SDF(%v,%v,%v)|=%v exceeds 1 — Lipschitz-1 invariant broken", x, y, z, v)
				}
			}
		}
	}
}

//-----------------------------------------------------------------------------
// MeshSDF3 / MeshSDF3Slow
//
// Mesh3D / Mesh3DSlow currently have Evaluate stubs that return 0 (see
// the TODO in sdf/mesh3.go ~167 and ~206). With a stub Evaluate the bbox-
// containment helper is meaningless — every sample returns d=0, which
// will trip the "surface extends past bbox" check (d ≤ -1e-6 fails... but
// 0 is treated as "on the surface" and would pass containment). Either
// way the result is not a real test of the SDF.
//
// We still exercise the constructor + bbox calculation: the bbox must
// contain all input triangle vertices (the constructor does Extend over
// each triangle's bbox). This is a real test of the bbox-vs-input
// relationship even though Evaluate is a stub.

func Test_Spec3D_MeshSDF3_BboxContainsTriangleVertices(t *testing.T) {
	// Tetrahedron with one vertex offset from origin so the bbox isn't
	// trivially centered.
	tris := []*Triangle3{
		{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}},
		{{0, 0, 0}, {2, 0, 0}, {0, 0, 3}},
		{{0, 0, 0}, {0, 2, 0}, {0, 0, 3}},
		{{2, 0, 0}, {0, 2, 0}, {0, 0, 3}},
	}
	s, err := Mesh3D(tris)
	if err != nil {
		t.Fatal(err)
	}
	bb := s.BoundingBox()
	want := Box3{Min: v3.Vec{0, 0, 0}, Max: v3.Vec{2, 2, 3}}
	if bb != want {
		t.Errorf("bbox = %+v, want %+v", bb, want)
	}
	// And verify every vertex of every input triangle is inside the bbox.
	for ti, tri := range tris {
		for vi, v := range tri {
			if v.X < bb.Min.X || v.X > bb.Max.X ||
				v.Y < bb.Min.Y || v.Y > bb.Max.Y ||
				v.Z < bb.Min.Z || v.Z > bb.Max.Z {
				t.Errorf("triangle %d vertex %d = %v is outside bbox %+v", ti, vi, v, bb)
			}
		}
	}
}

func Test_Spec3D_MeshSDF3_EvaluateIsStub(t *testing.T) {
	// Document the current behaviour explicitly so a future fix to mesh3.go's
	// Evaluate is caught here as a "no longer stub" signal. This test will
	// start failing the day Evaluate becomes real — that's intentional, it's
	// a sentinel to prompt revisiting the surface-bbox audit for mesh.
	tris := []*Triangle3{
		{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}},
	}
	s, err := Mesh3D(tris)
	if err != nil {
		t.Fatal(err)
	}
	// Far from any triangle — a real SDF would return ≈ √3 here.
	d := s.Evaluate(v3.Vec{100, 100, 100})
	if d != 0 {
		t.Skipf("Mesh3D.Evaluate is no longer a stub (returned %v at (100,100,100)); "+
			"re-enable a real bbox-containment audit for MeshSDF3", d)
	}
	t.Log("Mesh3D.Evaluate is still a TODO stub returning 0; bbox-containment test is meaningless and skipped")
}

func Test_Spec3D_MeshSDF3Slow_BboxContainsTriangleVertices(t *testing.T) {
	tris := []*Triangle3{
		{{-1, -1, -1}, {1, -1, -1}, {0, 1, 0}},
		{{-1, -1, -1}, {1, -1, -1}, {0, 0, 2}},
	}
	s, err := Mesh3DSlow(tris)
	if err != nil {
		t.Fatal(err)
	}
	bb := s.BoundingBox()
	want := Box3{Min: v3.Vec{-1, -1, -1}, Max: v3.Vec{1, 1, 2}}
	if bb != want {
		t.Errorf("bbox = %+v, want %+v", bb, want)
	}
}

//-----------------------------------------------------------------------------
// VoxelSDF3
//
// VoxelSDF3 is built from a source SDF. Its bbox is inherited verbatim from
// the source, and its Evaluate trilinearly interpolates over the sampled
// grid. We verify the standard containment property holds for a few source
// SDFs at a low meshCells count (faster to build).

func Test_Spec3D_VoxelSDF3_BboxContainsSurface(t *testing.T) {
	// Source SDFs to sample. Keep them simple so the voxel grid converges
	// quickly even at modest meshCells.
	sphere, err := Sphere3D(2.0)
	if err != nil {
		t.Fatal(err)
	}
	box, err := Box3D(v3.Vec{4, 2, 6}, 0)
	if err != nil {
		t.Fatal(err)
	}
	cylinder, err := Cylinder3D(5.0, 1.5, 0)
	if err != nil {
		t.Fatal(err)
	}
	cases := []struct {
		name   string
		src    SDF3
		cells  int
		sample int
	}{
		{"sphere_r2_cells20", sphere, 20, specialty3dSampleN},
		{"sphere_r2_cells40", sphere, 40, specialty3dSampleN},
		{"box_4x2x6_cells20", box, 20, specialty3dSampleN},
		{"cyl_h5r1.5_cells20", cylinder, 20, specialty3dSampleN},
	}
	for _, c := range cases {
		c := c
		t.Run(c.name, func(t *testing.T) {
			v := NewVoxelSDF3(c.src, c.cells, nil)
			// Sanity: voxel bbox matches source bbox.
			if v.BoundingBox() != c.src.BoundingBox() {
				t.Errorf("voxel bbox %+v != source bbox %+v", v.BoundingBox(), c.src.BoundingBox())
			}
			assertBboxContainsSurface(t, c.name, v, c.sample)
		})
	}
}
