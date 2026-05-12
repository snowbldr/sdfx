package sdf

import (
	"math"
	"testing"

	v3 "github.com/snowbldr/sdfx/vec/v3"
)

// 3D Boolean combinator bbox audit.
//
// These tests target Union3D / Difference3D / Intersect3D / Cut3D /
// Elongate3D and their smooth-min / smooth-max variants. The audit verifies
// that the bbox computed at construction time contains the entire surface,
// using assertBboxContainsSurface from extrude_test.go.
//
// The MinFunc / MaxFunc helpers exercised:
//   RoundMin / RoundMax
//   ChamferMin / ChamferMax
//   PolyMin   / PolyMax
//   ExpMin    / ExpMax
//   PowMin    / PowMax   ← suspected gap: PowMin returns 0 from min(0,0)
//                          so SetMin / SetMax don't expand the bbox even
//                          though the smooth surface protrudes.

// minFuncEntry pairs a named smooth-min with a k value.
type bool3dMinEntry struct {
	name string
	make func(k float64) MinFunc
}

type bool3dMaxEntry struct {
	name string
	make func(k float64) MaxFunc
}

func bool3dMinFuncs() []bool3dMinEntry {
	return []bool3dMinEntry{
		{"RoundMin", RoundMin},
		{"ChamferMin", ChamferMin},
		{"PolyMin", PolyMin},
		{"ExpMin", ExpMin},
		{"PowMin", PowMin},
	}
}

func bool3dMaxFuncs() []bool3dMaxEntry {
	return []bool3dMaxEntry{
		{"RoundMax", RoundMax},
		{"ChamferMax", ChamferMax},
		{"PolyMax", PolyMax},
		{"ExpMax", ExpMax},
		{"PowMax", PowMax},
	}
}

// bool3dKs returns k values to sample for each MinFunc / MaxFunc, chosen to
// cover small, medium, and large blend reach. ExpMin / ExpMax use k as a
// sharpness parameter (larger k = tighter blend) so their range is flipped.
func bool3dKs(name string) []float64 {
	switch name {
	case "ExpMin", "ExpMax":
		return []float64{1, 4, 16}
	case "PowMin", "PowMax":
		return []float64{2, 4, 8}
	default:
		return []float64{0.1, 0.3, 1.0}
	}
}

// twoNearSpheres returns two overlapping spheres positioned so a smooth
// fillet between them clearly protrudes past their hard-union bbox.
func twoNearSpheres() (SDF3, SDF3) {
	s1, _ := Sphere3D(1)
	s2raw, _ := Sphere3D(0.7)
	s2 := Transform3D(s2raw, Translate3d(v3.Vec{X: 1.4}))
	return s1, s2
}

//-----------------------------------------------------------------------------
// Union3D — smooth-min audit

func Test_Bool3D_UnionSmoothMinAllFuncs(t *testing.T) {
	for _, m := range bool3dMinFuncs() {
		for _, k := range bool3dKs(m.name) {
			name := "Union_" + m.name + "_k_" + ftoa(k)
			t.Run(name, func(t *testing.T) {
				s1, s2 := twoNearSpheres()
				u := Union3D(s1, s2)
				u.(*UnionSDF3).SetMin(m.make(k))
				assertBboxContainsSurface(t, name, u, 64)
			})
		}
	}
}

// Test_Bool3D_UnionSmoothMinDisjointInputs — when the children don't
// overlap, the smooth-min still reaches outward at the silhouette edges.
// Verifies bbox padding works even when there's no overlap region.
func Test_Bool3D_UnionSmoothMinDisjointInputs(t *testing.T) {
	for _, m := range bool3dMinFuncs() {
		for _, k := range bool3dKs(m.name) {
			name := "UnionDisjoint_" + m.name + "_k_" + ftoa(k)
			t.Run(name, func(t *testing.T) {
				a, _ := Sphere3D(0.6)
				braw, _ := Sphere3D(0.6)
				b := Transform3D(braw, Translate3d(v3.Vec{X: 3}))
				u := Union3D(a, b)
				u.(*UnionSDF3).SetMin(m.make(k))
				assertBboxContainsSurface(t, name, u, 64)
			})
		}
	}
}

// Test_Bool3D_UnionPowMinSurfaceLeak — PowMin(0,0)=0 means SetMin doesn't
// pad the bbox, but PowMin still produces values less than min(a,b) for
// positive a, b (which is the whole point of a smooth-min). Sphere
// surfaces touch min(s0, s1) on the line between them, where PowMin
// pushes the iso surface outward. This is the suspected gap.
func Test_Bool3D_UnionPowMinSurfaceLeak(t *testing.T) {
	// Pick configurations where PowMin's smooth-min reach is plainly
	// visible: two spheres near each other.
	for _, k := range []float64{2, 3, 4, 8} {
		name := "UnionPowMinLeak_k_" + ftoa(k)
		t.Run(name, func(t *testing.T) {
			a, _ := Sphere3D(1)
			braw, _ := Sphere3D(1)
			b := Transform3D(braw, Translate3d(v3.Vec{X: 1.5}))
			u := Union3D(a, b)
			u.(*UnionSDF3).SetMin(PowMin(k))
			assertBboxContainsSurface(t, name, u, 64)
		})
	}
}

// Test_Bool3D_UnionNilHandling — Union3D should short-circuit nil / single
// argument cases without panicking.
func Test_Bool3D_UnionNilHandling(t *testing.T) {
	defer func() {
		if r := recover(); r != nil {
			t.Fatalf("Union3D nil handling panicked: %v", r)
		}
	}()
	if Union3D() != nil {
		t.Error("Union3D() with zero args should return nil")
	}
	if Union3D(nil) != nil {
		t.Error("Union3D(nil) should return nil")
	}
	if Union3D(nil, nil, nil) != nil {
		t.Error("Union3D(nil, nil, nil) should return nil")
	}
	s, _ := Sphere3D(1)
	// One non-nil + several nils — should pass through the non-nil.
	out := Union3D(nil, s, nil)
	if out == nil {
		t.Fatal("Union3D(nil, s, nil) should pass through s, got nil")
	}
	// The pass-through path returns the original SDF3 directly — its bbox
	// must still contain its surface.
	assertBboxContainsSurface(t, "UnionNilPassthrough", out, 64)
}

//-----------------------------------------------------------------------------
// Difference3D — including cutter much larger than body, and SetMax

func Test_Bool3D_DifferenceHugeCutter(t *testing.T) {
	// Body is a small box; cutter is a sphere 10× bigger that completely
	// envelops the body's center. Outer bbox should still be the body's
	// bbox (Difference3D.BoundingBox returns s0.BoundingBox()).
	body, _ := Box3D(v3.Vec{X: 2, Y: 2, Z: 2}, 0)
	cutterRaw, _ := Sphere3D(20)
	// Translate the huge cutter so it slices off one side of the body.
	cutter := Transform3D(cutterRaw, Translate3d(v3.Vec{X: 19}))
	s := Difference3D(body, cutter)
	assertBboxContainsSurface(t, "DiffHugeCutter", s, 64)
}

func Test_Bool3D_DifferenceCutterFullyContainsBody(t *testing.T) {
	// Cutter strictly contains the body — the difference is empty. The
	// bbox is allowed to be the body's bbox (no surface to escape).
	body, _ := Box3D(v3.Vec{X: 1, Y: 1, Z: 1}, 0)
	cutter, _ := Sphere3D(5)
	s := Difference3D(body, cutter)
	// No surface to check — just verify no panic and bbox is finite.
	bb := s.BoundingBox()
	if math.IsNaN(bb.Min.X) || math.IsInf(bb.Min.X, 0) {
		t.Errorf("bbox has non-finite coord: %+v", bb)
	}
}

func Test_Bool3D_DifferenceSmoothMaxAllFuncs(t *testing.T) {
	for _, m := range bool3dMaxFuncs() {
		for _, k := range bool3dKs(m.name) {
			name := "Diff_" + m.name + "_k_" + ftoa(k)
			t.Run(name, func(t *testing.T) {
				// Body large enough to clearly carry a fillet at the cutout edge.
				body, _ := Sphere3D(2)
				cutter, _ := Sphere3D(1)
				s := Difference3D(body, cutter)
				s.(*DifferenceSDF3).SetMax(m.make(k))
				assertBboxContainsSurface(t, name, s, 64)
			})
		}
	}
}

func Test_Bool3D_DifferenceNilHandling(t *testing.T) {
	defer func() {
		if r := recover(); r != nil {
			t.Fatalf("Difference3D nil handling panicked: %v", r)
		}
	}()
	s, _ := Sphere3D(1)
	if Difference3D(s, nil) != s {
		t.Error("Difference3D(s, nil) should return s")
	}
	if Difference3D(nil, s) != nil {
		t.Error("Difference3D(nil, s) should return nil")
	}
}

//-----------------------------------------------------------------------------
// Intersect3D — smooth max audit

func Test_Bool3D_IntersectSmoothMaxAllFuncs(t *testing.T) {
	for _, m := range bool3dMaxFuncs() {
		for _, k := range bool3dKs(m.name) {
			name := "Intersect_" + m.name + "_k_" + ftoa(k)
			t.Run(name, func(t *testing.T) {
				a, _ := Sphere3D(1.5)
				braw, _ := Box3D(v3.Vec{X: 1.5, Y: 1.5, Z: 1.5}, 0)
				b := Transform3D(braw, Translate3d(v3.Vec{X: 0.4}))
				s := Intersect3D(a, b)
				s.(*IntersectionSDF3).SetMax(m.make(k))
				assertBboxContainsSurface(t, name, s, 64)
			})
		}
	}
}

func Test_Bool3D_IntersectNilHandling(t *testing.T) {
	defer func() {
		if r := recover(); r != nil {
			t.Fatalf("Intersect3D nil handling panicked: %v", r)
		}
	}()
	s, _ := Sphere3D(1)
	if Intersect3D(s, nil) != nil {
		t.Error("Intersect3D(s, nil) should return nil")
	}
	if Intersect3D(nil, s) != nil {
		t.Error("Intersect3D(nil, s) should return nil")
	}
}

//-----------------------------------------------------------------------------
// Cut3D — various plane angles, plane missing the shape, plane fully
// removing the shape.

func Test_Bool3D_CutPlaneMissesShape(t *testing.T) {
	// Plane is well past the shape; the cut keeps the side containing the
	// shape so the result is the original shape.
	sph, _ := Sphere3D(1)
	// Normal points +X; plane at X=10 is past the sphere → keep everything.
	s := Cut3D(sph, v3.Vec{X: 10}, v3.Vec{X: -1})
	assertBboxContainsSurface(t, "CutPlaneMissesShape_keepAll", s, 64)
}

func Test_Bool3D_CutPlaneRemovesAll(t *testing.T) {
	// Normal points +X; plane at X=10, keep n side → empty result.
	// bbox should clip to empty / degenerate without panicking.
	sph, _ := Sphere3D(1)
	s := Cut3D(sph, v3.Vec{X: 10}, v3.Vec{X: 1})
	bb := s.BoundingBox()
	// Bbox should be degenerate (empty) — Min == Max — and finite.
	if math.IsNaN(bb.Min.X) || math.IsInf(bb.Min.X, 0) ||
		math.IsNaN(bb.Max.X) || math.IsInf(bb.Max.X, 0) {
		t.Errorf("CutPlaneRemovesAll: bbox has non-finite coord: %+v", bb)
	}
	// No surface to contain — but the SDF at the plane sample point should
	// be ≥ 0 there since the half-space distance dominates.
	d := s.Evaluate(v3.Vec{X: 10})
	if d < -1e-6 {
		t.Errorf("CutPlaneRemovesAll: SDF should be ≥ 0 everywhere on kept side, got %v", d)
	}
}

func Test_Bool3D_CutVariousAngles(t *testing.T) {
	box, _ := Box3D(v3.Vec{X: 2, Y: 1.5, Z: 1}, 0)
	cases := []struct {
		name string
		a    v3.Vec
		n    v3.Vec
	}{
		{"axisX_at_origin", v3.Vec{}, v3.Vec{X: 1}},
		{"axisY_at_origin", v3.Vec{}, v3.Vec{Y: 1}},
		{"axisZ_at_origin", v3.Vec{}, v3.Vec{Z: 1}},
		{"diagonal_xy", v3.Vec{}, v3.Vec{X: 1, Y: 1}},
		{"diagonal_xyz", v3.Vec{}, v3.Vec{X: 1, Y: 1, Z: 1}},
		{"shallow_angle", v3.Vec{X: 0.5}, v3.Vec{X: 0.95, Y: 0.05, Z: 0.05}},
		{"offset_plane_inside", v3.Vec{X: 0.5}, v3.Vec{X: 1}},
		{"offset_plane_negative", v3.Vec{X: -0.5}, v3.Vec{X: -1}},
		{"tilted_off_origin", v3.Vec{X: 0.3, Y: 0.2, Z: 0.1}, v3.Vec{X: 1, Y: 2, Z: 3}},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s := Cut3D(box, c.a, c.n)
			assertBboxContainsSurface(t, "Cut_"+c.name, s, 64)
		})
	}
}

//-----------------------------------------------------------------------------
// Elongate3D — degenerate (axis = 0) and very large axes

func Test_Bool3D_ElongateZeroAxes(t *testing.T) {
	sph, _ := Sphere3D(1)
	// All-zero h should be the identity (no elongation).
	s := Elongate3D(sph, v3.Vec{})
	assertBboxContainsSurface(t, "ElongateAllZero", s, 64)

	// One axis = 0, others non-zero.
	cases := []struct {
		name string
		h    v3.Vec
	}{
		{"x_only", v3.Vec{X: 2}},
		{"y_only", v3.Vec{Y: 2}},
		{"z_only", v3.Vec{Z: 2}},
		{"yz_zero", v3.Vec{X: 1.5}},
		{"xy_zero", v3.Vec{Z: 1.5}},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s := Elongate3D(sph, c.h)
			assertBboxContainsSurface(t, "Elongate_"+c.name, s, 64)
		})
	}
}

func Test_Bool3D_ElongateLargeAxes(t *testing.T) {
	box, _ := Box3D(v3.Vec{X: 1, Y: 1, Z: 1}, 0)
	cases := []struct {
		name string
		h    v3.Vec
	}{
		{"huge_x", v3.Vec{X: 1000}},
		{"huge_all", v3.Vec{X: 1e4, Y: 1e4, Z: 1e4}},
		{"mixed_huge", v3.Vec{X: 1e3, Y: 0.5, Z: 1e3}},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s := Elongate3D(box, c.h)
			assertBboxContainsSurface(t, "Elongate_"+c.name, s, 64)
		})
	}
}

// Elongate3D documents that h.Abs() is taken — negative entries should
// behave like positive ones with the same magnitude.
func Test_Bool3D_ElongateNegativeAxes(t *testing.T) {
	sph, _ := Sphere3D(1)
	s := Elongate3D(sph, v3.Vec{X: -2, Y: -1, Z: -0.5})
	assertBboxContainsSurface(t, "ElongateNegativeH", s, 64)
}
