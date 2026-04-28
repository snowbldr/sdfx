package render

import (
	"testing"

	"github.com/snowbldr/sdfx/sdf"
	v3 "github.com/snowbldr/sdfx/vec/v3"
)

// Watertightness check for the smooth-min / smooth-max blend factories.
//
// The octree marching-cubes renderer skips a cube when |sdf(center)| ≥
// half-diagonal — sound only if the SDF is 1-Lipschitz. The original
// ChamferMin (factor √½) and RoundMin (max(k,min) – length(...)) both
// inflated the spatial gradient above 1 wherever the input gradients
// aligned (e.g. on the line between two overlapping spheres), so the
// pruning rule dropped surface cubes and the mesh came out with holes.
//
// Configurations exercise the worst-case ∇a∥∇b region: two overlapping
// spheres for the union (smooth-min) family, and a bigger sphere with a
// concentric smaller sphere subtracted for the difference (smooth-max)
// family. If a blend ever regresses to non-Lipschitz behavior, this test
// will report a non-zero boundary-edge count.

func twoOverlappingSpheres(t *testing.T) (sdf.SDF3, sdf.SDF3) {
	t.Helper()
	a, err := sdf.Sphere3D(1.0)
	if err != nil {
		t.Fatal(err)
	}
	b, err := sdf.Sphere3D(1.0)
	if err != nil {
		t.Fatal(err)
	}
	b = sdf.Transform3D(b, sdf.Translate3d(v3.Vec{X: 1.5}))
	return a, b
}

func sphereMinusSphere(t *testing.T) (sdf.SDF3, sdf.SDF3) {
	t.Helper()
	a, err := sdf.Sphere3D(1.5)
	if err != nil {
		t.Fatal(err)
	}
	b, err := sdf.Sphere3D(0.6)
	if err != nil {
		t.Fatal(err)
	}
	// Centers offset along x so ∇a and ∇b align on the line between
	// them — same configuration that broke ChamferMin / RoundMin.
	b = sdf.Transform3D(b, sdf.Translate3d(v3.Vec{X: 0.7}))
	return a, b
}

func Test_BlendMinFuncs_Watertight(t *testing.T) {
	const k = 0.4
	const cells = 60

	cases := []struct {
		name string
		min  sdf.MinFunc
	}{
		{"RoundMin", sdf.RoundMin(k)},
		{"ChamferMin", sdf.ChamferMin(k)},
		{"ExpMin", sdf.ExpMin(32)},
		{"PowMin", sdf.PowMin(8)},
		{"PolyMin", sdf.PolyMin(k)},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			a, b := twoOverlappingSpheres(t)
			u := sdf.Union3D(a, b)
			u.(*sdf.UnionSDF3).SetMin(c.min)
			assertWatertight(t, u, cells)
		})
	}
}

func Test_BlendMaxFuncs_Watertight(t *testing.T) {
	const k = 0.3
	const cells = 60

	cases := []struct {
		name string
		max  sdf.MaxFunc
	}{
		{"RoundMax", sdf.RoundMax(k)},
		{"ChamferMax", sdf.ChamferMax(k)},
		{"ExpMax", sdf.ExpMax(32)},
		{"PowMax", sdf.PowMax(8)},
		{"PolyMax", sdf.PolyMax(k)},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			a, b := sphereMinusSphere(t)
			d := sdf.Difference3D(a, b)
			d.(*sdf.DifferenceSDF3).SetMax(c.max)
			assertWatertight(t, d, cells)
		})
	}
}

// Test_BlendMinFuncs_BboxExtension_Watertight forces the smooth-min fillet
// to protrude outside the union of the children's bounding boxes, by using
// a smoothing scale comparable to the sphere radius.
//
// For two unit spheres at offset 1.5, UnionSDF3 reports bbox y,z ∈ [-1, 1].
// Smooth-min with large k inflates the surface at x=0 in y,z past 1.
// Octree marching cubes evaluates only inside the bbox and clips the bulge
// flat — producing boundary edges along the bbox face.
//
// This is independent of the Lipschitz fix exercised by the basic watertight
// test: even with a 1-Lipschitz smooth-min, UnionSDF3.BoundingBox does not
// account for the blend's outward extension. If this test fails, the fix is
// in UnionSDF3 (pad the bbox by the blend's reach), not in the MinFuncs.
func Test_BlendMinFuncs_BboxExtension_Watertight(t *testing.T) {
	const cells = 80

	// k chosen per family to give a comparably aggressive fillet
	// (round/chamfer/poly: rounding ∝ k; exp/pow: rounding ∝ 1/k).
	cases := []struct {
		name string
		min  sdf.MinFunc
	}{
		{"RoundMin", sdf.RoundMin(2.0)},
		{"ChamferMin", sdf.ChamferMin(2.0)},
		{"PolyMin", sdf.PolyMin(2.0)},
		{"ExpMin", sdf.ExpMin(2.0)},
		{"PowMin", sdf.PowMin(2.0)},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			a, b := twoOverlappingSpheres(t)
			u := sdf.Union3D(a, b)
			u.(*sdf.UnionSDF3).SetMin(c.min)
			assertWatertight(t, u, cells)
		})
	}
}

// Test_BlendMinFuncs_NaryUnion_Watertight checks that an n-ary smooth union
// (UnionSDF3 folds children pairwise) stays watertight. min is associative,
// smooth-min is not — pairwise reduction can produce a different surface than
// a true n-way smooth-min, and the resulting field may violate Lipschitz-1
// even when the pairwise blend itself is well-behaved.
func Test_BlendMinFuncs_NaryUnion_Watertight(t *testing.T) {
	const k = 0.4
	const cells = 80

	// Three unit spheres at the corners of an equilateral triangle in xy,
	// edge length 1.5. Each pair contributes a smooth fillet, and all
	// three meet near the centroid.
	build := func(t *testing.T) sdf.SDF3 {
		t.Helper()
		positions := []v3.Vec{
			{X: 0, Y: 0},
			{X: 1.5, Y: 0},
			{X: 0.75, Y: 1.299},
		}
		var children []sdf.SDF3
		for _, p := range positions {
			s, err := sdf.Sphere3D(1.0)
			if err != nil {
				t.Fatal(err)
			}
			children = append(children, sdf.Transform3D(s, sdf.Translate3d(p)))
		}
		return sdf.Union3D(children...)
	}

	cases := []struct {
		name string
		min  sdf.MinFunc
	}{
		{"RoundMin", sdf.RoundMin(k)},
		{"ChamferMin", sdf.ChamferMin(k)},
		{"ExpMin", sdf.ExpMin(32)},
		{"PowMin", sdf.PowMin(8)},
		{"PolyMin", sdf.PolyMin(k)},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			u := build(t)
			u.(*sdf.UnionSDF3).SetMin(c.min)
			assertWatertight(t, u, cells)
		})
	}
}
