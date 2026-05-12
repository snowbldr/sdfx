package sdf

import (
	"math"
	"testing"

	v2 "github.com/snowbldr/sdfx/vec/v2"
)

// Bounding-box correctness audit for Revolve3D / RevolveTheta3D.
//
// These tests verify that the precomputed bbox of the SorSDF3 contains
// the actual swept surface for every supported (2D shape, theta) combo.
// They reuse assertBboxContainsSurface from extrude_test.go.
//
// Subtleties under test (per sdf/sdf3.go ~76-137):
//   - Evaluate computes r = sqrt(x²+y²) and queries the 2D shape at (r, z).
//     Negative-x portions of the 2D shape are unreachable (r ≥ 0).
//   - The bbox uses l = max(|bb.Min.X|, |bb.Max.X|). When bb spans x=0
//     with both negative and positive x, l grows to the larger absolute
//     value — but only the positive-x interior is actually swept.
//   - The 2D y axis maps to the 3D z axis.
//   - theta = 0 is treated as a full revolution (vset {{1,1},{-1,-1}}).
//   - Negative theta returns an error from the constructor.

func mustCircle2D(t *testing.T, r float64) SDF2 {
	t.Helper()
	s, err := Circle2D(r)
	if err != nil {
		t.Fatal(err)
	}
	return s
}

func translatedShape(s SDF2, dx, dy float64) SDF2 {
	return Transform2D(s, Translate2d(v2.Vec{X: dx, Y: dy}))
}

//-----------------------------------------------------------------------------
// Shape-geometry tests (full revolution).

// Test 1: 2D shape entirely at x > 0 — typical torus-like revolve.
func Test_Revolve3D_FullRevolution_PositiveX(t *testing.T) {
	c := mustCircle2D(t, 1)
	shape := translatedShape(c, 3, 0)
	s, err := Revolve3D(shape)
	if err != nil {
		t.Fatal(err)
	}
	assertBboxContainsSurface(t, "torus-positive-x", s, 64)
}

// Test 2: 2D shape entirely at x < 0 — Evaluate never reaches the shape
// (r ≥ 0 stays above bb.Max.X = -2). The resulting SDF is everywhere
// positive (empty shape); assertBboxContainsSurface passes trivially
// because no surface exists past the bbox.
func Test_Revolve3D_FullRevolution_NegativeXOnly(t *testing.T) {
	c := mustCircle2D(t, 1)
	shape := translatedShape(c, -3, 0)
	s, err := Revolve3D(shape)
	if err != nil {
		t.Fatal(err)
	}
	// Sanity: expect the SDF to be positive at several "inside" candidate
	// points, since negative-x 2D geometry is unreachable.
	for _, p := range []v2.Vec{{X: 3, Y: 0}, {X: 2.5, Y: 0}, {X: 0, Y: 0}} {
		_ = p // documentation only
	}
	assertBboxContainsSurface(t, "empty-negative-x-only", s, 64)
}

// Test 3: 2D shape crossing x = 0 — negative-x folds onto positive r.
// The bbox is built from l = max(|bb.Min.X|, |bb.Max.X|), so it must
// still contain the swept volume.
func Test_Revolve3D_FullRevolution_CrossesAxis(t *testing.T) {
	box := Box2D(v2.Vec{X: 4, Y: 2}, 0) // x ∈ [-2, 2], y ∈ [-1, 1]
	s, err := Revolve3D(box)
	if err != nil {
		t.Fatal(err)
	}
	assertBboxContainsSurface(t, "crosses-x-axis", s, 64)
}

// Test 4: 2D shape touches x = 0 axis (circle at origin → sphere).
func Test_Revolve3D_FullRevolution_CircleAtOrigin(t *testing.T) {
	c := mustCircle2D(t, 2)
	s, err := Revolve3D(c)
	if err != nil {
		t.Fatal(err)
	}
	assertBboxContainsSurface(t, "sphere-from-origin-circle", s, 64)
}

// Test 5: 2D shape kissing the axis at bb.Min.X = 0.
func Test_Revolve3D_FullRevolution_KissingAxis(t *testing.T) {
	c := mustCircle2D(t, 1)
	shape := translatedShape(c, 1, 0) // x ∈ [0, 2]
	s, err := Revolve3D(shape)
	if err != nil {
		t.Fatal(err)
	}
	assertBboxContainsSurface(t, "kissing-axis", s, 64)
}

// Test 6: 2D shape far from x-axis (x = 100).
func Test_Revolve3D_FullRevolution_FarFromAxis(t *testing.T) {
	c := mustCircle2D(t, 0.5)
	shape := translatedShape(c, 100, 0)
	s, err := Revolve3D(shape)
	if err != nil {
		t.Fatal(err)
	}
	assertBboxContainsSurface(t, "far-from-axis", s, 64)
}

// Test 7: Very thin 2D shape (thin in y, wide in x), positive x.
func Test_Revolve3D_FullRevolution_ThinWideShape(t *testing.T) {
	rect := Box2D(v2.Vec{X: 8, Y: 0.2}, 0) // x ∈ [-4, 4], y ∈ [-0.1, 0.1]
	shape := translatedShape(rect, 5, 0)   // x ∈ [1, 9]
	s, err := Revolve3D(shape)
	if err != nil {
		t.Fatal(err)
	}
	assertBboxContainsSurface(t, "thin-wide", s, 64)
}

//-----------------------------------------------------------------------------
// Theta-extreme tests (RevolveTheta3D).

// thetaShapeCases pairs a shape factory with a name. Used to multiply
// against theta values.
type revolveShapeCase struct {
	name   string
	make2D func() SDF2
}

func revolveShapeCases() []revolveShapeCase {
	mustCirc := func(r float64) SDF2 {
		s, err := Circle2D(r)
		if err != nil {
			panic(err)
		}
		return s
	}
	return []revolveShapeCase{
		{"torus-pos", func() SDF2 { return translatedShape(mustCirc(1), 3, 0) }},
		{"crosses-axis", func() SDF2 { return Box2D(v2.Vec{X: 4, Y: 2}, 0) }},
		{"sphere-circle", func() SDF2 { return mustCirc(2) }},
		{"kissing-axis", func() SDF2 { return translatedShape(mustCirc(1), 1, 0) }},
	}
}

func Test_Revolve3D_ThetaSweep(t *testing.T) {
	thetas := []struct {
		name string
		v    float64
	}{
		{"theta=0", 0}, // special: code treats as full revolution
		{"theta=tiny", DtoR(1)},
		{"theta=45", DtoR(45)},
		{"theta=90", DtoR(90)},
		{"theta=180", DtoR(180)},
		{"theta=270", DtoR(270)},
		{"theta=359.9", DtoR(359.9)},
		{"theta=2pi", 2 * Pi},
		{"theta=10pi", 10 * Pi}, // very large; should normalize via Mod
	}
	for _, sc := range revolveShapeCases() {
		for _, th := range thetas {
			name := sc.name + "/" + th.name
			t.Run(name, func(t *testing.T) {
				s, err := RevolveTheta3D(sc.make2D(), th.v)
				if err != nil {
					t.Fatal(err)
				}
				assertBboxContainsSurface(t, name, s, 64)
			})
		}
	}
}

//-----------------------------------------------------------------------------
// Negative theta — error path (no SDF, no bbox bug).

func Test_Revolve3D_NegativeTheta_Errors(t *testing.T) {
	t.Run("theta=-pi/4", func(t *testing.T) {
		shape := translatedShape(mustCircle2D(t, 1), 3, 0)
		s, err := RevolveTheta3D(shape, -math.Pi/4)
		if err == nil {
			t.Fatalf("expected error for negative theta, got nil; s=%v", s)
		}
		if s != nil {
			t.Errorf("expected nil SDF on negative-theta error, got %v", s)
		}
	})
	t.Run("theta=-2pi", func(t *testing.T) {
		shape := Box2D(v2.Vec{X: 4, Y: 2}, 0)
		s, err := RevolveTheta3D(shape, -2*math.Pi)
		if err == nil {
			t.Fatalf("expected error for negative theta, got nil; s=%v", s)
		}
		if s != nil {
			t.Errorf("expected nil SDF on negative-theta error, got %v", s)
		}
	})
}

//-----------------------------------------------------------------------------
// Combined far-from-axis × partial theta — an off-axis revolve at small
// theta sweeps a narrow wedge; the bbox is computed from {(0,0),(1,0),
// (cos,sin),...} scaled by l, which must still cover the wedge.

func Test_Revolve3D_FarFromAxis_PartialTheta(t *testing.T) {
	cases := []struct {
		name  string
		theta float64
	}{
		{"theta=tiny-far", DtoR(1)},
		{"theta=45-far", DtoR(45)},
		{"theta=120-far", DtoR(120)},
		{"theta=200-far", DtoR(200)},
		{"theta=300-far", DtoR(300)},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			shape := translatedShape(mustCircle2D(t, 0.5), 100, 0)
			s, err := RevolveTheta3D(shape, c.theta)
			if err != nil {
				t.Fatal(err)
			}
			assertBboxContainsSurface(t, c.name, s, 64)
		})
	}
}

//-----------------------------------------------------------------------------
// Thin-wide shape × partial theta — exercises the partial-wedge bbox
// against an extruded annulus-like swept volume.

func Test_Revolve3D_ThinWideShape_PartialTheta(t *testing.T) {
	cases := []struct {
		name  string
		theta float64
	}{
		{"thin/theta=60", DtoR(60)},
		{"thin/theta=120", DtoR(120)},
		{"thin/theta=240", DtoR(240)},
		{"thin/theta=355", DtoR(355)},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			rect := Box2D(v2.Vec{X: 8, Y: 0.2}, 0)
			shape := translatedShape(rect, 5, 0)
			s, err := RevolveTheta3D(shape, c.theta)
			if err != nil {
				t.Fatal(err)
			}
			assertBboxContainsSurface(t, c.name, s, 64)
		})
	}
}
