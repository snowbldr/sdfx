package sdf

import (
	"math"
	"strconv"
	"testing"

	v2 "github.com/deadsy/sdfx/vec/v2"
	v3 "github.com/deadsy/sdfx/vec/v3"
)

// Bounding-box correctness tests for TwistExtrude3D and ScaleTwistExtrude3D.
//
// Two properties to verify:
//
//  1. Containment — the bbox must contain the surface. If the renderer
//     iterates only inside the bbox and the surface extends past it, the
//     mesh has holes. Operationally: every point just *outside* the bbox
//     must have SDF > 0 (no interior of the shape beyond the bbox).
//
//  2. Tightness — the bbox must be reasonably close to the surface, since
//     callers use BoundingBox() for placement and a phantom over-padded
//     region misposition objects. Operationally: each face of the bbox
//     must have at least one sample point where SDF is near zero (the
//     surface touches the face).
//
// The previous TwistExtrude3D bbox always returned the bb.Max.Length()
// disk regardless of twist. That over-sized for partial twists (the
// tightness check would fail) AND under-sized when bb.Min was farther
// from origin than bb.Max (the containment check would fail).
//
// Why no render-based watertight test: the deadsy MarchingCubes
// implementation drops triangles with two coincident vertices
// (`Degenerate(0)` filter in render/march3.go). The surface lands
// exactly on a cube corner at the extrusion's flat top/bottom caps
// (z = ±height/2) and at curved 2D shape boundaries, so adjacent cubes
// produce mismatched triangle sets and the rendered mesh shows boundary
// edges *unrelated to bbox correctness*. The high-density sampling
// containment test below catches the bbox bugs we care about; render-
// based watertightness would conflate them with the MC filter artifact.

//-----------------------------------------------------------------------------

// assertBboxContainsSurface samples points just outside each face of the
// bbox and verifies the SDF is positive there — no interior of the shape
// reaches past the bbox.
func assertBboxContainsSurface(t *testing.T, name string, s SDF3, sampleN int) {
	t.Helper()
	bb := s.BoundingBox()
	const margin = 1e-3 // 1 micron — small enough to land "just outside"
	type face struct {
		axis int
		dir  int // -1 = min face, +1 = max face
	}
	faces := []face{
		{0, +1}, {0, -1}, {1, +1}, {1, -1}, {2, +1}, {2, -1},
	}
	for _, f := range faces {
		var coord float64
		switch {
		case f.axis == 0 && f.dir > 0:
			coord = bb.Max.X + margin
		case f.axis == 0:
			coord = bb.Min.X - margin
		case f.axis == 1 && f.dir > 0:
			coord = bb.Max.Y + margin
		case f.axis == 1:
			coord = bb.Min.Y - margin
		case f.axis == 2 && f.dir > 0:
			coord = bb.Max.Z + margin
		default:
			coord = bb.Min.Z - margin
		}
		uA, vA := otherTwoAxes(f.axis)
		axMin := [3]float64{bb.Min.X, bb.Min.Y, bb.Min.Z}
		axMax := [3]float64{bb.Max.X, bb.Max.Y, bb.Max.Z}
		uMin, uMax := axMin[uA], axMax[uA]
		vMin, vMax := axMin[vA], axMax[vA]
		var worst float64
		var worstAt v3.Vec
		first := true
		for i := 0; i <= sampleN; i++ {
			u := uMin + (uMax-uMin)*float64(i)/float64(sampleN)
			for j := 0; j <= sampleN; j++ {
				v := vMin + (vMax-vMin)*float64(j)/float64(sampleN)
				p := assemblePoint(f.axis, coord, uA, u, vA, v)
				d := s.Evaluate(p)
				if first || d < worst {
					worst = d
					worstAt = p
					first = false
				}
			}
		}
		// Must be > 0 (outside the surface). Allow a tiny tolerance for
		// finite-difference round-off in the SDF.
		if worst < -1e-6 {
			t.Errorf("%s: face axis=%d dir=%+d: SDF=%v at %v outside bbox — surface extends past bbox (containment failed)",
				name, f.axis, f.dir, worst, worstAt)
		}
	}
}

// assertBboxNotOverlyLoose verifies the bbox is reasonably tight on each
// face — sampling on the face itself, the minimum SDF must be ≤ tol. If
// the minimum is much larger, the surface is far from this face and the
// bbox could be tightened (callers reading BoundingBox() for placement
// would misposition by min(SDF on face) units).
func assertBboxNotOverlyLoose(t *testing.T, name string, s SDF3, sampleN int, tol float64) {
	t.Helper()
	bb := s.BoundingBox()
	type face struct {
		axis int
		dir  int
	}
	faces := []face{
		{0, +1}, {0, -1}, {1, +1}, {1, -1}, {2, +1}, {2, -1},
	}
	for _, f := range faces {
		var coord float64
		switch {
		case f.axis == 0 && f.dir > 0:
			coord = bb.Max.X
		case f.axis == 0:
			coord = bb.Min.X
		case f.axis == 1 && f.dir > 0:
			coord = bb.Max.Y
		case f.axis == 1:
			coord = bb.Min.Y
		case f.axis == 2 && f.dir > 0:
			coord = bb.Max.Z
		default:
			coord = bb.Min.Z
		}
		uA, vA := otherTwoAxes(f.axis)
		axMin := [3]float64{bb.Min.X, bb.Min.Y, bb.Min.Z}
		axMax := [3]float64{bb.Max.X, bb.Max.Y, bb.Max.Z}
		uMin, uMax := axMin[uA], axMax[uA]
		vMin, vMax := axMin[vA], axMax[vA]
		minD := math.Inf(1)
		for i := 0; i <= sampleN; i++ {
			u := uMin + (uMax-uMin)*float64(i)/float64(sampleN)
			for j := 0; j <= sampleN; j++ {
				v := vMin + (vMax-vMin)*float64(j)/float64(sampleN)
				p := assemblePoint(f.axis, coord, uA, u, vA, v)
				if d := s.Evaluate(p); d < minD {
					minD = d
				}
			}
		}
		if minD > tol {
			t.Errorf("%s: face axis=%d dir=%+d: min SDF on face = %v (tol=%v) — bbox is over-padded by ~%v",
				name, f.axis, f.dir, minD, tol, minD)
		}
	}
}

func otherTwoAxes(axis int) (int, int) {
	switch axis {
	case 0:
		return 1, 2
	case 1:
		return 0, 2
	default:
		return 0, 1
	}
}

func assemblePoint(axisN int, coordN float64, uA int, uVal float64, vA int, vVal float64) v3.Vec {
	var p v3.Vec
	for ax, val := range map[int]float64{axisN: coordN, uA: uVal, vA: vVal} {
		switch ax {
		case 0:
			p.X = val
		case 1:
			p.Y = val
		case 2:
			p.Z = val
		}
	}
	return p
}

//-----------------------------------------------------------------------------

// twistCases is the shared shape × twist matrix for the bbox checks.
// Each shape exercises a different bbox-vs-origin configuration so the
// "old code used bb.Max.Length()" bug surfaces at least once.
//
// fillsBbox=true means the 2D shape touches every face of its own bbox
// (rectangles, polygons that hug the corners). For these the swept bbox
// can be tight to the surface. fillsBbox=false (circles, off-corner
// triangles) — the bbox helper rotates the *bbox*, not the shape, so the
// computed bbox is necessarily looser than the actual swept volume; we
// still verify containment but skip tightness.
type twistCase struct {
	name      string
	make2D    func() SDF2
	height    float64
	twist     float64
	fillsBbox bool
}

func twistCases() []twistCase {
	// Symmetric square — bb.Min and bb.Max are mirror images.
	square := func() SDF2 { return Box2D(v2.Vec{X: 4, Y: 4}, 0) }
	// Off-center rectangle — bb.Min is *farther* from origin than bb.Max.
	// This is the configuration the old code under-sized.
	offCenter := func() SDF2 {
		s := Box2D(v2.Vec{X: 2, Y: 2}, 0)
		return Transform2D(s, Translate2d(v2.Vec{X: -3, Y: -3}))
	}
	// Non-square aspect — exercises the corner-radius vs. axis-extent
	// distinction.
	thinRect := func() SDF2 { return Box2D(v2.Vec{X: 6, Y: 1}, 0) }
	// Off-axis thin rect — long thin shape translated away from origin,
	// stresses the "bb.Min farther from origin than bb.Max" pattern in
	// the most extreme way (long shape, off the rotation axis).
	offAxisThin := func() SDF2 {
		s := Box2D(v2.Vec{X: 8, Y: 0.5}, 0)
		return Transform2D(s, Translate2d(v2.Vec{X: 0, Y: -4}))
	}
	// Circle — doesn't fill its bbox. Tests that the rotation envelope
	// of the (square) bbox correctly contains the (round) actual shape;
	// tightness is necessarily looser here since the bbox itself is loose.
	circle := func() SDF2 {
		s, err := Circle2D(2)
		if err != nil {
			panic(err)
		}
		return s
	}
	// Triangle — irregular shape that doesn't fill its bbox. Exercises
	// the same case as circle but with a non-symmetric corner geometry,
	// so a buggy "use only bb.Max" formula would clip even the bbox itself.
	triangle := func() SDF2 {
		p := NewPolygon()
		p.Add(-3, -2)
		p.Add(3, -2)
		p.Add(0, 3)
		s, err := Polygon2D(p.Vertices())
		if err != nil {
			panic(err)
		}
		return s
	}
	cases := []twistCase{}
	for _, sh := range []struct {
		n     string
		f     func() SDF2
		fills bool
	}{
		{"sym-square", square, true},
		{"off-center", offCenter, true},
		{"thin-rect", thinRect, true},
		{"off-axis-thin", offAxisThin, true},
		{"circle", circle, false},
		{"triangle", triangle, false},
	} {
		for _, tw := range []struct {
			n string
			v float64
		}{
			{"twist=0", 0},
			{"twist=30deg", DtoR(30)},
			{"twist=90deg", DtoR(90)},
			{"twist=180deg", DtoR(180)},
			{"twist=270deg", DtoR(270)},
			{"twist=full", 2 * Pi},
			{"twist=2.5turn", 5 * Pi},
			{"twist=neg45", -DtoR(45)},
		} {
			cases = append(cases, twistCase{
				name:      sh.n + "/" + tw.n,
				make2D:    sh.f,
				height:    5,
				twist:     tw.v,
				fillsBbox: sh.fills,
			})
		}
	}
	return cases
}

func Test_TwistExtrude3D_BboxContainsSurface(t *testing.T) {
	// 128 samples per face axis = 16641 sample points per face, 6 faces.
	// Catches thin protrusions that a coarser grid would miss between
	// sample lines. Even at this density the test is sub-second.
	for _, c := range twistCases() {
		t.Run(c.name, func(t *testing.T) {
			s := TwistExtrude3D(c.make2D(), c.height, c.twist)
			assertBboxContainsSurface(t, c.name, s, 128)
		})
	}
}

func Test_TwistExtrude3D_BboxNotOverlyLoose(t *testing.T) {
	for _, c := range twistCases() {
		if !c.fillsBbox {
			// For shapes that don't fill their 2D bbox (circle, irregular
			// triangle), the helper rotates the bbox not the shape, so a
			// tightness check would fail by construction. Containment is
			// still verified for these cases by the other test.
			continue
		}
		t.Run(c.name, func(t *testing.T) {
			s := TwistExtrude3D(c.make2D(), c.height, c.twist)
			// Tolerance scaled to the bbox size — anything more than 5%
			// of the smallest bbox dimension is "phantom region".
			bb := s.BoundingBox()
			size := bb.Size()
			minSize := math.Min(size.X, math.Min(size.Y, size.Z))
			assertBboxNotOverlyLoose(t, c.name, s, 32, minSize*0.05)
		})
	}
}

func Test_ScaleTwistExtrude3D_BboxContainsSurface(t *testing.T) {
	for _, c := range twistCases() {
		t.Run(c.name, func(t *testing.T) {
			s := ScaleTwistExtrude3D(c.make2D(), c.height, c.twist, v2.Vec{X: 0.6, Y: 0.6})
			assertBboxContainsSurface(t, c.name, s, 128)
		})
	}
}

// Non-uniform scale + twist exercises the rotate-then-scale ordering of
// the inverse map: a "scale-then-rotate" envelope under-sizes the bbox
// because rotation can swing the input shape's long axis into the
// dimension getting the larger forward scale.
func Test_ScaleTwistExtrude3D_BboxContainsSurface_NonUniformScale(t *testing.T) {
	scales := []v2.Vec{
		{X: 1.5, Y: 2.5}, // grow, anisotropic — was failing
		{X: 2.5, Y: 1.5}, // grow, anisotropic, axis-swapped
		{X: 0.4, Y: 0.7}, // shrink, anisotropic
		{X: 2.0, Y: 0.5}, // grow one axis, shrink the other
	}
	for _, c := range twistCases() {
		for _, sc := range scales {
			name := c.name + "_scale_" + ftoa(sc.X) + "x" + ftoa(sc.Y)
			t.Run(name, func(t *testing.T) {
				s := ScaleTwistExtrude3D(c.make2D(), c.height, c.twist, sc)
				assertBboxContainsSurface(t, name, s, 128)
			})
		}
	}
}

func ftoa(f float64) string {
	return strconv.FormatFloat(f, 'g', -1, 64)
}

//-----------------------------------------------------------------------------

// Specific closed-form checks for twistedBoundingBox2.

func Test_TwistedBoundingBox2_FullRotation(t *testing.T) {
	// Square centered on origin, |twist| ≥ 2π → centered disk of radius √2·2.
	bb := Box2{Min: v2.Vec{X: -2, Y: -2}, Max: v2.Vec{X: 2, Y: 2}}
	rExpected := math.Sqrt(8) // corner at (2, 2), radius 2√2
	for _, twist := range []float64{2 * Pi, 3 * Pi, -4 * Pi} {
		got := twistedBoundingBox2(bb, twist)
		if math.Abs(got.Max.X-rExpected) > 1e-9 || math.Abs(got.Max.Y-rExpected) > 1e-9 ||
			math.Abs(got.Min.X+rExpected) > 1e-9 || math.Abs(got.Min.Y+rExpected) > 1e-9 {
			t.Errorf("twist=%v: got %+v, want disk of radius %v", twist, got, rExpected)
		}
	}
}

func Test_TwistedBoundingBox2_ZeroTwist(t *testing.T) {
	// |twist| = 0 → identity (the original bbox), modulo trig round-off.
	bb := Box2{Min: v2.Vec{X: -3, Y: -1}, Max: v2.Vec{X: 5, Y: 2}}
	got := twistedBoundingBox2(bb, 0)
	const tol = 1e-12
	if math.Abs(got.Min.X-bb.Min.X) > tol || math.Abs(got.Min.Y-bb.Min.Y) > tol ||
		math.Abs(got.Max.X-bb.Max.X) > tol || math.Abs(got.Max.Y-bb.Max.Y) > tol {
		t.Errorf("twist=0: got %+v, want ≈%+v", got, bb)
	}
}

func Test_TwistedBoundingBox2_NegativeQuadrant(t *testing.T) {
	// Off-center rectangle in the negative quadrant: bb.Min farther from
	// origin than bb.Max. This is the case the old code under-sized.
	bb := Box2{Min: v2.Vec{X: -5, Y: -5}, Max: v2.Vec{X: -1, Y: -1}}
	// Full rotation → disk of radius √(5² + 5²) = 5√2.
	got := twistedBoundingBox2(bb, 2*Pi)
	r := 5 * math.Sqrt(2)
	if math.Abs(got.Max.X-r) > 1e-9 {
		t.Errorf("got Max.X=%v, want %v", got.Max.X, r)
	}
	// Sanity: the old buggy formula bb.Max.Length() = √2 ≪ 5√2.
	if got.Max.X < 5 {
		t.Errorf("under-sized bbox: Max.X=%v should be ≥ 5", got.Max.X)
	}
}

func Test_TwistedBoundingBox2_PartialTwistTighterThanFull(t *testing.T) {
	// A 45° twist of an off-center rectangle should be strictly smaller
	// than the full-rotation disk.
	bb := Box2{Min: v2.Vec{X: 1, Y: 1}, Max: v2.Vec{X: 3, Y: 3}}
	full := twistedBoundingBox2(bb, 2*Pi)
	partial := twistedBoundingBox2(bb, DtoR(45))
	pSize := partial.Size()
	fSize := full.Size()
	if pSize.X >= fSize.X || pSize.Y >= fSize.Y {
		t.Errorf("partial twist not tighter: partial=%+v full=%+v", partial, full)
	}
}
