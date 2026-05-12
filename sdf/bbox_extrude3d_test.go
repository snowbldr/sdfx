package sdf

import (
	"testing"

	v2 "github.com/snowbldr/sdfx/vec/v2"
)

// Bounding-box correctness audit for the non-twist/non-scale 3D extrusion
// constructors: Extrude3D, ExtrudeRounded3D, Loft3D.
//
// All tests use the shared helper assertBboxContainsSurface (extrude_test.go)
// which samples points just outside every bbox face and fails if any one of
// them is inside the SDF surface (SDF < -1e-6). A failure means the
// constructor's bbox under-sizes — renderers walking only the bbox will
// produce holed meshes.
//
// Per task: write tests that EXPOSE bugs, do not fix anything. sampleN = 64.

//-----------------------------------------------------------------------------
// Shared 2D shape builders. Mirror the styles used in extrude_test.go but
// kept local so this file is self-contained and so we can vary parameters.

// e3dSquare returns a centered axis-aligned square of side `side`.
func e3dSquare(side float64) SDF2 { return Box2D(v2.Vec{X: side, Y: side}, 0) }

// e3dRect returns a centered axis-aligned rectangle of size (w,h).
func e3dRect(w, h float64) SDF2 { return Box2D(v2.Vec{X: w, Y: h}, 0) }

// e3dOffCenterBox returns a small box translated entirely into the negative
// quadrant (exposes "bbox is centered at origin" bugs).
func e3dOffCenterBox() SDF2 {
	s := Box2D(v2.Vec{X: 2, Y: 2}, 0)
	return Transform2D(s, Translate2d(v2.Vec{X: -5, Y: -5}))
}

// e3dCircle returns a circle of radius r at origin.
func e3dCircle(r float64) SDF2 {
	s, err := Circle2D(r)
	if err != nil {
		panic(err)
	}
	return s
}

// e3dOffCenterCircle returns a circle of radius r translated to (cx,cy).
func e3dOffCenterCircle(r, cx, cy float64) SDF2 {
	s := e3dCircle(r)
	return Transform2D(s, Translate2d(v2.Vec{X: cx, Y: cy}))
}

// e3dTriangle returns an irregular triangle that does not fill its bbox.
func e3dTriangle() SDF2 {
	p := NewPolygon()
	p.Add(-3, -2)
	p.Add(3, -2)
	p.Add(0, 4)
	s, err := Polygon2D(p.Vertices())
	if err != nil {
		panic(err)
	}
	return s
}

// e3dOffCenterTriangle is a triangle entirely in the negative quadrant.
func e3dOffCenterTriangle() SDF2 {
	p := NewPolygon()
	p.Add(-6, -6)
	p.Add(-2, -6)
	p.Add(-4, -1)
	s, err := Polygon2D(p.Vertices())
	if err != nil {
		panic(err)
	}
	return s
}

//-----------------------------------------------------------------------------
// Extrude3D — plain linear extrusion.

func Test_Extrude3D_Bbox_PlainCases(t *testing.T) {
	cases := []struct {
		name   string
		make2D func() SDF2
		height float64
	}{
		// Baseline — centered square, normal height.
		{"square_normal", func() SDF2 { return e3dSquare(2) }, 4},
		// Tiny height — well below 1 unit, exercises near-flat extrusions.
		{"square_tiny_height", func() SDF2 { return e3dSquare(2) }, 1e-3},
		// Huge height — orders of magnitude larger than xy extent.
		{"square_huge_height", func() SDF2 { return e3dSquare(2) }, 1e6},
		// Height ~= 0. Extrude3D doesn't validate, so the bbox should still
		// contain the (degenerate) surface.
		{"square_zero_height", func() SDF2 { return e3dSquare(2) }, 0},
		// Off-center input — entirely in the negative quadrant.
		{"off_center_box", e3dOffCenterBox, 3},
		// Asymmetric input — long thin rectangle far off-center.
		{"asymmetric_rect_off_center", func() SDF2 {
			s := e3dRect(20, 0.5)
			return Transform2D(s, Translate2d(v2.Vec{X: 15, Y: -8}))
		}, 2},
		// Irregular polygon — triangle.
		{"triangle", e3dTriangle, 3},
		// Off-center irregular polygon.
		{"triangle_off_center", e3dOffCenterTriangle, 2},
		// Circle — doesn't fill its bbox; still must be contained.
		{"circle", func() SDF2 { return e3dCircle(2) }, 4},
		// Extreme aspect ratio rectangle at origin.
		{"thin_long_rect", func() SDF2 { return e3dRect(20, 0.5) }, 1.5},
		// Negative height — Extrude3D simply halves it; behavior should
		// still produce a bbox that contains whatever surface exists.
		{"negative_height", func() SDF2 { return e3dSquare(2) }, -3},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s := Extrude3D(c.make2D(), c.height)
			assertBboxContainsSurface(t, c.name, s, 64)
		})
	}
}

//-----------------------------------------------------------------------------
// ExtrudeRounded3D — extrusion with rounded top/bottom edges.

func Test_Extrude3D_BboxRounded_VariousRounds(t *testing.T) {
	cases := []struct {
		name   string
		make2D func() SDF2
		height float64
		round  float64
	}{
		// round == 0 — function defers to Extrude3D internally; verify it
		// still produces a containing bbox.
		{"square_round_zero", func() SDF2 { return e3dSquare(4) }, 4, 0},
		// Round at maximum allowed (height/2).
		{"square_round_at_max", func() SDF2 { return e3dSquare(4) }, 4, 2},
		// Round just below max — exercises near-spherical end caps.
		{"square_round_near_max", func() SDF2 { return e3dSquare(4) }, 4, 1.999},
		// Round much smaller than max — typical thin chamfer case.
		{"square_round_thin", func() SDF2 { return e3dSquare(4) }, 4, 0.01},
		// Off-center input with significant round — bbox should follow
		// the translated shape AND the outward round expansion.
		{"off_center_box_with_round", e3dOffCenterBox, 3, 0.7},
		// Thin (tall, narrow) input with thick round — round is larger
		// than the shape's narrow dimension. Bbox must accommodate the
		// spherical caps that wrap entirely around the thin sliver.
		{"thin_rect_thick_round", func() SDF2 { return e3dRect(20, 0.2) }, 4, 1.9},
		// Off-center circle with max round.
		{"off_center_circle_max_round", func() SDF2 { return e3dOffCenterCircle(1, -3, -3) }, 2, 1},
		// Irregular polygon with sizable round.
		{"triangle_with_round", e3dTriangle, 3, 0.8},
		// Huge height with small round — bbox along z must be huge plus
		// 2*round.
		{"huge_height_tiny_round", func() SDF2 { return e3dSquare(2) }, 1000, 0.1},
		// Very small height (just above 2*round) — minimum legal height.
		{"min_legal_height", func() SDF2 { return e3dSquare(2) }, 2.0, 0.999},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s, err := ExtrudeRounded3D(c.make2D(), c.height, c.round)
			if err != nil {
				t.Fatal(err)
			}
			assertBboxContainsSurface(t, c.name, s, 64)
		})
	}
}

// Test_Extrude3D_BboxRounded_ZeroRoundMatchesPlain verifies that the
// special-case path (round == 0 → Extrude3D) returns the same bbox as the
// plain Extrude3D constructor. A drift here would indicate the rounded path
// is dispatched on an incorrect predicate.
func Test_Extrude3D_BboxRounded_ZeroRoundMatchesPlain(t *testing.T) {
	shape := e3dSquare(3)
	plain := Extrude3D(shape, 5)
	rounded, err := ExtrudeRounded3D(shape, 5, 0)
	if err != nil {
		t.Fatal(err)
	}
	bp := plain.BoundingBox()
	br := rounded.BoundingBox()
	if bp.Min != br.Min || bp.Max != br.Max {
		t.Errorf("round=0 dispatch mismatch: plain=%v rounded=%v", bp, br)
	}
}

//-----------------------------------------------------------------------------
// Loft3D — z-interpolated transition between two SDF2s.

func Test_Extrude3D_BboxLoft_VariousPairs(t *testing.T) {
	cases := []struct {
		name         string
		make0, make1 func() SDF2
		height       float64
		round        float64
	}{
		// Identity loft (sdf0 == sdf1) — should match a plain extrusion's
		// xy bbox (with z grown by round on each side).
		{"identity_square", func() SDF2 { return e3dSquare(2) }, func() SDF2 { return e3dSquare(2) }, 4, 0},
		// Two circles at very different xy positions — interpolated middle
		// shape may stretch across the gap.
		{"circle_origin_to_circle_far", func() SDF2 { return e3dCircle(1) }, func() SDF2 { return e3dOffCenterCircle(1, 5, 5) }, 4, 0},
		// Very different sizes — radius 0.1 vs 5. Stresses the maxAbsDifference
		// Lipschitz estimator that influences the Lipschitz correction.
		{"tiny_to_huge_circle", func() SDF2 { return e3dCircle(0.1) }, func() SDF2 { return e3dCircle(5) }, 4, 0},
		// Mismatched topology — square vs circle.
		{"square_to_circle", func() SDF2 { return e3dSquare(3) }, func() SDF2 { return e3dCircle(1.5) }, 4, 0},
		// Maximum round (height/2).
		{"max_round_two_circles", func() SDF2 { return e3dCircle(2) }, func() SDF2 { return e3dCircle(1) }, 4, 2},
		// Minimum height (height == 2*round + epsilon).
		{"min_height_with_round", func() SDF2 { return e3dCircle(1) }, func() SDF2 { return e3dCircle(0.5) }, 2.001, 1},
		// Off-center pair — both shapes in the negative quadrant.
		{"off_center_pair", e3dOffCenterBox, func() SDF2 { return e3dOffCenterCircle(1.5, -5, -5) }, 3, 0.3},
		// One off-center, one at origin — bbox must span both.
		{"asym_centers", func() SDF2 { return e3dCircle(1) }, func() SDF2 { return e3dOffCenterCircle(1, -4, 3) }, 3, 0.2},
		// Triangle to circle — irregular topology, neither fills its bbox.
		{"triangle_to_circle", e3dTriangle, func() SDF2 { return e3dCircle(2) }, 4, 0.5},
		// Extreme aspect ratio shapes.
		{"thin_to_thick_rect", func() SDF2 { return e3dRect(20, 0.5) }, func() SDF2 { return e3dRect(1, 20) }, 5, 0},
		// Very tall extrusion (huge height) — exposes z-axis bbox issues.
		{"huge_height_loft", func() SDF2 { return e3dCircle(2) }, func() SDF2 { return e3dCircle(0.5) }, 1e4, 0.5},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s, err := Loft3D(c.make0(), c.make1(), c.height, c.round)
			if err != nil {
				t.Fatal(err)
			}
			assertBboxContainsSurface(t, c.name, s, 64)
		})
	}
}

// Test_Extrude3D_BboxLoft_IdentityEqualsPlain verifies that lofting a shape
// to itself with round=0 produces the same xy bbox as a plain Extrude3D
// (z bbox is unchanged when round=0). Mismatch would suggest Loft3D's bbox
// pads spuriously when both inputs are identical.
func Test_Extrude3D_BboxLoft_IdentityEqualsPlain(t *testing.T) {
	shape := e3dSquare(3)
	plain := Extrude3D(shape, 5)
	loft, err := Loft3D(shape, shape, 5, 0)
	if err != nil {
		t.Fatal(err)
	}
	bp := plain.BoundingBox()
	bl := loft.BoundingBox()
	if bp.Min != bl.Min || bp.Max != bl.Max {
		t.Errorf("loft identity bbox mismatch: plain=%v loft=%v", bp, bl)
	}
}
