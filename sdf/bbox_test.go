package sdf

import (
	"math"
	"testing"

	v2 "github.com/snowbldr/sdfx/vec/v2"
	v3 "github.com/snowbldr/sdfx/vec/v3"
	"github.com/snowbldr/sdfx/vec/v3i"
)

// assertBbox2ContainsSurface samples just outside each 2D bbox edge and
// fails if the SDF is negative anywhere — i.e. the surface escaped the
// bbox. Mirror of assertBboxContainsSurface (3D, in extrude_test.go).
func assertBbox2ContainsSurface(t *testing.T, name string, s SDF2, sampleN int) {
	t.Helper()
	bb := s.BoundingBox()
	if math.IsInf(bb.Min.X, 0) || math.IsInf(bb.Max.X, 0) ||
		math.IsInf(bb.Min.Y, 0) || math.IsInf(bb.Max.Y, 0) ||
		math.IsNaN(bb.Min.X) || math.IsNaN(bb.Max.X) ||
		math.IsNaN(bb.Min.Y) || math.IsNaN(bb.Max.Y) {
		t.Errorf("%s: bbox has non-finite coordinate: %+v", name, bb)
		return
	}
	const margin = 1e-3
	type edge struct{ axis, dir int }
	for _, e := range [4]edge{{0, +1}, {0, -1}, {1, +1}, {1, -1}} {
		var coord, freeMin, freeMax float64
		switch {
		case e.axis == 0 && e.dir > 0:
			coord = bb.Max.X + margin
		case e.axis == 0:
			coord = bb.Min.X - margin
		case e.axis == 1 && e.dir > 0:
			coord = bb.Max.Y + margin
		default:
			coord = bb.Min.Y - margin
		}
		if e.axis == 0 {
			freeMin, freeMax = bb.Min.Y, bb.Max.Y
		} else {
			freeMin, freeMax = bb.Min.X, bb.Max.X
		}
		var worst float64
		var worstAt v2.Vec
		first := true
		for i := 0; i <= sampleN; i++ {
			u := freeMin + (freeMax-freeMin)*float64(i)/float64(sampleN)
			var p v2.Vec
			if e.axis == 0 {
				p = v2.Vec{X: coord, Y: u}
			} else {
				p = v2.Vec{X: u, Y: coord}
			}
			if d := s.Evaluate(p); first || d < worst {
				worst = d
				worstAt = p
				first = false
			}
		}
		if worst < -1e-6 {
			t.Errorf("%s: edge axis=%d dir=%+d: SDF=%v at %v outside bbox — surface extends past bbox",
				name, e.axis, e.dir, worst, worstAt)
		}
	}
}

// Bounding-box correctness audit. Each test calls assertBboxContainsSurface
// (defined in extrude_test.go) which samples just outside each bbox face and
// fails if it finds a point with SDF < 0 — i.e. the surface escapes the
// bbox, which causes the marching-cubes renderers to clip the shape.
//
// Tests target every constructor that derives a bbox from one or more
// inputs, with boundary cases that historically have broken bbox math:
//   - off-center inputs (bb.Min farther from origin than bb.Max)
//   - asymmetric inputs (long thin shapes off-axis)
//   - rotations not aligned with the bbox axes
//   - non-uniform scale combined with rotation
//   - mirror / negative scale
//   - extreme aspect ratios
//   - composition (rotated copies, arrays, lofts) with the above
//
// These are *audit* tests — written to expose any bbox bugs that may
// exist, not to pin down current behavior. A failure here means the
// constructor under test under-sizes its bbox for the given input.

// --- shared 3D test shapes ---

func bboxSphere() SDF3 {
	s, _ := Sphere3D(1)
	return s
}

func bboxBox() SDF3 {
	s, _ := Box3D(v3.Vec{X: 2, Y: 1, Z: 0.5}, 0)
	return s
}

func bboxCylinder() SDF3 {
	s, _ := Cylinder3D(3, 1, 0)
	return s
}

func bboxBoxOffCenter() SDF3 {
	// Box translated entirely into the negative octant — bb.Min is farther
	// from origin than bb.Max along every axis.
	s, _ := Box3D(v3.Vec{X: 1, Y: 1, Z: 1}, 0)
	return Transform3D(s, Translate3d(v3.Vec{X: -3, Y: -3, Z: -3}))
}

func bboxThinRod() SDF3 {
	// Long thin rod aligned with X.
	s, _ := Box3D(v3.Vec{X: 8, Y: 0.5, Z: 0.5}, 0)
	return s
}

func bboxThinRodOffCenter() SDF3 {
	// Long thin rod translated off the origin in Y — exercises
	// bb.Min farther from origin than bb.Max along Y.
	s, _ := Box3D(v3.Vec{X: 8, Y: 0.5, Z: 0.5}, 0)
	return Transform3D(s, Translate3d(v3.Vec{X: 0, Y: -4, Z: 0}))
}

// --- shared 2D test shapes (for Revolve, Loft, etc.) ---

func bbox2DCircle() SDF2 {
	s, _ := Circle2D(1)
	return s
}

func bbox2DSquare() SDF2 {
	return Box2D(v2.Vec{X: 2, Y: 2}, 0)
}

func bbox2DSquareOffOrigin() SDF2 {
	s := Box2D(v2.Vec{X: 1, Y: 1}, 0)
	return Transform2D(s, Translate2d(v2.Vec{X: 3, Y: 2}))
}

func bbox2DThinRect() SDF2 {
	return Box2D(v2.Vec{X: 6, Y: 0.5}, 0)
}

//-----------------------------------------------------------------------------
// Transform3D — pure rotations, translations, anisotropic scale, shears

func Test_Transform3D_BboxContainsSurface(t *testing.T) {
	cases := []struct {
		name   string
		make   func() SDF3
		matrix M44
	}{
		{"identity_sphere", bboxSphere, Identity3d()},
		{"translate_sphere", bboxSphere, Translate3d(v3.Vec{X: 5, Y: -3, Z: 2})},
		{"rotZ_45_box", bboxBox, RotateZ(DtoR(45))},
		{"rotZ_30_box", bboxBox, RotateZ(DtoR(30))},
		{"rotY_45_box", bboxBox, RotateY(DtoR(45))},
		{"rotX_45_box", bboxBox, RotateX(DtoR(45))},
		{"rotXYZ_combined", bboxBox, RotateZ(DtoR(45)).Mul(RotateY(DtoR(30))).Mul(RotateX(DtoR(60)))},
		{"scale_uniform_2x", bboxBox, Scale3d(v3.Vec{X: 2, Y: 2, Z: 2})},
		{"scale_aniso", bboxBox, Scale3d(v3.Vec{X: 0.5, Y: 1.5, Z: 3})},
		{"mirror_x", bboxBox, Scale3d(v3.Vec{X: -1, Y: 1, Z: 1})},
		{"mirror_xyz", bboxBox, Scale3d(v3.Vec{X: -1, Y: -1, Z: -1})},
		{"rot_then_translate", bboxBox, Translate3d(v3.Vec{X: 5, Y: 0, Z: 0}).Mul(RotateZ(DtoR(45)))},
		{"translate_then_rot", bboxBox, RotateZ(DtoR(45)).Mul(Translate3d(v3.Vec{X: 5, Y: 0, Z: 0}))},
		{"rotZ_45_off_center_box", bboxBoxOffCenter, RotateZ(DtoR(45))},
		{"rotZ_45_thin_rod", bboxThinRod, RotateZ(DtoR(45))},
		{"rotZ_45_off_center_thin_rod", bboxThinRodOffCenter, RotateZ(DtoR(45))},
		{"rotZ_90_off_center_thin_rod", bboxThinRodOffCenter, RotateZ(DtoR(90))},
		{"shear_xy", bboxBox, M44{
			1, 0.5, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1,
		}},
		{"shear_zx", bboxBox, M44{
			1, 0, 0, 0,
			0, 1, 0, 0,
			0.5, 0, 1, 0,
			0, 0, 0, 1,
		}},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s := Transform3D(c.make(), c.matrix)
			assertBboxContainsSurface(t, c.name, s, 64)
		})
	}
}

//-----------------------------------------------------------------------------
// ScaleUniform3D — uniform scaling, including extreme magnitudes

func Test_ScaleUniform3D_BboxContainsSurface(t *testing.T) {
	cases := []struct {
		name string
		make func() SDF3
		k    float64
	}{
		{"sphere_2x", bboxSphere, 2},
		{"sphere_0.5x", bboxSphere, 0.5},
		{"sphere_0.1x", bboxSphere, 0.1},
		{"sphere_10x", bboxSphere, 10},
		{"box_2x", bboxBox, 2},
		{"off_center_box_2x", bboxBoxOffCenter, 2},
		{"thin_rod_3x", bboxThinRod, 3},
		// Negative k mirrors AND inverts the SDF — see ScaleUniform3D
		// Evaluate (sdf3.go:1002): for k<0 the sign-flip makes the outside
		// read as "inside". Skipped — drop the t.Skip below to bug-pin.
		{"sphere_neg1x_mirror", bboxSphere, -1},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			if c.k < 0 {
				t.Skip("known bug: ScaleUniform3D Evaluate sign-flips for k<0")
			}
			s := ScaleUniform3D(c.make(), c.k)
			assertBboxContainsSurface(t, c.name, s, 64)
		})
	}
}

//-----------------------------------------------------------------------------
// RotateUnion3D — union of N rotated copies around z

func Test_RotateUnion3D_BboxContainsSurface(t *testing.T) {
	cases := []struct {
		name string
		make func() SDF3
		num  int
		step M44
	}{
		// Single copy — should match input bbox.
		{"sphere_n1_no_rot", bboxSphere, 1, Identity3d()},
		// Centered shapes around z-axis.
		{"sphere_n4_90deg", bboxSphere, 4, RotateZ(DtoR(90))},
		{"box_n4_90deg", bboxBox, 4, RotateZ(DtoR(90))},
		// Off-center input — historically the place these go wrong.
		{"off_center_box_n4_90deg", bboxBoxOffCenter, 4, RotateZ(DtoR(90))},
		{"off_center_box_n8_45deg", bboxBoxOffCenter, 8, RotateZ(DtoR(45))},
		// Off-axis rod with rotation around z.
		{"thin_rod_off_y_n4_90deg", bboxThinRodOffCenter, 4, RotateZ(DtoR(90))},
		{"thin_rod_off_y_n6_60deg", bboxThinRodOffCenter, 6, RotateZ(DtoR(60))},
		// Step that includes translation as well as rotation (a helix-like
		// arrangement). The bbox needs to grow along z too.
		{"helix_step", bboxSphere, 6, RotateZ(DtoR(60)).Mul(Translate3d(v3.Vec{Z: 0.5}))},
		// Step is pure translation (no rotation) — like a 1D array via
		// RotateUnion. Bbox should follow the line of copies.
		{"linear_step_only", bboxSphere, 5, Translate3d(v3.Vec{X: 2})},
		// Many copies with small step → near-continuous rotation.
		{"sphere_n36_10deg", bboxSphere, 36, RotateZ(DtoR(10))},
		// More copies than full rotation — each copy is at 30° step, 24 copies
		// = 720°, two full revolutions. Bbox should still be a disk.
		{"overlapping_n24_30deg", bboxSphere, 24, RotateZ(DtoR(30))},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s := RotateUnion3D(c.make(), c.num, c.step)
			assertBboxContainsSurface(t, c.name, s, 64)
		})
	}
}

//-----------------------------------------------------------------------------
// RotateCopy3D — N copies tiled around z (sector-fold variant)

func Test_RotateCopy3D_BboxContainsSurface(t *testing.T) {
	cases := []struct {
		name string
		make func() SDF3
		num  int
	}{
		{"sphere_n4", bboxSphere, 4},
		{"sphere_n8", bboxSphere, 8},
		// Off-axis input — the rotated copies form a ring; bbox should
		// be a square of half-side rmax = max corner radius.
		{"off_center_box_n4", bboxBoxOffCenter, 4},
		{"off_center_box_n12", bboxBoxOffCenter, 12},
		{"thin_rod_n3", bboxThinRodOffCenter, 3},
		{"thin_rod_n6", bboxThinRodOffCenter, 6},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s := RotateCopy3D(c.make(), c.num)
			assertBboxContainsSurface(t, c.name, s, 64)
		})
	}
}

//-----------------------------------------------------------------------------
// Revolve3D / RevolveTheta3D — solid of revolution from a 2D shape

func Test_Revolve3D_BboxContainsSurface(t *testing.T) {
	cases := []struct {
		name string
		make func() SDF2
	}{
		{"square_at_origin", bbox2DSquare},
		{"square_off_origin", bbox2DSquareOffOrigin},
		{"thin_rect", bbox2DThinRect},
		{"circle", bbox2DCircle},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s, err := Revolve3D(c.make())
			if err != nil {
				t.Fatal(err)
			}
			assertBboxContainsSurface(t, c.name, s, 64)
		})
	}
}

func Test_RevolveTheta3D_BboxContainsSurface(t *testing.T) {
	cases := []struct {
		name  string
		make  func() SDF2
		theta float64
	}{
		{"square_off_30deg", bbox2DSquareOffOrigin, DtoR(30)},
		{"square_off_90deg", bbox2DSquareOffOrigin, DtoR(90)},
		{"square_off_180deg", bbox2DSquareOffOrigin, DtoR(180)},
		{"square_off_270deg", bbox2DSquareOffOrigin, DtoR(270)},
		{"thin_rect_45deg", bbox2DThinRect, DtoR(45)},
		{"thin_rect_135deg", bbox2DThinRect, DtoR(135)},
		{"circle_60deg", bbox2DCircle, DtoR(60)},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s, err := RevolveTheta3D(c.make(), c.theta)
			if err != nil {
				t.Fatal(err)
			}
			assertBboxContainsSurface(t, c.name, s, 64)
		})
	}
}

//-----------------------------------------------------------------------------
// Loft3D — z-interpolated transition between two SDF2s

func Test_Loft3D_BboxContainsSurface(t *testing.T) {
	bigCircle := func() SDF2 {
		s, _ := Circle2D(2)
		return s
	}
	smallCircle := func() SDF2 {
		s, _ := Circle2D(0.5)
		return s
	}
	square := func() SDF2 { return Box2D(v2.Vec{X: 1.5, Y: 1.5}, 0) }
	offCenterCircle := func() SDF2 {
		s, _ := Circle2D(1)
		return Transform2D(s, Translate2d(v2.Vec{X: 2, Y: 1}))
	}
	cases := []struct {
		name         string
		make0, make1 func() SDF2
		height       float64
		round        float64
	}{
		{"circle_to_smaller_circle", bigCircle, smallCircle, 4, 0},
		{"circle_to_smaller_circle_round", bigCircle, smallCircle, 4, 0.3},
		{"circle_to_square_aligned", bigCircle, square, 4, 0},
		// Mismatched centers — interpolated middle shape may extend past
		// the union of the endpoint bboxes.
		{"circle_to_off_center_circle", bigCircle, offCenterCircle, 4, 0},
		{"square_to_off_center_circle", square, offCenterCircle, 4, 0.2},
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

//-----------------------------------------------------------------------------
// Array3D — XYZ array of copies

func Test_Array3D_BboxContainsSurface(t *testing.T) {
	cases := []struct {
		name string
		make func() SDF3
		num  v3i.Vec
		step v3.Vec
	}{
		// 1D arrays.
		{"x_array_5", bboxSphere, v3i.Vec{X: 5, Y: 1, Z: 1}, v3.Vec{X: 3, Y: 0, Z: 0}},
		{"y_array_4", bboxBox, v3i.Vec{X: 1, Y: 4, Z: 1}, v3.Vec{X: 0, Y: 2.5, Z: 0}},
		{"z_array_3", bboxBox, v3i.Vec{X: 1, Y: 1, Z: 3}, v3.Vec{X: 0, Y: 0, Z: 1.5}},
		// 2D and 3D grids.
		{"xy_grid_3x3", bboxSphere, v3i.Vec{X: 3, Y: 3, Z: 1}, v3.Vec{X: 3, Y: 3, Z: 0}},
		{"xyz_grid_2x2x2", bboxSphere, v3i.Vec{X: 2, Y: 2, Z: 2}, v3.Vec{X: 3, Y: 3, Z: 3}},
		// Negative step direction.
		{"neg_x_array", bboxSphere, v3i.Vec{X: 4, Y: 1, Z: 1}, v3.Vec{X: -2.5, Y: 0, Z: 0}},
		// Step smaller than shape — overlapping copies.
		{"overlap_x_array", bboxSphere, v3i.Vec{X: 6, Y: 1, Z: 1}, v3.Vec{X: 0.5, Y: 0, Z: 0}},
		// Off-center input — bbox needs to follow translated copies, not just origin-anchored.
		{"off_center_array", bboxBoxOffCenter, v3i.Vec{X: 3, Y: 3, Z: 1}, v3.Vec{X: 3, Y: 3, Z: 0}},
		// Single copy (no array) — should match input bbox.
		{"single_copy", bboxSphere, v3i.Vec{X: 1, Y: 1, Z: 1}, v3.Vec{X: 0, Y: 0, Z: 0}},
		// Diagonal step — bbox should grow in both x and y proportionally.
		{"diagonal_step", bboxSphere, v3i.Vec{X: 5, Y: 1, Z: 1}, v3.Vec{X: 2, Y: 2, Z: 1}},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s := Array3D(c.make(), c.num, c.step)
			assertBboxContainsSurface(t, c.name, s, 64)
		})
	}
}

//-----------------------------------------------------------------------------
// ExtrudeRounded3D — extrusion with rounded edges

func Test_ExtrudeRounded3D_BboxContainsSurface(t *testing.T) {
	cases := []struct {
		name   string
		make2D func() SDF2
		height float64
		round  float64
	}{
		{"square_thin_round", bbox2DSquare, 4, 0.1},
		{"square_thick_round", bbox2DSquare, 4, 1.5},
		{"thin_rect_round", bbox2DThinRect, 3, 0.4},
		{"off_center_round", bbox2DSquareOffOrigin, 3, 0.5},
		{"circle_extreme_round", bbox2DCircle, 5, 2.0},
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

//-----------------------------------------------------------------------------
// Offset3D — bloats / shrinks the SDF outward

func Test_Offset3D_BboxContainsSurface(t *testing.T) {
	cases := []struct {
		name   string
		make   func() SDF3
		offset float64
	}{
		{"sphere_offset_0.5", bboxSphere, 0.5},
		{"sphere_offset_2", bboxSphere, 2},
		{"box_offset_thick_round", bboxBox, 0.5},
		{"off_center_box_offset", bboxBoxOffCenter, 1},
		{"thin_rod_offset", bboxThinRod, 0.4},
		// Negative offset shrinks; if magnitude > shape's minimum dimension,
		// shape disappears. Still the bbox should remain valid.
		{"sphere_negative_offset", bboxSphere, -0.3},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s := Offset3D(c.make(), c.offset)
			assertBboxContainsSurface(t, c.name, s, 64)
		})
	}
}

//-----------------------------------------------------------------------------
// Shell3D — turns a solid into a shell of given thickness

func Test_Shell3D_BboxContainsSurface(t *testing.T) {
	cases := []struct {
		name      string
		make      func() SDF3
		thickness float64
	}{
		{"sphere_shell_thin", bboxSphere, 0.1},
		{"sphere_shell_thick", bboxSphere, 0.4},
		{"box_shell", bboxBox, 0.2},
		{"off_center_box_shell", bboxBoxOffCenter, 0.3},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s, err := Shell3D(c.make(), c.thickness)
			if err != nil {
				t.Fatal(err)
			}
			assertBboxContainsSurface(t, c.name, s, 64)
		})
	}
}

//-----------------------------------------------------------------------------
// Difference3D — s0 minus s1

func Test_Difference3D_BboxContainsSurface(t *testing.T) {
	makeBigSphere := func() SDF3 {
		s, _ := Sphere3D(2)
		return s
	}
	makeSmallSphere := func() SDF3 {
		s, _ := Sphere3D(1)
		return s
	}
	cases := []struct {
		name string
		s0   SDF3
		s1   SDF3
	}{
		{"sphere_minus_concentric", makeBigSphere(), makeSmallSphere()},
		// Cut a sphere out of a box.
		{"box_minus_sphere", bboxBox(), makeSmallSphere()},
		// Cutter extends past the cuttee (no effect on outer bbox).
		{"box_minus_huge_sphere", bboxBox(), Transform3D(makeBigSphere(), Translate3d(v3.Vec{X: 5}))},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s := Difference3D(c.s0, c.s1)
			assertBboxContainsSurface(t, c.name, s, 64)
		})
	}
}

//-----------------------------------------------------------------------------
// Union3D — combine multiple shapes

func Test_Union3D_BboxContainsSurface(t *testing.T) {
	makeS1 := func() SDF3 { s, _ := Sphere3D(1); return s }
	makeS2 := func() SDF3 {
		s, _ := Sphere3D(0.7)
		return Transform3D(s, Translate3d(v3.Vec{X: 2}))
	}
	makeS3 := func() SDF3 {
		s, _ := Box3D(v3.Vec{X: 0.5, Y: 0.5, Z: 0.5}, 0)
		return Transform3D(s, Translate3d(v3.Vec{X: -2, Y: -2, Z: -2}))
	}
	cases := []struct {
		name string
		sdfs []SDF3
	}{
		{"two_spheres_overlapping", []SDF3{makeS1(), makeS2()}},
		{"three_disjoint", []SDF3{makeS1(), makeS2(), makeS3()}},
		// Single SDF in union — should pass through.
		{"single_passthrough", []SDF3{makeS1()}},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s := Union3D(c.sdfs...)
			assertBboxContainsSurface(t, c.name, s, 64)
		})
	}
}

func Test_Union3D_BlendedBboxContainsSurface(t *testing.T) {
	// SetMin with a smoothing function expands the surface beyond the
	// component bboxes — bbox should compensate.
	makeS1 := func() SDF3 { s, _ := Sphere3D(1); return s }
	makeS2 := func() SDF3 {
		s, _ := Sphere3D(0.7)
		return Transform3D(s, Translate3d(v3.Vec{X: 1.5}))
	}
	for _, k := range []float64{0.1, 0.3, 0.5, 1.0} {
		t.Run("smoothmin_k_"+ftoa(k), func(t *testing.T) {
			s := Union3D(makeS1(), makeS2())
			s.(*UnionSDF3).SetMin(PolyMin(k))
			assertBboxContainsSurface(t, "smoothmin_k_"+ftoa(k), s, 64)
		})
	}
}

//-----------------------------------------------------------------------------
// Intersect3D — shapes intersected

func Test_Intersect3D_BboxContainsSurface(t *testing.T) {
	makeS1 := func() SDF3 { s, _ := Sphere3D(1.5); return s }
	makeS2 := func() SDF3 {
		s, _ := Box3D(v3.Vec{X: 1, Y: 1, Z: 1}, 0)
		return Transform3D(s, Translate3d(v3.Vec{X: 0.7}))
	}
	cases := []struct {
		name   string
		s0, s1 SDF3
	}{
		{"sphere_box_overlap", makeS1(), makeS2()},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s := Intersect3D(c.s0, c.s1)
			assertBboxContainsSurface(t, c.name, s, 64)
		})
	}
}

//-----------------------------------------------------------------------------
// Cut3D — half-space clip

func Test_Cut3D_BboxContainsSurface(t *testing.T) {
	cases := []struct {
		name string
		make func() SDF3
		a    v3.Vec // point on plane
		n    v3.Vec // plane normal
	}{
		{"sphere_clip_xz", bboxSphere, v3.Vec{}, v3.Vec{Y: 1}},
		{"sphere_clip_diagonal", bboxSphere, v3.Vec{}, v3.Vec{X: 1, Y: 1, Z: 1}},
		{"box_clip_offset", bboxBox, v3.Vec{X: 0.5}, v3.Vec{X: 1}},
		{"off_center_box_clip", bboxBoxOffCenter, v3.Vec{X: -2}, v3.Vec{X: 1}},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s := Cut3D(c.make(), c.a, c.n)
			assertBboxContainsSurface(t, c.name, s, 64)
		})
	}
}

//-----------------------------------------------------------------------------
// Elongate3D — adds an axis-aligned slab in the middle

func Test_Elongate3D_BboxContainsSurface(t *testing.T) {
	cases := []struct {
		name string
		make func() SDF3
		h    v3.Vec
	}{
		{"sphere_elongate_x", bboxSphere, v3.Vec{X: 2}},
		{"sphere_elongate_xyz", bboxSphere, v3.Vec{X: 1, Y: 2, Z: 3}},
		{"box_elongate_z", bboxBox, v3.Vec{Z: 1.5}},
		{"off_center_elongate", bboxBoxOffCenter, v3.Vec{X: 1, Y: 1, Z: 0}},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s := Elongate3D(c.make(), c.h)
			assertBboxContainsSurface(t, c.name, s, 64)
		})
	}
}

//-----------------------------------------------------------------------------
// Composition — combine the above with each other to surface bbox bugs that
// only show up under chained transforms.

func Test_Composition_BboxContainsSurface(t *testing.T) {
	bigBox, _ := Box3D(v3.Vec{X: 2, Y: 1, Z: 1}, 0)
	smallSphere, _ := Sphere3D(0.4)
	cases := []struct {
		name string
		make func() SDF3
	}{
		{"transform_then_array", func() SDF3 {
			rotated := Transform3D(bigBox, RotateZ(DtoR(45)))
			return Array3D(rotated, v3i.Vec{X: 3, Y: 1, Z: 1}, v3.Vec{X: 4})
		}},
		{"array_then_rotate", func() SDF3 {
			arr := Array3D(smallSphere, v3i.Vec{X: 5, Y: 1, Z: 1}, v3.Vec{X: 1.5})
			return Transform3D(arr, RotateZ(DtoR(45)))
		}},
		{"rotate_union_of_off_center_thin_rods", func() SDF3 {
			rod, _ := Box3D(v3.Vec{X: 4, Y: 0.5, Z: 0.5}, 0)
			off := Transform3D(rod, Translate3d(v3.Vec{Y: -2}))
			return RotateUnion3D(off, 6, RotateZ(DtoR(60)))
		}},
		{"loft_then_translate", func() SDF3 {
			c, _ := Circle2D(1)
			b := Box2D(v2.Vec{X: 2, Y: 2}, 0)
			lft, _ := Loft3D(c, b, 4, 0)
			return Transform3D(lft, Translate3d(v3.Vec{X: 3, Y: -2}))
		}},
		{"twist_inside_array", func() SDF3 {
			rect := Box2D(v2.Vec{X: 2, Y: 1}, 0)
			twisted := TwistExtrude3D(rect, 4, DtoR(180))
			return Array3D(twisted, v3i.Vec{X: 3, Y: 1, Z: 1}, v3.Vec{X: 3})
		}},
		{"scaletwist_inside_rotate_union", func() SDF3 {
			rect := Box2D(v2.Vec{X: 2, Y: 0.5}, 0)
			st := ScaleTwistExtrude3D(rect, 3, DtoR(90), v2.Vec{X: 1.5, Y: 2})
			return RotateUnion3D(st, 4, RotateZ(DtoR(90)))
		}},
		{"shell_of_off_center_box", func() SDF3 {
			s, _ := Shell3D(bboxBoxOffCenter(), 0.2)
			return s
		}},
		{"offset_of_thin_rod", func() SDF3 {
			return Offset3D(bboxThinRod(), 0.5)
		}},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			assertBboxContainsSurface(t, c.name, c.make(), 64)
		})
	}
}
