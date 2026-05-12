package sdf

import (
	"testing"

	v2 "github.com/snowbldr/sdfx/vec/v2"
	"github.com/snowbldr/sdfx/vec/v2i"
	v3 "github.com/snowbldr/sdfx/vec/v3"
)

// Bbox-correctness audit tests for 2D boolean combinators, arrays/rotates,
// and Slice2D. All tests use assertBbox2ContainsSurface (sdf/bbox_test.go)
// to verify that the constructor's BoundingBox actually contains the surface.

//-----------------------------------------------------------------------------
// Shared 2D test shapes specific to combinator audits.

func auditCircleR(r float64) SDF2 {
	s, _ := Circle2D(r)
	return s
}

func auditCircleAt(r, x, y float64) SDF2 {
	s, _ := Circle2D(r)
	return Transform2D(s, Translate2d(v2.Vec{X: x, Y: y}))
}

func auditBoxAt(w, h, x, y float64) SDF2 {
	s := Box2D(v2.Vec{X: w, Y: h}, 0)
	return Transform2D(s, Translate2d(v2.Vec{X: x, Y: y}))
}

//-----------------------------------------------------------------------------
// Union2D — bbox should be extend(child bboxes); pad by -min(0,0) when blended.

func Test_Comb2D_Union2D_BboxContainsSurface(t *testing.T) {
	cases := []struct {
		name string
		make func() SDF2
	}{
		{"two_circles_overlap", func() SDF2 {
			return Union2D(auditCircleR(1), auditCircleAt(0.7, 1.2, 0))
		}},
		{"three_disjoint", func() SDF2 {
			return Union2D(auditCircleR(1), auditCircleAt(0.5, 3, 0), auditBoxAt(0.5, 0.5, -2, -2))
		}},
		{"five_shapes_mixed", func() SDF2 {
			return Union2D(
				auditCircleR(1),
				auditCircleAt(0.7, 2.0, 0),
				auditBoxAt(0.5, 0.5, -2, -2),
				auditBoxAt(0.4, 1.0, 0, 3),
				auditCircleAt(0.3, -3, 2),
			)
		}},
		{"single_passthrough", func() SDF2 {
			return Union2D(auditCircleR(1))
		}},
		{"two_off_center", func() SDF2 {
			return Union2D(auditCircleAt(1, -3, -3), auditCircleAt(0.5, 3, 3))
		}},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			assertBbox2ContainsSurface(t, c.name, c.make(), 64)
		})
	}
}

func Test_Comb2D_Union2D_BlendedBboxContainsSurface(t *testing.T) {
	// SetMin with a smoothing function expands the surface beyond the
	// component bboxes — bbox should compensate via -min(0,0) padding.
	make := func() SDF2 {
		return Union2D(auditCircleR(1), auditCircleAt(0.7, 1.2, 0))
	}
	for _, k := range []float64{0.05, 0.1, 0.3, 0.5, 1.0} {
		t.Run("polymin_k_"+ftoa(k), func(t *testing.T) {
			s := make()
			s.(*UnionSDF2).SetMin(PolyMin(k))
			assertBbox2ContainsSurface(t, "polymin_k_"+ftoa(k), s, 64)
		})
	}
	for _, k := range []float64{0.05, 0.1, 0.3, 0.5} {
		t.Run("roundmin_k_"+ftoa(k), func(t *testing.T) {
			s := make()
			s.(*UnionSDF2).SetMin(RoundMin(k))
			assertBbox2ContainsSurface(t, "roundmin_k_"+ftoa(k), s, 64)
		})
	}
	for _, k := range []float64{0.1, 0.5} {
		t.Run("chamfermin_k_"+ftoa(k), func(t *testing.T) {
			s := make()
			s.(*UnionSDF2).SetMin(ChamferMin(k))
			assertBbox2ContainsSurface(t, "chamfermin_k_"+ftoa(k), s, 64)
		})
	}
	// PowMin is scale-dependent: -PowMin(0,0)=0 so no padding; surface may
	// extend past bbox — this is the documented audit interest.
	for _, k := range []float64{2, 4, 8} {
		t.Run("powmin_k_"+ftoa(k), func(t *testing.T) {
			s := make()
			s.(*UnionSDF2).SetMin(PowMin(k))
			assertBbox2ContainsSurface(t, "powmin_k_"+ftoa(k), s, 64)
		})
	}
	for _, k := range []float64{8, 32} {
		t.Run("expmin_k_"+ftoa(k), func(t *testing.T) {
			s := make()
			s.(*UnionSDF2).SetMin(ExpMin(k))
			assertBbox2ContainsSurface(t, "expmin_k_"+ftoa(k), s, 64)
		})
	}
	t.Run("polymin_disjoint", func(t *testing.T) {
		// Disjoint shapes with large smoothing radius — surface extends
		// into the gap.
		s := Union2D(auditCircleR(1), auditCircleAt(0.7, 4, 0))
		s.(*UnionSDF2).SetMin(PolyMin(1.0))
		assertBbox2ContainsSurface(t, "polymin_disjoint", s, 64)
	})
}

//-----------------------------------------------------------------------------
// Difference2D — bbox is s0's bbox; cutter can only shrink the result.

func Test_Comb2D_Difference2D_BboxContainsSurface(t *testing.T) {
	cases := []struct {
		name string
		make func() SDF2
	}{
		{"box_minus_circle", func() SDF2 {
			return Difference2D(Box2D(v2.Vec{X: 2, Y: 2}, 0), auditCircleR(0.5))
		}},
		{"circle_minus_concentric_circle", func() SDF2 {
			return Difference2D(auditCircleR(2), auditCircleR(1))
		}},
		{"cutter_much_larger", func() SDF2 {
			return Difference2D(Box2D(v2.Vec{X: 2, Y: 2}, 0), auditCircleR(10))
		}},
		{"cutter_outside_body", func() SDF2 {
			return Difference2D(Box2D(v2.Vec{X: 2, Y: 2}, 0), auditCircleAt(0.5, 5, 5))
		}},
		{"off_center_body_minus_circle", func() SDF2 {
			return Difference2D(auditBoxAt(2, 2, 3, 3), auditCircleR(0.5))
		}},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			assertBbox2ContainsSurface(t, c.name, c.make(), 64)
		})
	}
}

func Test_Comb2D_Difference2D_BlendedBboxContainsSurface(t *testing.T) {
	// SetMax with a smoothing function — surface may extend past s0's bbox
	// because the smooth max can "pull" the surface outward near the cut.
	make := func() SDF2 {
		return Difference2D(Box2D(v2.Vec{X: 2, Y: 2}, 0), auditCircleR(0.5))
	}
	for _, k := range []float64{0.05, 0.1, 0.3, 0.5} {
		t.Run("polymax_k_"+ftoa(k), func(t *testing.T) {
			s := make()
			s.(*DifferenceSDF2).SetMax(PolyMax(k))
			assertBbox2ContainsSurface(t, "polymax_k_"+ftoa(k), s, 64)
		})
	}
	for _, k := range []float64{0.1, 0.3} {
		t.Run("roundmax_k_"+ftoa(k), func(t *testing.T) {
			s := make()
			s.(*DifferenceSDF2).SetMax(RoundMax(k))
			assertBbox2ContainsSurface(t, "roundmax_k_"+ftoa(k), s, 64)
		})
	}
	for _, k := range []float64{0.1, 0.5} {
		t.Run("chamfermax_k_"+ftoa(k), func(t *testing.T) {
			s := make()
			s.(*DifferenceSDF2).SetMax(ChamferMax(k))
			assertBbox2ContainsSurface(t, "chamfermax_k_"+ftoa(k), s, 64)
		})
	}
}

//-----------------------------------------------------------------------------
// Intersect2D — bbox is the intersection of child bboxes.

func Test_Comb2D_Intersect2D_BboxContainsSurface(t *testing.T) {
	cases := []struct {
		name string
		make func() SDF2
	}{
		{"circle_box_overlap", func() SDF2 {
			return Intersect2D(auditCircleR(1.5), auditBoxAt(1, 1, 0.5, 0))
		}},
		{"two_overlapping_circles", func() SDF2 {
			return Intersect2D(auditCircleR(1), auditCircleAt(1, 0.8, 0))
		}},
		{"empty_intersection_far_apart", func() SDF2 {
			return Intersect2D(auditCircleR(1), auditCircleAt(1, 10, 10))
		}},
		{"cutter_much_larger_than_body", func() SDF2 {
			return Intersect2D(auditCircleR(0.5), Box2D(v2.Vec{X: 20, Y: 20}, 0))
		}},
		{"off_center_intersect", func() SDF2 {
			return Intersect2D(auditCircleAt(2, 2, 2), auditBoxAt(2, 2, 3, 3))
		}},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s := c.make()
			if s == nil {
				t.Skip("nil sdf")
			}
			assertBbox2ContainsSurface(t, c.name, s, 64)
		})
	}
}

func Test_Comb2D_Intersect2D_BlendedBboxContainsSurface(t *testing.T) {
	make := func() SDF2 {
		return Intersect2D(auditCircleR(1.5), auditBoxAt(1, 1, 0.5, 0))
	}
	for _, k := range []float64{0.05, 0.1, 0.3} {
		t.Run("polymax_k_"+ftoa(k), func(t *testing.T) {
			s := make()
			s.(*IntersectionSDF2).SetMax(PolyMax(k))
			assertBbox2ContainsSurface(t, "polymax_k_"+ftoa(k), s, 64)
		})
	}
	for _, k := range []float64{0.1, 0.3} {
		t.Run("roundmax_k_"+ftoa(k), func(t *testing.T) {
			s := make()
			s.(*IntersectionSDF2).SetMax(RoundMax(k))
			assertBbox2ContainsSurface(t, "roundmax_k_"+ftoa(k), s, 64)
		})
	}
}

//-----------------------------------------------------------------------------
// Cut2D — half-plane clip. The kept side is to the right of v (Cut2D(sdf,a,v)).

func Test_Comb2D_Cut2D_BboxContainsSurface(t *testing.T) {
	cases := []struct {
		name string
		make func() SDF2
		a    v2.Vec // point on line
		v    v2.Vec // direction along the line
	}{
		{"circle_plane_through_origin_x", func() SDF2 { return auditCircleR(1) }, v2.Vec{}, v2.Vec{X: 1, Y: 0}},
		{"circle_plane_through_origin_y", func() SDF2 { return auditCircleR(1) }, v2.Vec{}, v2.Vec{X: 0, Y: 1}},
		{"circle_plane_diag_45", func() SDF2 { return auditCircleR(1) }, v2.Vec{}, v2.Vec{X: 1, Y: 1}},
		{"box_plane_offset", func() SDF2 { return Box2D(v2.Vec{X: 2, Y: 2}, 0) }, v2.Vec{X: 0.5, Y: 0}, v2.Vec{X: 0, Y: 1}},
		{"box_plane_misses_shape_kept_side", func() SDF2 {
			// Direction +x, normal = (-vy, vx) = (0, 1). Right side is where
			// (p - a).(vy, -vx) = (p - a).(0, -1) ≥ 0, i.e. p.Y ≤ a.Y.
			// Place plane at y=10 → entire box (within y∈[-1,1]) is kept.
			return Box2D(v2.Vec{X: 2, Y: 2}, 0)
		}, v2.Vec{X: 0, Y: 10}, v2.Vec{X: 1, Y: 0}},
		{"box_plane_misses_shape_cut_side", func() SDF2 {
			// Same line direction, but place plane at y=-10 → kept side is
			// where p.Y ≤ -10, which excludes the box entirely. Bbox should
			// be empty / degenerate.
			return Box2D(v2.Vec{X: 2, Y: 2}, 0)
		}, v2.Vec{X: 0, Y: -10}, v2.Vec{X: 1, Y: 0}},
		{"off_center_box_cut", func() SDF2 { return auditBoxAt(2, 2, 3, 3) }, v2.Vec{X: 3, Y: 3}, v2.Vec{X: 1, Y: 0}},
		{"thin_rect_diag_cut", func() SDF2 { return Box2D(v2.Vec{X: 6, Y: 0.5}, 0) }, v2.Vec{}, v2.Vec{X: 1, Y: 1}},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s := Cut2D(c.make(), c.a, c.v)
			assertBbox2ContainsSurface(t, c.name, s, 64)
		})
	}
}

//-----------------------------------------------------------------------------
// Elongate2D — extends the SDF along the axes with positive offsets.

func Test_Comb2D_Elongate2D_BboxContainsSurface(t *testing.T) {
	cases := []struct {
		name string
		make func() SDF2
		h    v2.Vec
	}{
		{"circle_h_zero", func() SDF2 { return auditCircleR(1) }, v2.Vec{}},
		{"circle_elongate_x_only", func() SDF2 { return auditCircleR(1) }, v2.Vec{X: 2}},
		{"circle_elongate_y_only", func() SDF2 { return auditCircleR(1) }, v2.Vec{Y: 3}},
		{"circle_elongate_xy", func() SDF2 { return auditCircleR(1) }, v2.Vec{X: 2, Y: 3}},
		{"circle_elongate_very_large", func() SDF2 { return auditCircleR(1) }, v2.Vec{X: 100, Y: 50}},
		{"box_elongate", func() SDF2 { return Box2D(v2.Vec{X: 2, Y: 1}, 0) }, v2.Vec{X: 1, Y: 1}},
		{"off_center_box_elongate", func() SDF2 { return auditBoxAt(1, 1, 3, 3) }, v2.Vec{X: 2, Y: 2}},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s := Elongate2D(c.make(), c.h)
			assertBbox2ContainsSurface(t, c.name, s, 64)
		})
	}
}

//-----------------------------------------------------------------------------
// Array2D — XY grid of copies.

func Test_Comb2D_Array2D_BboxContainsSurface(t *testing.T) {
	cases := []struct {
		name string
		make func() SDF2
		num  v2i.Vec
		step v2.Vec
	}{
		// 1D arrays.
		{"x_array_5", func() SDF2 { return auditCircleR(1) }, v2i.Vec{X: 5, Y: 1}, v2.Vec{X: 3, Y: 0}},
		{"y_array_4", func() SDF2 { return Box2D(v2.Vec{X: 2, Y: 1}, 0) }, v2i.Vec{X: 1, Y: 4}, v2.Vec{X: 0, Y: 2.5}},
		// 2D grid.
		{"xy_grid_3x3", func() SDF2 { return auditCircleR(1) }, v2i.Vec{X: 3, Y: 3}, v2.Vec{X: 3, Y: 3}},
		{"xy_grid_2x5", func() SDF2 { return auditCircleR(0.5) }, v2i.Vec{X: 2, Y: 5}, v2.Vec{X: 1.5, Y: 1.5}},
		// Negative step direction.
		{"neg_x_array", func() SDF2 { return auditCircleR(1) }, v2i.Vec{X: 4, Y: 1}, v2.Vec{X: -2.5, Y: 0}},
		{"neg_xy_array", func() SDF2 { return auditCircleR(1) }, v2i.Vec{X: 3, Y: 3}, v2.Vec{X: -2.5, Y: -2.5}},
		// Overlapping copies.
		{"overlap_array", func() SDF2 { return auditCircleR(1) }, v2i.Vec{X: 6, Y: 1}, v2.Vec{X: 0.5, Y: 0}},
		// Off-center input.
		{"off_center_array", func() SDF2 { return auditBoxAt(1, 1, 3, 3) }, v2i.Vec{X: 3, Y: 3}, v2.Vec{X: 2, Y: 2}},
		// Single copy.
		{"single_copy", func() SDF2 { return auditCircleR(1) }, v2i.Vec{X: 1, Y: 1}, v2.Vec{X: 0, Y: 0}},
		// Diagonal step.
		{"diagonal_step", func() SDF2 { return auditCircleR(1) }, v2i.Vec{X: 5, Y: 1}, v2.Vec{X: 2, Y: 2}},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s := Array2D(c.make(), c.num, c.step)
			if s == nil {
				t.Skip("nil sdf (num.X<=0 or num.Y<=0)")
			}
			assertBbox2ContainsSurface(t, c.name, s, 64)
		})
	}
}

func Test_Comb2D_Array2D_ZeroNumReturnsNil(t *testing.T) {
	// Array2D returns nil for num.X<=0 or num.Y<=0. Verify the nil contract
	// so we don't accidentally test a nil SDF below.
	if s := Array2D(auditCircleR(1), v2i.Vec{X: 0, Y: 0}, v2.Vec{X: 1, Y: 1}); s != nil {
		t.Errorf("Array2D(num=(0,0)) returned non-nil: %v", s)
	}
	if s := Array2D(auditCircleR(1), v2i.Vec{X: 0, Y: 2}, v2.Vec{X: 1, Y: 1}); s != nil {
		t.Errorf("Array2D(num=(0,2)) returned non-nil: %v", s)
	}
	if s := Array2D(auditCircleR(1), v2i.Vec{X: 2, Y: 0}, v2.Vec{X: 1, Y: 1}); s != nil {
		t.Errorf("Array2D(num=(2,0)) returned non-nil: %v", s)
	}
}

func Test_Comb2D_Array2D_BlendedBboxContainsSurface(t *testing.T) {
	make := func() SDF2 {
		return Array2D(auditCircleR(1), v2i.Vec{X: 3, Y: 3}, v2.Vec{X: 3, Y: 3})
	}
	for _, k := range []float64{0.1, 0.3, 0.5} {
		t.Run("polymin_k_"+ftoa(k), func(t *testing.T) {
			s := make()
			s.(*ArraySDF2).SetMin(PolyMin(k))
			assertBbox2ContainsSurface(t, "polymin_k_"+ftoa(k), s, 64)
		})
	}
}

//-----------------------------------------------------------------------------
// RotateUnion2D — N rotated copies (step matrix supplied).

func Test_Comb2D_RotateUnion2D_BboxContainsSurface(t *testing.T) {
	cases := []struct {
		name string
		make func() SDF2
		num  int
		step M33
	}{
		// Single copy — should match input bbox.
		{"circle_n1_no_rot", func() SDF2 { return auditCircleR(1) }, 1, Identity2d()},
		{"box_n1_no_rot", func() SDF2 { return Box2D(v2.Vec{X: 2, Y: 1}, 0) }, 1, Identity2d()},
		// Centered shapes around origin.
		{"box_n4_90deg", func() SDF2 { return Box2D(v2.Vec{X: 2, Y: 1}, 0) }, 4, Rotate2d(DtoR(90))},
		// Off-center input — historically where bbox math fails.
		{"off_center_box_n4_90deg", func() SDF2 { return auditBoxAt(1, 1, 3, 0) }, 4, Rotate2d(DtoR(90))},
		{"off_center_box_n8_45deg", func() SDF2 { return auditBoxAt(1, 1, 3, 0) }, 8, Rotate2d(DtoR(45))},
		{"thin_rect_off_y_n4_90deg", func() SDF2 { return auditBoxAt(6, 0.5, 0, -3) }, 4, Rotate2d(DtoR(90))},
		// Many copies wrapping past 360°.
		{"circle_n36_10deg", func() SDF2 { return auditCircleR(1) }, 36, Rotate2d(DtoR(10))},
		{"overlapping_n24_30deg", func() SDF2 { return auditCircleR(1) }, 24, Rotate2d(DtoR(30))},
		// Step is pure translation (no rotation) — like a 1D array.
		{"linear_step_only", func() SDF2 { return auditCircleR(1) }, 5, Translate2d(v2.Vec{X: 2})},
		{"linear_step_diag", func() SDF2 { return auditCircleR(1) }, 4, Translate2d(v2.Vec{X: 1.5, Y: 1.5})},
		// Step is rotation + translation (spiral-like).
		{"rot_plus_translate", func() SDF2 { return auditCircleR(0.5) }, 6,
			Rotate2d(DtoR(60)).Mul(Translate2d(v2.Vec{X: 1}))},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s := RotateUnion2D(c.make(), c.num, c.step)
			if s == nil {
				t.Skip("nil sdf (num<=0)")
			}
			assertBbox2ContainsSurface(t, c.name, s, 64)
		})
	}
}

func Test_Comb2D_RotateUnion2D_BlendedBboxContainsSurface(t *testing.T) {
	make := func() SDF2 {
		return RotateUnion2D(auditBoxAt(1, 1, 3, 0), 4, Rotate2d(DtoR(90)))
	}
	for _, k := range []float64{0.1, 0.3, 0.5} {
		t.Run("polymin_k_"+ftoa(k), func(t *testing.T) {
			s := make()
			s.(*RotateUnionSDF2).SetMin(PolyMin(k))
			assertBbox2ContainsSurface(t, "polymin_k_"+ftoa(k), s, 64)
		})
	}
}

//-----------------------------------------------------------------------------
// RotateCopy2D — N sector-folded copies around origin.

func Test_Comb2D_RotateCopy2D_BboxContainsSurface(t *testing.T) {
	cases := []struct {
		name string
		make func() SDF2
		num  int
	}{
		{"circle_n1", func() SDF2 { return auditCircleR(1) }, 1},
		{"circle_n2", func() SDF2 { return auditCircleR(1) }, 2},
		{"circle_n4", func() SDF2 { return auditCircleR(1) }, 4},
		{"circle_n12", func() SDF2 { return auditCircleR(1) }, 12},
		// Off-axis input — bbox should be a square of side 2·rmax.
		{"off_center_box_n4", func() SDF2 { return auditBoxAt(1, 1, 3, 0) }, 4},
		{"off_center_box_n12", func() SDF2 { return auditBoxAt(1, 1, 3, 0) }, 12},
		{"off_center_circle_n6", func() SDF2 { return auditCircleAt(0.5, 2, 0) }, 6},
		{"thin_rect_off_n3", func() SDF2 { return auditBoxAt(4, 0.5, 0, -2) }, 3},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s := RotateCopy2D(c.make(), c.num)
			if s == nil {
				t.Skip("nil sdf (num<=0)")
			}
			assertBbox2ContainsSurface(t, c.name, s, 64)
		})
	}
}

//-----------------------------------------------------------------------------
// Slice2D — planar slice through an SDF3 yielding an SDF2.

func Test_Comb2D_Slice2D_BboxContainsSurface(t *testing.T) {
	// 3D shapes used in slicing.
	makeSphere := func(r float64) SDF3 {
		s, _ := Sphere3D(r)
		return s
	}
	makeBox := func() SDF3 {
		s, _ := Box3D(v3.Vec{X: 2, Y: 1, Z: 0.5}, 0)
		return s
	}
	makeExtrudedCircle := func() SDF3 {
		c, _ := Circle2D(1)
		return Extrude3D(c, 4)
	}
	makeExtrudedBox := func() SDF3 {
		return Extrude3D(Box2D(v2.Vec{X: 2, Y: 1}, 0), 4)
	}
	cases := []struct {
		name string
		make func() SDF3
		a    v3.Vec
		n    v3.Vec
	}{
		// Sphere sliced through center → circle of radius r.
		{"sphere_through_center_z", func() SDF3 { return makeSphere(1) }, v3.Vec{}, v3.Vec{Z: 1}},
		{"sphere_through_center_x", func() SDF3 { return makeSphere(1) }, v3.Vec{}, v3.Vec{X: 1}},
		{"sphere_through_center_y", func() SDF3 { return makeSphere(1) }, v3.Vec{}, v3.Vec{Y: 1}},
		{"sphere_through_center_diag", func() SDF3 { return makeSphere(1) }, v3.Vec{}, v3.Vec{X: 1, Y: 1, Z: 1}},
		// Sphere grazed near surface — slice is a small circle.
		{"sphere_grazing", func() SDF3 { return makeSphere(1) }, v3.Vec{Z: 0.95}, v3.Vec{Z: 1}},
		// Plane misses the sphere — bbox should still be sensible (degenerate or empty).
		{"sphere_plane_misses", func() SDF3 { return makeSphere(1) }, v3.Vec{Z: 5}, v3.Vec{Z: 1}},
		// Box sliced with axis-aligned plane.
		{"box_axis_aligned_z", makeBox, v3.Vec{}, v3.Vec{Z: 1}},
		{"box_axis_aligned_x", makeBox, v3.Vec{}, v3.Vec{X: 1}},
		{"box_axis_aligned_y", makeBox, v3.Vec{}, v3.Vec{Y: 1}},
		// Extruded shape sliced perpendicular to extrusion → recovers original 2D shape.
		{"extruded_circle_perp_z", makeExtrudedCircle, v3.Vec{}, v3.Vec{Z: 1}},
		{"extruded_box_perp_z", makeExtrudedBox, v3.Vec{}, v3.Vec{Z: 1}},
		// Extruded shape sliced parallel to extrusion axis.
		{"extruded_circle_parallel_x", makeExtrudedCircle, v3.Vec{}, v3.Vec{X: 1}},
		// Slice with diagonal normal through box.
		{"box_diag_normal", makeBox, v3.Vec{}, v3.Vec{X: 1, Y: 1, Z: 1}},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s := Slice2D(c.make(), c.a, c.n)
			assertBbox2ContainsSurface(t, c.name, s, 64)
		})
	}
}
