package sdf

import (
	"math"
	"testing"

	v3 "github.com/snowbldr/sdfx/vec/v3"
	"github.com/snowbldr/sdfx/vec/v3i"
)

// Bbox-correctness audit for the 3D array / rotate-copy constructors:
//   - Array3D / ArraySDF3
//   - RotateUnion3D / RotateUnionSDF3
//   - RotateCopy3D / RotateCopySDF3
//
// Focuses on edge cases the existing bbox_test.go file does not
// already cover: degenerate num counts, zero / negative step components,
// SetMin variants (PolyMin, PowMin, RoundMin, ChamferMin), helices,
// non-orthogonal step matrices (shear), wraparound (num·step > 360°),
// off-axis inputs that cross sector boundaries, and inputs touching the
// z-axis.
//
// All shapes use the assertBboxContainsSurface helper (extrude_test.go)
// with sampleN = 64.

// --- shared shapes ---

func arr3dSphere() SDF3 {
	s, _ := Sphere3D(1)
	return s
}

func arr3dSmallSphere() SDF3 {
	s, _ := Sphere3D(0.4)
	return s
}

func arr3dBox() SDF3 {
	s, _ := Box3D(v3.Vec{X: 2, Y: 1, Z: 0.5}, 0)
	return s
}

func arr3dRodOffX(offset float64) SDF3 {
	s, _ := Box3D(v3.Vec{X: 0.4, Y: 0.4, Z: 2}, 0)
	return Transform3D(s, Translate3d(v3.Vec{X: offset, Y: 0, Z: 0}))
}

// Box that just touches the z-axis (bb.Min.X = 0).
func arr3dBoxTouchingZ() SDF3 {
	s, _ := Box3D(v3.Vec{X: 1, Y: 0.5, Z: 1}, 0)
	return Transform3D(s, Translate3d(v3.Vec{X: 0.5, Y: 0, Z: 0}))
}

// Box that crosses the z-axis (bb.Min.X < 0 < bb.Max.X).
func arr3dBoxCrossesZ() SDF3 {
	s, _ := Box3D(v3.Vec{X: 2, Y: 0.5, Z: 1}, 0)
	return s
}

//-----------------------------------------------------------------------------
// Array3D — degenerate num and step edges.

func Test_Arr3D_Array3D_NumZero_ReturnsNil(t *testing.T) {
	// num.X = 0 → constructor must return nil (early-return path).
	s := Array3D(arr3dSphere(), v3i.Vec{X: 0, Y: 1, Z: 1}, v3.Vec{X: 2})
	if s != nil {
		t.Fatalf("Array3D(num.X=0): expected nil, got %T", s)
	}
	s = Array3D(arr3dSphere(), v3i.Vec{X: 1, Y: 0, Z: 1}, v3.Vec{X: 2})
	if s != nil {
		t.Fatalf("Array3D(num.Y=0): expected nil, got %T", s)
	}
	s = Array3D(arr3dSphere(), v3i.Vec{X: 1, Y: 1, Z: 0}, v3.Vec{X: 2})
	if s != nil {
		t.Fatalf("Array3D(num.Z=0): expected nil, got %T", s)
	}
	s = Array3D(arr3dSphere(), v3i.Vec{X: 0, Y: 0, Z: 0}, v3.Vec{X: 2})
	if s != nil {
		t.Fatalf("Array3D(num=000): expected nil, got %T", s)
	}
}

func Test_Arr3D_Array3D_NumNegative_ReturnsNil(t *testing.T) {
	s := Array3D(arr3dSphere(), v3i.Vec{X: -1, Y: 1, Z: 1}, v3.Vec{X: 2})
	if s != nil {
		t.Fatalf("Array3D(num.X<0): expected nil, got %T", s)
	}
}

func Test_Arr3D_Array3D_Num111_ZeroStep(t *testing.T) {
	// num = (1,1,1), step irrelevant: bbox should equal child bbox exactly.
	child := arr3dSphere()
	s := Array3D(child, v3i.Vec{X: 1, Y: 1, Z: 1}, v3.Vec{})
	if s == nil {
		t.Fatal("expected non-nil")
	}
	bb := s.BoundingBox()
	cb := child.BoundingBox()
	if !bb.Min.Equals(cb.Min, 1e-12) || !bb.Max.Equals(cb.Max, 1e-12) {
		t.Errorf("expected bbox = child bbox; got %+v vs %+v", bb, cb)
	}
	assertBboxContainsSurface(t, "num111_zeroStep", s, 64)
}

func Test_Arr3D_Array3D_ZeroStep_AllOverlap(t *testing.T) {
	// step=(0,0,0): all copies sit at origin. Bbox should still contain
	// the union (== child bbox).
	s := Array3D(arr3dSphere(), v3i.Vec{X: 4, Y: 3, Z: 2}, v3.Vec{})
	if s == nil {
		t.Fatal("expected non-nil")
	}
	assertBboxContainsSurface(t, "array_zero_step", s, 64)
}

func Test_Arr3D_Array3D_MixedSignStep(t *testing.T) {
	// Negative x, positive y, zero z — the array snakes "backwards" in x.
	s := Array3D(arr3dSphere(), v3i.Vec{X: 3, Y: 3, Z: 1}, v3.Vec{X: -2, Y: 3, Z: 0})
	assertBboxContainsSurface(t, "mixed_sign_step", s, 64)
}

func Test_Arr3D_Array3D_LargeNumOneDim(t *testing.T) {
	// 100 copies in x — bbox should still be correct, even though we
	// don't care about performance.
	s := Array3D(arr3dSmallSphere(), v3i.Vec{X: 100, Y: 1, Z: 1}, v3.Vec{X: 1})
	assertBboxContainsSurface(t, "large_num_x", s, 64)
}

func Test_Arr3D_Array3D_StepEqualsShape(t *testing.T) {
	// Touching copies: step exactly equals the bounding extent.
	s := Array3D(arr3dSphere(), v3i.Vec{X: 4, Y: 4, Z: 4}, v3.Vec{X: 2, Y: 2, Z: 2})
	assertBboxContainsSurface(t, "step_equals_shape", s, 64)
}

func Test_Arr3D_Array3D_HeavyOverlap(t *testing.T) {
	// Step << shape: heavy overlap. Bbox still must contain everything.
	s := Array3D(arr3dSphere(), v3i.Vec{X: 5, Y: 5, Z: 1}, v3.Vec{X: 0.1, Y: 0.1, Z: 0})
	assertBboxContainsSurface(t, "heavy_overlap", s, 64)
}

func Test_Arr3D_Array3D_SetMin_PolyMin(t *testing.T) {
	// PolyMin(0.5) → reach = 0.25 (from -min(0,0) = -poly(0,0,0.5) = 0.125
	// actually returns 0.5/4 = 0.125 → reach 0.125, bbox enlarges by 0.25
	// on each side). Just verify containment under the blended evaluator.
	s := Array3D(arr3dSphere(), v3i.Vec{X: 3, Y: 3, Z: 1}, v3.Vec{X: 2.5, Y: 2.5, Z: 0})
	if as, ok := s.(*ArraySDF3); ok {
		as.SetMin(PolyMin(0.5))
	} else {
		t.Fatal("expected *ArraySDF3")
	}
	assertBboxContainsSurface(t, "array_polymin", s, 64)
}

func Test_Arr3D_Array3D_SetMin_RoundMin(t *testing.T) {
	// RoundMin produces a fillet; -RoundMin(0,0) = k*(some factor) > 0
	// for non-zero k. Just check containment.
	s := Array3D(arr3dSphere(), v3i.Vec{X: 3, Y: 3, Z: 1}, v3.Vec{X: 2.2, Y: 2.2, Z: 0})
	if as, ok := s.(*ArraySDF3); ok {
		as.SetMin(RoundMin(0.4))
	}
	assertBboxContainsSurface(t, "array_roundmin", s, 64)
}

func Test_Arr3D_Array3D_SetMin_ChamferMin(t *testing.T) {
	// ChamferMin: -min(0,0) = -((0+0-k)*0.5) = k/2 > 0 → enlarges bbox.
	s := Array3D(arr3dSphere(), v3i.Vec{X: 3, Y: 3, Z: 1}, v3.Vec{X: 2.3, Y: 2.3, Z: 0})
	if as, ok := s.(*ArraySDF3); ok {
		as.SetMin(ChamferMin(0.5))
	}
	assertBboxContainsSurface(t, "array_chamfermin", s, 64)
}

func Test_Arr3D_Array3D_SetMin_PowMin(t *testing.T) {
	// PowMin(0,0) = 0 → reach = 0 → bbox NOT enlarged. If PowMin's
	// blend extends the surface past the hard-min envelope (heavy
	// overlap puts neighbouring sphere SDFs both negative near the
	// midpoint), containment may fail. This documents the gap.
	s := Array3D(arr3dSphere(), v3i.Vec{X: 3, Y: 3, Z: 1}, v3.Vec{X: 1.8, Y: 1.8, Z: 0})
	if as, ok := s.(*ArraySDF3); ok {
		as.SetMin(PowMin(8))
	}
	assertBboxContainsSurface(t, "array_powmin", s, 64)
}

func Test_Arr3D_Array3D_OffCenterRodMixedStep(t *testing.T) {
	// Off-center rod with negative-z and positive-x step.
	rod := arr3dRodOffX(2.0)
	s := Array3D(rod, v3i.Vec{X: 4, Y: 1, Z: 3}, v3.Vec{X: 3, Y: 0, Z: -2})
	assertBboxContainsSurface(t, "offcenter_rod_mixed_step", s, 64)
}

//-----------------------------------------------------------------------------
// RotateUnion3D — edge cases.

func Test_Arr3D_RotateUnion3D_NumZero_ReturnsNil(t *testing.T) {
	s := RotateUnion3D(arr3dSphere(), 0, RotateZ(DtoR(30)))
	if s != nil {
		t.Fatalf("RotateUnion3D(num=0): expected nil, got %T", s)
	}
	s = RotateUnion3D(arr3dSphere(), -3, RotateZ(DtoR(30)))
	if s != nil {
		t.Fatalf("RotateUnion3D(num<0): expected nil, got %T", s)
	}
}

func Test_Arr3D_RotateUnion3D_Num1_Identity(t *testing.T) {
	// One copy, identity step: bbox should equal child bbox.
	child := arr3dBox()
	s := RotateUnion3D(child, 1, Identity3d())
	bb := s.BoundingBox()
	cb := child.BoundingBox()
	if !bb.Min.Equals(cb.Min, 1e-9) || !bb.Max.Equals(cb.Max, 1e-9) {
		t.Errorf("num=1 identity: bbox %+v != child %+v", bb, cb)
	}
	assertBboxContainsSurface(t, "rotunion_n1_identity", s, 64)
}

func Test_Arr3D_RotateUnion3D_PureTranslationStep(t *testing.T) {
	// Step matrix is a pure translation (no rotation) → 1D-like array.
	s := RotateUnion3D(arr3dSphere(), 5, Translate3d(v3.Vec{X: 1.5, Y: 0.5, Z: 0.3}))
	assertBboxContainsSurface(t, "rotunion_pure_translation", s, 64)
}

func Test_Arr3D_RotateUnion3D_HelixStep(t *testing.T) {
	// Helix: rotate 45° per copy AND translate +1 in z per copy.
	step := Translate3d(v3.Vec{Z: 1}).Mul(RotateZ(DtoR(45)))
	s := RotateUnion3D(arr3dRodOffX(2.5), 8, step)
	assertBboxContainsSurface(t, "rotunion_helix", s, 64)
}

func Test_Arr3D_RotateUnion3D_ShearStep(t *testing.T) {
	// Non-orthogonal step matrix (a small shear). The bbox computation
	// uses bb.Vertices() + mulVertices3 which transforms each corner,
	// so a shear should still be captured correctly.
	shear := M44{
		1, 0.3, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1,
	}
	// Combine shear with a small rotation so copies don't all overlap.
	step := shear.Mul(RotateZ(DtoR(20)))
	s := RotateUnion3D(arr3dSphere(), 6, step)
	assertBboxContainsSurface(t, "rotunion_shear", s, 64)
}

func Test_Arr3D_RotateUnion3D_Wraparound_720deg(t *testing.T) {
	// 30° × 24 copies = 720°. The path traces the same disk twice;
	// bbox should still cover the disk and not blow up.
	s := RotateUnion3D(arr3dRodOffX(3), 24, RotateZ(DtoR(30)))
	assertBboxContainsSurface(t, "rotunion_wrap_720", s, 64)
}

func Test_Arr3D_RotateUnion3D_OffCenterRod(t *testing.T) {
	// Rod centered at x=4 → rotated 6× by 60° forms a ring/disk.
	rod := arr3dRodOffX(4)
	s := RotateUnion3D(rod, 6, RotateZ(DtoR(60)))
	assertBboxContainsSurface(t, "rotunion_offcenter_rod", s, 64)
}

func Test_Arr3D_RotateUnion3D_SetMin_PolyMin(t *testing.T) {
	s := RotateUnion3D(arr3dSphere(), 6, RotateZ(DtoR(60)).Mul(Translate3d(v3.Vec{X: 1.5})))
	if rs, ok := s.(*RotateUnionSDF3); ok {
		rs.SetMin(PolyMin(0.6))
	} else {
		t.Fatal("expected *RotateUnionSDF3")
	}
	assertBboxContainsSurface(t, "rotunion_polymin", s, 64)
}

func Test_Arr3D_RotateUnion3D_SetMin_PowMin(t *testing.T) {
	// PowMin(0,0)=0 → no enlargement. With overlapping spheres along
	// a ring this may show the same kind of gap PolyMin avoids.
	s := RotateUnion3D(arr3dSphere(), 6, RotateZ(DtoR(60)).Mul(Translate3d(v3.Vec{X: 1.2})))
	if rs, ok := s.(*RotateUnionSDF3); ok {
		rs.SetMin(PowMin(8))
	}
	assertBboxContainsSurface(t, "rotunion_powmin", s, 64)
}

func Test_Arr3D_RotateUnion3D_SetMin_ChamferMin(t *testing.T) {
	s := RotateUnion3D(arr3dSphere(), 6, RotateZ(DtoR(60)).Mul(Translate3d(v3.Vec{X: 1.5})))
	if rs, ok := s.(*RotateUnionSDF3); ok {
		rs.SetMin(ChamferMin(0.5))
	}
	assertBboxContainsSurface(t, "rotunion_chamfermin", s, 64)
}

//-----------------------------------------------------------------------------
// RotateCopy3D — sector-fold variant.

func Test_Arr3D_RotateCopy3D_NumZero_ReturnsNil(t *testing.T) {
	s := RotateCopy3D(arr3dSphere(), 0)
	if s != nil {
		t.Fatalf("RotateCopy3D(num=0): expected nil, got %T", s)
	}
	s = RotateCopy3D(arr3dSphere(), -2)
	if s != nil {
		t.Fatalf("RotateCopy3D(num<0): expected nil, got %T", s)
	}
}

func Test_Arr3D_RotateCopy3D_Num1_FullSector(t *testing.T) {
	// num=1 → theta = 2π → the "first sector" is the entire plane;
	// no folding. The surface should match the input but the bbox is
	// always the rmax disk per the constructor logic.
	child := arr3dRodOffX(2)
	s := RotateCopy3D(child, 1)
	assertBboxContainsSurface(t, "rotcopy_n1", s, 64)
	// Sanity: bbox should be a square disk of side 2·rmax.
	bb := s.BoundingBox()
	if math.Abs(bb.Min.X+bb.Max.X) > 1e-9 || math.Abs(bb.Min.Y+bb.Max.Y) > 1e-9 {
		t.Errorf("expected symmetric XY bbox; got %+v", bb)
	}
}

func Test_Arr3D_RotateCopy3D_Num2_HalfFold(t *testing.T) {
	// num=2: 180° sectors.
	s := RotateCopy3D(arr3dRodOffX(2), 2)
	assertBboxContainsSurface(t, "rotcopy_n2", s, 64)
}

func Test_Arr3D_RotateCopy3D_ManySmallSectors(t *testing.T) {
	// num=24: 15° sectors — narrow.
	s := RotateCopy3D(arr3dRodOffX(2), 24)
	assertBboxContainsSurface(t, "rotcopy_n24", s, 64)
}

func Test_Arr3D_RotateCopy3D_TouchingZAxis(t *testing.T) {
	// Input touches the z-axis at x=0. SawTooth fold maps |angle|≥0,
	// so the rod should appear N times around the axis.
	s := RotateCopy3D(arr3dBoxTouchingZ(), 6)
	assertBboxContainsSurface(t, "rotcopy_touching_z", s, 64)
}

func Test_Arr3D_RotateCopy3D_CrossesZAxis(t *testing.T) {
	// Input straddles the z-axis. SawTooth folds points on the "wrong"
	// side of the axis into the first sector — the folded SDF differs
	// from the literal rotated-union, but containment in the rmax disk
	// must still hold.
	s := RotateCopy3D(arr3dBoxCrossesZ(), 4)
	assertBboxContainsSurface(t, "rotcopy_crosses_z", s, 64)
}

func Test_Arr3D_RotateCopy3D_OneSideOfZ(t *testing.T) {
	// Input fully on +x side; bbox is the rmax disk.
	rod := arr3dRodOffX(3)
	s := RotateCopy3D(rod, 5)
	assertBboxContainsSurface(t, "rotcopy_one_side", s, 64)
}

func Test_Arr3D_RotateCopy3D_AngularWidthBiggerThanSector(t *testing.T) {
	// Input that spans more than one sector (a wide box at modest
	// radius). With num=12 (30° sectors), the box subtends > 30°.
	s, _ := Box3D(v3.Vec{X: 3, Y: 2, Z: 1}, 0)
	off := Transform3D(s, Translate3d(v3.Vec{X: 1.5}))
	rc := RotateCopy3D(off, 12)
	assertBboxContainsSurface(t, "rotcopy_wide_input", rc, 64)
}
