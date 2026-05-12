package sdf

import (
	"testing"

	v3 "github.com/snowbldr/sdfx/vec/v3"
)

// Bbox containment audits for 3D primitive constructors.
// Each constructor is exercised across boundary parameter values; the
// assertBboxContainsSurface helper (extrude_test.go) samples points just
// outside each bbox face and fails if SDF < -1e-6 anywhere — i.e. the
// surface protrudes past the claimed bbox.

const prim3dSampleN = 64

//-----------------------------------------------------------------------------

func Test_Prim3D_Box3D_BboxContainsSurface(t *testing.T) {
	cases := []struct {
		name  string
		size  v3.Vec
		round float64
	}{
		{"unit_zero_round", v3.Vec{1, 1, 1}, 0},
		{"unit_round_half_min", v3.Vec{1, 1, 1}, 0.5},
		{"anisotropic_thin", v3.Vec{10, 0.1, 0.1}, 0},
		{"anisotropic_thin_rounded", v3.Vec{10, 0.1, 0.1}, 0.05},
		{"huge", v3.Vec{1e6, 1e6, 1e6}, 0},
		{"tiny", v3.Vec{1e-3, 1e-3, 1e-3}, 0},
		{"round_just_below_halfmin", v3.Vec{2, 4, 6}, 0.99 * 1.0},
		{"round_equals_halfmin", v3.Vec{2, 4, 6}, 1.0},
		{"mixed_sizes_rounded", v3.Vec{4, 2, 8}, 0.5},
	}
	for _, c := range cases {
		c := c
		t.Run(c.name, func(t *testing.T) {
			s, err := Box3D(c.size, c.round)
			if err != nil {
				t.Fatalf("Box3D(%v,%v): %v", c.size, c.round, err)
			}
			assertBboxContainsSurface(t, c.name, s, prim3dSampleN)
		})
	}
}

//-----------------------------------------------------------------------------

func Test_Prim3D_Sphere3D_BboxContainsSurface(t *testing.T) {
	cases := []struct {
		name   string
		radius float64
	}{
		{"tiny", 1e-6},
		{"sub_unit", 0.01},
		{"unit", 1.0},
		{"medium", 100.0},
		{"huge", 1e6},
	}
	for _, c := range cases {
		c := c
		t.Run(c.name, func(t *testing.T) {
			s, err := Sphere3D(c.radius)
			if err != nil {
				t.Fatalf("Sphere3D(%v): %v", c.radius, err)
			}
			assertBboxContainsSurface(t, c.name, s, prim3dSampleN)
		})
	}
}

//-----------------------------------------------------------------------------

func Test_Prim3D_Cylinder3D_BboxContainsSurface(t *testing.T) {
	cases := []struct {
		name           string
		height, radius float64
		round          float64
	}{
		{"unit_no_round", 2, 1, 0},
		{"round_just_below_radius", 2, 1, 0.99},
		{"round_equals_radius", 2, 1, 1.0},
		{"tall_thin", 100, 0.5, 0},
		{"short_fat", 0.1, 10, 0},
		{"short_fat_rounded", 0.2, 10, 0.1},
		{"tall_thin_rounded", 100, 0.5, 0.25},
		{"tiny", 1e-3, 1e-3, 0},
		{"huge", 1e6, 1e6, 0},
	}
	for _, c := range cases {
		c := c
		t.Run(c.name, func(t *testing.T) {
			s, err := Cylinder3D(c.height, c.radius, c.round)
			if err != nil {
				t.Fatalf("Cylinder3D(h=%v,r=%v,round=%v): %v", c.height, c.radius, c.round, err)
			}
			assertBboxContainsSurface(t, c.name, s, prim3dSampleN)
		})
	}
}

//-----------------------------------------------------------------------------

func Test_Prim3D_Capsule3D_BboxContainsSurface(t *testing.T) {
	// Capsule3D is Cylinder3D(height, radius, radius) — degenerate round==radius.
	cases := []struct {
		name           string
		height, radius float64
	}{
		{"unit", 2, 1},
		{"tall", 100, 1},
		{"squat", 2, 1}, // height == 2*radius — capsule is a sphere
		{"tiny", 1e-3, 1e-4},
		{"huge", 1e6, 1e3},
	}
	for _, c := range cases {
		c := c
		t.Run(c.name, func(t *testing.T) {
			s, err := Capsule3D(c.height, c.radius)
			if err != nil {
				t.Fatalf("Capsule3D(h=%v,r=%v): %v", c.height, c.radius, err)
			}
			assertBboxContainsSurface(t, c.name, s, prim3dSampleN)
		})
	}
}

//-----------------------------------------------------------------------------

func Test_Prim3D_Cone3D_BboxContainsSurface(t *testing.T) {
	cases := []struct {
		name           string
		height, r0, r1 float64
		round          float64
	}{
		{"cylinder_r0_eq_r1", 2, 1, 1, 0},
		{"point_top", 2, 1, 0, 0},
		{"point_bottom", 2, 0, 1, 0},
		{"normal_truncated", 4, 2, 1, 0},
		{"extreme_aspect_tall", 100, 1, 0.1, 0},
		{"extreme_aspect_squat", 0.1, 10, 5, 0},
		{"rounded_truncated", 4, 2, 1, 0.1},
		{"rounded_point_top", 4, 2, 0, 0.1},
		{"rounded_point_bottom", 4, 0, 2, 0.1},
		{"tiny", 1e-3, 1e-3, 5e-4, 0},
		{"huge", 1e6, 1e6, 5e5, 0},
		{"inverted_r1_gt_r0", 2, 0.5, 2, 0},
	}
	for _, c := range cases {
		c := c
		t.Run(c.name, func(t *testing.T) {
			s, err := Cone3D(c.height, c.r0, c.r1, c.round)
			if err != nil {
				t.Fatalf("Cone3D(h=%v,r0=%v,r1=%v,round=%v): %v", c.height, c.r0, c.r1, c.round, err)
			}
			assertBboxContainsSurface(t, c.name, s, prim3dSampleN)
		})
	}
}
