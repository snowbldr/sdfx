package render

import (
	"testing"

	"github.com/deadsy/sdfx/sdf"
	v2 "github.com/deadsy/sdfx/vec/v2"
)

// ScaleExtrude3D and ScaleTwistExtrude3D un-scale (and un-twist) the
// query before forwarding to the 2D SDF. The Jacobian of the inverse
// map has off-diagonal entries from ∂/∂z, so σ_max > 1 even at modest
// scale ratios; for the twisted variant the rotation × scale composite
// inflates further. Without correction the result overstates 3D
// distance and the octree marching-cubes renderer's |sdf(center)| ≥
// half-diagonal pruning skips cubes that contain surface — holes.

func Test_ScaleExtrude3D_Watertight(t *testing.T) {
	const cells = 80
	box := func() sdf.SDF2 { return sdf.Box2D(v2.Vec{X: 4, Y: 4}, 0) }
	cases := []struct {
		name   string
		height float64
		scale  v2.Vec
	}{
		{"shrink_2x_uniform", 5, v2.Vec{X: 0.5, Y: 0.5}},
		{"shrink_3x_uniform", 5, v2.Vec{X: 1.0 / 3, Y: 1.0 / 3}},
		{"shrink_anisotropic", 5, v2.Vec{X: 0.4, Y: 0.7}},
		{"grow_2x_uniform", 5, v2.Vec{X: 2, Y: 2}},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s := sdf.ScaleExtrude3D(box(), c.height, c.scale)
			tris := CollectTriangles(s, NewMarchingCubesOctree(cells))
			be := CountBoundaryEdges(tris)
			if be != 0 {
				t.Errorf("octree mesh has %d boundary edges (want 0); %d tris", be, len(tris))
			}
			t.Logf("%d tris, %d boundary edges", len(tris), be)
		})
	}
}

func Test_ScaleTwistExtrude3D_Watertight(t *testing.T) {
	const cells = 80
	box := func() sdf.SDF2 { return sdf.Box2D(v2.Vec{X: 4, Y: 4}, 0) }
	cases := []struct {
		name   string
		height float64
		twist  float64
		scale  v2.Vec
	}{
		{"shrink_30deg", 5, sdf.DtoR(30), v2.Vec{X: 0.5, Y: 0.5}},
		{"shrink_90deg", 5, sdf.DtoR(90), v2.Vec{X: 0.5, Y: 0.5}},
		{"shrink_180deg", 5, sdf.DtoR(180), v2.Vec{X: 0.5, Y: 0.5}},
		{"grow_360deg", 5, 2 * sdf.Pi, v2.Vec{X: 1.5, Y: 1.5}},
		{"shrink_aniso_90deg", 5, sdf.DtoR(90), v2.Vec{X: 0.4, Y: 0.7}},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s := sdf.ScaleTwistExtrude3D(box(), c.height, c.twist, c.scale)
			tris := CollectTriangles(s, NewMarchingCubesOctree(cells))
			be := CountBoundaryEdges(tris)
			if be != 0 {
				t.Errorf("octree mesh has %d boundary edges (want 0); %d tris", be, len(tris))
			}
			t.Logf("%d tris, %d boundary edges", len(tris), be)
		})
	}
}
