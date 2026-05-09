package render

import (
	"testing"

	"github.com/deadsy/sdfx/sdf"
	v2 "github.com/deadsy/sdfx/vec/v2"
)

// Loft3D linearly blends between two 2D SDFs along z. The xy gradient is
// a convex combination of the two Lipschitz-1 inner gradients (so still
// ≤ 1), but the z gradient grows with max|a1−a0|/(2·h_inner). The outer
// d = √(a² + b²) end-cap composition further inflates the Lipschitz
// constant. Without correction, the result overstates 3D distance and
// the octree marching-cubes renderer's |sdf(center)| ≥ half-diagonal
// pruning skips cubes that contain surface, producing holes.
//
// This test exercises the regime where the bug shows up: lofting
// between two shapes of significantly different size (so max|a1−a0|
// is substantial) with a short height (so L_z is large).

func Test_Loft3D_Watertight(t *testing.T) {
	const cells = 80
	smallBox := func() sdf.SDF2 { return sdf.Box2D(v2.Vec{X: 1, Y: 1}, 0.1) }
	bigBox := func() sdf.SDF2 { return sdf.Box2D(v2.Vec{X: 5, Y: 5}, 0.1) }
	smallCircle := func() sdf.SDF2 { c, _ := sdf.Circle2D(0.5); return c }
	bigCircle := func() sdf.SDF2 { c, _ := sdf.Circle2D(3); return c }
	cases := []struct {
		name          string
		sdf0, sdf1    func() sdf.SDF2
		height, round float64
	}{
		{"small_to_big_box_h2", smallBox, bigBox, 2, 0},
		{"small_to_big_box_h5", smallBox, bigBox, 5, 0.05},
		{"box_to_circle", bigBox, bigCircle, 5, 0.05},
		{"circle_to_circle_growing", smallCircle, bigCircle, 3, 0.05},
		{"circle_to_circle_short", smallCircle, bigCircle, 1, 0},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s, err := sdf.Loft3D(c.sdf0(), c.sdf1(), c.height, c.round)
			if err != nil {
				t.Fatal(err)
			}
			tris := CollectTriangles(s, NewMarchingCubesOctree(cells))
			be := CountBoundaryEdges(tris)
			if be != 0 {
				t.Errorf("octree mesh has %d boundary edges (want 0); %d tris", be, len(tris))
			}
			t.Logf("%d tris, %d boundary edges", len(tris), be)
		})
	}
}
