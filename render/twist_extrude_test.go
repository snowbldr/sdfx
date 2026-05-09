package render

import (
	"testing"

	"github.com/deadsy/sdfx/sdf"
	v2 "github.com/deadsy/sdfx/vec/v2"
)

// TwistExtrude3D is not a 1-Lipschitz SDF: the un-twist mapping has
// Jacobian σ_max = √(1 + k²r²) where k = twist/height, so the 2D-side
// distance overstates the true 3D distance by up to that factor. The
// octree marching-cubes renderer prunes cubes by |sdf(center)| ≥
// half-diagonal, so an over-stated distance causes it to skip cubes
// that contain the surface — holes. The fix divides the Evaluate
// result by σ_max (computed at construction from the bbox-bounded
// rMax) so the SDF stays a valid Lipschitz-1 distance estimator.
//
// This test exercises the regime that exposes the bug: a 2D shape sized
// to put rMax near the bbox extreme, twisted enough that k²r² is order
// one, rendered via the octree at moderate cell density.

func twistExtrudeWatertightCases() []struct {
	name   string
	twist  float64
	height float64
	make2D func() sdf.SDF2
} {
	box4 := func() sdf.SDF2 { return sdf.Box2D(v2.Vec{X: 4, Y: 4}, 0) }
	thin := func() sdf.SDF2 { return sdf.Box2D(v2.Vec{X: 8, Y: 1}, 0) }
	return []struct {
		name   string
		twist  float64
		height float64
		make2D func() sdf.SDF2
	}{
		{"box4_30deg", sdf.DtoR(30), 5, box4},
		{"box4_90deg", sdf.DtoR(90), 5, box4},
		{"box4_180deg", sdf.DtoR(180), 5, box4},
		{"box4_360deg", 2 * sdf.Pi, 5, box4},
		{"box4_2turns", 4 * sdf.Pi, 10, box4},
		{"thin_90deg", sdf.DtoR(90), 5, thin},
		{"thin_180deg", sdf.DtoR(180), 5, thin},
		{"thin_360deg", 2 * sdf.Pi, 10, thin},
	}
}

func Test_TwistExtrude3D_Watertight(t *testing.T) {
	const cells = 80
	for _, c := range twistExtrudeWatertightCases() {
		t.Run(c.name, func(t *testing.T) {
			s := sdf.TwistExtrude3D(c.make2D(), c.height, c.twist)
			tris := CollectTriangles(s, NewMarchingCubesOctree(cells))
			be := CountBoundaryEdges(tris)
			if be != 0 {
				t.Errorf("octree mesh has %d boundary edges (want 0 for watertight); %d tris", be, len(tris))
			}
			t.Logf("%d tris, %d boundary edges", len(tris), be)
		})
	}
}
