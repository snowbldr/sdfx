package render

import (
	"testing"

	"github.com/deadsy/sdfx/sdf"
	v3 "github.com/deadsy/sdfx/vec/v3"
)

// VoxelSDF3 trilinear-interpolates corner values from a coarse grid. Even
// when the source SDF is Lipschitz-1, the trilinear interpolant's gradient
// can reach √(Lx² + Ly² + Lz²) where each Lᵢ approaches 1 — i.e. up to √3
// — so unrescaled it overstates 3D distance. The octree marching-cubes
// renderer's |sdf(center)| ≥ half-diagonal pruning then skips cubes that
// contain surface, producing holes. The fix divides Evaluate by the
// measured per-axis Lipschitz bound (computed at construction from
// adjacent-corner deltas).

func Test_VoxelSDF3_Watertight(t *testing.T) {
	const renderCells = 80
	cases := []struct {
		name     string
		src      func() sdf.SDF3
		voxCells int
	}{
		{
			"sphere_coarse_voxels",
			func() sdf.SDF3 { s, _ := sdf.Sphere3D(2); return s },
			16,
		},
		{
			"sphere_fine_voxels",
			func() sdf.SDF3 { s, _ := sdf.Sphere3D(2); return s },
			32,
		},
		{
			"rounded_box",
			func() sdf.SDF3 { s, _ := sdf.Box3D(v3.Vec{X: 4, Y: 3, Z: 2}, 0.3); return s },
			32,
		},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			v := sdf.NewVoxelSDF3(c.src(), c.voxCells, nil)
			tris := CollectTriangles(v, NewMarchingCubesOctree(renderCells))
			be := CountBoundaryEdges(tris)
			if be != 0 {
				t.Errorf("octree mesh has %d boundary edges (want 0); %d tris", be, len(tris))
			}
			t.Logf("%d tris, %d boundary edges", len(tris), be)
		})
	}
}
