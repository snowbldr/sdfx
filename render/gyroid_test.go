package render

import (
	"testing"

	"github.com/deadsy/sdfx/sdf"
	v3 "github.com/deadsy/sdfx/vec/v3"
)

// Gyroid3D is not a true SDF — its implicit function
//   g(p) = sin(kx·x)·cos(ky·y) + sin(ky·y)·cos(kz·z) + sin(kz·z)·cos(kx·x)
// has gradient up to √3·max(kᵢ), so unrescaled it overstates 3D distance
// by that factor. The octree marching-cubes renderer's isEmpty pruning
// (|sdf(center)| ≥ half-diagonal) then skips cubes that contain surface,
// producing holes. The fix divides Evaluate by √3·max(kᵢ).
//
// Gyroid is infinite, so we intersect with a box to get a renderable
// finite shape and assert the resulting mesh is watertight.

func Test_Gyroid3D_Watertight(t *testing.T) {
	const cells = 80
	box, err := sdf.Box3D(v3.Vec{X: 10, Y: 10, Z: 10}, 0)
	if err != nil {
		t.Fatal(err)
	}
	cases := []struct {
		name  string
		scale v3.Vec
	}{
		{"isotropic_2", v3.Vec{X: 2, Y: 2, Z: 2}},
		{"isotropic_3", v3.Vec{X: 3, Y: 3, Z: 3}},
		{"anisotropic_2_3_4", v3.Vec{X: 2, Y: 3, Z: 4}},
		{"fine_1", v3.Vec{X: 1, Y: 1, Z: 1}},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			g, err := sdf.Gyroid3D(c.scale)
			if err != nil {
				t.Fatal(err)
			}
			// Gyroid has empty bbox; put the box first so Intersect3D's
			// `s0.BoundingBox()` heuristic gives the box's bbox.
			s := sdf.Intersect3D(box, g)
			tris := CollectTriangles(s, NewMarchingCubesOctree(cells))
			be := CountBoundaryEdges(tris)
			if be != 0 {
				t.Errorf("octree mesh has %d boundary edges (want 0); %d tris", be, len(tris))
			}
			t.Logf("%d tris, %d boundary edges", len(tris), be)
		})
	}
}
