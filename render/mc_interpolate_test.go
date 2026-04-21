package render

import (
	"testing"

	"github.com/snowbldr/sdfx/sdf"
	v3 "github.com/snowbldr/sdfx/vec/v3"
)

// Box3D at thin-slab aspect ratios with non-trivial rounding places MC
// surface vertices near 1e-4-aligned coordinates (e.g. Y=1 for a 6x2x2
// box). Without canonicalized mcInterpolate argument order, two adjacent
// cubes sharing a swap-ordered MC edge compute interpolated points that
// differ by ~1 ULP, and the int32(c * 1e4) vertex quantization buckets
// them separately — producing phantom boundary edges on flat faces.
//
// These resolutions all failed before the fix, with BE counts of
// 1170 / 4824 / 4320 / 14012 / 20128 / 17280. All zero after.
func Test_Box3D_622_SweepResolutions(t *testing.T) {
	b, err := sdf.Box3D(v3.Vec{X: 6, Y: 2, Z: 2}, 0.3)
	if err != nil {
		t.Fatal(err)
	}
	for _, cells := range []int{50, 75, 100, 125, 150, 200} {
		tris := CollectTriangles(b, NewMarchingCubesOctree(cells))
		if be := CountBoundaryEdges(tris); be != 0 {
			t.Errorf("c=%d: %d boundary edges (want 0)", cells, be)
		}
	}
	for _, cells := range []int{50, 100, 125, 150} {
		tris := CollectTriangles(b, NewMarchingCubesUniform(cells))
		if be := CountBoundaryEdges(tris); be != 0 {
			t.Errorf("uniform c=%d: %d boundary edges (want 0)", cells, be)
		}
	}
}
