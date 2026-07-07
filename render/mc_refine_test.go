package render

import (
	"math"
	"testing"

	"github.com/snowbldr/sdfx/sdf"
	v3 "github.com/snowbldr/sdfx/vec/v3"
)

// maxVertexError renders s and returns the maximum |sdf| over all mesh
// vertices — how far marching-cubes vertices land from the true surface.
func maxVertexError(t *testing.T, s sdf.SDF3, r Render3) float64 {
	tris := CollectTriangles(s, r)
	worst := 0.0
	for _, tri := range tris {
		for _, p := range tri {
			if d := math.Abs(s.Evaluate(p)); d > worst {
				worst = d
			}
		}
	}
	return worst
}

// Test_MC_EdgeRefinement: on a CSG model the SDF is kinked (the active
// primitive switches mid-edge), so plain linear interpolation lands mesh
// vertices well off the true surface. Bisection refinement along the edge
// must place every vertex within a small fraction of a cell.
func Test_MC_EdgeRefinement(t *testing.T) {
	box, _ := sdf.Box3D(v3.Vec{X: 20, Y: 20, Z: 20}, 0)
	ball, _ := sdf.Sphere3D(11.5)
	s := sdf.Difference3D(box, ball)

	const cells = 64
	cell := 20.0 / cells

	plain := maxVertexError(t, s, NewMarchingCubesOctree(cells).Refine(0))
	refined := maxVertexError(t, s, NewMarchingCubesOctree(cells))
	t.Logf("cell=%.3f  max vertex error: plain=%.5f refined=%.6f (%.0fx better)",
		cell, plain, refined, plain/refined)

	if refined > cell/100 {
		t.Errorf("refined vertices should sit within cell/100=%.5f of the surface, got %.5f", cell/100, refined)
	}
	if refined > plain/5 {
		t.Errorf("refinement should be at least 5x more accurate: plain=%.5f refined=%.5f", plain, refined)
	}
}
