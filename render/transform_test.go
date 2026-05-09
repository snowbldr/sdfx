package render

import (
	"math"
	"testing"

	"github.com/deadsy/sdfx/sdf"
	v3 "github.com/deadsy/sdfx/vec/v3"
)

// Transform3D applies M⁻¹ to the query point and forwards the result. If
// M is non-orthonormal (any scaling), |Evaluate(M⁻¹·p)| can stretch true
// 3D distance by σ_max(M⁻¹) > 1, which makes the SDF a non-Lipschitz-1
// distance estimator. The octree marching-cubes renderer's
// |sdf(center)| ≥ half-diagonal pruning then drops cubes containing
// surface — holes. The fix multiplies the result by 1/σ_max(M⁻¹).
//
// This test exercises non-uniform scaling (where σ_max > 1) on a sphere,
// rendered via the octree, asserting zero boundary edges.

func Test_Transform3D_Watertight(t *testing.T) {
	const cells = 80
	sphere, err := sdf.Sphere3D(1)
	if err != nil {
		t.Fatal(err)
	}
	cases := []struct {
		name  string
		scale v3.Vec
	}{
		{"scale_2x_uniform", v3.Vec{X: 2, Y: 2, Z: 2}},
		{"scale_3_1_1", v3.Vec{X: 3, Y: 1, Z: 1}},
		{"scale_1_1_3", v3.Vec{X: 1, Y: 1, Z: 3}},
		{"scale_2_3_4", v3.Vec{X: 2, Y: 3, Z: 4}},
		{"scale_5_1_5", v3.Vec{X: 5, Y: 1, Z: 5}},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			s := sdf.Transform3D(sphere, sdf.Scale3d(c.scale))
			tris := CollectTriangles(s, NewMarchingCubesOctree(cells))
			be := CountBoundaryEdges(tris)
			if be != 0 {
				t.Errorf("octree mesh has %d boundary edges (want 0); %d tris", be, len(tris))
			}
			t.Logf("%d tris, %d boundary edges", len(tris), be)
		})
	}
}

// Transform3D with rotation+translation only (orthonormal) must NOT
// require any correction — invStretch should equal 1 exactly. Pin this
// so a future tightening of the σ_max calc doesn't accidentally apply
// a < 1 factor to rigid transforms (which would shrink the SDF below
// the true distance and make the renderer over-evaluate, slow but not
// break anything — still worth keeping the path tight).
func Test_Transform3D_RigidPreservesDistance(t *testing.T) {
	sphere, err := sdf.Sphere3D(1)
	if err != nil {
		t.Fatal(err)
	}
	rotated := sdf.Transform3D(sphere, sdf.RotateX(sdf.DtoR(37)))
	// Outside the sphere by 1 unit: SDF must be ≈ 1.
	d := rotated.Evaluate(v3.Vec{X: 2, Y: 0, Z: 0})
	if math.Abs(d-1) > 1e-9 {
		t.Errorf("rotated sphere SDF at distance 1 = %v, want ≈1", d)
	}
}
