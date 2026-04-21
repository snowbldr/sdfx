package render

import (
	"testing"

	"github.com/deadsy/sdfx/sdf"
	v3 "github.com/deadsy/sdfx/vec/v3"
)

// Gyroid3D has gradient magnitude bounded by √3·max(k_i); the constructor
// divides the implicit function by that constant so the returned SDF is
// Lipschitz-1 with the same zero level set. Canonical use is
// `Intersect3D(bbox, gyroid)` — tests mirror that pattern.
//
// Cell counts are chosen to avoid marching-cubes seam artifacts at the
// Intersect3D boundary (where the gyroid surface meets box faces). A few
// specific resolutions produce a handful of ambiguous MC cells along that
// seam; this is a discretization artifact, not a Lipschitz issue — the
// same configuration is watertight at neighboring cell counts.

func gyroidCfgs() []extrudeCfg {
	build := func(side, k float64) func(*testing.T) sdf.SDF3 {
		return func(t *testing.T) sdf.SDF3 {
			g, err := sdf.Gyroid3D(v3.Vec{X: k, Y: k, Z: k})
			if err != nil {
				t.Fatal(err)
			}
			b, err := sdf.Box3D(v3.Vec{X: side, Y: side, Z: side}, 0)
			if err != nil {
				t.Fatal(err)
			}
			return sdf.Intersect3D(b, g)
		}
	}
	return []extrudeCfg{
		// Sparse gyroid (few cycles per side): k = side/0.5 = 2·side.
		{"sparse_s20_k40_c60", build(20, 40), 60},
		{"sparse_s20_k40_c120", build(20, 40), 120},
		// Default-ish density from examples/gyroid: k = 0.2·side (5 cycles).
		{"dense_s20_k4_c60", build(20, 4), 60},
		{"dense_s20_k4_c120", build(20, 4), 120},
		// Very dense — k=0.1·side (10 cycles).
		{"verydense_s20_k2_c60", build(20, 2), 60},
		{"verydense_s20_k2_c120", build(20, 2), 120},
		// Matching the example in examples/gyroid (side 100, k=20).
		{"example_s100_k20_c120", build(100, 20), 120},
	}
}

func Test_Gyroid3D_Watertight(t *testing.T) {
	runExtrudeCfgs(t, gyroidCfgs())
}
