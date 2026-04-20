package render

import (
	"testing"

	"github.com/snowbldr/sdfx/sdf"
	v3 "github.com/snowbldr/sdfx/vec/v3"
)

// VoxelSDF3 trilinear-interpolates a grid of pre-sampled SDF values. Even
// when the source is a valid Lipschitz-1 SDF, the trilinear gradient in a
// voxel can reach magnitude √3 — one partial per axis can approach 1
// simultaneously. Without correction, the octree's isEmpty skip can drop
// cubes near the surface (holes). These tests exercise the fix against
// sources whose Lipschitz is near 1 (sphere, box, cylinder) across grid
// densities; uncorrected, the coarse-grid sphere case leaks ~9 edges.
//
// VoxelSDF3 cannot faithfully represent sources with discontinuous
// gradients (e.g. Intersect3D of two primitives), which produce MC
// ambiguity regardless of grid resolution — that limitation is inherent
// to trilinear interp, not a Lipschitz issue, and is out of scope here.

func voxelCfgs() []extrudeCfg {
	sphere := func(t *testing.T) sdf.SDF3 {
		s, err := sdf.Sphere3D(5)
		if err != nil {
			t.Fatal(err)
		}
		return s
	}
	cylinder := func(t *testing.T) sdf.SDF3 {
		c, err := sdf.Cylinder3D(8, 4, 0.5)
		if err != nil {
			t.Fatal(err)
		}
		return c
	}
	roundedBox := func(t *testing.T) sdf.SDF3 {
		b, err := sdf.Box3D(v3.Vec{X: 8, Y: 6, Z: 4}, 1)
		if err != nil {
			t.Fatal(err)
		}
		return b
	}
	wrap := func(src func(*testing.T) sdf.SDF3, voxels int) func(*testing.T) sdf.SDF3 {
		return func(t *testing.T) sdf.SDF3 {
			return sdf.NewVoxelSDF3(src(t), voxels, nil)
		}
	}
	return []extrudeCfg{
		// Sphere — smooth, Lipschitz-1. Without the √3 correction, v=20
		// leaks edges at c=50. With the correction, all pass.
		{"sphere_v20_c50", wrap(sphere, 20), 50},
		{"sphere_v40_c100", wrap(sphere, 40), 100},
		{"sphere_v80_c100", wrap(sphere, 80), 100},
		// Cylinder — mixed smooth/flat regions, two sharp rim circles.
		{"cylinder_v40_c80", wrap(cylinder, 40), 80},
		{"cylinder_v80_c120", wrap(cylinder, 80), 120},
		// Rounded box — rounded edges avoid the MC aliasing that sharp
		// box faces at specific cell counts can induce.
		{"roundedbox_v40_c80", wrap(roundedBox, 40), 80},
		{"roundedbox_v80_c120", wrap(roundedBox, 80), 120},
	}
}

func Test_VoxelSDF3_Watertight(t *testing.T) {
	runExtrudeCfgs(t, voxelCfgs())
}
