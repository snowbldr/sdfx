package render

import (
	"testing"

	"github.com/deadsy/sdfx/sdf"
	v2 "github.com/deadsy/sdfx/vec/v2"
	v3 "github.com/deadsy/sdfx/vec/v3"
	"github.com/deadsy/sdfx/vec/v3i"
)

// Covers combinators whose impl isn't trivially Lipschitz-1 on inspection:
// ExtrudeRounded3D (branching sqrt), Elongate3D (clamp-based domain shift),
// Array3D (union + bbox pruning), RotateUnion3D (rotation composed with
// union). Trivial combinators (Offset3D, Shell3D, default Union/Intersect/
// Difference, Cut3D, ScaleUniform3D) are exercised transitively elsewhere.

func combinatorCfgs() []extrudeCfg {
	return []extrudeCfg{
		// --- ExtrudeRounded3D: sqrt(a² + b²) - round with a/b-sign branches.
		{"extrudeRounded_box_c50", func(t *testing.T) sdf.SDF3 {
			s, err := sdf.ExtrudeRounded3D(sdf.Box2D(v2.Vec{X: 10, Y: 4}, 0), 8, 1)
			if err != nil {
				t.Fatal(err)
			}
			return s
		}, 50},
		{"extrudeRounded_star_c100", func(t *testing.T) sdf.SDF3 {
			s, err := sdf.ExtrudeRounded3D(starProfile(t), 6, 0.8)
			if err != nil {
				t.Fatal(err)
			}
			return s
		}, 100},

		// --- Elongate3D: q = p - clamp(p, -h/2, h/2); shifts domain onto axis.
		{"elongate_sphere_c50", func(t *testing.T) sdf.SDF3 {
			s, err := sdf.Sphere3D(2)
			if err != nil {
				t.Fatal(err)
			}
			return sdf.Elongate3D(s, v3.Vec{X: 6, Y: 0, Z: 0})
		}, 50},
		{"elongate_sphere_3axis_c100", func(t *testing.T) sdf.SDF3 {
			s, err := sdf.Sphere3D(1.5)
			if err != nil {
				t.Fatal(err)
			}
			return sdf.Elongate3D(s, v3.Vec{X: 5, Y: 3, Z: 2})
		}, 100},
		// A cube elongated to 3:3:3-aspect-ratio stays away from the Box3D
		// MC aliasing at thin-rectangular aspect ratios (see box 6:2:2 vs
		// 6:3:3 — the latter is clean, the former has pre-existing MC
		// artifacts unrelated to Elongate3D).
		{"elongate_cube_c100", func(t *testing.T) sdf.SDF3 {
			s, err := sdf.Box3D(v3.Vec{X: 3, Y: 3, Z: 3}, 0.3)
			if err != nil {
				t.Fatal(err)
			}
			return sdf.Elongate3D(s, v3.Vec{X: 3, Y: 0, Z: 0})
		}, 100},

		// --- Array3D: union of translated copies. Bbox pruning inside Union
		// relies on child bboxes being tight — a packed array stresses that.
		{"array_sphere_3x3x3_c100", func(t *testing.T) sdf.SDF3 {
			s, err := sdf.Sphere3D(0.8)
			if err != nil {
				t.Fatal(err)
			}
			return sdf.Array3D(s, v3i.Vec{3, 3, 3}, v3.Vec{X: 2, Y: 2, Z: 2})
		}, 100},
		{"array_box_4x1x1_c100", func(t *testing.T) sdf.SDF3 {
			s, err := sdf.Box3D(v3.Vec{X: 1, Y: 1, Z: 1}, 0.2)
			if err != nil {
				t.Fatal(err)
			}
			return sdf.Array3D(s, v3i.Vec{4, 1, 1}, v3.Vec{X: 1.5, Y: 0, Z: 0})
		}, 100},
		// Overlapping copies — boundary between siblings is the union seam.
		{"array_overlap_c100", func(t *testing.T) sdf.SDF3 {
			s, err := sdf.Sphere3D(1)
			if err != nil {
				t.Fatal(err)
			}
			return sdf.Array3D(s, v3i.Vec{3, 1, 1}, v3.Vec{X: 1.2, Y: 0, Z: 0})
		}, 100},

		// --- RotateUnion3D: union of N rotated copies around an axis.
		{"rotateUnion_box_6x_c100", func(t *testing.T) sdf.SDF3 {
			s, err := sdf.Box3D(v3.Vec{X: 1, Y: 1, Z: 4}, 0.1)
			if err != nil {
				t.Fatal(err)
			}
			s = sdf.Transform3D(s, sdf.Translate3d(v3.Vec{X: 3, Y: 0, Z: 0}))
			return sdf.RotateUnion3D(s, 6, sdf.RotateZ(sdf.Tau/6))
		}, 100},
		{"rotateUnion_sphere_12x_c100", func(t *testing.T) sdf.SDF3 {
			s, err := sdf.Sphere3D(0.6)
			if err != nil {
				t.Fatal(err)
			}
			s = sdf.Transform3D(s, sdf.Translate3d(v3.Vec{X: 2.5, Y: 0, Z: 0}))
			return sdf.RotateUnion3D(s, 12, sdf.RotateZ(sdf.Tau/12))
		}, 100},
	}
}

func Test_Combinators_Watertight(t *testing.T) {
	runExtrudeCfgs(t, combinatorCfgs())
}
