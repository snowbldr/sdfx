package render

import (
	"testing"

	"github.com/deadsy/sdfx/sdf"
	v3 "github.com/deadsy/sdfx/vec/v3"
)

// Transform3D evaluates the underlying SDF at M⁻¹·p and returns the result
// unscaled. The Lipschitz factor of the composite is σ_max(M⁻¹). Rigid
// transforms (rotation, translation) give σ=1 and are safe. For a pure
// scale:
//   • factor ≥ 1: M⁻¹ shrinks → σ ≤ 1 → SDF underestimates, no holes
//   • factor < 1: M⁻¹ expands → σ > 1 → SDF overestimates → octree holes
// Shears can introduce σ > 1 along the sheared axis.
//
// These tests exercise both safe and unsafe regimes with a simple underlying
// SDF so any hole must be attributable to the transform.

func transformCfgs() []extrudeCfg {
	sphere := func(t *testing.T) sdf.SDF3 {
		s, err := sdf.Sphere3D(1)
		if err != nil {
			t.Fatal(err)
		}
		return s
	}
	box := func(t *testing.T, size float64) sdf.SDF3 {
		b, err := sdf.Box3D(v3.Vec{X: size, Y: size, Z: size}, 0.1)
		if err != nil {
			t.Fatal(err)
		}
		return b
	}
	return []extrudeCfg{
		// --- Safe baselines: rigid transforms keep Lipschitz=1 ---
		{"identity_sphere_c50", func(t *testing.T) sdf.SDF3 {
			return sdf.Transform3D(sphere(t), sdf.Identity3d())
		}, 50},
		{"translate_sphere_c50", func(t *testing.T) sdf.SDF3 {
			return sdf.Transform3D(sphere(t), sdf.Translate3d(v3.Vec{X: 2, Y: 0, Z: 0}))
		}, 50},
		{"rotateY_sphere_c50", func(t *testing.T) sdf.SDF3 {
			return sdf.Transform3D(sphere(t), sdf.RotateY(sdf.Pi/4))
		}, 50},
		{"rotateY_box_c100", func(t *testing.T) sdf.SDF3 {
			return sdf.Transform3D(box(t, 4), sdf.RotateY(sdf.Pi/6))
		}, 100},

		// --- Safe: expansion (factor > 1) gives σ < 1, underestimates ---
		{"expand_2x_sphere_c50", func(t *testing.T) sdf.SDF3 {
			return sdf.Transform3D(sphere(t), sdf.Scale3d(v3.Vec{X: 2, Y: 2, Z: 2}))
		}, 50},
		{"expand_3x_box_c100", func(t *testing.T) sdf.SDF3 {
			return sdf.Transform3D(box(t, 2), sdf.Scale3d(v3.Vec{X: 3, Y: 3, Z: 3}))
		}, 100},

		// --- Unsafe: uniform shrink overestimates distance ---
		// Scale3d(0.5): σ_max = 2 → SDF twice the true distance.
		{"shrink_0.5_sphere_c50", func(t *testing.T) sdf.SDF3 {
			return sdf.Transform3D(sphere(t), sdf.Scale3d(v3.Vec{X: 0.5, Y: 0.5, Z: 0.5}))
		}, 50},
		{"shrink_0.5_sphere_c100", func(t *testing.T) sdf.SDF3 {
			return sdf.Transform3D(sphere(t), sdf.Scale3d(v3.Vec{X: 0.5, Y: 0.5, Z: 0.5}))
		}, 100},
		// Scale3d(0.3): σ_max ≈ 3.33 — strong hole risk.
		{"shrink_0.3_sphere_c50", func(t *testing.T) sdf.SDF3 {
			return sdf.Transform3D(sphere(t), sdf.Scale3d(v3.Vec{X: 0.3, Y: 0.3, Z: 0.3}))
		}, 50},
		{"shrink_0.3_sphere_c100", func(t *testing.T) sdf.SDF3 {
			return sdf.Transform3D(sphere(t), sdf.Scale3d(v3.Vec{X: 0.3, Y: 0.3, Z: 0.3}))
		}, 100},
		{"shrink_0.3_box_c100", func(t *testing.T) sdf.SDF3 {
			return sdf.Transform3D(box(t, 4), sdf.Scale3d(v3.Vec{X: 0.3, Y: 0.3, Z: 0.3}))
		}, 100},

		// --- Unsafe: anisotropic shrink overestimates along one axis ---
		{"anisotropic_0.3_1_1_c100", func(t *testing.T) sdf.SDF3 {
			return sdf.Transform3D(sphere(t), sdf.Scale3d(v3.Vec{X: 0.3, Y: 1, Z: 1}))
		}, 100},
		{"anisotropic_0.1_10_1_c100", func(t *testing.T) sdf.SDF3 {
			return sdf.Transform3D(sphere(t), sdf.Scale3d(v3.Vec{X: 0.1, Y: 10, Z: 1}))
		}, 100},

		// --- Unsafe: shear. M = I + e_xy; M⁻¹ = I - e_xy has σ_max > 1.
		// Matrix layout (row-major): row i at index 4i..4i+3. To add a
		// y-on-x shear of magnitude 2 we set M[1] = 2 (row 0, col 1).
		{"shear_xy_2_box_c100", func(t *testing.T) sdf.SDF3 {
			m := sdf.Identity3d()
			m[1] = 2 // x += 2y
			return sdf.Transform3D(box(t, 4), m)
		}, 100},
		// Shear + rotation combined — σ on sheared axis still > 1.
		{"shear_plus_rot_c100", func(t *testing.T) sdf.SDF3 {
			m := sdf.Identity3d()
			m[1] = 1 // x += y
			m = m.Mul(sdf.RotateZ(sdf.Pi / 6))
			return sdf.Transform3D(sphere(t), m)
		}, 100},
	}
}

func Test_Transform3D_Watertight(t *testing.T) {
	runExtrudeCfgs(t, transformCfgs())
}
