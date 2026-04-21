package render

import (
	"testing"

	"github.com/deadsy/sdfx/sdf"
	v3 "github.com/deadsy/sdfx/vec/v3"
)

// Cone3D.Evaluate has five branches over the (r, z) cone-silhouette cross
// section (above-cap, below-cap, interior, slope-side, base-vertex,
// top-vertex). Each branch uses a different distance formula, so seams at
// the branch boundaries are where Lipschitz violations or wrong-formula
// bugs show up. These configs cover pointed cones (r1=0), taper-up,
// taper-down, cylindrical (r0≈r1), and fat-round cones that exercise the
// slope-inset math (`ofs = round / n.X`).
//
// RotateCopy3D maps p through (√(x²+y²), sawtooth(atan2(y,x), θ)) into the
// first angular sector before delegating. The mapping is discontinuous at
// the sector seam, so the child SDF must be entirely contained within a
// single sector for the render to be watertight. These tests give child
// shapes that live well inside one sector.

func primitiveCfgs() []extrudeCfg {
	return []extrudeCfg{
		// --- Cone3D: pointed tip (r1=0). Branch: top vertex case.
		{"cone_tip_c80", func(t *testing.T) sdf.SDF3 {
			s, err := sdf.Cone3D(4, 2, 0, 0)
			if err != nil {
				t.Fatal(err)
			}
			return s
		}, 80},
		// Pointed tip with rounded base edge (exercises slope-inset ofs).
		{"cone_tip_rounded_c100", func(t *testing.T) sdf.SDF3 {
			s, err := sdf.Cone3D(4, 2, 0, 0.3)
			if err != nil {
				t.Fatal(err)
			}
			return s
		}, 100},
		// Truncated taper-up (r1 > r0).
		{"cone_taper_up_c80", func(t *testing.T) sdf.SDF3 {
			s, err := sdf.Cone3D(4, 1, 2.5, 0)
			if err != nil {
				t.Fatal(err)
			}
			return s
		}, 80},
		// Truncated taper-down (r1 < r0).
		{"cone_taper_down_c100", func(t *testing.T) sdf.SDF3 {
			s, err := sdf.Cone3D(3, 2, 0.5, 0.2)
			if err != nil {
				t.Fatal(err)
			}
			return s
		}, 100},
		// Near-cylinder (r0 ≈ r1) with rounded caps.
		{"cone_nearcyl_c100", func(t *testing.T) sdf.SDF3 {
			s, err := sdf.Cone3D(4, 2, 2.1, 0.3)
			if err != nil {
				t.Fatal(err)
			}
			return s
		}, 100},
		// Steep taper with small round — stresses the (r-r)/height slope math.
		{"cone_steep_c100", func(t *testing.T) sdf.SDF3 {
			s, err := sdf.Cone3D(8, 3, 0.2, 0.1)
			if err != nil {
				t.Fatal(err)
			}
			return s
		}, 100},

		// --- RotateCopy3D: child must live inside one 2π/n sector.
		// A small box offset along +X, copied 6x (60° sectors).
		{"rotateCopy_box_6x_c100", func(t *testing.T) sdf.SDF3 {
			b, err := sdf.Box3D(v3.Vec{X: 1, Y: 1, Z: 2}, 0.1)
			if err != nil {
				t.Fatal(err)
			}
			b = sdf.Transform3D(b, sdf.Translate3d(v3.Vec{X: 3, Y: 0, Z: 0}))
			return sdf.RotateCopy3D(b, 6)
		}, 100},
		// Smaller child, more copies — thinner sectors.
		{"rotateCopy_sphere_12x_c100", func(t *testing.T) sdf.SDF3 {
			s, err := sdf.Sphere3D(0.5)
			if err != nil {
				t.Fatal(err)
			}
			s = sdf.Transform3D(s, sdf.Translate3d(v3.Vec{X: 3, Y: 0, Z: 0}))
			return sdf.RotateCopy3D(s, 12)
		}, 100},
		// Cone offset along X, copied — exercises RotateCopy over non-trivial
		// primitive shape with polar asymmetry.
		{"rotateCopy_cone_8x_c100", func(t *testing.T) sdf.SDF3 {
			c, err := sdf.Cone3D(3, 0.8, 0.2, 0.1)
			if err != nil {
				t.Fatal(err)
			}
			c = sdf.Transform3D(c, sdf.Translate3d(v3.Vec{X: 3, Y: 0, Z: 0}))
			return sdf.RotateCopy3D(c, 8)
		}, 100},
	}
}

func Test_Primitives_Watertight(t *testing.T) {
	runExtrudeCfgs(t, primitiveCfgs())
}
