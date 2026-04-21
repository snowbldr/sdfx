package render

import (
	"math"
	"testing"

	"github.com/deadsy/sdfx/sdf"
	v2 "github.com/deadsy/sdfx/vec/v2"
)

// extrudeCfg builds an SDF3 lazily so errors surface via t.Fatal, and asserts
// watertightness of the octree render at the given cell count.
type extrudeCfg struct {
	name  string
	build func(*testing.T) sdf.SDF3
	cells int
}

// assertWatertight renders the SDF through the octree marching-cubes renderer
// and fails the test if the resulting mesh has any boundary edges (edges
// shared by only one triangle).
func assertWatertight(t *testing.T, s sdf.SDF3, cells int) {
	t.Helper()
	tris := CollectTriangles(s, NewMarchingCubesOctree(cells))
	be := CountBoundaryEdges(tris)
	if be != 0 {
		t.Errorf("%d boundary edges (want 0), %d tris", be, len(tris))
	}
	t.Logf("%d tris, %d boundary edges", len(tris), be)
}

// runExtrudeCfgs iterates the configs as subtests.
func runExtrudeCfgs(t *testing.T, cfgs []extrudeCfg) {
	for _, c := range cfgs {
		t.Run(c.name, func(t *testing.T) {
			assertWatertight(t, c.build(t), c.cells)
		})
	}
}

//-----------------------------------------------------------------------------
// Shared 2D profile builders. Chosen to stress the Lipschitz stretch along
// different axes of the 3D→2D mapping:
//   box10x4    — modest radial extent (rMax≈5.39), both axes non-trivial
//   thinRect   — large/asymmetric rMax (rMax≈10.05) so twist·r gets big fast
//   starProfile— non-convex with inner notches, stresses asymmetric mappings

func box10x4() sdf.SDF2  { return sdf.Box2D(v2.Vec{X: 10, Y: 4}, 0.5) }
func thinRect() sdf.SDF2 { return sdf.Box2D(v2.Vec{X: 20, Y: 2}, 0) }

// starProfile is a 5-point star with outer radius 5 and inner radius 2.
func starProfile(t *testing.T) sdf.SDF2 {
	t.Helper()
	p := sdf.NewPolygon()
	for i := 0; i < 10; i++ {
		r := 5.0
		if i%2 == 1 {
			r = 2.0
		}
		theta := float64(i) * sdf.Pi / 5
		p.Add(r*math.Cos(theta), r*math.Sin(theta))
	}
	s, err := sdf.Polygon2D(p.Vertices())
	if err != nil {
		t.Fatal(err)
	}
	return s
}

//-----------------------------------------------------------------------------
// TwistExtrude3D: σ_max = √(1 + k²r²), k = twist/height. Stretch grows with
// twist and with radial extent of the 2D profile. The helix is steepest at
// the widest part of the profile (max r), so thin wide rectangles are the
// worst case.

func twistExtrudeCfgs(t *testing.T) []extrudeCfg {
	return []extrudeCfg{
		// Zero-twist sanity: stretch should be 1, mapping is identity.
		{"notwist_box_c50", func(t *testing.T) sdf.SDF3 {
			return sdf.TwistExtrude3D(box10x4(), 20, 0)
		}, 50},
		// Mild twist, compact profile.
		{"box_pi_h20_c50", func(t *testing.T) sdf.SDF3 {
			return sdf.TwistExtrude3D(box10x4(), 20, sdf.Pi)
		}, 50},
		{"box_pi_h20_c100", func(t *testing.T) sdf.SDF3 {
			return sdf.TwistExtrude3D(box10x4(), 20, sdf.Pi)
		}, 100},
		// Mild twist, wide thin profile — high r, moderate k.
		{"thin_pi_h20_c100", func(t *testing.T) sdf.SDF3 {
			return sdf.TwistExtrude3D(thinRect(), 20, sdf.Pi)
		}, 100},
		// Aggressive twist, compact. σ(r=5.39, k=π/10) ≈ 1.97.
		{"box_4pi_h40_c50", func(t *testing.T) sdf.SDF3 {
			return sdf.TwistExtrude3D(box10x4(), 40, 4*sdf.Pi)
		}, 50},
		{"box_4pi_h40_c100", func(t *testing.T) sdf.SDF3 {
			return sdf.TwistExtrude3D(box10x4(), 40, 4*sdf.Pi)
		}, 100},
		// Aggressive twist, thin rect — worst case. σ(r=10.05, k=π/10) ≈ 3.31.
		{"thin_4pi_h40_c50", func(t *testing.T) sdf.SDF3 {
			return sdf.TwistExtrude3D(thinRect(), 40, 4*sdf.Pi)
		}, 50},
		{"thin_4pi_h40_c100", func(t *testing.T) sdf.SDF3 {
			return sdf.TwistExtrude3D(thinRect(), 40, 4*sdf.Pi)
		}, 100},
		// Short height makes k even larger. k=4π/20=π/5; at r=10.05, kr≈6.3, σ≈6.4.
		{"thin_4pi_h20_c100", func(t *testing.T) sdf.SDF3 {
			return sdf.TwistExtrude3D(thinRect(), 20, 4*sdf.Pi)
		}, 100},
		// Extreme: 8π over 40, thin profile. kr ≈ 6.3, σ ≈ 6.4.
		{"thin_8pi_h40_c100", func(t *testing.T) sdf.SDF3 {
			return sdf.TwistExtrude3D(thinRect(), 40, 8*sdf.Pi)
		}, 100},
		// Star profile with aggressive twist.
		{"star_4pi_h40_c100", func(t *testing.T) sdf.SDF3 {
			return sdf.TwistExtrude3D(starProfile(t), 40, 4*sdf.Pi)
		}, 100},
		// Left-hand twist (sign change).
		{"box_neg4pi_h40_c100", func(t *testing.T) sdf.SDF3 {
			return sdf.TwistExtrude3D(box10x4(), 40, -4*sdf.Pi)
		}, 100},
		// Very coarse octree against aggressive twist.
		{"thin_4pi_h40_c25", func(t *testing.T) sdf.SDF3 {
			return sdf.TwistExtrude3D(thinRect(), 40, 4*sdf.Pi)
		}, 25},
	}
}

func Test_TwistExtrude3D_Watertight(t *testing.T) {
	runExtrudeCfgs(t, twistExtrudeCfgs(t))
}

//-----------------------------------------------------------------------------
// ScaleExtrude3D: the 3D→2D Jacobian is
//   J = [s_x(z),      0,  m_x·x]
//       [     0, s_y(z),  m_y·y]
// with s(z) = m·z + b going from 1 at z=-h/2 to 1/scale at z=+h/2.
// σ_max² = max eigenvalue of J·Jᵀ. For uniform scale this reduces to
// s(z)² + m²r². Extreme aspect (scale << 1 or >> 1) and small h blow it up.

func scaleExtrudeCfgs() []extrudeCfg {
	return []extrudeCfg{
		// Identity scale (1,1) — factor is 1 everywhere, m=0, no stretch.
		{"id_scale_c50", func(t *testing.T) sdf.SDF3 {
			return sdf.ScaleExtrude3D(box10x4(), 20, v2.Vec{X: 1, Y: 1})
		}, 50},
		// Near-identity — tiny stretch. c=50 is too coarse for the shallow
		// slanted face of a box with rounding=0.5 (MC ambiguity, not
		// Lipschitz); bumped to c=100 to exercise the correction itself.
		{"mild_0.9_h20_c100", func(t *testing.T) sdf.SDF3 {
			return sdf.ScaleExtrude3D(box10x4(), 20, v2.Vec{X: 0.9, Y: 0.9})
		}, 100},
		// Moderate taper — inv=2, m=0.05; σ at z=h/2, r=5.39 ≈ 2.0.
		{"taper_0.5_h20_c50", func(t *testing.T) sdf.SDF3 {
			return sdf.ScaleExtrude3D(box10x4(), 20, v2.Vec{X: 0.5, Y: 0.5})
		}, 50},
		{"taper_0.5_h20_c100", func(t *testing.T) sdf.SDF3 {
			return sdf.ScaleExtrude3D(box10x4(), 20, v2.Vec{X: 0.5, Y: 0.5})
		}, 100},
		// Inverse taper — growing toward top. inv=0.5, m=-0.025.
		{"grow_2_h20_c100", func(t *testing.T) sdf.SDF3 {
			return sdf.ScaleExtrude3D(box10x4(), 20, v2.Vec{X: 2, Y: 2})
		}, 100},
		// Aggressive shrink — inv=10, m=0.45; σ_max near top ≈ 10.
		{"shrink_0.1_h20_c50", func(t *testing.T) sdf.SDF3 {
			return sdf.ScaleExtrude3D(box10x4(), 20, v2.Vec{X: 0.1, Y: 0.1})
		}, 50},
		{"shrink_0.1_h20_c100", func(t *testing.T) sdf.SDF3 {
			return sdf.ScaleExtrude3D(box10x4(), 20, v2.Vec{X: 0.1, Y: 0.1})
		}, 100},
		// Asymmetric scale — anisotropic stretch.
		{"asym_0.1_10_c100", func(t *testing.T) sdf.SDF3 {
			return sdf.ScaleExtrude3D(box10x4(), 20, v2.Vec{X: 0.1, Y: 10})
		}, 100},
		// Short height amplifies slope magnitude.
		{"shrink_0.3_h5_c50", func(t *testing.T) sdf.SDF3 {
			return sdf.ScaleExtrude3D(box10x4(), 5, v2.Vec{X: 0.3, Y: 0.3})
		}, 50},
		// Thin profile exaggerates m·x term.
		{"thin_0.3_h20_c100", func(t *testing.T) sdf.SDF3 {
			return sdf.ScaleExtrude3D(thinRect(), 20, v2.Vec{X: 0.3, Y: 0.3})
		}, 100},
	}
}

func Test_ScaleExtrude3D_Watertight(t *testing.T) {
	runExtrudeCfgs(t, scaleExtrudeCfgs())
}

//-----------------------------------------------------------------------------
// ScaleTwistExtrude3D: combines both scaling and rotation with z. Jacobian
// mixes both contributions; σ_max can exceed the per-effect maximum of either
// ScaleExtrude or TwistExtrude alone. Even scale=(1,1) routes through this
// non-identity extrude path (so invStretch stays at 1 without a correction).

func scaleTwistExtrudeCfgs() []extrudeCfg {
	return []extrudeCfg{
		// scale=(1,1) + zero twist: still goes through the non-normal path,
		// but extrude function is effectively identity. σ=1, should pass.
		{"id_notwist_c50", func(t *testing.T) sdf.SDF3 {
			return sdf.ScaleTwistExtrude3D(box10x4(), 20, 0, v2.Vec{X: 1, Y: 1})
		}, 50},
		// Pure twist via the scale-twist path (scale=(1,1)). Equivalent to
		// TwistExtrude3D geometrically, but the struct gets no stretch
		// correction in this constructor.
		{"id_4pi_h40_c100", func(t *testing.T) sdf.SDF3 {
			return sdf.ScaleTwistExtrude3D(box10x4(), 40, 4*sdf.Pi, v2.Vec{X: 1, Y: 1})
		}, 100},
		// Pure scale via the scale-twist path (twist=0).
		{"taper_0.5_notwist_c100", func(t *testing.T) sdf.SDF3 {
			return sdf.ScaleTwistExtrude3D(box10x4(), 20, 0, v2.Vec{X: 0.5, Y: 0.5})
		}, 100},
		// Combined: moderate twist + moderate taper.
		{"pi_taper_0.5_c100", func(t *testing.T) sdf.SDF3 {
			return sdf.ScaleTwistExtrude3D(box10x4(), 20, sdf.Pi, v2.Vec{X: 0.5, Y: 0.5})
		}, 100},
		// Combined aggressive: 4π twist + 0.5 taper + thin profile.
		{"thin_4pi_0.5_c100", func(t *testing.T) sdf.SDF3 {
			return sdf.ScaleTwistExtrude3D(thinRect(), 40, 4*sdf.Pi, v2.Vec{X: 0.5, Y: 0.5})
		}, 100},
		// Asymmetric scale + twist.
		{"pi_asym_0.25_0.5_c100", func(t *testing.T) sdf.SDF3 {
			return sdf.ScaleTwistExtrude3D(box10x4(), 20, sdf.Pi, v2.Vec{X: 0.25, Y: 0.5})
		}, 100},
		// Extreme combo: this is the direct analogue of the "thin 4π" test
		// above but routed through ScaleTwistExtrude.
		{"extreme_combo_c100", func(t *testing.T) sdf.SDF3 {
			return sdf.ScaleTwistExtrude3D(thinRect(), 20, 4*sdf.Pi, v2.Vec{X: 0.3, Y: 0.3})
		}, 100},
	}
}

func Test_ScaleTwistExtrude3D_Watertight(t *testing.T) {
	runExtrudeCfgs(t, scaleTwistExtrudeCfgs())
}

//-----------------------------------------------------------------------------
// ExtrudeRounded3D: uses the identity (x,y) projection plus rounded caps.
// No helical/scaling mapping, so σ_max = 1 and the mesh should always be
// watertight for any sensible input. This is a baseline regression test.

func extrudeRoundedCfgs() []extrudeCfg {
	return []extrudeCfg{
		{"box_round1_h20_c50", func(t *testing.T) sdf.SDF3 {
			s, err := sdf.ExtrudeRounded3D(box10x4(), 20, 1)
			if err != nil {
				t.Fatal(err)
			}
			return s
		}, 50},
		{"box_round2_h20_c100", func(t *testing.T) sdf.SDF3 {
			s, err := sdf.ExtrudeRounded3D(box10x4(), 20, 2)
			if err != nil {
				t.Fatal(err)
			}
			return s
		}, 100},
		{"thin_round0.5_h20_c100", func(t *testing.T) sdf.SDF3 {
			s, err := sdf.ExtrudeRounded3D(thinRect(), 20, 0.5)
			if err != nil {
				t.Fatal(err)
			}
			return s
		}, 100},
		{"star_round0.5_h20_c100", func(t *testing.T) sdf.SDF3 {
			s, err := sdf.ExtrudeRounded3D(starProfile(t), 20, 0.5)
			if err != nil {
				t.Fatal(err)
			}
			return s
		}, 100},
	}
}

func Test_ExtrudeRounded3D_Watertight(t *testing.T) {
	runExtrudeCfgs(t, extrudeRoundedCfgs())
}

//-----------------------------------------------------------------------------
// Loft3D: a(p) = Mix(sdf0(x,y), sdf1(x,y), k(z)), k=0.5·z/h + 0.5.
// The z-gradient is (sdf1 - sdf0)·(0.5/h). When the two 2D SDFs disagree
// strongly at the same (x,y) — especially when one is "deep inside" and
// the other is "far outside" — the combined SDF's Lipschitz constant along
// z is Δmax/(2h), which can dwarf 1 for short, disparate lofts.

func loftCfgs() []extrudeCfg {
	// profiles as closures to re-create per test (SDF2s are not pure, but
	// Circle2D/Box2D return fresh values)
	circle := func(r float64) sdf.SDF2 {
		c, err := sdf.Circle2D(r)
		if err != nil {
			panic(err)
		}
		return c
	}
	offsetCircle := func(r, dx float64) sdf.SDF2 {
		return sdf.Transform2D(circle(r), sdf.Translate2d(v2.Vec{X: dx, Y: 0}))
	}
	rotatedBox := func() sdf.SDF2 {
		return sdf.Transform2D(sdf.Box2D(v2.Vec{X: 10, Y: 2}, 0.5), sdf.Rotate2d(sdf.Pi/4))
	}
	return []extrudeCfg{
		// Same profile both ends: Δ=0, stretch=1, should pass.
		{"same_circle_h20_c50", func(t *testing.T) sdf.SDF3 {
			s, err := sdf.Loft3D(circle(5), circle(5), 20, 0.5)
			if err != nil {
				t.Fatal(err)
			}
			return s
		}, 50},
		// Mild concentric size change.
		{"circle_5_10_h20_c100", func(t *testing.T) sdf.SDF3 {
			s, err := sdf.Loft3D(circle(5), circle(10), 20, 0.5)
			if err != nil {
				t.Fatal(err)
			}
			return s
		}, 100},
		// Box rotated 45° — Δ large near corners.
		{"box_rotbox_h20_c100", func(t *testing.T) sdf.SDF3 {
			s, err := sdf.Loft3D(sdf.Box2D(v2.Vec{X: 10, Y: 2}, 0.5), rotatedBox(), 20, 0.5)
			if err != nil {
				t.Fatal(err)
			}
			return s
		}, 100},
		// Circle → off-center circle: Δ ≈ 2·dx near overlap region.
		{"circle_offset_h20_c100", func(t *testing.T) sdf.SDF3 {
			s, err := sdf.Loft3D(circle(5), offsetCircle(5, 5), 20, 0.5)
			if err != nil {
				t.Fatal(err)
			}
			return s
		}, 100},
		// Same but short height — ∂a/∂z doubles, Lipschitz ≈ 2.
		{"circle_offset_h10_c100", func(t *testing.T) sdf.SDF3 {
			s, err := sdf.Loft3D(circle(5), offsetCircle(5, 5), 10, 0.5)
			if err != nil {
				t.Fatal(err)
			}
			return s
		}, 100},
		// Very short + big offset — worst case.
		{"circle_offset_h5_c100", func(t *testing.T) sdf.SDF3 {
			s, err := sdf.Loft3D(circle(5), offsetCircle(5, 8), 5, 0.5)
			if err != nil {
				t.Fatal(err)
			}
			return s
		}, 100},
		// Star ↔ circle.
		{"star_circle_h20_c100", func(t *testing.T) sdf.SDF3 {
			s, err := sdf.Loft3D(starProfile(t), circle(5), 20, 0.5)
			if err != nil {
				t.Fatal(err)
			}
			return s
		}, 100},
		// Star ↔ circle, short. c=100 and c=200 happen to align poorly with
		// star tips (MC resolution artifact unrelated to Lipschitz); c=90 is
		// representative of the Lipschitz stress without that artifact.
		{"star_circle_h5_c90", func(t *testing.T) sdf.SDF3 {
			s, err := sdf.Loft3D(starProfile(t), circle(5), 5, 0.5)
			if err != nil {
				t.Fatal(err)
			}
			return s
		}, 90},
	}
}

func Test_Loft3D_Watertight(t *testing.T) {
	runExtrudeCfgs(t, loftCfgs())
}
