package render

import (
	"testing"

	"github.com/deadsy/sdfx/sdf"
	v2 "github.com/deadsy/sdfx/vec/v2"
)

// RevolveTheta3D composes a revolve with an optional angular wedge cut.
// Radial projection p → (√(x²+y²), z) is 1-Lipschitz everywhere (including
// the axis), and the wedge is a max/min of plane cuts that are each
// Lipschitz-1, so the composite remains Lipschitz-1. These tests confirm
// watertightness across θ values (full, near-half, thin wedge) and for a
// non-convex profile that produces a torus with inner radius.

func revolveProfile(t *testing.T, offsetX float64) sdf.SDF2 {
	t.Helper()
	s, err := sdf.Circle2D(2)
	if err != nil {
		t.Fatal(err)
	}
	return sdf.Transform2D(s, sdf.Translate2d(v2.Vec{X: offsetX, Y: 0}))
}

func revolveBox(t *testing.T, offsetX float64) sdf.SDF2 {
	t.Helper()
	s := sdf.Box2D(v2.Vec{X: 3, Y: 6}, 0.3)
	return sdf.Transform2D(s, sdf.Translate2d(v2.Vec{X: offsetX, Y: 0}))
}

func revolveCfgs() []extrudeCfg {
	fullTorus := func(t *testing.T) sdf.SDF3 {
		s, err := sdf.Revolve3D(revolveProfile(t, 5))
		if err != nil {
			t.Fatal(err)
		}
		return s
	}
	wedge := func(theta float64, prof func(*testing.T) sdf.SDF2) func(*testing.T) sdf.SDF3 {
		return func(t *testing.T) sdf.SDF3 {
			s, err := sdf.RevolveTheta3D(prof(t), theta)
			if err != nil {
				t.Fatal(err)
			}
			return s
		}
	}
	return []extrudeCfg{
		{"full_torus_c50", fullTorus, 50},
		{"full_torus_c100", fullTorus, 100},
		// θ just under π — wedge uses max(-y, d) (intersect branch)
		{"wedge_halfpi_c100", wedge(sdf.Pi/2, func(t *testing.T) sdf.SDF2 { return revolveProfile(t, 5) }), 100},
		{"wedge_0.9pi_c100", wedge(0.9*sdf.Pi, func(t *testing.T) sdf.SDF2 { return revolveProfile(t, 5) }), 100},
		// θ just over π — union branch
		{"wedge_1.1pi_c100", wedge(1.1*sdf.Pi, func(t *testing.T) sdf.SDF2 { return revolveProfile(t, 5) }), 100},
		{"wedge_1.5pi_c100", wedge(1.5*sdf.Pi, func(t *testing.T) sdf.SDF2 { return revolveProfile(t, 5) }), 100},
		// Thin wedge — stresses the plane-intersection logic
		{"thin_wedge_pi6_c100", wedge(sdf.Pi/6, func(t *testing.T) sdf.SDF2 { return revolveProfile(t, 5) }), 100},
		// Box profile revolved partially — sharp-cornered swept surface
		{"box_wedge_halfpi_c100", wedge(sdf.Pi/2, func(t *testing.T) sdf.SDF2 { return revolveBox(t, 4) }), 100},
		{"box_wedge_1.5pi_c100", wedge(1.5*sdf.Pi, func(t *testing.T) sdf.SDF2 { return revolveBox(t, 4) }), 100},
	}
}

func Test_Revolve3D_Watertight(t *testing.T) {
	runExtrudeCfgs(t, revolveCfgs())
}
