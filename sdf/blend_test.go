package sdf

import (
	"math"
	"math/rand"
	"testing"

	v3 "github.com/snowbldr/sdfx/vec/v3"
)

// blendNum tests numerical properties of MinFunc/MaxFunc implementations
// independent of any SDF context: limit-as-k→0 behavior, min/max duality,
// and that no implementation returns NaN for typical SDF input ranges.
//
// The Lipschitz-in-space check (Test_Blend_Lipschitz1) lives below and runs
// the blend on actual SDFs (two spheres) — a numerical-only check on the
// scalar MinFunc can't see the spatial-gradient amplification that produces
// the ChamferMin "holes" issue, because the same input pair (a,b) can come
// from gradients ∇a, ∇b with any angle between them.

//-----------------------------------------------------------------------------

// minFuncs / maxFuncs are factories so each test gets a fresh closure (some
// of these capture state via the builder; closures aren't shared).

type minBuild struct {
	name string
	make func() MinFunc
}

type maxBuild struct {
	name string
	make func() MaxFunc
}

func minFuncs(k float64) []minBuild {
	return []minBuild{
		{"RoundMin", func() MinFunc { return RoundMin(k) }},
		{"ChamferMin", func() MinFunc { return ChamferMin(k) }},
		{"ExpMin", func() MinFunc { return ExpMin(k) }},
		{"PowMin", func() MinFunc { return PowMin(k) }},
		{"PolyMin", func() MinFunc { return PolyMin(k) }},
	}
}

func maxFuncs(k float64) []maxBuild {
	return []maxBuild{
		{"RoundMax", func() MaxFunc { return RoundMax(k) }},
		{"ChamferMax", func() MaxFunc { return ChamferMax(k) }},
		{"ExpMax", func() MaxFunc { return ExpMax(k) }},
		{"PowMax", func() MaxFunc { return PowMax(k) }},
		{"PolyMax", func() MaxFunc { return PolyMax(k) }},
	}
}

//-----------------------------------------------------------------------------

// As k→0 every smooth-min should approach math.Min, and every smooth-max
// math.Max. The k chosen below is small enough that the smooth deviation
// shrinks below the comparison tolerance for inputs of order 1, but big
// enough not to be lost in float64 round-off.
//
// ExpMin/ExpMax and PowMin/PowMax are excluded — their k parameter has
// the opposite convention (large k → hard, small k → smooth/degenerate),
// so the k→0 limit doesn't apply. They get a hard-limit test below.
func Test_BlendMinMaxSmoothLimit(t *testing.T) {
	const k = 1e-4
	const tol = 1e-3

	pairs := [][2]float64{
		{0, 0}, {1, 2}, {2, 1}, {-1, 1}, {-1, -2},
		{0.5, 0.5}, {-0.5, 0.5}, {-0.5, -0.5},
		{10, -10}, {-10, 10}, {0.01, -0.01},
	}

	skip := map[string]bool{"ExpMin": true, "ExpMax": true, "PowMin": true, "PowMax": true}

	for _, mb := range minFuncs(k) {
		if skip[mb.name] {
			continue
		}
		t.Run(mb.name, func(t *testing.T) {
			f := mb.make()
			for _, p := range pairs {
				got := f(p[0], p[1])
				want := math.Min(p[0], p[1])
				if math.Abs(got-want) > tol {
					t.Errorf("%s(%v, %v) = %v, want ≈ %v", mb.name, p[0], p[1], got, want)
				}
			}
		})
	}

	for _, mb := range maxFuncs(k) {
		if skip[mb.name] {
			continue
		}
		t.Run(mb.name, func(t *testing.T) {
			f := mb.make()
			for _, p := range pairs {
				got := f(p[0], p[1])
				want := math.Max(p[0], p[1])
				if math.Abs(got-want) > tol {
					t.Errorf("%s(%v, %v) = %v, want ≈ %v", mb.name, p[0], p[1], got, want)
				}
			}
		})
	}
}

// Hard-limit test for the "large k = hard" family — ExpMin/ExpMax via
// log-sum-exp, PowMin/PowMax via the harmonic-power formula. Both
// approach math.Min/Max as k grows, with residual ∼ log(2)/k.
//
// ExpMin handles every sign combination cleanly. PowMin agrees with
// math.Min only when the picked input is non-negative (and therefore
// PowMax agrees with math.Max only when the picked input is non-positive)
// — see the note on PowMin's docstring. The pairs below avoid the
// other-sign-wins cases so the test pins what the formula actually
// promises rather than what its name suggests.
func Test_BlendHardLimit(t *testing.T) {
	const k = 1024.0
	const tol = 1e-2

	emin := ExpMin(k)
	emax := ExpMax(k)
	pmin := PowMin(k)
	pmax := PowMax(k)

	// Same-sign-positive: math.Min picks a non-negative; PowMin agrees.
	// Mixed-sign with negative-wins: math.Min picks the negative one.
	// Both regimes work for ExpMin and PowMin.
	minPairs := [][2]float64{
		{0.5, 0.5}, {1, 2}, {2, 1}, {0.1, 0.7},
		{-1, 2}, {3, -3}, {-0.5, 0.5},
	}
	for _, p := range minPairs {
		if got, want := emin(p[0], p[1]), math.Min(p[0], p[1]); math.Abs(got-want) > tol {
			t.Errorf("ExpMin(%v,%v) = %v, want ≈ %v", p[0], p[1], got, want)
		}
		if got, want := pmin(p[0], p[1]), math.Min(p[0], p[1]); math.Abs(got-want) > tol {
			t.Errorf("PowMin(%v,%v) = %v, want ≈ %v", p[0], p[1], got, want)
		}
	}

	// Same-sign-negative: math.Max picks a non-positive; PowMax agrees.
	// Mixed-sign with positive-wins: math.Max picks the positive one.
	maxPairs := [][2]float64{
		{-0.5, -0.5}, {-1, -2}, {-2, -1}, {-0.1, -0.7},
		{1, -2}, {-3, 3}, {0.5, -0.5},
	}
	for _, p := range maxPairs {
		if got, want := emax(p[0], p[1]), math.Max(p[0], p[1]); math.Abs(got-want) > tol {
			t.Errorf("ExpMax(%v,%v) = %v, want ≈ %v", p[0], p[1], got, want)
		}
		if got, want := pmax(p[0], p[1]), math.Max(p[0], p[1]); math.Abs(got-want) > tol {
			t.Errorf("PowMax(%v,%v) = %v, want ≈ %v", p[0], p[1], got, want)
		}
	}

	// ExpMin/ExpMax also work in the cases PowMin/PowMax can't —
	// log-sum-exp is sign-symmetric in a way the harmonic-power formula
	// isn't.
	expOnly := [][2]float64{{-1, -2}, {-0.5, -0.5}, {1, 2}, {0.5, 0.5}}
	for _, p := range expOnly {
		if got, want := emin(p[0], p[1]), math.Min(p[0], p[1]); math.Abs(got-want) > tol {
			t.Errorf("ExpMin(%v,%v) = %v, want ≈ %v", p[0], p[1], got, want)
		}
		if got, want := emax(p[0], p[1]), math.Max(p[0], p[1]); math.Abs(got-want) > tol {
			t.Errorf("ExpMax(%v,%v) = %v, want ≈ %v", p[0], p[1], got, want)
		}
	}
}

//-----------------------------------------------------------------------------

// Negation duality: MaxFunc(a,b) = -MinFunc(-a,-b). This is how PolyMax was
// already defined; the new ports must respect the same identity, otherwise
// the named pairs wouldn't actually be the smooth-max counterparts.
func Test_BlendMinMaxDuality(t *testing.T) {
	const k = 0.3
	const tol = 1e-12

	r := rand.New(rand.NewSource(1))
	mins := minFuncs(k)
	maxes := maxFuncs(k)
	if len(mins) != len(maxes) {
		t.Fatalf("min/max factory length mismatch: %d vs %d", len(mins), len(maxes))
	}
	for i := range mins {
		mn := mins[i].make()
		mx := maxes[i].make()
		t.Run(mins[i].name+"/"+maxes[i].name, func(t *testing.T) {
			for j := 0; j < 200; j++ {
				a := r.Float64()*4 - 2
				b := r.Float64()*4 - 2
				got := mx(a, b)
				want := -mn(-a, -b)
				if math.Abs(got-want) > tol {
					t.Errorf("%s(%v,%v) = %v, dual gives %v", maxes[i].name, a, b, got, want)
				}
			}
		})
	}
}

//-----------------------------------------------------------------------------

// PowMin used to call math.Pow on negative arguments with a non-integer k,
// silently returning NaN. SDFs are negative for any point inside a shape,
// so the function was unusable in its primary role (smoothing the corner
// where two surfaces meet — both inputs negative just inside the corner).
//
// The fix takes |a|, |b| and applies the sign of the smaller-in-abs-value
// input. This test pins both: no NaN out, and the result agrees with
// math.Min when the inputs differ sharply (where smoothing has nothing to do).
func Test_PowMin_NoNaN(t *testing.T) {
	pm := PowMin(8)
	cases := [][2]float64{
		{-1, -1}, {-0.5, -0.5}, {-2, -3},
		{-1, 1}, {1, -1}, {-3, 5},
		{0.5, 0.5}, {2, 3},
	}
	for _, c := range cases {
		v := pm(c[0], c[1])
		if math.IsNaN(v) || math.IsInf(v, 0) {
			t.Errorf("PowMin(%v,%v) = %v (NaN/Inf)", c[0], c[1], v)
		}
	}
	// On opposite-sign inputs PowMin should land near math.Min — both for
	// correctness and so it composes sanely with the rest of the smooth-min
	// family. The k=8 power makes |min| dominate, but not exactly.
	got := pm(-3, 5)
	if got > -2 || got < -3.5 {
		t.Errorf("PowMin(-3, 5) = %v, expected near -3", got)
	}
}

// Same pin for PowMax — it's a thin wrapper on PowMin via the duality.
func Test_PowMax_NoNaN(t *testing.T) {
	pm := PowMax(8)
	cases := [][2]float64{
		{1, 1}, {0.5, 0.5}, {2, 3},
		{-1, 1}, {1, -1}, {3, -5},
		{-0.5, -0.5}, {-2, -3},
	}
	for _, c := range cases {
		v := pm(c[0], c[1])
		if math.IsNaN(v) || math.IsInf(v, 0) {
			t.Errorf("PowMax(%v,%v) = %v (NaN/Inf)", c[0], c[1], v)
		}
	}
}

//-----------------------------------------------------------------------------

// Test_Blend_Lipschitz1 is the test that motivated the ChamferMin fix.
//
// The renderer's octree pruning (render/march3p.go isEmpty) classifies a
// cube as empty when |sdf(center)| ≥ half-diagonal. That rule is sound
// only if the SDF is 1-Lipschitz: |f(p₁)-f(p₂)| ≤ |p₁-p₂|. If a blend
// function inflates the spatial gradient above 1, the SDF reports a
// further distance to the surface than truly exists, and isEmpty drops
// cubes that actually contain triangles → holes in the mesh.
//
// The original ChamferMin used (a+b-k)·√½, which has spatial gradient up
// to √2 (worst case ∇a∥∇b — gradients of two concentric sphere SDFs do
// this on the line between centers). The corrected ChamferMin uses
// (a+b-k)·½, gradient ≤ 1 in every configuration.
//
// We exercise this with two overlapping spheres, where ∇SDF_A and ∇SDF_B
// are nearly parallel along the line between centers — the configuration
// that breaks the textbook formula. The gradient of the composite is
// estimated by central differences and compared to 1 + a small tolerance
// to absorb the O(eps²) finite-difference truncation error.
func Test_Blend_Lipschitz1(t *testing.T) {
	sa, err := Sphere3D(1.0)
	if err != nil {
		t.Fatal(err)
	}
	sb, err := Sphere3D(1.0)
	if err != nil {
		t.Fatal(err)
	}
	sb = Transform3D(sb, Translate3d(v3.Vec{X: 1.5}))

	const eps = 1e-4
	// Allow a small slack: 1 + 5·eps covers the O(eps²) bias of central
	// differences on these inputs (max observed empirically ≈ 2e-4).
	const slack = 5e-4

	checkLip := func(t *testing.T, name string, f func(p v3.Vec) float64) {
		t.Helper()
		r := rand.New(rand.NewSource(42))
		const samples = 4000
		var maxGrad float64
		var maxAt v3.Vec
		// Sample box covers both spheres with margin so we hit the
		// region where ∇a ≈ ∇b (the line between centers) plus the
		// regions where ∇a ⊥ ∇b (off-axis), exercising every gradient
		// configuration.
		for i := 0; i < samples; i++ {
			p := v3.Vec{
				X: r.Float64()*4 - 1,
				Y: r.Float64()*3 - 1.5,
				Z: r.Float64()*3 - 1.5,
			}
			dx := (f(v3.Vec{p.X + eps, p.Y, p.Z}) - f(v3.Vec{p.X - eps, p.Y, p.Z})) / (2 * eps)
			dy := (f(v3.Vec{p.X, p.Y + eps, p.Z}) - f(v3.Vec{p.X, p.Y - eps, p.Z})) / (2 * eps)
			dz := (f(v3.Vec{p.X, p.Y, p.Z + eps}) - f(v3.Vec{p.X, p.Y, p.Z - eps})) / (2 * eps)
			g := math.Sqrt(dx*dx + dy*dy + dz*dz)
			if g > maxGrad {
				maxGrad = g
				maxAt = p
			}
		}
		if maxGrad > 1+slack {
			t.Errorf("%s: max |∇f| = %v at %v (must be ≤ 1 for renderer pruning)", name, maxGrad, maxAt)
		} else {
			t.Logf("%s: max |∇f| = %v", name, maxGrad)
		}
	}

	for _, mb := range minFuncs(0.5) {
		f := mb.make()
		t.Run(mb.name, func(t *testing.T) {
			checkLip(t, mb.name, func(p v3.Vec) float64 {
				return f(sa.Evaluate(p), sb.Evaluate(p))
			})
		})
	}
	for _, mb := range maxFuncs(0.5) {
		f := mb.make()
		t.Run(mb.name, func(t *testing.T) {
			checkLip(t, mb.name, func(p v3.Vec) float64 {
				return f(sa.Evaluate(p), sb.Evaluate(p))
			})
		})
	}
}

//-----------------------------------------------------------------------------
