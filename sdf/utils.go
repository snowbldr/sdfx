//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------

package sdf

import (
	"fmt"
	"math"
	"math/rand"
	"runtime"

	"github.com/snowbldr/sdfx/vec/conv"
	v2 "github.com/snowbldr/sdfx/vec/v2"
	v3 "github.com/snowbldr/sdfx/vec/v3"
)

//-----------------------------------------------------------------------------
// constants

// Pi (3.14159...)
const Pi = math.Pi

// Tau (2 * Pi).
const Tau = 2 * math.Pi

// MillimetresPerInch is millimetres per inch (25.4)
const MillimetresPerInch = 25.4

// InchesPerMillimetre is inches per millimetre
const InchesPerMillimetre = 1.0 / MillimetresPerInch

// Mil is millimetres per 1/1000 of an inch
const Mil = MillimetresPerInch / 1000.0

const sqrtHalf = 0.7071067811865476
const tolerance = 1e-9
const epsilon = 1e-12

//-----------------------------------------------------------------------------

// From go 1.20 the rand.* are initialized to a random seed.
// Different results are generated every run. We want consistent
// results from run to run for binary verification, so we have
// our own local random source.

var sdfRand = rand.New(rand.NewSource(1))

// randomRange returns a random float64 [a,b)
func randomRange(a, b float64) float64 {
	return a + (b-a)*sdfRand.Float64()
}

//-----------------------------------------------------------------------------

// DtoR converts degrees to radians
func DtoR(degrees float64) float64 {
	return (Pi / 180) * degrees
}

// RtoD converts radians to degrees
func RtoD(radians float64) float64 {
	return (180 / Pi) * radians
}

//-----------------------------------------------------------------------------

func minInt(a, b int) int {
	if a < b {
		return a
	}
	return b
}

func maxInt(a, b int) int {
	if a > b {
		return a
	}
	return b
}

// fmin/fmax are branch-based min/max for float64.
//
// They replace math.Min/math.Max on SDF hot paths (Union/Difference
// Evaluate, Box MinDist2). The stdlib versions handle NaN and signed
// zero, which costs measurable time (picorx profile showed math.archMax
// at 32% of CPU). These helpers assume finite, non-NaN inputs — true
// for all SDF values — and inline to a single compare-and-select.
//
// Trade-off: if an SDF ever returns NaN, fmin/fmax return the non-NaN
// argument instead of propagating NaN. That is arguably more useful
// (NaN in a distance field makes marching cubes produce garbage) and
// the library's test suite catches no cases where NaN propagation is
// relied on.
func fmin(a, b float64) float64 {
	if a < b {
		return a
	}
	return b
}

func fmax(a, b float64) float64 {
	if a > b {
		return a
	}
	return b
}

//-----------------------------------------------------------------------------

// Clamp x between a and b, assume a <= b
func Clamp(x, a, b float64) float64 {
	if x < a {
		return a
	}
	if x > b {
		return b
	}
	return x
}

// Mix does a linear interpolation from x to y, a = [0,1]
func Mix(x, y, a float64) float64 {
	return x + (a * (y - x))
}

//-----------------------------------------------------------------------------

// Sign returns the sign of x
func Sign(x float64) float64 {
	if x < 0 {
		return -1
	}
	if x > 0 {
		return 1
	}
	return 0
}

//-----------------------------------------------------------------------------

// SawTooth generates a sawtooth function. Returns [-period/2, period/2)
func SawTooth(x, period float64) float64 {
	x += period / 2
	t := x / period
	return period*(t-math.Floor(t)) - period/2
}

//-----------------------------------------------------------------------------

// MinFunc is a minimum functions for SDF blending.
type MinFunc func(a, b float64) float64

// RoundMin returns a minimum function that uses a quarter-circle to join
// the two objects smoothly.
//
// Implementation is Inigo Quilez's `sminCircular`. The previous formula
// (max(k, min(a,b)) - length(max(k-a,0), max(k-b,0))) had a spatial
// gradient up to √2 wherever ∇a ≈ ∇b inside both shapes — the configuration
// you hit on the line between two overlapping spheres — so the SDF wasn't
// a valid distance estimator and the renderer's octree pruning
// (|f| ≥ half-diagonal ⇒ skip) dropped surface cells, producing holes.
// sminCircular is 1-Lipschitz everywhere.
//
// The fillet extends ~0.293·k outward from the sharp-corner point — the
// same surface offset the previous formula produced at coincident inputs,
// chosen so existing call sites' k tunings keep their look. (The IQ
// article's "k = fillet radius" rescaling — k *= 1/(1-√½) — is omitted
// for that reason.)
func RoundMin(k float64) MinFunc {
	return func(a, b float64) float64 {
		h := math.Max(k-math.Abs(a-b), 0) / k
		return math.Min(a, b) - k*0.5*(1+h-math.Sqrt(1-h*(h-2)))
	}
}

// ChamferMin returns a minimum function that makes a chamfered edge.
//
// The factor on the chamfer plane is 1/2 rather than the textbook 1/√2 so
// the result stays a valid distance estimator for arbitrary input SDFs.
// (a+b-k)/√2 has gradient magnitude up to √2 — exactly 1 only when ∇a⊥∇b
// — so combining curved SDFs would let it overshoot the true distance and
// produce holes when the renderer's octree pruning trusts |f| as a lower
// bound on distance to the surface. Dividing by 2 caps the gradient at 1
// in every input configuration; the geometric tradeoff is a slightly
// shallower chamfer angle than 45° when the input surfaces are planar.
func ChamferMin(k float64) MinFunc {
	return func(a, b float64) float64 {
		return math.Min(math.Min(a, b), (a+b-k)*0.5)
	}
}

// ExpMin returns a minimum function with exponential smoothing (k = 32).
//
// The implementation factors out max(a,b) so exp arguments stay non-positive,
// avoiding overflow when k·max(|a|,|b|) is large (without the shift,
// k=32, a=-23 already overflows double).
func ExpMin(k float64) MinFunc {
	return func(a, b float64) float64 {
		// -log(e^-ka + e^-kb)/k  =  m - log(e^-k(a-m) + e^-k(b-m))/k  with m = min(a,b).
		// Picking m = min keeps both shifted args ≥ 0 → e^-k·shift ∈ (0, 1].
		m := math.Min(a, b)
		return m - math.Log(math.Exp(-k*(a-m))+math.Exp(-k*(b-m)))/k
	}
}

// PowMin returns a minimum function (k = 8).
//
// The IQ "power smooth min" is only defined for positive distances — the
// raw a^k / b^k formula returns NaN whenever either argument is negative
// and k is non-integer, which happens at any SDF point inside either shape.
// Operating on |a|, |b| keeps the algebra well-defined and keeps the
// resulting field 1-Lipschitz so the renderer's pruning stays sound, but
// the both-negative branch is conservative rather than exact: |result| is
// the smaller of |a|, |b|, whereas math.Min picks the larger-magnitude
// (more-deeply-interior) value. PolyMin is the recommended smooth-min for
// SDF blending; this function is kept as the "soft minimum on positive
// distances" variant (e.g. shadow / AO blending).
func PowMin(k float64) MinFunc {
	return func(a, b float64) float64 {
		ax, bx := math.Abs(a), math.Abs(b)
		M, m := ax, bx
		if bx > ax {
			M, m = bx, ax
		}
		// Algebraically equivalent to the textbook
		//   ((|a|^k · |b|^k) / (|a|^k + |b|^k))^(1/k)
		// but factored as m / (1 + (m/M)^k)^(1/k) to avoid the underflow
		// that hits both numerator and denominator when k|log a| ≳ 700.
		// (m/M) ≤ 1 → the inner pow stays in [0, 1] regardless of k.
		var mag float64
		if M == 0 {
			mag = 0
		} else {
			mag = m / math.Pow(1+math.Pow(m/M, k), 1/k)
		}
		if a < 0 || b < 0 {
			return -mag
		}
		return mag
	}
}

func poly(a, b, k float64) float64 {
	h := Clamp(0.5+0.5*(b-a)/k, 0.0, 1.0)
	return Mix(b, a, h) - k*h*(1.0-h)
}

// PolyMin returns a minimum function (Try k = 0.1, a bigger k gives a bigger fillet).
func PolyMin(k float64) MinFunc {
	return func(a, b float64) float64 {
		return poly(a, b, k)
	}
}

//-----------------------------------------------------------------------------

// MaxFunc is a maximum function for SDF blending.
type MaxFunc func(a, b float64) float64

// Each MaxFunc is the negation-dual of the matching MinFunc:
//   MaxFunc(a, b) = -MinFunc(-a, -b)
// The duality preserves Lipschitz-1 (negation is an isometry), so any
// distance-estimator property of the MinFunc carries over.

// RoundMax returns a maximum function that joins with a quarter-circle fillet.
// Useful for smoothing the corner of a Difference3D / Intersection3D.
func RoundMax(k float64) MaxFunc {
	min := RoundMin(k)
	return func(a, b float64) float64 {
		return -min(-a, -b)
	}
}

// ChamferMax returns a maximum function that makes a chamfered edge at
// the corner of a Difference3D / Intersection3D. See ChamferMin for the
// note on Lipschitz-correctness.
func ChamferMax(k float64) MaxFunc {
	min := ChamferMin(k)
	return func(a, b float64) float64 {
		return -min(-a, -b)
	}
}

// ExpMax returns a maximum function with exponential smoothing (k = 32).
func ExpMax(k float64) MaxFunc {
	min := ExpMin(k)
	return func(a, b float64) float64 {
		return -min(-a, -b)
	}
}

// PowMax returns a maximum function (k = 8). See PowMin for the note on
// negative-input handling.
func PowMax(k float64) MaxFunc {
	min := PowMin(k)
	return func(a, b float64) float64 {
		return -min(-a, -b)
	}
}

// PolyMax returns a maximum function (Try k = 0.1, a bigger k gives a bigger fillet).
func PolyMax(k float64) MaxFunc {
	return func(a, b float64) float64 {
		return -poly(-a, -b, k)
	}
}

//-----------------------------------------------------------------------------

// ExtrudeFunc maps v3.Vec to v2.Vec - the point used to evaluate the SDF2.
type ExtrudeFunc func(p v3.Vec) v2.Vec

// NormalExtrude returns an extrusion function.
func NormalExtrude(p v3.Vec) v2.Vec {
	return v2.Vec{p.X, p.Y}
}

// TwistExtrude returns an extrusion function that twists with z.
func TwistExtrude(height, twist float64) ExtrudeFunc {
	k := twist / height
	return func(p v3.Vec) v2.Vec {
		m := Rotate(p.Z * k)
		return m.MulPosition(v2.Vec{p.X, p.Y})
	}
}

// ScaleExtrude returns an extrusion functions that scales with z.
func ScaleExtrude(height float64, scale v2.Vec) ExtrudeFunc {
	inv := v2.Vec{1 / scale.X, 1 / scale.Y}
	m := inv.Sub(v2.Vec{1, 1}).DivScalar(height) // slope
	b := inv.MulScalar(0.5).AddScalar(0.5)       // intercept
	return func(p v3.Vec) v2.Vec {
		return v2.Vec{p.X, p.Y}.Mul(m.MulScalar(p.Z).Add(b))
	}
}

// ScaleTwistExtrude returns an extrusion function that scales and twists with z.
func ScaleTwistExtrude(height, twist float64, scale v2.Vec) ExtrudeFunc {
	k := twist / height
	inv := v2.Vec{1 / scale.X, 1 / scale.Y}
	m := inv.Sub(v2.Vec{1, 1}).DivScalar(height) // slope
	b := inv.MulScalar(0.5).AddScalar(0.5)       // intercept
	return func(p v3.Vec) v2.Vec {
		// Scale and then Twist
		pnew := v2.Vec{p.X, p.Y}.Mul(m.MulScalar(p.Z).Add(b)) // Scale
		return Rotate(p.Z * k).MulPosition(pnew)              // Twist

		// Twist and then scale
		//pnew := Rotate(p.Z * k).MulPosition(v2.Vec{p.X, p.Y})
		//return pnew.Mul(m.MulScalar(p.Z).Add(b))
	}
}

//-----------------------------------------------------------------------------
// Raycasting

func sigmoidScaled(x float64) float64 {
	return 2/(1+math.Exp(-x)) - 1
}

// Raycast3 collides a ray (with an origin point from and a direction dir) with an SDF3.
// sigmoid is useful for fixing bad distance functions (those that do not accurately represent the distance to the
// closest surface, but will probably imply more evaluations)
// stepScale controls precision (less stepSize, more precision, but more SDF evaluations): use 1 if SDF indicates
// distance to the closest surface.
// It returns the collision point, how many normalized distances to reach it (t), and the number of steps performed
// If no surface is found (in maxDist and maxSteps), t is < 0
func Raycast3(s SDF3, from, dir v3.Vec, scaleAndSigmoid, stepScale, epsilon, maxDist float64, maxSteps int) (collision v3.Vec, t float64, steps int) {
	t = 0
	dirN := dir.Normalize()
	pos := from
	for {
		val := math.Abs(s.Evaluate(pos))
		//log.Print("Raycast step #", steps, " at ", pos, " with value ", val, "\n")
		if val < epsilon {
			collision = pos // Success
			break
		}
		steps++
		if steps == maxSteps {
			t = -1 // Failure
			break
		}
		if scaleAndSigmoid > 0 {
			val = sigmoidScaled(val * 10)
		}
		delta := val * stepScale
		t += delta
		pos = pos.Add(dirN.MulScalar(delta))
		if t < 0 || t > maxDist {
			t = -1 // Failure
			break
		}
	}
	//log.Println("Raycast did", steps, "steps")
	return
}

// Raycast2 see Raycast3. NOTE: implementation using Raycast3 (inefficient?)
func Raycast2(s SDF2, from, dir v2.Vec, scaleAndSigmoid, stepScale, epsilon, maxDist float64, maxSteps int) (v2.Vec, float64, int) {
	collision, t, steps := Raycast3(Extrude3D(s, 1), conv.V2ToV3(from, 0), conv.V2ToV3(dir, 0), scaleAndSigmoid, stepScale, epsilon, maxDist, maxSteps)
	return v2.Vec{collision.X, collision.Y}, t, steps
}

//-----------------------------------------------------------------------------
// Normals

// Normal3 returns the normal of an SDF3 at a point (doesn't need to be on the surface).
// Computed by sampling it several times inside a box of side 2*eps centered on p.
func Normal3(s SDF3, p v3.Vec, eps float64) v3.Vec {
	return v3.Vec{
		X: s.Evaluate(p.Add(v3.Vec{X: eps})) - s.Evaluate(p.Add(v3.Vec{X: -eps})),
		Y: s.Evaluate(p.Add(v3.Vec{Y: eps})) - s.Evaluate(p.Add(v3.Vec{Y: -eps})),
		Z: s.Evaluate(p.Add(v3.Vec{Z: eps})) - s.Evaluate(p.Add(v3.Vec{Z: -eps})),
	}.Normalize()
}

// Normal2 returns the normal of an SDF3 at a point (doesn't need to be on the surface).
// Computed by sampling it several times inside a box of side 2*eps centered on p.
func Normal2(s SDF2, p v2.Vec, eps float64) v2.Vec {
	return v2.Vec{
		X: s.Evaluate(p.Add(v2.Vec{X: eps})) - s.Evaluate(p.Add(v2.Vec{X: -eps})),
		Y: s.Evaluate(p.Add(v2.Vec{Y: eps})) - s.Evaluate(p.Add(v2.Vec{Y: -eps})),
	}.Normalize()
}

//-----------------------------------------------------------------------------

// FloatDecode returns a string that decodes the float64 bitfields.
func FloatDecode(x float64) string {
	i := math.Float64bits(x)
	s := int((i >> 63) & 1)
	f := i & ((1 << 52) - 1)
	e := int((i>>52)&((1<<11)-1)) - 1023
	return fmt.Sprintf("s %d f 0x%013x e %d", s, f, e)
}

// FloatEncode encodes a float64 from sign, fraction and exponent values.
func FloatEncode(s int, f uint64, e int) float64 {
	s &= 1
	exp := uint64(e+1023) & ((1 << 11) - 1)
	f &= (1 << 52) - 1
	return math.Float64frombits(uint64(s)<<63 | exp<<52 | f)
}

//-----------------------------------------------------------------------------
// Floating Point Comparisons
// See: http://floating-point-gui.de/errors/NearlyEqualsTest.java

/*

const minNormal = 2.2250738585072014e-308 // 2**-1022

// EqualFloat64 compares two float64 values for equality.
func EqualFloat64(a, b, epsilon float64) bool {
	if a == b {
		return true
	}
	absA := math.Abs(a)
	absB := math.Abs(b)
	diff := math.Abs(a - b)
	if a == 0 || b == 0 || diff < minNormal {
		// a or b is zero or both are extremely close to it
		// relative error is less meaningful here
		return diff < (epsilon * minNormal)
	}
	// use relative error
	return diff/math.Min((absA+absB), math.MaxFloat64) < epsilon
}

*/

// EqualFloat64 compares two float64 values for equality.
func EqualFloat64(a, b, epsilon float64) bool {
	if a == b {
		return true
	}
	return math.Abs(a-b) < epsilon
}

// SnapFloat64 snaps a float value to b if it is within epsilon of b.
func SnapFloat64(a, b, epsilon float64) float64 {
	if EqualFloat64(a, b, epsilon) {
		return b
	}
	return a
}

//-----------------------------------------------------------------------------

// ZeroSmall zeroes out values that are small relative to a quantity.
func ZeroSmall(x, y, epsilon float64) float64 {
	if math.Abs(x)/y < epsilon {
		return 0
	}
	return x
}

//-----------------------------------------------------------------------------

// ErrMsg returns an error with a message function name and line number.
func ErrMsg(msg string) error {
	pc, _, line, ok := runtime.Caller(1)
	if !ok {
		return fmt.Errorf("?: %s", msg)
	}
	fn := runtime.FuncForPC(pc)
	return fmt.Errorf("%s line %d: %s", fn.Name(), line, msg)
}

//-----------------------------------------------------------------------------
