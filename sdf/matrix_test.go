//-----------------------------------------------------------------------------
/*

Matrix rotation tests — proper-rotation invariants for RotateToVector.

*/
//-----------------------------------------------------------------------------

package sdf

import (
	"math"
	"testing"

	v3 "github.com/snowbldr/sdfx/vec/v3"
)

//-----------------------------------------------------------------------------

// A proper rotation matrix has determinant +1. A determinant of -1 means
// the transform contains a reflection / point-inversion, which mirrors
// chiral geometry.
func TestRotateToVector_ProperRotation(t *testing.T) {
	cases := []struct {
		name string
		from v3.Vec
		to   v3.Vec
	}{
		{"same +Z", v3.Vec{X: 0, Y: 0, Z: 1}, v3.Vec{X: 0, Y: 0, Z: 1}},
		{"+Y to +Z", v3.Vec{X: 0, Y: 1, Z: 0}, v3.Vec{X: 0, Y: 0, Z: 1}},
		{"+X to +Z", v3.Vec{X: 1, Y: 0, Z: 0}, v3.Vec{X: 0, Y: 0, Z: 1}},
		{"+Z to -Z", v3.Vec{X: 0, Y: 0, Z: 1}, v3.Vec{X: 0, Y: 0, Z: -1}},
		{"+X to -X", v3.Vec{X: 1, Y: 0, Z: 0}, v3.Vec{X: -1, Y: 0, Z: 0}},
		{"+Y to -Y", v3.Vec{X: 0, Y: 1, Z: 0}, v3.Vec{X: 0, Y: -1, Z: 0}},
		{"oblique antiparallel", v3.Vec{X: 1, Y: 2, Z: 3}.Normalize(), v3.Vec{X: -1, Y: -2, Z: -3}.Normalize()},
		{"oblique general", v3.Vec{X: 1, Y: 2, Z: 3}.Normalize(), v3.Vec{X: 3, Y: -1, Z: 2}.Normalize()},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			m := RotateToVector(c.from, c.to)

			// Determinant of the upper-left 3x3 must be +1 (proper rotation).
			r := M33{
				m[0], m[1], m[2],
				m[4], m[5], m[6],
				m[8], m[9], m[10],
			}
			det := r.Determinant()
			if math.Abs(det-1) > 1e-9 {
				t.Errorf("determinant = %v, want +1 (got reflection if -1)", det)
			}

			// The matrix must actually rotate `from` onto `to`.
			got := m.MulPosition(c.from)
			if !got.Equals(c.to, 1e-9) {
				t.Errorf("from %v -> %v, want %v", c.from, got, c.to)
			}

			// Chirality check: a right-handed basis must stay right-handed.
			// Rotate the standard basis and check (Rx × Ry) · Rz == +1.
			rx := m.MulPosition(v3.Vec{X: 1, Y: 0, Z: 0})
			ry := m.MulPosition(v3.Vec{X: 0, Y: 1, Z: 0})
			rz := m.MulPosition(v3.Vec{X: 0, Y: 0, Z: 1})
			handedness := rx.Cross(ry).Dot(rz)
			if math.Abs(handedness-1) > 1e-9 {
				t.Errorf("basis handedness = %v, want +1 (mirrored if -1)", handedness)
			}
		})
	}
}

//-----------------------------------------------------------------------------
