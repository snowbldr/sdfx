//-----------------------------------------------------------------------------
/*

RotateToVector chirality preservation demo.

A rotation, by definition, must preserve chirality — a right-handed object
must come out right-handed. This example verifies that property visually for
RotateToVector by applying it to a chiral object (a right-handed ISO screw
thread) and comparing the result against an independently-constructed
reference rotation.

Renders three copies of the same thread side by side:

  left   — original, untouched right-handed thread.
  middle — thread transformed by sdf.RotateToVector(+Z, -Z). This is the
           function under test. A correct rotation produces an upside-down
           but still right-handed thread.
  right  — reference: thread transformed by sdf.RotateX(π), which also
           sends +Z to -Z. Known to be a proper rotation; right-handed.

If RotateToVector is correct, the middle and right threads spiral the same
way as the original (all three right-handed). If RotateToVector is broken
in its antiparallel branch (returns a reflection instead of a rotation),
the middle thread will spiral the OPPOSITE way — clearly visible as a
mirrored, left-handed helix.

*/
//-----------------------------------------------------------------------------

package main

import (
	"fmt"

	"github.com/snowbldr/sdfx/render"
	"github.com/snowbldr/sdfx/sdf"
	v3 "github.com/snowbldr/sdfx/vec/v3"
)

//-----------------------------------------------------------------------------

const quality = 200

func mustRightHandedThread() sdf.SDF3 {
	const (
		radius = 5.0  // M10 nominal radius
		pitch  = 1.5  // M10 coarse pitch
		length = 20.0 // bolt length
	)
	thread, err := sdf.ISOThread(radius, pitch, true)
	if err != nil {
		panic(err)
	}
	// starts > 0 → right-handed thread.
	screw, err := sdf.Screw3D(thread, length, 0, pitch, 1)
	if err != nil {
		panic(err)
	}
	return screw
}

func main() {
	const spacing = 18.0

	thread := mustRightHandedThread()
	plusZ := v3.Vec{Z: 1}
	minusZ := v3.Vec{Z: -1}

	// Original, untouched.
	original := sdf.Transform3D(thread, sdf.Translate3d(v3.Vec{X: -spacing}))

	// Function under test: RotateToVector(+Z, -Z).
	// A correct rotation matrix here has determinant +1 and preserves
	// the thread's right-handed chirality. A reflection matrix
	// (determinant -1) silently mirrors it to left-handed.
	underTest := sdf.Transform3D(thread, sdf.RotateToVector(plusZ, minusZ))

	// Reference: a proper 180° rotation around X also sends +Z to -Z, and
	// is unambiguously a rotation (not a reflection). The thread should
	// look identical to the underTest copy if RotateToVector is correct.
	reference := sdf.Transform3D(
		thread,
		sdf.Translate3d(v3.Vec{X: spacing}).Mul(sdf.RotateX(sdf.Pi)),
	)

	out := sdf.Union3D(original, underTest, reference)
	render.ToSTL(out, "rotate_to_vector_chirality.stl", render.NewMarchingCubesUniform(quality))

	det := sdf.RotateToVector(plusZ, minusZ).Determinant()
	fmt.Println("wrote rotate_to_vector_chirality.stl")
	fmt.Printf("det(RotateToVector(+Z, -Z)) = %v  (a rotation has det = +1)\n", det)
	fmt.Println("Compare the threads: all three should spiral the same way.")
	fmt.Println("If the middle one spirals the opposite way, RotateToVector is")
	fmt.Println("returning a reflection instead of a rotation in its 180° branch.")
}

//-----------------------------------------------------------------------------
