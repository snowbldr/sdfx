package sdf

import (
	"testing"

	v2 "github.com/deadsy/sdfx/vec/v2"
	v3 "github.com/deadsy/sdfx/vec/v3"
)

func TestUnion2DFlattens(t *testing.T) {
	c1, _ := Circle2D(1)
	c2, _ := Circle2D(1)
	c3, _ := Circle2D(1)
	c4, _ := Circle2D(1)
	a := Transform2D(c1, Translate2d(v2.Vec{0, 0}))
	b := Transform2D(c2, Translate2d(v2.Vec{5, 0}))
	c := Transform2D(c3, Translate2d(v2.Vec{0, 5}))
	d := Transform2D(c4, Translate2d(v2.Vec{5, 5}))

	inner1 := Union2D(a, b)
	inner2 := Union2D(c, d)
	outer := Union2D(inner1, inner2).(*UnionSDF2)

	if got := len(outer.sdf); got != 4 {
		t.Errorf("want 4 flat children, got %d", got)
	}
}

func TestUnion3DFlattens(t *testing.T) {
	b1, _ := Box3D(v3.Vec{1, 1, 1}, 0)
	b2, _ := Box3D(v3.Vec{1, 1, 1}, 0)
	b3, _ := Box3D(v3.Vec{1, 1, 1}, 0)
	b4, _ := Box3D(v3.Vec{1, 1, 1}, 0)
	a := Transform3D(b1, Translate3d(v3.Vec{0, 0, 0}))
	b := Transform3D(b2, Translate3d(v3.Vec{5, 0, 0}))
	c := Transform3D(b3, Translate3d(v3.Vec{0, 5, 0}))
	d := Transform3D(b4, Translate3d(v3.Vec{5, 5, 0}))

	inner1 := Union3D(a, b)
	inner2 := Union3D(c, d)
	outer := Union3D(inner1, inner2).(*UnionSDF3)

	if got := len(outer.sdf); got != 4 {
		t.Errorf("want 4 flat children, got %d", got)
	}
}
