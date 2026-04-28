//-----------------------------------------------------------------------------
/*

3D Boxes

*/
//-----------------------------------------------------------------------------

package sdf

import (
	"math"

	v3 "github.com/snowbldr/sdfx/vec/v3"
)

//-----------------------------------------------------------------------------

// Box3 is a 3d bounding box.
type Box3 struct {
	Min, Max v3.Vec
}

// NewBox3 creates a 3d box with a given center and size.
func NewBox3(center, size v3.Vec) Box3 {
	half := size.MulScalar(0.5)
	return Box3{center.Sub(half), center.Add(half)}
}

// Extend returns a box enclosing two 3d boxes.
func (a Box3) Extend(b Box3) Box3 {
	return Box3{a.Min.Min(b.Min), a.Max.Max(b.Max)}
}

// Intersect returns the AABB intersection of two 3d boxes. If the boxes are
// disjoint, returns a degenerate box (Min == Max) at the midpoint of the
// closest gap so callers can still construct a valid (empty-surface) SDF.
func (a Box3) Intersect(b Box3) Box3 {
	lo := a.Min.Max(b.Min)
	hi := a.Max.Min(b.Max)
	if lo.X > hi.X || lo.Y > hi.Y || lo.Z > hi.Z {
		mid := lo.Add(hi).MulScalar(0.5)
		return Box3{Min: mid, Max: mid}
	}
	return Box3{Min: lo, Max: hi}
}

// Include enlarges a 3d box to include a point.
func (a Box3) Include(v v3.Vec) Box3 {
	return Box3{a.Min.Min(v), a.Max.Max(v)}
}

// Translate translates a 3d box.
func (a Box3) Translate(v v3.Vec) Box3 {
	return Box3{a.Min.Add(v), a.Max.Add(v)}
}

// Size returns the size of a 3d box.
func (a Box3) Size() v3.Vec {
	return a.Max.Sub(a.Min)
}

// Center returns the center of a 3d box.
func (a Box3) Center() v3.Vec {
	return a.Min.Add(a.Size().MulScalar(0.5))
}

// ScaleAboutCenter returns a new 3d box scaled about the center of a box.
func (a Box3) ScaleAboutCenter(k float64) Box3 {
	return NewBox3(a.Center(), a.Size().MulScalar(k))
}

// Enlarge returns a new 3d box enlarged by a size vector.
func (a Box3) Enlarge(v v3.Vec) Box3 {
	v = v.MulScalar(0.5)
	return Box3{a.Min.Sub(v), a.Max.Add(v)}
}

// Cube returns a cubical box larger than the original box.
func (a Box3) Cube() Box3 {
	side := a.Size().MaxComponent()
	return Box3{a.Min, a.Min.Add(v3.Vec{side, side, side})}
}

// Contains checks if the 3d box contains the point.
func (a Box3) Contains(v v3.Vec) bool {
	return a.Min.X <= v.X &&
		a.Min.Y <= v.Y &&
		a.Min.Z <= v.Z &&
		v.X <= a.Max.X &&
		v.Y <= a.Max.Y &&
		v.Z <= a.Max.Z
}

// Clip returns the smallest axis-aligned box containing the intersection of
// this box with the half-space {p : (p - point) . normal >= 0}. If the box
// lies entirely on the negative side, a degenerate empty box at point is
// returned.
func (a Box3) Clip(point, normal v3.Vec) Box3 {
	verts := a.Vertices()
	dists := make([]float64, len(verts))
	for i, v := range verts {
		dists[i] = v.Sub(point).Dot(normal)
	}
	var pts []v3.Vec
	for i, v := range verts {
		if dists[i] >= 0 {
			pts = append(pts, v)
		}
	}
	// Edges connect vertices that differ in exactly one coordinate.
	// Vertex order from Vertices(): bit 0 = Z, bit 1 = Y, bit 2 = X.
	edges := [12][2]int{
		{0, 1}, {2, 3}, {4, 5}, {6, 7}, // Z edges
		{0, 2}, {1, 3}, {4, 6}, {5, 7}, // Y edges
		{0, 4}, {1, 5}, {2, 6}, {3, 7}, // X edges
	}
	for _, e := range edges {
		d0, d1 := dists[e[0]], dists[e[1]]
		if (d0 > 0 && d1 < 0) || (d0 < 0 && d1 > 0) {
			t := d0 / (d0 - d1)
			v0, v1 := verts[e[0]], verts[e[1]]
			pts = append(pts, v0.Add(v1.Sub(v0).MulScalar(t)))
		}
	}
	if len(pts) == 0 {
		return Box3{Min: point, Max: point}
	}
	out := Box3{Min: pts[0], Max: pts[0]}
	for _, p := range pts[1:] {
		out = out.Include(p)
	}
	return out
}

// Vertices returns a slice of 3d box corner vertices.
func (a Box3) Vertices() v3.VecSet {
	return []v3.Vec{
		a.Min,
		{a.Min.X, a.Min.Y, a.Max.Z},
		{a.Min.X, a.Max.Y, a.Min.Z},
		{a.Min.X, a.Max.Y, a.Max.Z},
		{a.Max.X, a.Min.Y, a.Min.Z},
		{a.Max.X, a.Min.Y, a.Max.Z},
		{a.Max.X, a.Max.Y, a.Min.Z},
		a.Max,
	}
}

// Snap a point to the box edges
func (a *Box3) Snap(p v3.Vec, epsilon float64) v3.Vec {
	p.X = SnapFloat64(p.X, a.Min.X, epsilon)
	p.X = SnapFloat64(p.X, a.Max.X, epsilon)
	p.Y = SnapFloat64(p.Y, a.Min.Y, epsilon)
	p.Y = SnapFloat64(p.Y, a.Max.Y, epsilon)
	p.Z = SnapFloat64(p.Z, a.Min.Z, epsilon)
	p.Z = SnapFloat64(p.Z, a.Max.Z, epsilon)
	return p
}

// Equals test the equality of 3d boxes.
func (a Box3) Equals(b Box3, delta float64) bool {
	return a.Min.Equals(b.Min, delta) && a.Max.Equals(b.Max, delta)
}

//-----------------------------------------------------------------------------
// Box Sub-Octants

func (a Box3) oct0() Box3 {
	delta := a.Size().MulScalar(0.5)
	ll := a.Min
	return Box3{ll, ll.Add(delta)}
}

func (a Box3) oct1() Box3 {
	delta := a.Size().MulScalar(0.5)
	ll := v3.Vec{a.Min.X + delta.X, a.Min.Y, a.Min.Z}
	return Box3{ll, ll.Add(delta)}
}

func (a Box3) oct2() Box3 {
	delta := a.Size().MulScalar(0.5)
	ll := v3.Vec{a.Min.X, a.Min.Y + delta.Y, a.Min.Z}
	return Box3{ll, ll.Add(delta)}
}

func (a Box3) oct3() Box3 {
	delta := a.Size().MulScalar(0.5)
	ll := v3.Vec{a.Min.X + delta.X, a.Min.Y + delta.Y, a.Min.Z}
	return Box3{ll, ll.Add(delta)}
}

func (a Box3) oct4() Box3 {
	delta := a.Size().MulScalar(0.5)
	ll := v3.Vec{a.Min.X, a.Min.Y, a.Min.Z + delta.Z}
	return Box3{ll, ll.Add(delta)}
}

func (a Box3) oct5() Box3 {
	delta := a.Size().MulScalar(0.5)
	ll := v3.Vec{a.Min.X + delta.X, a.Min.Y, a.Min.Z + delta.Z}
	return Box3{ll, ll.Add(delta)}
}

func (a Box3) oct6() Box3 {
	delta := a.Size().MulScalar(0.5)
	ll := v3.Vec{a.Min.X, a.Min.Y + delta.Y, a.Min.Z + delta.Z}
	return Box3{ll, ll.Add(delta)}
}

func (a Box3) oct7() Box3 {
	delta := a.Size().MulScalar(0.5)
	ll := a.Min.Add(delta)
	return Box3{ll, ll.Add(delta)}
}

// MinDist2 returns the squared minimum distance from a point to the box.
// Returns 0 if the point is inside the box.
//
// Per axis we take the larger of (min-p) and (p-max); exactly one is
// non-negative when p is outside the slab, both are non-positive when
// p is inside, so clamping to 0 gives the slab gap. The flat style
// (vs nested if/else) keeps inliner cost under budget — critical
// because UnionSDF3 / DifferenceSDF3 call this once per child per
// evaluation on the rendering hot path.
func (a Box3) MinDist2(p v3.Vec) float64 {
	dx := max(a.Min.X-p.X, p.X-a.Max.X, 0)
	dy := max(a.Min.Y-p.Y, p.Y-a.Max.Y, 0)
	dz := max(a.Min.Z-p.Z, p.Z-a.Max.Z, 0)
	return dx*dx + dy*dy + dz*dz
}

// MinDist2GT reports whether MinDist2(p) > bound. Short-circuits after the
// XY-plane contribution so the Z slab math is skipped when XY alone already
// exceeds bound — callers only want the boolean. Kept lean enough to inline.
func (a Box3) MinDist2GT(p v3.Vec, bound float64) bool {
	dx := max(a.Min.X-p.X, p.X-a.Max.X, 0)
	dy := max(a.Min.Y-p.Y, p.Y-a.Max.Y, 0)
	d2 := dx*dx + dy*dy
	if d2 > bound {
		return true
	}
	dz := max(a.Min.Z-p.Z, p.Z-a.Max.Z, 0)
	return d2+dz*dz > bound
}

//-----------------------------------------------------------------------------
// Minimum/Maximum distances from a point to a box

// MinMaxDist2 returns the minimum and maximum dist * dist from a point to a box.
// Points within the box have minimum distance = 0.
func (a Box3) MinMaxDist2(p v3.Vec) Interval {
	maxDist2 := 0.0
	minDist2 := 0.0

	// translate the box so p is at the origin
	a = a.Translate(p.Neg())

	// consider the vertices
	vs := a.Vertices()
	for i := range vs {
		d2 := vs[i].Length2()
		if i == 0 {
			minDist2 = d2
		} else {
			minDist2 = math.Min(minDist2, d2)
		}
		maxDist2 = math.Max(maxDist2, d2)
	}

	// consider the faces (for the minimum)
	withinX := a.Min.X < 0 && a.Max.X > 0
	withinY := a.Min.Y < 0 && a.Max.Y > 0
	withinZ := a.Min.Z < 0 && a.Max.Z > 0

	if withinX && withinY && withinZ {
		minDist2 = 0
	} else {
		if withinX && withinY {
			d := math.Min(math.Abs(a.Max.Z), math.Abs(a.Min.Z))
			minDist2 = math.Min(minDist2, d*d)
		}
		if withinX && withinZ {
			d := math.Min(math.Abs(a.Max.Y), math.Abs(a.Min.Y))
			minDist2 = math.Min(minDist2, d*d)
		}
		if withinY && withinZ {
			d := math.Min(math.Abs(a.Max.X), math.Abs(a.Min.X))
			minDist2 = math.Min(minDist2, d*d)
		}
	}

	return Interval{minDist2, maxDist2}
}

//-----------------------------------------------------------------------------

// Random returns a random point within 3d box.
func (a *Box3) Random() v3.Vec {
	return v3.Vec{
		randomRange(a.Min.X, a.Max.X),
		randomRange(a.Min.Y, a.Max.Y),
		randomRange(a.Min.Z, a.Max.Z),
	}
}

// RandomTriangle returns a random triangle that lies within the box
func (a *Box3) RandomTriangle() Triangle3 {
	return Triangle3{
		a.Random(),
		a.Random(),
		a.Random(),
	}
}

// RandomSet returns a set of random points from within a 3d box.
func (a *Box3) RandomSet(n int) v3.VecSet {
	s := make([]v3.Vec, n)
	for i := range s {
		s[i] = a.Random()
	}
	return s
}

//-----------------------------------------------------------------------------
