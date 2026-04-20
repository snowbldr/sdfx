//-----------------------------------------------------------------------------
/*

3D Signed Distance Functions

*/
//-----------------------------------------------------------------------------

package sdf

import (
	"errors"
	"math"

	"github.com/snowbldr/sdfx/vec/conv"
	"github.com/snowbldr/sdfx/vec/p2"
	v2 "github.com/snowbldr/sdfx/vec/v2"
	v3 "github.com/snowbldr/sdfx/vec/v3"
	"github.com/snowbldr/sdfx/vec/v3i"
)

//-----------------------------------------------------------------------------

// SDF3 is the interface to a 3d signed distance function object.
type SDF3 interface {
	Evaluate(p v3.Vec) float64
	BoundingBox() Box3
}

//-----------------------------------------------------------------------------
// Basic SDF Functions

/*
func sdfBox3d(p, s v3.Vec) float64 {
	d := p.Abs().Sub(s)
	return d.Max(v3.Vec{0, 0, 0}).Length() + Min(d.MaxComponent(), 0)
}
*/

func sdfBox3d(p, s v3.Vec) float64 {
	d := p.Abs().Sub(s)
	if d.X > 0 && d.Y > 0 && d.Z > 0 {
		return d.Length()
	}
	if d.X > 0 && d.Y > 0 {
		return v2.Vec{d.X, d.Y}.Length()
	}
	if d.X > 0 && d.Z > 0 {
		return v2.Vec{d.X, d.Z}.Length()
	}
	if d.Y > 0 && d.Z > 0 {
		return v2.Vec{d.Y, d.Z}.Length()
	}
	if d.X > 0 {
		return d.X
	}
	if d.Y > 0 {
		return d.Y
	}
	if d.Z > 0 {
		return d.Z
	}
	return d.MaxComponent()
}

//-----------------------------------------------------------------------------

// SorSDF3 solid of revolution, SDF2 to SDF3.
type SorSDF3 struct {
	sdf   SDF2
	theta float64 // angle for partial revolutions
	norm  v2.Vec  // pre-calculated normal to theta line
	bb    Box3
}

// RevolveTheta3D returns an SDF3 for a solid of revolution.
func RevolveTheta3D(sdf SDF2, theta float64) (SDF3, error) {
	if sdf == nil {
		return nil, nil
	}
	if theta < 0 {
		return nil, ErrMsg("theta < 0")
	}
	s := SorSDF3{}
	s.sdf = sdf
	// normalize theta
	s.theta = math.Mod(math.Abs(theta), Tau)
	sin := math.Sin(s.theta)
	cos := math.Cos(s.theta)
	// pre-calculate the normal to the theta line
	s.norm = v2.Vec{-sin, cos}
	// work out the bounding box
	var vset v2.VecSet
	if s.theta == 0 {
		vset = []v2.Vec{{1, 1}, {-1, -1}}
	} else {
		vset = []v2.Vec{{0, 0}, {1, 0}, {cos, sin}}
		if s.theta > 0.5*Pi {
			vset = append(vset, v2.Vec{0, 1})
		}
		if s.theta > Pi {
			vset = append(vset, v2.Vec{-1, 0})
		}
		if s.theta > 1.5*Pi {
			vset = append(vset, v2.Vec{0, -1})
		}
	}
	bb := s.sdf.BoundingBox()
	l := math.Max(math.Abs(bb.Min.X), math.Abs(bb.Max.X))
	vmin := vset.Min().MulScalar(l)
	vmax := vset.Max().MulScalar(l)
	s.bb = Box3{v3.Vec{vmin.X, vmin.Y, bb.Min.Y}, v3.Vec{vmax.X, vmax.Y, bb.Max.Y}}
	return &s, nil
}

// Revolve3D returns an SDF3 for a solid of revolution.
func Revolve3D(sdf SDF2) (SDF3, error) {
	return RevolveTheta3D(sdf, 0)
}

// Evaluate returns the minimum distance to a solid of revolution.
func (s *SorSDF3) Evaluate(p v3.Vec) float64 {
	x := math.Sqrt(p.X*p.X + p.Y*p.Y)
	a := s.sdf.Evaluate(v2.Vec{x, p.Z})
	b := a
	if s.theta != 0 {
		// combine two vertical planes to give an intersection wedge
		d := s.norm.Dot(v2.Vec{p.X, p.Y})
		if s.theta < Pi {
			b = math.Max(-p.Y, d) // intersect
		} else {
			b = math.Min(-p.Y, d) // union
		}
	}
	// return the intersection
	return math.Max(a, b)
}

// BoundingBox returns the bounding box for a solid of revolution.
func (s *SorSDF3) BoundingBox() Box3 {
	return s.bb
}

//-----------------------------------------------------------------------------

// ExtrudeSDF3 extrudes an SDF2 to an SDF3.
type ExtrudeSDF3 struct {
	sdf        SDF2
	height     float64
	extrude    ExtrudeFunc
	bb         Box3
	isNormal   bool    // default NormalExtrude — skip function-pointer call in Evaluate
	invStretch float64 // 1/(Lipschitz stretch of the 3D→2D projection); 1 for identity
}

// Extrude3D does a linear extrude on an SDF3.
func Extrude3D(sdf SDF2, height float64) SDF3 {
	s := ExtrudeSDF3{}
	s.sdf = sdf
	s.height = height / 2
	s.extrude = NormalExtrude
	s.isNormal = true
	s.invStretch = 1
	// work out the bounding box
	bb := sdf.BoundingBox()
	s.bb = Box3{v3.Vec{bb.Min.X, bb.Min.Y, -s.height}, v3.Vec{bb.Max.X, bb.Max.Y, s.height}}
	return &s
}

// TwistExtrude3D extrudes an SDF2 while rotating by twist radians over the height of the extrusion.
func TwistExtrude3D(sdf SDF2, height, twist float64) SDF3 {
	s := ExtrudeSDF3{}
	s.sdf = sdf
	s.height = height / 2
	s.extrude = TwistExtrude(height, twist)
	// work out the bounding box
	bb := sdf.BoundingBox()
	l := bb.Max.Length()
	s.bb = Box3{v3.Vec{-l, -l, -s.height}, v3.Vec{l, l, s.height}}
	// The twist mapping rotates (x,y) by θ=k·z where k=twist/height. Its
	// Jacobian has maximum singular value σ_max = √(1 + k²r²) with
	// r² = x² + y². Without correction, the returned SDF overestimates
	// true 3D distance by up to σ_max, causing the octree isEmpty check
	// to skip cubes that contain the surface (holes). All surfaces of the
	// extruded shape lie within the 2D bounding box, so r ≤ rMax and we
	// can use a single global correction constant.
	k := twist / height
	rMax2 := math.Max(bb.Min.X*bb.Min.X, bb.Max.X*bb.Max.X) +
		math.Max(bb.Min.Y*bb.Min.Y, bb.Max.Y*bb.Max.Y)
	s.invStretch = 1 / math.Sqrt(1+k*k*rMax2)
	return &s
}

// ScaleExtrude3D extrudes an SDF2 and scales it over the height of the extrusion.
func ScaleExtrude3D(sdf SDF2, height float64, scale v2.Vec) SDF3 {
	s := ExtrudeSDF3{}
	s.sdf = sdf
	s.height = height / 2
	s.extrude = ScaleExtrude(height, scale)
	// work out the bounding box
	bb := sdf.BoundingBox()
	bb = bb.Extend(Box2{bb.Min.Mul(scale), bb.Max.Mul(scale)})
	s.bb = Box3{v3.Vec{bb.Min.X, bb.Min.Y, -s.height}, v3.Vec{bb.Max.X, bb.Max.Y, s.height}}
	// Lipschitz correction for the scale map f(x,y,z) = (x·s_x(z), y·s_y(z))
	// with s_x(z) = m_x·z + b_x (b_x makes s_x(-h/2)=1, s_x(h/2)=1/scale_x).
	// Jacobian rows: (s_x, 0, m_x·x), (0, s_y, m_y·y). For 2x2 JJᵀ:
	//   A = s_x² + m_x²x², C = s_y² + m_y²y², B² = m_x²m_y²x²y²
	//   λ_max = (A+C)/2 + √((A-C)²/4 + B²).
	// λ_max is monotone in x², y² and in s_x², s_y². Its max over the volume
	// is at one of the two z endpoints with |x|, |y| at their extreme.
	invX := 1 / scale.X
	invY := 1 / scale.Y
	mx := (invX - 1) / height
	my := (invY - 1) / height
	xMax := math.Max(math.Abs(s.bb.Min.X), math.Abs(s.bb.Max.X))
	yMax := math.Max(math.Abs(s.bb.Min.Y), math.Abs(s.bb.Max.Y))
	a := mx * mx * xMax * xMax
	c := my * my * yMax * yMax
	sigma2 := math.Max(
		scaleExtrudeLambdaMax(1, 1, a, c),
		scaleExtrudeLambdaMax(invX*invX, invY*invY, a, c),
	)
	s.invStretch = 1
	if sigma2 > 1 {
		s.invStretch = 1 / math.Sqrt(sigma2)
	}
	return &s
}

// scaleExtrudeLambdaMax returns λ_max of the 2x2 matrix JJᵀ for the scale
// extrude Jacobian with diagonal scale entries sx², sy² and z-column squared
// entries a = m_x²x², c = m_y²y² (so B² = a·c).
func scaleExtrudeLambdaMax(sx2, sy2, a, c float64) float64 {
	A := sx2 + a
	C := sy2 + c
	return (A+C)/2 + math.Sqrt((A-C)*(A-C)/4+a*c)
}

// ScaleTwistExtrude3D extrudes an SDF2 and scales and twists it over the height of the extrusion.
func ScaleTwistExtrude3D(sdf SDF2, height, twist float64, scale v2.Vec) SDF3 {
	s := ExtrudeSDF3{}
	s.sdf = sdf
	s.height = height / 2
	s.extrude = ScaleTwistExtrude(height, twist, scale)
	// work out the bounding box
	bb := sdf.BoundingBox()
	bb = bb.Extend(Box2{bb.Min.Mul(scale), bb.Max.Mul(scale)})
	l := bb.Max.Length()
	s.bb = Box3{v3.Vec{-l, -l, -s.height}, v3.Vec{l, l, s.height}}
	// Lipschitz correction for the composed map
	//   f(x,y,z) = R(k·z) · (x·s_x(z), y·s_y(z)),   k = twist/height.
	// Rotation is orthogonal so σ_max is invariant to R. The rotation-free
	// Jacobian columns are (s_x,0), (0,s_y), and
	//   ∂z = (m_x·x − k·y·s_y, k·x·s_x + m_y·y).
	// At fixed z, λ_max(x,y) of JJᵀ is a convex quadratic form in (x,y) so
	// its max over the 2D bbox is attained at a corner. z is checked at
	// the two endpoints (s_x², s_y² are parabolas with interior minima).
	k := twist / height
	invX := 1 / scale.X
	invY := 1 / scale.Y
	mx := (invX - 1) / height
	my := (invY - 1) / height
	sigma2 := 1.0
	zs := [2]float64{-s.height, s.height}
	xs := [2]float64{s.bb.Min.X, s.bb.Max.X}
	ys := [2]float64{s.bb.Min.Y, s.bb.Max.Y}
	for _, z := range zs {
		sx := mx*z + (invX+1)/2
		sy := my*z + (invY+1)/2
		for _, x := range xs {
			for _, y := range ys {
				j13 := mx*x - k*sy*y
				j23 := k*sx*x + my*y
				A := sx*sx + j13*j13
				C := sy*sy + j23*j23
				B := j13 * j23
				lam := (A+C)/2 + math.Sqrt((A-C)*(A-C)/4+B*B)
				if lam > sigma2 {
					sigma2 = lam
				}
			}
		}
	}
	s.invStretch = 1 / math.Sqrt(sigma2)
	return &s
}

// Evaluate returns the minimum distance to an extrusion.
func (s *ExtrudeSDF3) Evaluate(p v3.Vec) float64 {
	var q v2.Vec
	if s.isNormal {
		q = v2.Vec{X: p.X, Y: p.Y}
	} else {
		q = s.extrude(p)
	}
	a := s.sdf.Evaluate(q)
	z := p.Z
	if z < 0 {
		z = -z
	}
	b := z - s.height
	d := b
	if a > b {
		d = a
	}
	return d * s.invStretch
}

// SetExtrude sets the extrusion control function.
func (s *ExtrudeSDF3) SetExtrude(extrude ExtrudeFunc) {
	s.extrude = extrude
	s.isNormal = false
}

// BoundingBox returns the bounding box for an extrusion.
func (s *ExtrudeSDF3) BoundingBox() Box3 {
	return s.bb
}

//-----------------------------------------------------------------------------
// Linear extrude an SDF2 with rounded edges.
// Note: The height of the extrusion is adjusted for the rounding.
// The underlying SDF2 shape is not modified.

// ExtrudeRoundedSDF3 extrudes an SDF2 to an SDF3 with rounded edges.
type ExtrudeRoundedSDF3 struct {
	sdf    SDF2
	height float64
	round  float64
	bb     Box3
}

// ExtrudeRounded3D extrudes an SDF2 to an SDF3 with rounded edges.
func ExtrudeRounded3D(sdf SDF2, height, round float64) (SDF3, error) {
	if round == 0 {
		// revert to non-rounded case
		return Extrude3D(sdf, height), nil
	}
	if sdf == nil {
		return nil, errors.New("sdf == nil")
	}
	if height <= 0 {
		return nil, errors.New("height <= 0")
	}
	if round < 0 {
		return nil, errors.New("round < 0")
	}
	if height < 2*round {
		return nil, errors.New("height < 2 * round")
	}
	s := ExtrudeRoundedSDF3{
		sdf:    sdf,
		height: (height / 2) - round,
		round:  round,
	}
	// work out the bounding box
	bb := sdf.BoundingBox()
	s.bb = Box3{v3.Vec{bb.Min.X, bb.Min.Y, -s.height}.SubScalar(round), v3.Vec{bb.Max.X, bb.Max.Y, s.height}.AddScalar(round)}
	return &s, nil
}

// Evaluate returns the minimum distance to a rounded extrusion.
func (s *ExtrudeRoundedSDF3) Evaluate(p v3.Vec) float64 {
	a := s.sdf.Evaluate(v2.Vec{p.X, p.Y})
	z := p.Z
	if z < 0 {
		z = -z
	}
	b := z - s.height
	var d float64
	if b > 0 {
		if a < 0 {
			d = b
		} else {
			d = math.Sqrt((a * a) + (b * b))
		}
	} else {
		if a < 0 {
			if a > b {
				d = a
			} else {
				d = b
			}
		} else {
			d = a
		}
	}
	return d - s.round
}

// BoundingBox returns the bounding box for a rounded extrusion.
func (s *ExtrudeRoundedSDF3) BoundingBox() Box3 {
	return s.bb
}

//-----------------------------------------------------------------------------
// Extrude/Loft (with rounded edges)
// Blend between sdf0 and sdf1 as we move from bottom to top.

// LoftSDF3 is an extrusion between two SDF2s.
type LoftSDF3 struct {
	sdf0, sdf1 SDF2
	height     float64
	round      float64
	bb         Box3
	invStretch float64
}

// Loft3D extrudes an SDF3 that transitions between two SDF2 shapes.
func Loft3D(sdf0, sdf1 SDF2, height, round float64) (SDF3, error) {
	if sdf0 == nil {
		return nil, errors.New("sdf0 == nil")
	}
	if sdf1 == nil {
		return nil, errors.New("sdf1 == nil")
	}
	if height <= 0 {
		return nil, errors.New("height <= 0")
	}
	if round < 0 {
		return nil, errors.New("round < 0")
	}
	if height < 2*round {
		return nil, errors.New("height < 2 * round")
	}
	s := LoftSDF3{
		sdf0:   sdf0,
		sdf1:   sdf1,
		height: (height / 2) - round,
		round:  round,
	}
	// work out the bounding box
	bb0 := sdf0.BoundingBox()
	bb1 := sdf1.BoundingBox()
	bb := bb0.Extend(bb1)
	s.bb = Box3{v3.Vec{bb.Min.X, bb.Min.Y, -s.height}.SubScalar(round), v3.Vec{bb.Max.X, bb.Max.Y, s.height}.AddScalar(round)}
	// Lipschitz correction. For a = Mix(a0, a1, k), k = 0.5·z/h + 0.5 with
	// h = height/2 − round, the xy gradient is a convex combination of the
	// two Lipschitz-1 gradients so Lip_xy ≤ 1; the z gradient is bounded by
	// L_z = max|a1−a0|/(2h). The outer d = √(a² + b²) end-cap gives a
	// Lipschitz of √(1 + L_z(L_z + √(L_z²+4))/2) (≥ √(1+L_z²)).
	lz := maxAbsDifference(sdf0, sdf1, bb, 16) / (2 * s.height)
	lend2 := 1 + lz*(lz+math.Sqrt(lz*lz+4))/2
	s.invStretch = 1
	if lend2 > 1 {
		s.invStretch = 1 / math.Sqrt(lend2)
	}
	return &s, nil
}

// maxAbsDifference returns an upper bound on max|a0(p) − a1(p)| over bb by
// sampling an n×n grid and adding a Lipschitz-2 between-samples safety term
// (a0 − a1 is Lipschitz-2 since each sdf is Lipschitz-1).
func maxAbsDifference(a0, a1 SDF2, bb Box2, n int) float64 {
	maxDiff := 0.0
	dx := bb.Max.X - bb.Min.X
	dy := bb.Max.Y - bb.Min.Y
	for i := 0; i < n; i++ {
		for j := 0; j < n; j++ {
			ti := float64(i) / float64(n-1)
			tj := float64(j) / float64(n-1)
			p := v2.Vec{bb.Min.X + dx*ti, bb.Min.Y + dy*tj}
			d := math.Abs(a0.Evaluate(p) - a1.Evaluate(p))
			if d > maxDiff {
				maxDiff = d
			}
		}
	}
	halfDiag := math.Hypot(dx/float64(n-1), dy/float64(n-1)) / 2
	return maxDiff + 2*halfDiag
}

// Evaluate returns the minimum distance to a loft extrusion.
func (s *LoftSDF3) Evaluate(p v3.Vec) float64 {
	k := Clamp((0.5*p.Z/s.height)+0.5, 0, 1)
	a0 := s.sdf0.Evaluate(v2.Vec{p.X, p.Y})
	a1 := s.sdf1.Evaluate(v2.Vec{p.X, p.Y})
	a := Mix(a0, a1, k)

	z := p.Z
	if z < 0 {
		z = -z
	}
	b := z - s.height
	var d float64
	if b > 0 {
		if a < 0 {
			d = b
		} else {
			d = math.Sqrt((a * a) + (b * b))
		}
	} else {
		if a < 0 {
			if a > b {
				d = a
			} else {
				d = b
			}
		} else {
			d = a
		}
	}
	return (d - s.round) * s.invStretch
}

// BoundingBox returns the bounding box for a loft extrusion.
func (s *LoftSDF3) BoundingBox() Box3 {
	return s.bb
}

//-----------------------------------------------------------------------------
// Box (exact distance field)

// BoxSDF3 is a 3d box.
type BoxSDF3 struct {
	size  v3.Vec
	round float64
	bb    Box3
}

// Box3D return an SDF3 for a 3d box (rounded corners with round > 0).
func Box3D(size v3.Vec, round float64) (SDF3, error) {
	if size.LTEZero() {
		return nil, ErrMsg("size <= 0")
	}
	if round < 0 {
		return nil, ErrMsg("round < 0")
	}
	size = size.MulScalar(0.5)
	s := BoxSDF3{}
	s.size = size.SubScalar(round)
	s.round = round
	s.bb = Box3{size.Neg(), size}
	return &s, nil
}

// Evaluate returns the minimum distance to a 3d box.
func (s *BoxSDF3) Evaluate(p v3.Vec) float64 {
	return sdfBox3d(p, s.size) - s.round
}

// BoundingBox returns the bounding box for a 3d box.
func (s *BoxSDF3) BoundingBox() Box3 {
	return s.bb
}

//-----------------------------------------------------------------------------
// Sphere (exact distance field)

// SphereSDF3 is a sphere.
type SphereSDF3 struct {
	radius float64
	bb     Box3
}

// Sphere3D return an SDF3 for a sphere.
func Sphere3D(radius float64) (SDF3, error) {
	if radius <= 0 {
		return nil, ErrMsg("radius <= 0")
	}
	s := SphereSDF3{}
	s.radius = radius
	d := v3.Vec{radius, radius, radius}
	s.bb = Box3{d.Neg(), d}
	return &s, nil
}

// Evaluate returns the minimum distance to a sphere.
func (s *SphereSDF3) Evaluate(p v3.Vec) float64 {
	return p.Length() - s.radius
}

// BoundingBox returns the bounding box for a sphere.
func (s *SphereSDF3) BoundingBox() Box3 {
	return s.bb
}

//-----------------------------------------------------------------------------
// Cylinder (exact distance field)

// CylinderSDF3 is a cylinder.
type CylinderSDF3 struct {
	height float64
	radius float64
	round  float64
	bb     Box3
}

// Cylinder3D return an SDF3 for a cylinder (rounded edges with round > 0).
func Cylinder3D(height, radius, round float64) (SDF3, error) {
	if radius <= 0 {
		return nil, ErrMsg("radius <= 0")
	}
	if round < 0 {
		return nil, ErrMsg("round < 0")
	}
	if round > radius {
		return nil, ErrMsg("round > radius")
	}
	if height < 2.0*round {
		return nil, ErrMsg("height < 2 * round")
	}
	s := CylinderSDF3{}
	s.height = (height / 2) - round
	s.radius = radius - round
	s.round = round
	d := v3.Vec{radius, radius, height / 2}
	s.bb = Box3{d.Neg(), d}
	return &s, nil
}

// Capsule3D return an SDF3 for a capsule.
func Capsule3D(height, radius float64) (SDF3, error) {
	return Cylinder3D(height, radius, radius)
}

// Evaluate returns the minimum distance to a cylinder.
//
// Inlines sdfBox2d with two bit-exact simplifications:
//  1. sqrt(p.X²+p.Y²) is already nonneg so math.Abs is a no-op and dropped.
//  2. The v2.Vec intermediaries become plain floats, skipping struct
//     construction that the inliner otherwise has to track.
//
// Profiled at 7% flat CPU in the parallel renderer, so trimming allocation
// shape alone is worth the flattening.
func (s *CylinderSDF3) Evaluate(p v3.Vec) float64 {
	px := math.Sqrt(p.X*p.X + p.Y*p.Y)
	py := math.Abs(p.Z)
	dx := px - s.radius
	dy := py - s.height
	var d float64
	if dx > 0 && dy > 0 {
		d = math.Sqrt(dx*dx + dy*dy)
	} else if py-px > s.height-s.radius {
		d = dy
	} else {
		d = dx
	}
	return d - s.round
}

// BoundingBox returns the bounding box for a cylinder.
func (s *CylinderSDF3) BoundingBox() Box3 {
	return s.bb
}

//-----------------------------------------------------------------------------
// Truncated Cone (exact distance field)

// ConeSDF3 is a truncated cone.
type ConeSDF3 struct {
	r0     float64 // base radius
	r1     float64 // top radius
	height float64 // half height
	round  float64 // rounding offset
	u      v2.Vec  // normalized cone slope vector
	n      v2.Vec  // normal to cone slope (points outward)
	l      float64 // length of cone slope
	bb     Box3    // bounding box
}

// Cone3D returns the SDF3 for a trucated cone (round > 0 gives rounded edges).
func Cone3D(height, r0, r1, round float64) (SDF3, error) {
	if height <= 0 {
		return nil, ErrMsg("height <= 0")
	}
	if round < 0 {
		return nil, ErrMsg("round < 0")
	}
	if height < 2.0*round {
		return nil, ErrMsg("height < 2 * round")
	}
	s := ConeSDF3{}
	s.height = (height / 2) - round
	s.round = round
	// cone slope vector and normal
	s.u = v2.Vec{r1, height / 2}.Sub(v2.Vec{r0, -height / 2}).Normalize()
	s.n = v2.Vec{s.u.Y, -s.u.X}
	// inset the radii for the rounding
	ofs := round / s.n.X
	s.r0 = r0 - (1+s.n.Y)*ofs
	s.r1 = r1 - (1-s.n.Y)*ofs
	// cone slope length
	s.l = v2.Vec{s.r1, s.height}.Sub(v2.Vec{s.r0, -s.height}).Length()
	// work out the bounding box
	r := math.Max(s.r0+round, s.r1+round)
	s.bb = Box3{v3.Vec{-r, -r, -height / 2}, v3.Vec{r, r, height / 2}}
	return &s, nil
}

// Evaluate returns the minimum distance to a trucated cone.
func (s *ConeSDF3) Evaluate(p v3.Vec) float64 {
	// convert to SoR 2d coordinates
	p2 := v2.Vec{v2.Vec{p.X, p.Y}.Length(), p.Z}
	// is p2 above the cone?
	if p2.Y >= s.height && p2.X <= s.r1 {
		return p2.Y - s.height - s.round
	}
	// is p2 below the cone?
	if p2.Y <= -s.height && p2.X <= s.r0 {
		return -p2.Y - s.height - s.round
	}
	// distance to slope line
	v := p2.Sub(v2.Vec{s.r0, -s.height})
	dSlope := v.Dot(s.n)
	// is p2 inside the cone?
	if dSlope < 0 && math.Abs(p2.Y) < s.height {
		return -math.Min(-dSlope, s.height-math.Abs(p2.Y)) - s.round
	}
	// is p2 closest to the slope line?
	t := v.Dot(s.u)
	if t >= 0 && t <= s.l {
		return dSlope - s.round
	}
	// is p2 closest to the base radius vertex?
	if t < 0 {
		return v.Length() - s.round
	}
	// p2 is closest to the top radius vertex
	return p2.Sub(v2.Vec{s.r1, s.height}).Length() - s.round
}

// BoundingBox return the bounding box for the trucated cone..
func (s *ConeSDF3) BoundingBox() Box3 {
	return s.bb
}

//-----------------------------------------------------------------------------
// Transform SDF3 (rotation, translation - distance preserving)

// TransformSDF3 is an SDF3 transformed with a 4x4 transformation matrix.
type TransformSDF3 struct {
	sdf       SDF3
	matrix    M44
	inverse   M44
	translate v3.Vec // inverse translation when isTranslate (-v for Translate3d(v))
	bb        Box3
	// invStretch is 1/σ_max(M⁻¹_3x3) clamped to ≤ 1. Evaluating the inner
	// SDF at M⁻¹·p has Lipschitz factor σ_max of that inverse linear map;
	// scaling by invStretch restores Lipschitz-1 (safe for isEmpty skip).
	invStretch float64
	// isTranslate marks transforms that are pure translations so Evaluate
	// can skip the 12-multiply matrix-vector product and just add translate.
	// Translate3d is by far the most common Transform3D usage (243 call
	// sites across the repo as of this change), so specializing it wins.
	isTranslate bool
}

// Transform3D applies a transformation matrix to an SDF3.
func Transform3D(sdf SDF3, matrix M44) SDF3 {
	s := TransformSDF3{}
	s.sdf = sdf
	s.matrix = matrix
	s.inverse = matrix.Inverse()
	s.bb = matrix.MulBox(sdf.BoundingBox())
	// Detect translation-only inverse. MulPosition on such a matrix
	// reduces to p + (tx,ty,tz), which is what isTranslate fast-paths.
	inv := &s.inverse
	if inv[0] == 1 && inv[1] == 0 && inv[2] == 0 &&
		inv[4] == 0 && inv[5] == 1 && inv[6] == 0 &&
		inv[8] == 0 && inv[9] == 0 && inv[10] == 1 &&
		inv[12] == 0 && inv[13] == 0 && inv[14] == 0 && inv[15] == 1 {
		s.isTranslate = true
		s.translate = v3.Vec{X: inv[3], Y: inv[7], Z: inv[11]}
		s.invStretch = 1
	} else {
		sigma2 := m44LinearSigmaMax2(inv)
		s.invStretch = 1
		if sigma2 > 1 {
			s.invStretch = 1 / math.Sqrt(sigma2)
		}
	}
	return &s
}

// m44LinearSigmaMax2 returns σ_max² of the 3x3 linear block of a 4x4 matrix,
// i.e. the largest eigenvalue of MᵀM. Uses the closed-form eigenvalue formula
// for a 3x3 symmetric matrix (Smith 1961) — avoids iterative SVD.
func m44LinearSigmaMax2(m *M44) float64 {
	a, b, c := m[0], m[1], m[2]
	d, e, f := m[4], m[5], m[6]
	g, h, i := m[8], m[9], m[10]
	// A = MᵀM (symmetric)
	a00 := a*a + d*d + g*g
	a11 := b*b + e*e + h*h
	a22 := c*c + f*f + i*i
	a01 := a*b + d*e + g*h
	a02 := a*c + d*f + g*i
	a12 := b*c + e*f + h*i
	p1 := a01*a01 + a02*a02 + a12*a12
	if p1 == 0 {
		// diagonal
		return math.Max(math.Max(a00, a11), a22)
	}
	q := (a00 + a11 + a22) / 3
	d0, d1, d2 := a00-q, a11-q, a22-q
	p2 := d0*d0 + d1*d1 + d2*d2 + 2*p1
	p := math.Sqrt(p2 / 6)
	// det((A - qI)/p) / 2
	b00, b11, b22 := d0/p, d1/p, d2/p
	b01, b02, b12 := a01/p, a02/p, a12/p
	detB := b00*(b11*b22-b12*b12) - b01*(b01*b22-b12*b02) + b02*(b01*b12-b11*b02)
	r := detB / 2
	if r < -1 {
		r = -1
	} else if r > 1 {
		r = 1
	}
	phi := math.Acos(r) / 3
	return q + 2*p*math.Cos(phi)
}

// Evaluate returns the minimum distance to a transformed SDF3.
// Distance is *not* preserved with scaling.
func (s *TransformSDF3) Evaluate(p v3.Vec) float64 {
	if s.isTranslate {
		return s.sdf.Evaluate(v3.Vec{X: p.X + s.translate.X, Y: p.Y + s.translate.Y, Z: p.Z + s.translate.Z})
	}
	return s.sdf.Evaluate(s.inverse.MulPosition(p)) * s.invStretch
}

// BoundingBox returns the bounding box of a transformed SDF3.
func (s *TransformSDF3) BoundingBox() Box3 {
	return s.bb
}

//-----------------------------------------------------------------------------
// Uniform XYZ Scaling of SDF3s (we can work out the distance)

// ScaleUniformSDF3 is an SDF3 scaled uniformly in XYZ directions.
type ScaleUniformSDF3 struct {
	sdf     SDF3
	k, invK float64
	bb      Box3
}

// ScaleUniform3D uniformly scales an SDF3 on all axes.
func ScaleUniform3D(sdf SDF3, k float64) SDF3 {
	m := Scale3d(v3.Vec{k, k, k})
	return &ScaleUniformSDF3{
		sdf:  sdf,
		k:    k,
		invK: 1.0 / k,
		bb:   m.MulBox(sdf.BoundingBox()),
	}
}

// Evaluate returns the minimum distance to a uniformly scaled SDF3.
// The distance is correct with scaling.
func (s *ScaleUniformSDF3) Evaluate(p v3.Vec) float64 {
	q := p.MulScalar(s.invK)
	return s.sdf.Evaluate(q) * s.k
}

// BoundingBox returns the bounding box of a uniformly scaled SDF3.
func (s *ScaleUniformSDF3) BoundingBox() Box3 {
	return s.bb
}

//-----------------------------------------------------------------------------

// UnionSDF3 is a union of SDF3s.
//
// When using the default hard min (math.Min), Evaluate skips children whose
// bounding box is farther from the query point than the current best distance.
// This avoids most child evaluations for unions of spatially separated objects,
// which is the common case in CAD (e.g. arrays of holes, mounting posts).
//
// Profiling showed UnionSDF3.Evaluate consuming ~48% cumulative CPU time on
// typical models because every point was evaluated against all children,
// including those millimeters away. Bbox pruning reduces this to only
// evaluating nearby children.
type UnionSDF3 struct {
	sdf     []SDF3
	min     MinFunc
	bb      Box3
	boxes   []Box3 // per-child bounding boxes, cached at construction time
	blended bool   // true after SetMin — disables pruning (see below)
}

// Union3D returns the union of multiple SDF3 objects.
func Union3D(sdf ...SDF3) SDF3 {
	if len(sdf) == 0 {
		return nil
	}
	s := UnionSDF3{}
	// strip out any nils
	s.sdf = make([]SDF3, 0, len(sdf))
	for _, x := range sdf {
		if x != nil {
			s.sdf = append(s.sdf, x)
		}
	}
	if len(s.sdf) == 0 {
		return nil
	}
	if len(s.sdf) == 1 {
		// only one sdf - not really a union
		return s.sdf[0]
	}
	// work out the bounding box, cache per-child boxes for pruning
	s.boxes = make([]Box3, len(s.sdf))
	bb := s.sdf[0].BoundingBox()
	s.boxes[0] = bb
	for i := 1; i < len(s.sdf); i++ {
		s.boxes[i] = s.sdf[i].BoundingBox()
		bb = bb.Extend(s.boxes[i])
	}
	s.bb = bb
	s.min = math.Min
	return &s
}

// Evaluate returns the minimum distance to an SDF3 union.
func (s *UnionSDF3) Evaluate(p v3.Vec) float64 {
	sdfs := s.sdf
	d := sdfs[0].Evaluate(p)
	if s.blended {
		// Blended/smooth min functions can produce distances smaller than
		// either input, so we can't safely skip any child — evaluate all.
		for i := 1; i < len(sdfs); i++ {
			d = s.min(d, sdfs[i].Evaluate(p))
		}
		return d
	}
	// Hard min with bbox pruning. If a child's bounding box minimum distance
	// exceeds the current best, the child's SDF value must also exceed it
	// (SDF values are always >= distance to the bounding box for exterior
	// points). The d*d comparison works for both positive d (outside all
	// children so far) and negative d (inside a child), since d*d = |d|^2.
	boxes := s.boxes[:len(sdfs)] // tell the compiler boxes[i] is in-range
	bound := d * d
	for i := 1; i < len(sdfs); i++ {
		if boxes[i].MinDist2GT(p, bound) {
			continue
		}
		if v := sdfs[i].Evaluate(p); v < d {
			d = v
			bound = d * d
		}
	}
	return d
}

// SetMin sets the minimum function to control blending.
// Bbox pruning is disabled because blended min functions (e.g. smooth union)
// can produce distances smaller than either input, making it unsafe to skip
// children based on bounding box distance alone.
func (s *UnionSDF3) SetMin(min MinFunc) {
	s.min = min
	s.blended = true
}

// BoundingBox returns the bounding box of an SDF3 union.
func (s *UnionSDF3) BoundingBox() Box3 {
	return s.bb
}

//-----------------------------------------------------------------------------

// DifferenceSDF3 is the difference of two SDF3s, s0 - s1.
//
// When using the default hard max (math.Max), Evaluate skips the s1
// evaluation when p is far enough from s1's bounding box that -s1 can't
// exceed the current s0 distance. In CAD workflow s1 is typically a
// small hole or cutout, so this avoids most child evaluations.
type DifferenceSDF3 struct {
	s0      SDF3
	s1      SDF3
	max     MaxFunc
	bb      Box3
	s1bb    Box3 // cached bounding box of s1 for pruning
	blended bool // true after SetMax — disables pruning
}

// Difference3D returns the difference of two SDF3s, s0 - s1.
func Difference3D(s0, s1 SDF3) SDF3 {
	if s1 == nil {
		return s0
	}
	if s0 == nil {
		return nil
	}
	s := DifferenceSDF3{}
	s.s0 = s0
	s.s1 = s1
	s.max = math.Max
	s.bb = s0.BoundingBox()
	s.s1bb = s1.BoundingBox()
	return &s
}

// Evaluate returns the minimum distance to the SDF3 difference.
func (s *DifferenceSDF3) Evaluate(p v3.Vec) float64 {
	d0 := s.s0.Evaluate(p)
	if s.blended {
		return s.max(d0, -s.s1.Evaluate(p))
	}
	// Hard max pruning. result = max(d0, -d1).
	// We know d1 >= sqrt(MinDist2(s1.bb, p)) for p outside s1's bbox, so
	// -d1 <= -sqrt(MinDist2). If MinDist2 > d0*d0, then d1 > |d0|, so
	// -d1 < d0 whether d0 is positive or negative — result = d0.
	if s.s1bb.MinDist2GT(p, d0*d0) {
		return d0
	}
	d1 := -s.s1.Evaluate(p)
	if d1 > d0 {
		return d1
	}
	return d0
}

// SetMax sets the maximum function to control blending.
// Bbox pruning is disabled because blended max functions can produce
// distances larger than either input.
func (s *DifferenceSDF3) SetMax(max MaxFunc) {
	s.max = max
	s.blended = true
}

// BoundingBox returns the bounding box of the SDF3 difference.
func (s *DifferenceSDF3) BoundingBox() Box3 {
	return s.bb
}

//-----------------------------------------------------------------------------

// ElongateSDF3 is the elongation of an SDF3.
type ElongateSDF3 struct {
	sdf    SDF3   // the sdf being elongated
	hp, hn v3.Vec // positive/negative elongation vector
	bb     Box3   // bounding box
}

// Elongate3D returns the elongation of an SDF3.
func Elongate3D(sdf SDF3, h v3.Vec) SDF3 {
	h = h.Abs()
	s := ElongateSDF3{
		sdf: sdf,
		hp:  h.MulScalar(0.5),
		hn:  h.MulScalar(-0.5),
	}
	// bounding box
	bb := sdf.BoundingBox()
	bb0 := bb.Translate(s.hp)
	bb1 := bb.Translate(s.hn)
	s.bb = bb0.Extend(bb1)
	return &s
}

// Evaluate returns the minimum distance to a elongated SDF2.
func (s *ElongateSDF3) Evaluate(p v3.Vec) float64 {
	q := p.Sub(p.Clamp(s.hn, s.hp))
	return s.sdf.Evaluate(q)
}

// BoundingBox returns the bounding box of an elongated SDF3.
func (s *ElongateSDF3) BoundingBox() Box3 {
	return s.bb
}

//-----------------------------------------------------------------------------

// IntersectionSDF3 is the intersection of two SDF3s.
type IntersectionSDF3 struct {
	s0      SDF3
	s1      SDF3
	max     MaxFunc
	blended bool
	bb      Box3
}

// Intersect3D returns the intersection of two SDF3s.
func Intersect3D(s0, s1 SDF3) SDF3 {
	if s0 == nil || s1 == nil {
		return nil
	}
	s := IntersectionSDF3{}
	s.s0 = s0
	s.s1 = s1
	s.max = math.Max
	// TODO fix bounding box
	s.bb = s0.BoundingBox()
	return &s
}

// Evaluate returns the minimum distance to the SDF3 intersection.
func (s *IntersectionSDF3) Evaluate(p v3.Vec) float64 {
	a := s.s0.Evaluate(p)
	b := s.s1.Evaluate(p)
	if s.blended {
		return s.max(a, b)
	}
	if a > b {
		return a
	}
	return b
}

// SetMax sets the maximum function to control blending.
func (s *IntersectionSDF3) SetMax(max MaxFunc) {
	s.max = max
	s.blended = true
}

// BoundingBox returns the bounding box of an SDF3 intersection.
func (s *IntersectionSDF3) BoundingBox() Box3 {
	return s.bb
}

//-----------------------------------------------------------------------------

// CutSDF3 makes a planar cut through an SDF3.
type CutSDF3 struct {
	sdf SDF3
	a   v3.Vec // point on plane
	n   v3.Vec // normal to plane
	bb  Box3   // bounding box
}

// Cut3D cuts an SDF3 along a plane passing through a with normal n.
// The SDF3 on the same side as the normal remains.
func Cut3D(sdf SDF3, a, n v3.Vec) SDF3 {
	s := CutSDF3{}
	s.sdf = sdf
	s.a = a
	s.n = n.Normalize().Neg()
	// TODO - cut the bounding box
	s.bb = sdf.BoundingBox()
	return &s
}

// Evaluate returns the minimum distance to the cut SDF3.
func (s *CutSDF3) Evaluate(p v3.Vec) float64 {
	a := p.Sub(s.a).Dot(s.n)
	b := s.sdf.Evaluate(p)
	if a > b {
		return a
	}
	return b
}

// BoundingBox returns the bounding box of the cut SDF3.
func (s *CutSDF3) BoundingBox() Box3 {
	return s.bb
}

//-----------------------------------------------------------------------------

// ArraySDF3 stores an XYZ array of a given SDF3
type ArraySDF3 struct {
	sdf     SDF3
	num     v3i.Vec
	step    v3.Vec
	min     MinFunc
	blended bool
	bb      Box3
}

// Array3D returns an XYZ array of a given SDF3
func Array3D(sdf SDF3, num v3i.Vec, step v3.Vec) SDF3 {
	// check the number of steps
	if num.X <= 0 || num.Y <= 0 || num.Z <= 0 {
		return nil
	}
	s := ArraySDF3{}
	s.sdf = sdf
	s.num = num
	s.step = step
	s.min = math.Min
	// work out the bounding box
	bb0 := sdf.BoundingBox()
	bb1 := bb0.Translate(step.Mul(conv.V3iToV3(num.SubScalar(1))))
	s.bb = bb0.Extend(bb1)
	return &s
}

// SetMin sets the minimum function to control blending.
func (s *ArraySDF3) SetMin(min MinFunc) {
	s.min = min
	s.blended = true
}

// Evaluate returns the minimum distance to an XYZ SDF3 array.
func (s *ArraySDF3) Evaluate(p v3.Vec) float64 {
	d := math.MaxFloat64
	if s.blended {
		for j := 0; j < s.num.X; j++ {
			for k := 0; k < s.num.Y; k++ {
				for l := 0; l < s.num.Z; l++ {
					x := p.Sub(v3.Vec{float64(j) * s.step.X, float64(k) * s.step.Y, float64(l) * s.step.Z})
					d = s.min(d, s.sdf.Evaluate(x))
				}
			}
		}
		return d
	}
	for j := 0; j < s.num.X; j++ {
		for k := 0; k < s.num.Y; k++ {
			for l := 0; l < s.num.Z; l++ {
				x := p.Sub(v3.Vec{float64(j) * s.step.X, float64(k) * s.step.Y, float64(l) * s.step.Z})
				if v := s.sdf.Evaluate(x); v < d {
					d = v
				}
			}
		}
	}
	return d
}

// BoundingBox returns the bounding box of an XYZ SDF3 array.
func (s *ArraySDF3) BoundingBox() Box3 {
	return s.bb
}

//-----------------------------------------------------------------------------

// RotateUnionSDF3 creates a union of SDF3s rotated about the z-axis.
type RotateUnionSDF3 struct {
	sdf     SDF3
	num     int
	mats    []M44 // mats[i] = step^i (inverse), precomputed
	childBB Box3  // sdf.BoundingBox(); tested per copy in local space
	min     MinFunc
	blended bool
	bb      Box3
}

// RotateUnion3D creates a union of SDF3s rotated about the z-axis.
func RotateUnion3D(sdf SDF3, num int, step M44) SDF3 {
	// check the number of steps
	if num <= 0 {
		return nil
	}
	s := RotateUnionSDF3{}
	s.sdf = sdf
	s.num = num
	s.min = math.Min
	s.childBB = sdf.BoundingBox()
	// precompute inverse rotation powers so Evaluate skips per-call Mul
	invStep := step.Inverse()
	s.mats = make([]M44, num)
	s.mats[0] = Identity3d()
	for i := 1; i < num; i++ {
		s.mats[i] = s.mats[i-1].Mul(invStep)
	}
	// work out the bounding box
	v := s.childBB.Vertices()
	bbMin := v[0]
	bbMax := v[0]
	for i := 0; i < s.num; i++ {
		bbMin = bbMin.Min(v.Min())
		bbMax = bbMax.Max(v.Max())
		mulVertices3(v, step)
	}
	s.bb = Box3{bbMin, bbMax}
	return &s
}

// Evaluate returns the minimum distance to a rotate/union object.
func (s *RotateUnionSDF3) Evaluate(p v3.Vec) float64 {
	d := math.MaxFloat64
	mats := s.mats
	if s.blended {
		for i := range mats {
			x := mats[i].MulPosition(p)
			d = s.min(d, s.sdf.Evaluate(x))
		}
		return d
	}
	// Hard-min pruning: x sits in the child's LOCAL frame (mats[i] is the
	// inverse of step^i), so every copy shares the same childBB. Skip a copy
	// when x is too far from childBB for its SDF value to improve d.
	bound := d * d
	bb := s.childBB
	for i := range mats {
		x := mats[i].MulPosition(p)
		if bb.MinDist2GT(x, bound) {
			continue
		}
		if v := s.sdf.Evaluate(x); v < d {
			d = v
			bound = d * d
		}
	}
	return d
}

// SetMin sets the minimum function to control blending.
func (s *RotateUnionSDF3) SetMin(min MinFunc) {
	s.min = min
	s.blended = true
}

// BoundingBox returns the bounding box of a rotate/union object.
func (s *RotateUnionSDF3) BoundingBox() Box3 {
	return s.bb
}

//-----------------------------------------------------------------------------

// RotateCopySDF3 rotates and creates N copies of an SDF3 about the z-axis.
type RotateCopySDF3 struct {
	sdf   SDF3
	theta float64
	bb    Box3
}

// RotateCopy3D rotates and creates N copies of an SDF3 about the z-axis.
func RotateCopy3D(
	sdf SDF3, // SDF3 to rotate and copy
	num int, // number of copies
) SDF3 {
	// check the number of steps
	if num <= 0 {
		return nil
	}
	s := RotateCopySDF3{}
	s.sdf = sdf
	s.theta = Tau / float64(num)
	// work out the bounding box
	bb := sdf.BoundingBox()
	zmax := bb.Max.Z
	zmin := bb.Min.Z
	rmax := 0.0
	// find the bounding box vertex with the greatest distance from the z-axis
	// TODO - revisit - should go by real vertices
	for _, v := range bb.Vertices() {
		l := v2.Vec{v.X, v.Y}.Length()
		if l > rmax {
			rmax = l
		}
	}
	s.bb = Box3{v3.Vec{-rmax, -rmax, zmin}, v3.Vec{rmax, rmax, zmax}}
	return &s
}

// Evaluate returns the minimum distance to a rotate/copy SDF3.
func (s *RotateCopySDF3) Evaluate(p v3.Vec) float64 {
	// Map p to a point in the first copy sector.
	p2d := v2.Vec{p.X, p.Y}
	p2d = conv.P2ToV2(p2.Vec{p2d.Length(), SawTooth(math.Atan2(p2d.Y, p2d.X), s.theta)})
	return s.sdf.Evaluate(v3.Vec{p2d.X, p2d.Y, p.Z})
}

// BoundingBox returns the bounding box of a rotate/copy SDF3.
func (s *RotateCopySDF3) BoundingBox() Box3 {
	return s.bb
}

//-----------------------------------------------------------------------------

/* WIP

// Connector3 defines a 3d connection point.
type Connector3 struct {
	Name     string
	Position v3.Vec
	Vector   v3.Vec
	Angle    float64
}

// ConnectedSDF3 is an SDF3 with connection points defined.
type ConnectedSDF3 struct {
	sdf        SDF3
	connectors []Connector3
}

// AddConnector adds connection points to an SDF3.
func AddConnector(sdf SDF3, connectors ...Connector3) SDF3 {
	// is the sdf already connected?
	if s, ok := sdf.(*ConnectedSDF3); ok {
		// append connection points
		s.connectors = append(s.connectors, connectors...)
		return s
	}
	// return a new connected sdf
	return &ConnectedSDF3{
		sdf:        sdf,
		connectors: connectors,
	}
}

// Evaluate returns the minimum distance to a connected SDF3.
func (s *ConnectedSDF3) Evaluate(p v3.Vec) float64 {
	return s.sdf.Evaluate(p)
}

// BoundingBox returns the bounding box of a connected SDF3.
func (s *ConnectedSDF3) BoundingBox() Box3 {
	return s.sdf.BoundingBox()
}

*/

//-----------------------------------------------------------------------------

// OffsetSDF3 offsets the distance function of an existing SDF3.
type OffsetSDF3 struct {
	sdf    SDF3    // the underlying SDF
	offset float64 // the distance the SDF is offset by
	bb     Box3    // bounding box
}

// Offset3D returns an SDF3 that offsets the distance function of another SDF3.
func Offset3D(sdf SDF3, offset float64) SDF3 {
	s := OffsetSDF3{
		sdf:    sdf,
		offset: offset,
	}
	// bounding box
	bb := sdf.BoundingBox()
	s.bb = NewBox3(bb.Center(), bb.Size().AddScalar(2*offset))
	return &s
}

// Evaluate returns the minimum distance to an offset SDF3.
func (s *OffsetSDF3) Evaluate(p v3.Vec) float64 {
	return s.sdf.Evaluate(p) - s.offset
}

// BoundingBox returns the bounding box of an offset SDF3.
func (s *OffsetSDF3) BoundingBox() Box3 {
	return s.bb
}

//-----------------------------------------------------------------------------

// ShellSDF3 shells the surface of an existing SDF3.
type ShellSDF3 struct {
	sdf   SDF3    // parent sdf3
	delta float64 // half shell thickness
	bb    Box3    // bounding box
}

// Shell3D returns an SDF3 that shells the surface of an existing SDF3.
func Shell3D(sdf SDF3, thickness float64) (SDF3, error) {
	if thickness <= 0 {
		return nil, ErrMsg("thickness <= 0")
	}
	return &ShellSDF3{
		sdf:   sdf,
		delta: 0.5 * thickness,
		bb:    sdf.BoundingBox().Enlarge(v3.Vec{thickness, thickness, thickness}),
	}, nil
}

// Evaluate returns the minimum distance to a shelled SDF3.
func (s *ShellSDF3) Evaluate(p v3.Vec) float64 {
	return math.Abs(s.sdf.Evaluate(p)) - s.delta
}

// BoundingBox returns the bounding box of a shelled SDF3.
func (s *ShellSDF3) BoundingBox() Box3 {
	return s.bb
}

//-----------------------------------------------------------------------------

// LineOf3D returns a union of 3D objects positioned along a line from p0 to p1.
func LineOf3D(s SDF3, p0, p1 v3.Vec, pattern string) SDF3 {
	var objects []SDF3
	if pattern != "" {
		x := p0
		dx := p1.Sub(p0).DivScalar(float64(len(pattern)))
		for _, c := range pattern {
			if c == 'x' {
				objects = append(objects, Transform3D(s, Translate3d(x)))
			}
			x = x.Add(dx)
		}
	}
	return Union3D(objects...)
}

//-----------------------------------------------------------------------------

// Multi3D creates a union of an SDF3 at translated positions.
func Multi3D(s SDF3, positions v3.VecSet) SDF3 {
	if (s == nil) || (len(positions) == 0) {
		return nil
	}
	objects := make([]SDF3, len(positions))
	for i, p := range positions {
		objects[i] = Transform3D(s, Translate3d(p))
	}
	return Union3D(objects...)
}

//-----------------------------------------------------------------------------

// Orient3D creates a union of an SDF3 at oriented directions.
func Orient3D(s SDF3, base v3.Vec, directions v3.VecSet) SDF3 {
	if (s == nil) || (len(directions) == 0) {
		return nil
	}
	objects := make([]SDF3, len(directions))
	for i, d := range directions {
		objects[i] = Transform3D(s, RotateToVector(base, d))
	}
	return Union3D(objects...)
}

//-----------------------------------------------------------------------------
