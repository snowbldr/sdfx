//-----------------------------------------------------------------------------
/*

Cut3D / Cut2D testing — bounding-box tightening and side semantics.

*/
//-----------------------------------------------------------------------------

package sdf

import (
	"math"
	"math/rand"
	"testing"

	v2 "github.com/snowbldr/sdfx/vec/v2"
	v3 "github.com/snowbldr/sdfx/vec/v3"
)

//-----------------------------------------------------------------------------
// helpers

func boxesApproxEqual(a, b Box3, tol float64) bool {
	return a.Min.Equals(b.Min, tol) && a.Max.Equals(b.Max, tol)
}

func boxes2ApproxEqual(a, b Box2, tol float64) bool {
	return a.Min.Equals(b.Min, tol) && a.Max.Equals(b.Max, tol)
}

//-----------------------------------------------------------------------------
// Box3.Clip

func Test_Box3_Clip(t *testing.T) {
	box := Box3{Min: v3.Vec{X: -1, Y: -1, Z: -1}, Max: v3.Vec{X: 1, Y: 1, Z: 1}}

	tests := []struct {
		name        string
		point       v3.Vec
		normal      v3.Vec
		expectedMin v3.Vec
		expectedMax v3.Vec
	}{
		{
			name:        "+Z half through origin keeps top",
			point:       v3.Vec{},
			normal:      v3.Vec{X: 0, Y: 0, Z: 1},
			expectedMin: v3.Vec{X: -1, Y: -1, Z: 0},
			expectedMax: v3.Vec{X: 1, Y: 1, Z: 1},
		},
		{
			name:        "-Z half through origin keeps bottom",
			point:       v3.Vec{},
			normal:      v3.Vec{X: 0, Y: 0, Z: -1},
			expectedMin: v3.Vec{X: -1, Y: -1, Z: -1},
			expectedMax: v3.Vec{X: 1, Y: 1, Z: 0},
		},
		{
			name:        "+X half through origin keeps positive X",
			point:       v3.Vec{},
			normal:      v3.Vec{X: 1, Y: 0, Z: 0},
			expectedMin: v3.Vec{X: 0, Y: -1, Z: -1},
			expectedMax: v3.Vec{X: 1, Y: 1, Z: 1},
		},
		{
			name:        "+Y half through origin keeps positive Y",
			point:       v3.Vec{},
			normal:      v3.Vec{X: 0, Y: 1, Z: 0},
			expectedMin: v3.Vec{X: -1, Y: 0, Z: -1},
			expectedMax: v3.Vec{X: 1, Y: 1, Z: 1},
		},
		{
			name:        "+Z plane offset 0.5 keeps quarter",
			point:       v3.Vec{X: 0, Y: 0, Z: 0.5},
			normal:      v3.Vec{X: 0, Y: 0, Z: 1},
			expectedMin: v3.Vec{X: -1, Y: -1, Z: 0.5},
			expectedMax: v3.Vec{X: 1, Y: 1, Z: 1},
		},
		{
			name:        "+Z plane fully above box leaves empty",
			point:       v3.Vec{X: 0, Y: 0, Z: 5},
			normal:      v3.Vec{X: 0, Y: 0, Z: 1},
			expectedMin: v3.Vec{X: 0, Y: 0, Z: 5},
			expectedMax: v3.Vec{X: 0, Y: 0, Z: 5},
		},
		{
			name:        "+Z plane fully below box leaves whole box",
			point:       v3.Vec{X: 0, Y: 0, Z: -5},
			normal:      v3.Vec{X: 0, Y: 0, Z: 1},
			expectedMin: v3.Vec{X: -1, Y: -1, Z: -1},
			expectedMax: v3.Vec{X: 1, Y: 1, Z: 1},
		},
		{
			// keeping the corner at (1,1,1) — the slice plane at origin with
			// normal (1,1,1) carves a triangular pyramid; its AABB still
			// reaches every face of the cube the corner touches.
			name:        "diagonal cut keeps cube AABB",
			point:       v3.Vec{},
			normal:      v3.Vec{X: 1, Y: 1, Z: 1}.Normalize(),
			expectedMin: v3.Vec{X: -1, Y: -1, Z: -1},
			expectedMax: v3.Vec{X: 1, Y: 1, Z: 1},
		},
	}

	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			got := box.Clip(tc.point, tc.normal)
			want := Box3{Min: tc.expectedMin, Max: tc.expectedMax}
			if !boxesApproxEqual(got, want, 1e-9) {
				t.Errorf("Clip(%v, %v):\n  got  %+v\n  want %+v", tc.point, tc.normal, got, want)
			}
		})
	}
}

//-----------------------------------------------------------------------------
// Box2.Clip

func Test_Box2_Clip(t *testing.T) {
	box := Box2{Min: v2.Vec{X: -1, Y: -1}, Max: v2.Vec{X: 1, Y: 1}}

	tests := []struct {
		name        string
		point       v2.Vec
		normal      v2.Vec
		expectedMin v2.Vec
		expectedMax v2.Vec
	}{
		{
			name:        "+Y keeps top",
			point:       v2.Vec{},
			normal:      v2.Vec{X: 0, Y: 1},
			expectedMin: v2.Vec{X: -1, Y: 0},
			expectedMax: v2.Vec{X: 1, Y: 1},
		},
		{
			name:        "-Y keeps bottom",
			point:       v2.Vec{},
			normal:      v2.Vec{X: 0, Y: -1},
			expectedMin: v2.Vec{X: -1, Y: -1},
			expectedMax: v2.Vec{X: 1, Y: 0},
		},
		{
			name:        "+X keeps right",
			point:       v2.Vec{},
			normal:      v2.Vec{X: 1, Y: 0},
			expectedMin: v2.Vec{X: 0, Y: -1},
			expectedMax: v2.Vec{X: 1, Y: 1},
		},
		{
			name:        "diagonal +XY keeps top-right",
			point:       v2.Vec{},
			normal:      v2.Vec{X: 1, Y: 1}.Normalize(),
			expectedMin: v2.Vec{X: -1, Y: -1},
			expectedMax: v2.Vec{X: 1, Y: 1},
		},
		{
			name:        "+Y above box → empty",
			point:       v2.Vec{X: 0, Y: 5},
			normal:      v2.Vec{X: 0, Y: 1},
			expectedMin: v2.Vec{X: 0, Y: 5},
			expectedMax: v2.Vec{X: 0, Y: 5},
		},
	}

	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			got := box.Clip(tc.point, tc.normal)
			want := Box2{Min: tc.expectedMin, Max: tc.expectedMax}
			if !boxes2ApproxEqual(got, want, 1e-9) {
				t.Errorf("Clip(%v, %v):\n  got  %+v\n  want %+v", tc.point, tc.normal, got, want)
			}
		})
	}
}

//-----------------------------------------------------------------------------
// Cut3D — bbox tightening, side semantics, volume ratio

func Test_Cut3D_BBoxTightens(t *testing.T) {
	cube, err := Box3D(v3.Vec{X: 2, Y: 2, Z: 2}, 0)
	if err != nil {
		t.Fatal(err)
	}

	// Cut at z=0 with +Z normal — kept half should have z ∈ [0,1].
	upper := Cut3D(cube, v3.Vec{}, v3.Vec{X: 0, Y: 0, Z: 1})
	bb := upper.BoundingBox()
	want := Box3{Min: v3.Vec{X: -1, Y: -1, Z: 0}, Max: v3.Vec{X: 1, Y: 1, Z: 1}}
	if !boxesApproxEqual(bb, want, 1e-9) {
		t.Errorf("upper-half bbox: got %+v, want %+v", bb, want)
	}

	// Cut with -Z normal — kept half z ∈ [-1,0].
	lower := Cut3D(cube, v3.Vec{}, v3.Vec{X: 0, Y: 0, Z: -1})
	bb = lower.BoundingBox()
	want = Box3{Min: v3.Vec{X: -1, Y: -1, Z: -1}, Max: v3.Vec{X: 1, Y: 1, Z: 0}}
	if !boxesApproxEqual(bb, want, 1e-9) {
		t.Errorf("lower-half bbox: got %+v, want %+v", bb, want)
	}
}

func Test_Cut3D_KeepsCorrectSide(t *testing.T) {
	cube, err := Box3D(v3.Vec{X: 2, Y: 2, Z: 2}, 0)
	if err != nil {
		t.Fatal(err)
	}

	// Cut at z=0 with +Z normal — z>0 inside, z<0 outside.
	upper := Cut3D(cube, v3.Vec{}, v3.Vec{X: 0, Y: 0, Z: 1})

	// Point well inside upper half: should be inside (eval < 0).
	if d := upper.Evaluate(v3.Vec{X: 0, Y: 0, Z: 0.5}); d >= 0 {
		t.Errorf("expected inside (d<0) at +Z point, got d=%v", d)
	}
	// Point well inside lower half: should be outside (eval > 0).
	if d := upper.Evaluate(v3.Vec{X: 0, Y: 0, Z: -0.5}); d <= 0 {
		t.Errorf("expected outside (d>0) at -Z point, got d=%v", d)
	}

	// Now flip — keep -Z half.
	lower := Cut3D(cube, v3.Vec{}, v3.Vec{X: 0, Y: 0, Z: -1})
	if d := lower.Evaluate(v3.Vec{X: 0, Y: 0, Z: -0.5}); d >= 0 {
		t.Errorf("expected inside on -Z half, got d=%v", d)
	}
	if d := lower.Evaluate(v3.Vec{X: 0, Y: 0, Z: 0.5}); d <= 0 {
		t.Errorf("expected outside on +Z half, got d=%v", d)
	}
}

func Test_Cut3D_VolumeRatio(t *testing.T) {
	// Cube 2x2x2, cut at z=0 with +Z normal. Kept volume is exactly half.
	cube, err := Box3D(v3.Vec{X: 2, Y: 2, Z: 2}, 0)
	if err != nil {
		t.Fatal(err)
	}
	upper := Cut3D(cube, v3.Vec{}, v3.Vec{X: 0, Y: 0, Z: 1})

	rng := rand.New(rand.NewSource(42))
	const N = 200000
	bb := cube.BoundingBox()
	bbSize := bb.Size()

	cubeIn, upperIn := 0, 0
	for i := 0; i < N; i++ {
		p := v3.Vec{
			X: bb.Min.X + rng.Float64()*bbSize.X,
			Y: bb.Min.Y + rng.Float64()*bbSize.Y,
			Z: bb.Min.Z + rng.Float64()*bbSize.Z,
		}
		if cube.Evaluate(p) <= 0 {
			cubeIn++
		}
		if upper.Evaluate(p) <= 0 {
			upperIn++
		}
	}
	ratio := float64(upperIn) / float64(cubeIn)
	if math.Abs(ratio-0.5) > 0.01 {
		t.Errorf("expected volume ratio ~0.5, got %v (cubeIn=%d upperIn=%d)", ratio, cubeIn, upperIn)
	}
}

func Test_Cut3D_ArbitraryPlane(t *testing.T) {
	// Cube cut at offset 0.5 on +Z normal — kept volume = 0.25 of original.
	cube, err := Box3D(v3.Vec{X: 2, Y: 2, Z: 2}, 0)
	if err != nil {
		t.Fatal(err)
	}
	cut := Cut3D(cube, v3.Vec{X: 0, Y: 0, Z: 0.5}, v3.Vec{X: 0, Y: 0, Z: 1})

	bb := cut.BoundingBox()
	want := Box3{Min: v3.Vec{X: -1, Y: -1, Z: 0.5}, Max: v3.Vec{X: 1, Y: 1, Z: 1}}
	if !boxesApproxEqual(bb, want, 1e-9) {
		t.Errorf("offset-cut bbox: got %+v, want %+v", bb, want)
	}

	rng := rand.New(rand.NewSource(7))
	const N = 200000
	cubeBB := cube.BoundingBox()
	size := cubeBB.Size()
	cubeIn, cutIn := 0, 0
	for i := 0; i < N; i++ {
		p := v3.Vec{
			X: cubeBB.Min.X + rng.Float64()*size.X,
			Y: cubeBB.Min.Y + rng.Float64()*size.Y,
			Z: cubeBB.Min.Z + rng.Float64()*size.Z,
		}
		if cube.Evaluate(p) <= 0 {
			cubeIn++
		}
		if cut.Evaluate(p) <= 0 {
			cutIn++
		}
	}
	ratio := float64(cutIn) / float64(cubeIn)
	if math.Abs(ratio-0.25) > 0.01 {
		t.Errorf("expected volume ratio ~0.25, got %v", ratio)
	}
}

func Test_Cut3D_TwoOpposingCutsCoverWhole(t *testing.T) {
	// A solid cut into two halves should have volumes that sum to the whole.
	cube, err := Box3D(v3.Vec{X: 2, Y: 2, Z: 2}, 0)
	if err != nil {
		t.Fatal(err)
	}
	upper := Cut3D(cube, v3.Vec{}, v3.Vec{X: 0, Y: 0, Z: 1})
	lower := Cut3D(cube, v3.Vec{}, v3.Vec{X: 0, Y: 0, Z: -1})

	rng := rand.New(rand.NewSource(123))
	const N = 200000
	bb := cube.BoundingBox()
	size := bb.Size()
	cubeIn, halvesIn := 0, 0
	for i := 0; i < N; i++ {
		p := v3.Vec{
			X: bb.Min.X + rng.Float64()*size.X,
			Y: bb.Min.Y + rng.Float64()*size.Y,
			Z: bb.Min.Z + rng.Float64()*size.Z,
		}
		c := cube.Evaluate(p) <= 0
		u := upper.Evaluate(p) <= 0
		l := lower.Evaluate(p) <= 0
		if c {
			cubeIn++
		}
		if u || l {
			halvesIn++
		}
		if c && !(u || l) {
			t.Fatalf("cube point %v not covered by either half", p)
		}
	}
	if cubeIn != halvesIn {
		t.Errorf("union of two halves should equal cube: cubeIn=%d halvesIn=%d", cubeIn, halvesIn)
	}
}

//-----------------------------------------------------------------------------
// Cut2D — bbox tightening, side semantics, area ratio

func Test_Cut2D_BBoxTightens(t *testing.T) {
	square := Box2D(v2.Vec{X: 2, Y: 2}, 0)
	// Direction +X means the line runs along +X. Right side of that
	// directed line is -Y. So kept half should be y ∈ [-1, 0].
	right := Cut2D(square, v2.Vec{}, v2.Vec{X: 1, Y: 0})
	bb := right.BoundingBox()
	want := Box2{Min: v2.Vec{X: -1, Y: -1}, Max: v2.Vec{X: 1, Y: 0}}
	if !boxes2ApproxEqual(bb, want, 1e-9) {
		t.Errorf("right-of-+X bbox: got %+v, want %+v", bb, want)
	}

	// Direction -X — right side is now +Y.
	other := Cut2D(square, v2.Vec{}, v2.Vec{X: -1, Y: 0})
	bb = other.BoundingBox()
	want = Box2{Min: v2.Vec{X: -1, Y: 0}, Max: v2.Vec{X: 1, Y: 1}}
	if !boxes2ApproxEqual(bb, want, 1e-9) {
		t.Errorf("right-of--X bbox: got %+v, want %+v", bb, want)
	}
}

func Test_Cut2D_KeepsRightSide(t *testing.T) {
	square := Box2D(v2.Vec{X: 2, Y: 2}, 0)
	// "Right of +X line through origin" = -Y half.
	right := Cut2D(square, v2.Vec{}, v2.Vec{X: 1, Y: 0})

	if d := right.Evaluate(v2.Vec{X: 0, Y: -0.5}); d >= 0 {
		t.Errorf("expected inside on right (-Y) side, got d=%v", d)
	}
	if d := right.Evaluate(v2.Vec{X: 0, Y: 0.5}); d <= 0 {
		t.Errorf("expected outside on left (+Y) side, got d=%v", d)
	}
}

func Test_Cut2D_AreaRatio(t *testing.T) {
	square := Box2D(v2.Vec{X: 2, Y: 2}, 0)
	right := Cut2D(square, v2.Vec{}, v2.Vec{X: 1, Y: 0})

	rng := rand.New(rand.NewSource(99))
	const N = 200000
	bb := square.BoundingBox()
	size := bb.Size()
	in, cutIn := 0, 0
	for i := 0; i < N; i++ {
		p := v2.Vec{
			X: bb.Min.X + rng.Float64()*size.X,
			Y: bb.Min.Y + rng.Float64()*size.Y,
		}
		if square.Evaluate(p) <= 0 {
			in++
		}
		if right.Evaluate(p) <= 0 {
			cutIn++
		}
	}
	ratio := float64(cutIn) / float64(in)
	if math.Abs(ratio-0.5) > 0.01 {
		t.Errorf("expected area ratio ~0.5, got %v", ratio)
	}
}

func Test_Cut2D_DiagonalLine(t *testing.T) {
	// 2x2 square cut by line through origin with direction (1,1).
	// Right side of that directed line is the half where x > y.
	square := Box2D(v2.Vec{X: 2, Y: 2}, 0)
	right := Cut2D(square, v2.Vec{}, v2.Vec{X: 1, Y: 1})

	// Bbox of {(x,y) ∈ [-1,1]^2 : x >= y} reaches all four extents:
	// corner (1,-1) keeps min Y, corner (1,1) gives max X/Y, corner (-1,-1) min X.
	bb := right.BoundingBox()
	want := Box2{Min: v2.Vec{X: -1, Y: -1}, Max: v2.Vec{X: 1, Y: 1}}
	if !boxes2ApproxEqual(bb, want, 1e-9) {
		t.Errorf("diagonal cut bbox: got %+v, want %+v", bb, want)
	}

	// Side check: (0.5, -0.5) is on the right (x > y), should be inside.
	if d := right.Evaluate(v2.Vec{X: 0.5, Y: -0.5}); d >= 0 {
		t.Errorf("expected inside, got d=%v", d)
	}
	// (-0.5, 0.5) is on the left (x < y), should be outside.
	if d := right.Evaluate(v2.Vec{X: -0.5, Y: 0.5}); d <= 0 {
		t.Errorf("expected outside, got d=%v", d)
	}
}

//-----------------------------------------------------------------------------
