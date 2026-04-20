package render

import (
	"testing"

	"github.com/deadsy/sdfx/sdf"
	v3 "github.com/deadsy/sdfx/vec/v3"
)

type discardW struct{}

func (discardW) Write(in []*sdf.Triangle3) error { return nil }
func (discardW) Close() error                    { return nil }

// buildComplex3D approximates a panel-style SDF composition: extruded
// rounded polygon + array of extruded cylinders + drilled holes. It's a
// stand-in for the picorx panels in benchmarks.
func buildComplex3D(b *testing.B) sdf.SDF3 {
	b.Helper()
	poly := sdf.NewPolygon()
	poly.Add(-40, -20)
	poly.Add(40, -20)
	poly.Add(40, 20)
	poly.Add(-40, 20)
	p2d, err := sdf.Polygon2D(poly.Vertices())
	if err != nil {
		b.Fatal(err)
	}
	body := sdf.Extrude3D(p2d, 8)

	cyl, err := sdf.Cylinder3D(8, 6, 0.5)
	if err != nil {
		b.Fatal(err)
	}
	var cyls []sdf.SDF3
	for x := -30.0; x <= 30; x += 15 {
		for y := -15.0; y <= 15; y += 15 {
			c := sdf.Transform3D(cyl, sdf.Translate3d(v3.Vec{x, y, 0}))
			cyls = append(cyls, c)
		}
	}
	cylU := sdf.Union3D(cyls...)

	hole, err := sdf.Cylinder3D(10, 1.5, 0)
	if err != nil {
		b.Fatal(err)
	}
	holeU := sdf.Multi3D(hole, []v3.Vec{{-35, 15, 0}, {35, 15, 0}, {-35, -15, 0}, {35, -15, 0}})

	s := sdf.Union3D(body, cylU)
	s = sdf.Difference3D(s, holeU)
	return s
}

func BenchmarkParallel200(b *testing.B) {
	s := buildComplex3D(b)
	dw := discardW{}
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		r := NewMarchingCubesOctreeParallel(200)
		r.Render(s, dw)
	}
}

func BenchmarkParallel400(b *testing.B) {
	s := buildComplex3D(b)
	dw := discardW{}
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		r := NewMarchingCubesOctreeParallel(400)
		r.Render(s, dw)
	}
}
