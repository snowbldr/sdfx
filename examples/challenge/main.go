//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------

package main

import (
	"log"

	"github.com/snowbldr/sdfx/render"
)

//-----------------------------------------------------------------------------

func main() {

	s, err := cc16a()
	if err != nil {
		log.Fatalf("error: %s", err)
	}
	render.ToSTL(s, "cc16a.stl", render.NewMarchingCubesOctreeParallel(200))

	s, err = cc16b()
	if err != nil {
		log.Fatalf("error: %s", err)
	}
	render.ToSTL(s, "cc16b.stl", render.NewMarchingCubesOctreeParallel(200))

	cc18a()

	s, err = cc18b()
	if err != nil {
		log.Fatalf("error: %s", err)
	}
	render.ToSTL(s, "cc18b.stl", render.NewMarchingCubesOctreeParallel(200))

	s, err = cc18c()
	if err != nil {
		log.Fatalf("error: %s", err)
	}
	render.ToSTL(s, "cc18c.stl", render.NewMarchingCubesOctreeParallel(200))
}

//-----------------------------------------------------------------------------
