//-----------------------------------------------------------------------------
/*

Raspberry Pi Parts

*/
//-----------------------------------------------------------------------------

package main

import (
	"log"

	"github.com/snowbldr/sdfx/examples/rpi/stand"
	"github.com/snowbldr/sdfx/render"
	"github.com/snowbldr/sdfx/sdf"
)

//-----------------------------------------------------------------------------

// material shrinkage
const shrink = 1.0 / 0.999 // PLA ~0.1%
//const shrink = 1.0/0.995; // ABS ~0.5%

//-----------------------------------------------------------------------------

func main() {
	s, err := stand.DisplayStand()
	if err != nil {
		log.Fatalf("error: %s", err)
	}
	render.ToSTL(sdf.ScaleUniform3D(s, shrink), "display_stand.stl", render.NewMarchingCubesOctree(300))
}

//-----------------------------------------------------------------------------
