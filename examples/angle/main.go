//-----------------------------------------------------------------------------
/*


 */
//-----------------------------------------------------------------------------

package main

import (
	"log"

	"github.com/snowbldr/sdfx/obj"
	"github.com/snowbldr/sdfx/render"
	"github.com/snowbldr/sdfx/sdf"
)

//-----------------------------------------------------------------------------

// material shrinkage
const shrink = 1.0 / 0.999 // PLA ~0.1%
//const shrink = 1.0/0.995; // ABS ~0.5%

//-----------------------------------------------------------------------------

func main() {

	const l = 1.25 * sdf.MillimetresPerInch
	const t = 0.125 * sdf.MillimetresPerInch
	const r = 0.125 * sdf.MillimetresPerInch

	k := obj.AngleParms{
		X:          obj.AngleLeg{l, t},
		Y:          obj.AngleLeg{l, t},
		RootRadius: r,
		Length:     12 * sdf.MillimetresPerInch,
	}

	s, err := obj.Angle3D(&k)
	if err != nil {
		log.Fatalf("error: %s", err)
	}
	s = sdf.ScaleUniform3D(s, shrink)
	render.ToSTL(s, "angle.stl", render.NewMarchingCubesOctreeParallel(300))
}

//-----------------------------------------------------------------------------
