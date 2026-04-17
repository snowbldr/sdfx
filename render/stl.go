//-----------------------------------------------------------------------------
/*

STL Load/Save

*/
//-----------------------------------------------------------------------------

package render

import (
	"bufio"
	"encoding/binary"
	"fmt"
	"math"
	"os"
	"strconv"
	"strings"
	"sync"

	"github.com/deadsy/sdfx/sdf"
	v3 "github.com/deadsy/sdfx/vec/v3"
)

// encodeSTLTriangle writes one 50-byte STL triangle record (12 float32s
// for normal + 3 vertices, then a 2-byte attribute) directly into buf.
// Avoids binary.Write's reflection path, which profiled at ~16% of CPU
// in picorx.lhs. buf must have at least 50 bytes of capacity.
func encodeSTLTriangle(buf []byte, t *sdf.Triangle3) {
	le := binary.LittleEndian
	n := t.Normal()
	le.PutUint32(buf[0:], math.Float32bits(float32(n.X)))
	le.PutUint32(buf[4:], math.Float32bits(float32(n.Y)))
	le.PutUint32(buf[8:], math.Float32bits(float32(n.Z)))
	le.PutUint32(buf[12:], math.Float32bits(float32(t[0].X)))
	le.PutUint32(buf[16:], math.Float32bits(float32(t[0].Y)))
	le.PutUint32(buf[20:], math.Float32bits(float32(t[0].Z)))
	le.PutUint32(buf[24:], math.Float32bits(float32(t[1].X)))
	le.PutUint32(buf[28:], math.Float32bits(float32(t[1].Y)))
	le.PutUint32(buf[32:], math.Float32bits(float32(t[1].Z)))
	le.PutUint32(buf[36:], math.Float32bits(float32(t[2].X)))
	le.PutUint32(buf[40:], math.Float32bits(float32(t[2].Y)))
	le.PutUint32(buf[44:], math.Float32bits(float32(t[2].Z)))
	buf[48] = 0
	buf[49] = 0
}

//-----------------------------------------------------------------------------

// STLHeader defines the STL file header.
type STLHeader struct {
	_     [80]uint8 // Header
	Count uint32    // Number of triangles
}

// STLTriangle defines the triangle data within an STL file.
type STLTriangle struct {
	Normal, Vertex1, Vertex2, Vertex3 [3]float32
	_                                 uint16 // Attribute byte count
}

//-----------------------------------------------------------------------------

// parseFloats converts float value strings to []float64.
func parseFloats(in []string) ([]float64, error) {
	out := make([]float64, len(in))
	for i := range in {
		val, err := strconv.ParseFloat(in[i], 64)
		if err != nil {
			return nil, err
		}
		out[i] = val
	}
	return out, nil
}

// loadSTLAscii loads an STL file created in ASCII format.
func loadSTLAscii(file *os.File) ([]*sdf.Triangle3, error) {
	var v []v3.Vec
	scanner := bufio.NewScanner(file)
	for scanner.Scan() {
		line := scanner.Text()
		fields := strings.Fields(line)
		if len(fields) == 4 && fields[0] == "vertex" {
			f, err := parseFloats(fields[1:])
			if err != nil {
				return nil, err
			}
			v = append(v, v3.Vec{f[0], f[1], f[2]})
		}
	}
	// make triangles out of every 3 vertices
	var mesh []*sdf.Triangle3
	for i := 0; i < len(v); i += 3 {
		mesh = append(mesh, &sdf.Triangle3{v[i+0], v[i+1], v[i+2]})
	}
	return mesh, scanner.Err()
}

// loadSTLBinary loads an STL file created in binary format.
func loadSTLBinary(file *os.File) ([]*sdf.Triangle3, error) {
	r := bufio.NewReader(file)
	header := STLHeader{}
	if err := binary.Read(r, binary.LittleEndian, &header); err != nil {
		return nil, err
	}
	mesh := make([]*sdf.Triangle3, int(header.Count))
	for i := range mesh {
		d := STLTriangle{}
		if err := binary.Read(r, binary.LittleEndian, &d); err != nil {
			return nil, err
		}
		v1 := v3.Vec{float64(d.Vertex1[0]), float64(d.Vertex1[1]), float64(d.Vertex1[2])}
		v2 := v3.Vec{float64(d.Vertex2[0]), float64(d.Vertex2[1]), float64(d.Vertex2[2])}
		v3 := v3.Vec{float64(d.Vertex3[0]), float64(d.Vertex3[1]), float64(d.Vertex3[2])}
		mesh[i] = &sdf.Triangle3{v1, v2, v3}
	}
	return mesh, nil
}

// LoadSTL loads an STL file (ascii or binary) and returns the triangle mesh.
func LoadSTL(path string) ([]*sdf.Triangle3, error) {
	// open file
	file, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer file.Close()

	// get file size
	info, err := file.Stat()
	if err != nil {
		return nil, err
	}
	size := info.Size()

	// read header, get expected binary size
	header := STLHeader{}
	if err := binary.Read(file, binary.LittleEndian, &header); err != nil {
		return nil, err
	}
	expectedSize := int64(header.Count)*50 + 84

	// rewind to start of file
	_, err = file.Seek(0, 0)
	if err != nil {
		return nil, err
	}

	// parse ascii or binary stl
	if size == expectedSize {
		return loadSTLBinary(file)
	}
	return loadSTLAscii(file)
}

//-----------------------------------------------------------------------------

// SaveSTL writes a triangle mesh to an STL file.
func SaveSTL(path string, mesh []*sdf.Triangle3) error {
	file, err := os.Create(path)
	if err != nil {
		return err
	}
	defer file.Close()

	buf := bufio.NewWriter(file)
	var header [84]byte
	binary.LittleEndian.PutUint32(header[80:], uint32(len(mesh)))
	if _, err := buf.Write(header[:]); err != nil {
		return err
	}

	var rec [50]byte
	for _, triangle := range mesh {
		encodeSTLTriangle(rec[:], triangle)
		if _, err := buf.Write(rec[:]); err != nil {
			return err
		}
	}

	return buf.Flush()
}

//-----------------------------------------------------------------------------

// writeSTL writes a stream of triangles to an STL file.
func writeSTL(wg *sync.WaitGroup, path string) (chan<- []*sdf.Triangle3, error) {

	f, err := os.Create(path)
	if err != nil {
		return nil, err
	}

	// Use buffered IO for optimal IO writes.
	// The default buffer size doesn't appear to limit performance.
	buf := bufio.NewWriter(f)

	// write an empty 84-byte header (80 bytes padding + uint32 count)
	var zeroHdr [84]byte
	if _, err := buf.Write(zeroHdr[:]); err != nil {
		return nil, err
	}

	// External code writes triangles to this channel.
	// This goroutine reads the channel and writes triangles to the file.
	c := make(chan []*sdf.Triangle3)

	wg.Add(1)
	go func() {
		defer wg.Done()
		defer f.Close()

		var count uint32
		var rec [50]byte
		// read triangles from the channel and write them to the file
		for ts := range c {
			for _, t := range ts {
				encodeSTLTriangle(rec[:], t)
				if _, err := buf.Write(rec[:]); err != nil {
					fmt.Printf("%s\n", err)
					return
				}
				count++
			}
		}
		// flush the triangles
		buf.Flush()

		// back to the start of the file
		if _, err := f.Seek(0, 0); err != nil {
			fmt.Printf("%s\n", err)
			return
		}
		// rewrite the header with the correct mesh count
		var hdrBytes [84]byte
		binary.LittleEndian.PutUint32(hdrBytes[80:], count)
		if _, err := f.Write(hdrBytes[:]); err != nil {
			fmt.Printf("%s\n", err)
			return
		}
	}()

	return c, nil
}

//-----------------------------------------------------------------------------
