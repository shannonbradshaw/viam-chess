package viamchess

import (
	"fmt"
	"image"
	"sort"
	"testing"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/rimage"
	"go.viam.com/test"
)

// Probe: unproject the detected corners onto the fitted plane, reproject with
// the production PointToPixel, and report the pixel error. Large errors mean
// the manual ray model diverges from the camera model (e.g. distortion).
func TestMetricPartitionCornerRoundtrip(t *testing.T) {
	input, err := rimage.ReadImageFromFile("data/board17.jpg")
	test.That(t, err, test.ShouldBeNil)
	pc, err := pointcloud.NewFromFile("data/board17.pcd", "")
	test.That(t, err, test.ShouldBeNil)
	props, err := ReadCameraProperties("data/board17_props.json")
	test.That(t, err, test.ShouldBeNil)

	corners, err := FindBoard(input)
	test.That(t, err, test.ShouldBeNil)

	pl, ok := fitBoardPlane(pc, corners, props)
	test.That(t, ok, test.ShouldBeTrue)
	t.Logf("plane: z = %.6f*x + %.6f*y + %.2f", pl.A, pl.B, pl.C)

	for i, c := range corners {
		p, ok := unprojectToPlaneModel(c, props, props.IntrinsicParams, pl)
		test.That(t, ok, test.ShouldBeTrue)
		px, py, err := props.PointToPixel(p)
		test.That(t, err, test.ShouldBeNil)
		t.Logf("corner %d: pixel (%d,%d) -> 3D (%.1f,%.1f,%.1f) -> reprojected (%.1f,%.1f)",
			i, c.X, c.Y, p.X, p.Y, p.Z, px, py)
	}
}

// Probe: for every point, compare its pixel-rect square against its metric
// square and print the migration counts, to reveal the shape of any systematic
// mis-assignment.
func TestMetricPartitionMigration(t *testing.T) {
	input, err := rimage.ReadImageFromFile("data/board17.jpg")
	test.That(t, err, test.ShouldBeNil)
	pc, err := pointcloud.NewFromFile("data/board17.pcd", "")
	test.That(t, err, test.ShouldBeNil)
	props, err := ReadCameraProperties("data/board17_props.json")
	test.That(t, err, test.ShouldBeNil)
	corners, err := FindBoard(input)
	test.That(t, err, test.ShouldBeNil)

	cc := defaultClassifyConfig()
	pl, ok := fitBoardPlane(pc, corners, props)
	test.That(t, ok, test.ShouldBeTrue)
	bf, ok := newBoardFrame(corners, props, pl, cc.SquareInset)
	test.That(t, ok, test.ShouldBeTrue)

	names := map[int]string{}
	rects := map[int]image.Rectangle{}
	for rank := 1; rank <= 8; rank++ {
		for file := 'a'; file <= 'h'; file++ {
			idx := (rank-1)*8 + int(file-'a')
			names[idx] = fmt.Sprintf("%c%d", file, rank)
			rects[idx] = computeSquareBounds(corners, int('h'-file), rank-1, cc.SquareInset)
		}
	}

	migration := map[string]int{}
	pc.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		metricIdx := -1
		if col, row, ok := bf.cell(p); ok {
			metricIdx = squareIndexFor(col, row)
		}
		pixelIdx := -1
		if x, y, err := props.PointToPixel(p); err == nil {
			ix, iy := int(x), int(y)
			for idx, b := range rects {
				if ix >= b.Min.X && ix <= b.Max.X && iy >= b.Min.Y && iy <= b.Max.Y {
					pixelIdx = idx
					break
				}
			}
		}
		if metricIdx != pixelIdx {
			from, to := "off", "off"
			if pixelIdx >= 0 {
				from = names[pixelIdx]
			}
			if metricIdx >= 0 {
				to = names[metricIdx]
			}
			migration[from+"->"+to]++
		}
		return true
	})

	type kv struct {
		k string
		v int
	}
	var pairs []kv
	for k, v := range migration {
		pairs = append(pairs, kv{k, v})
	}
	sort.Slice(pairs, func(i, j int) bool { return pairs[i].v > pairs[j].v })
	for i, p := range pairs {
		if i >= 25 {
			break
		}
		t.Logf("%6d  %s", p.v, p.k)
	}
}
