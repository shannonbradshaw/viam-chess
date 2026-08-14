package viamchess

import (
	"fmt"
	"image"
	"testing"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/rimage"
	"go.viam.com/test"
)

// Probe: for each square still failing under metric partitioning, print the
// full 3D classification intermediates for BOTH partitionings so the failures
// can be attributed (composition change vs heuristic threshold).
func TestMetricPartitionFailingSquares(t *testing.T) {
	cases := []struct{ board, square string }{
		{"board22", "e2"},
		{"board23", "d2"},
		{"board23", "e4"},
		{"board28", "g6"},
	}
	_ = logging.NewTestLogger(t)
	for _, tc := range cases {
		t.Run(tc.board+"/"+tc.square, func(t *testing.T) {
			input, err := rimage.ReadImageFromFile("data/" + tc.board + ".jpg")
			test.That(t, err, test.ShouldBeNil)
			pc, err := pointcloud.NewFromFile("data/"+tc.board+".pcd", "")
			test.That(t, err, test.ShouldBeNil)
			props, err := ReadCameraProperties("data/" + tc.board + "_props.json")
			test.That(t, err, test.ShouldBeNil)
			corners, err := FindBoard(input)
			test.That(t, err, test.ShouldBeNil)
			cc := defaultClassifyConfig()

			pl, ok := fitBoardPlane(pc, corners, props)
			test.That(t, ok, test.ShouldBeTrue)
			bf, ok := newBoardFrame(corners, props, pl, cc.SquareInset)
			test.That(t, ok, test.ShouldBeTrue)

			var file rune = rune(tc.square[0])
			rank := int(tc.square[1] - '0')
			rect := computeSquareBounds(corners, int('h'-file), rank-1, cc.SquareInset)
			wantIdx := (rank-1)*8 + int(file-'a')

			metricPc := pointcloud.NewBasicEmpty()
			pixelPc := pointcloud.NewBasicEmpty()
			pc.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
				if col, row, ok := bf.cell(p); ok && squareIndexFor(col, row) == wantIdx {
					metricPc.Set(p, d)
				}
				if x, y, err := props.PointToPixel(p); err == nil {
					ix, iy := int(x), int(y)
					if ix >= rect.Min.X && ix <= rect.Max.X && iy >= rect.Min.Y && iy <= rect.Max.Y {
						pixelPc.Set(p, d)
					}
				}
				return true
			})

			for label, spc := range map[string]pointcloud.PointCloud{"metric": metricPc, "pixel": pixelPc} {
				d3x := pcDiagnose3D(spc, input, props, 0, cc.MinPieceSize)
				final := classifyPieceColor(spc, input, rect, props, cc)
				reject := d3x.rejectReason(cc.ColorDivergenceGuard, cc.MinTopFootprintMM)
				t.Logf("%-6s size=%5d top=%4d topColored=%4d boardCnt=%4d final=%d reject=%q", label, spc.Size(), d3x.TopCount, d3x.TopColoredCount, d3x.BoardColoredCount, final, reject)
				t.Logf("   topMean RGB=(%.0f,%.0f,%.0f) boardMean RGB=(%.0f,%.0f,%.0f) warmth=%.1f zTop=[%.0f,%.0f] planeZ=%.0f footprint=%.0fx%.0f",
					d3x.TopMeanAttachedR, d3x.TopMeanAttachedG, d3x.TopMeanAttachedB,
					d3x.BoardMeanAttachedR, d3x.BoardMeanAttachedG, d3x.BoardMeanAttachedB,
					d3x.TopMeanAttachedR-d3x.TopMeanAttachedB,
					d3x.TopMinZ, d3x.TopMaxZ, d3x.BoardPlaneZ,
					d3x.TopMaxX-d3x.TopMinX, d3x.TopMaxY-d3x.TopMinY)
			}
			d2 := colorFromImage2D(input, rect, cc.OtsuSeparationThreshold, cc.BrightnessThreshold)
			minority := d2.CntDark
			if d2.CntLight < minority {
				minority = d2.CntLight
			}
			frac := 0.0
			if d2.Total > 0 {
				frac = float64(minority) / float64(d2.Total)
			}
			t.Logf("2D: total=%d dark=%d light=%d sep=%.1f color=%d minorityFrac=%.3f",
				d2.Total, d2.CntDark, d2.CntLight, d2.Separation, d2.Color, frac)
			_ = fmt.Sprintf("%v", image.Point{})
		})
	}
}
