package viamchess

import (
	"image"
	"math"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/rimage/transform"
)

// Metric point-to-square partitioning.
//
// Assigning a point to a square by its projected pixel is wrong for anything
// tall: a king's crown ~95mm above the board projects outward (away from the
// image center) into the neighboring square's image rect, so sparse boards grow
// phantom pieces next to tall ones (fixtures board27/board28). A point's metric
// position over the board plane has no such parallax: the crown sits above its
// own square no matter where the camera is. So: fit the board plane, unproject
// the detected corners onto it, and bucket each point by which of the 8x8 cells
// its in-plane position falls in.

// boardPlane is z = A*x + B*y + C in the camera frame (z away from the camera;
// the board is roughly perpendicular to the optical axis so this form is
// well-conditioned).
type boardPlane struct {
	A, B, C float64
}

// fitBoardPlane least-squares fits the board plane using only points whose
// (production-model) projection lands inside the detected corner quad — the
// image region we know is board. Fitting the whole cloud instead lets the
// table and background drag the plane away from the board surface, which
// skews the unprojected corners into a non-square quad (the board is square;
// newBoardFrame checks that). A second pass refits on the points within
// refineBandMM of the first fit so pieces inside the quad (a minority of its
// points) don't tilt it. ok=false when there are too few points or the system
// is degenerate.
func fitBoardPlane(pc pointcloud.PointCloud, corners []image.Point, props camera.Properties) (boardPlane, bool) {
	const minPoints = 500
	const refineBandMM = 8.0

	if len(corners) != 4 {
		return boardPlane{}, false
	}
	inQuad := func(p r3.Vector) bool {
		x, y, err := props.PointToPixel(p)
		if err != nil {
			return false
		}
		return pointInConvexQuad(x, y, corners)
	}

	fit := func(keep func(p r3.Vector) bool) (boardPlane, int, bool) {
		var sxx, sxy, sx, syy, sy, n, sxz, syz, sz float64
		pc.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
			if !inQuad(p) {
				return true
			}
			if keep != nil && !keep(p) {
				return true
			}
			sxx += p.X * p.X
			sxy += p.X * p.Y
			sx += p.X
			syy += p.Y * p.Y
			sy += p.Y
			n++
			sxz += p.X * p.Z
			syz += p.Y * p.Z
			sz += p.Z
			return true
		})
		if n < minPoints {
			return boardPlane{}, int(n), false
		}
		sol, ok := solve3x3(
			[3][3]float64{{sxx, sxy, sx}, {sxy, syy, sy}, {sx, sy, n}},
			[3]float64{sxz, syz, sz})
		if !ok {
			return boardPlane{}, int(n), false
		}
		return boardPlane{A: sol[0], B: sol[1], C: sol[2]}, int(n), true
	}

	rough, _, ok := fit(nil)
	if !ok {
		return boardPlane{}, false
	}
	refined, _, ok := fit(func(p r3.Vector) bool {
		return math.Abs(p.Z-(rough.A*p.X+rough.B*p.Y+rough.C)) <= refineBandMM
	})
	if !ok {
		// Enough points overall but too few near the rough plane: trust the
		// rough fit rather than bailing to pixel partitioning.
		return rough, true
	}
	return refined, true
}

// pointInConvexQuad reports whether pixel (x,y) lies inside the corner quad
// (TL, TR, BR, BL order — a convex polygon; inside means all cross products
// have the same sign as we walk the edges).
func pointInConvexQuad(x, y float64, q []image.Point) bool {
	sign := 0
	for i := 0; i < 4; i++ {
		a, b := q[i], q[(i+1)%4]
		cross := float64(b.X-a.X)*(y-float64(a.Y)) - float64(b.Y-a.Y)*(x-float64(a.X))
		switch {
		case cross > 0:
			if sign < 0 {
				return false
			}
			sign = 1
		case cross < 0:
			if sign > 0 {
				return false
			}
			sign = -1
		}
	}
	return true
}

// solve3x3 solves m*x = r by Gaussian elimination with partial pivoting.
func solve3x3(m [3][3]float64, r [3]float64) ([3]float64, bool) {
	for col := 0; col < 3; col++ {
		pivot := col
		for row := col + 1; row < 3; row++ {
			if math.Abs(m[row][col]) > math.Abs(m[pivot][col]) {
				pivot = row
			}
		}
		if math.Abs(m[pivot][col]) < 1e-12 {
			return [3]float64{}, false
		}
		m[col], m[pivot] = m[pivot], m[col]
		r[col], r[pivot] = r[pivot], r[col]
		for row := col + 1; row < 3; row++ {
			f := m[row][col] / m[col][col]
			for k := col; k < 3; k++ {
				m[row][k] -= f * m[col][k]
			}
			r[row] -= f * r[col]
		}
	}
	var x [3]float64
	for row := 2; row >= 0; row-- {
		v := r[row]
		for k := row + 1; k < 3; k++ {
			v -= m[row][k] * x[k]
		}
		x[row] = v / m[row][row]
	}
	return x, true
}

// boardFrame is the board's 2D coordinate system on the fitted plane. origin is
// the 3D top-left corner; u spans the full TL->TR edge (computeSquareBounds'
// column axis, col = 'h'-file) and v the full TL->BL edge (row axis,
// row = rank-1), so cell indices here mean exactly what they mean in the image
// partitioning.
type boardFrame struct {
	origin, u, v r3.Vector
	n            r3.Vector // unit normal
	uu, uv, vv   float64   // Gram matrix of (u, v)
	det          float64
	fracU, fracV float64 // per-cell inset fractions (mirrors the pixel inset)
}

// newBoardFrame unprojects the four detected image corners onto the plane and
// builds the frame. ok=false on missing intrinsics, a corner ray parallel to
// the plane, or a degenerate edge.
func newBoardFrame(corners []image.Point, props camera.Properties, pl boardPlane, squareInset float64) (*boardFrame, bool) {
	in := props.IntrinsicParams
	if in == nil || len(corners) != 4 || in.Fx == 0 || in.Fy == 0 {
		return nil, false
	}
	pts := make([]r3.Vector, 4)
	for i, c := range corners {
		p, ok := unprojectToPlaneModel(c, props, in, pl)
		if !ok {
			return nil, false
		}
		pts[i] = p
	}
	tl, tr, br, bl := pts[0], pts[1], pts[2], pts[3]
	u := tr.Sub(tl)
	v := bl.Sub(tl)
	// The physical board is square: all four edges must come out near-equal or
	// the plane/corners are wrong and metric partitioning would misfile
	// everything. Bail to the pixel fallback instead.
	edges := []float64{u.Norm(), v.Norm(), br.Sub(tr).Norm(), br.Sub(bl).Norm()}
	minE, maxE := edges[0], edges[0]
	for _, e := range edges[1:] {
		minE = math.Min(minE, e)
		maxE = math.Max(maxE, e)
	}
	if minE <= 0 || maxE/minE > 1.15 {
		return nil, false
	}
	n := u.Cross(v)
	if n.Norm() < 1e-9 {
		return nil, false
	}
	n = n.Normalize()
	bf := &boardFrame{
		origin: tl, u: u, v: v, n: n,
		uu: u.Dot(u), uv: u.Dot(v), vv: v.Dot(v),
	}
	bf.det = bf.uu*bf.vv - bf.uv*bf.uv
	if math.Abs(bf.det) < 1e-9 {
		return nil, false
	}
	// Mirror computeSquareBounds' pixel inset as a fraction of a cell edge so
	// both partitioners drop the same border/alignment noise.
	cellPxU := dist2D(corners[0], corners[1]) / 8
	cellPxV := dist2D(corners[0], corners[3]) / 8
	if cellPxU > 0 {
		bf.fracU = math.Min(squareInset, cellPxU/10) / cellPxU
	}
	if cellPxV > 0 {
		bf.fracV = math.Min(squareInset, cellPxV/10) / cellPxV
	}
	return bf, true
}

func dist2D(a, b image.Point) float64 {
	dx := float64(a.X - b.X)
	dy := float64(a.Y - b.Y)
	return math.Sqrt(dx*dx + dy*dy)
}

// unprojectToPlaneModel finds the plane point whose *production* projection
// (props.PointToPixel, including any distortion) lands on the target pixel.
// The ideal-pinhole inverse below is only the starting guess; each iteration
// reprojects with the real model and walks the working pixel by the error.
// Distortion is smooth and near-identity, so this converges in a few rounds;
// without it the corners land 15-27px off (see the roundtrip probe test).
func unprojectToPlaneModel(target image.Point, props camera.Properties, in *transform.PinholeCameraIntrinsics, pl boardPlane) (r3.Vector, bool) {
	px, py := float64(target.X), float64(target.Y)
	gx, gy := px, py
	var p r3.Vector
	for i := 0; i < 8; i++ {
		var ok bool
		p, ok = unprojectToPlane(gx, gy, in, pl)
		if !ok {
			return r3.Vector{}, false
		}
		rx, ry, err := props.PointToPixel(p)
		if err != nil {
			return r3.Vector{}, false
		}
		ex, ey := rx-px, ry-py
		if ex*ex+ey*ey < 0.25 { // within half a pixel
			return p, true
		}
		gx -= ex
		gy -= ey
	}
	return p, true // best effort; sub-pixel convergence usually hits by iter 2-3
}

// unprojectToPlane intersects the ideal-pinhole camera ray through pixel
// (px,py) with the plane. Ray: t*(dx,dy,1) with dx=(px-Ppx)/Fx, dy=(py-Ppy)/Fy;
// substituting into z = A*x+B*y+C gives t = C / (1 - A*dx - B*dy).
func unprojectToPlane(px, py float64, in *transform.PinholeCameraIntrinsics, pl boardPlane) (r3.Vector, bool) {
	dx := (px - in.Ppx) / in.Fx
	dy := (py - in.Ppy) / in.Fy
	denom := 1 - pl.A*dx - pl.B*dy
	if math.Abs(denom) < 1e-9 {
		return r3.Vector{}, false
	}
	t := pl.C / denom
	if t <= 0 {
		return r3.Vector{}, false
	}
	return r3.Vector{X: dx * t, Y: dy * t, Z: t}, true
}

// cell returns the (col, row) cell containing p's in-plane position, or
// ok=false when p is off the board or inside the inset border. col follows the
// TL->TR axis ('h'-file), row the TL->BL axis (rank-1).
func (bf *boardFrame) cell(p r3.Vector) (col, row int, ok bool) {
	w := p.Sub(bf.origin)
	w = w.Sub(bf.n.Mul(w.Dot(bf.n)))
	bu := w.Dot(bf.u)
	bv := w.Dot(bf.v)
	s := (bf.vv*bu - bf.uv*bv) / bf.det
	t := (bf.uu*bv - bf.uv*bu) / bf.det
	if s < 0 || s >= 1 || t < 0 || t >= 1 {
		return 0, 0, false
	}
	sc := s * 8
	tc := t * 8
	col = int(sc)
	row = int(tc)
	fs := sc - float64(col)
	ft := tc - float64(row)
	if fs < bf.fracU || fs > 1-bf.fracU || ft < bf.fracV || ft > 1-bf.fracV {
		return 0, 0, false
	}
	return col, row, true
}

// boardBandBlankets reports whether the square's board-surface points cover
// its full footprint: divide the band's XY extent into a grid and require
// every sub-cell occupied. A standing piece occludes the board beneath it, so
// even when glossy surfaces drop the piece's own depth points, the board band
// shows a hole where the piece stands — only a truly empty square blankets.
func boardBandBlankets(pc pointcloud.PointCloud, boardZ float64) bool {
	const boardBandHalfMM = 5.0
	minX, maxX := math.Inf(1), math.Inf(-1)
	minY, maxY := math.Inf(1), math.Inf(-1)
	var band []r3.Vector
	pc.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		if math.Abs(p.Z-boardZ) > boardBandHalfMM {
			return true
		}
		band = append(band, p)
		minX = math.Min(minX, p.X)
		maxX = math.Max(maxX, p.X)
		minY = math.Min(minY, p.Y)
		maxY = math.Max(maxY, p.Y)
		return true
	})
	if len(band) == 0 || maxX-minX <= 0 || maxY-minY <= 0 {
		return false
	}
	// 5x5: a piece's depth-dropout hole spans roughly half the cell, so at
	// least one interior sub-cell falls fully inside it; 3x3 was too coarse
	// (ring points at a center sub-cell's edges masked the hole).
	const g = 5
	var grid [g][g]bool
	for _, p := range band {
		gx := int((p.X - minX) / (maxX - minX) * g)
		gy := int((p.Y - minY) / (maxY - minY) * g)
		if gx > g-1 {
			gx = g - 1
		}
		if gy > g-1 {
			gy = g - 1
		}
		grid[gx][gy] = true
	}
	for gx := 0; gx < g; gx++ {
		for gy := 0; gy < g; gy++ {
			if !grid[gx][gy] {
				return false
			}
		}
	}
	return true
}

// squareIndexFor maps a (col, row) cell to the index used by the rank-major
// squares slice built in findBoardAndPieces: index = (rank-1)*8 + (file-'a')
// with file = 'h'-col and rank = row+1.
func squareIndexFor(col, row int) int {
	return row*8 + (7 - col)
}
