package main

import (
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	generic "go.viam.com/rdk/services/generic"
	"viamchess"
)

func main() {

	module.ModularMain(resource.APIModel{camera.API, viamchess.BoardCameraModel})
	module.ModularMain(resource.APIModel{generic.API, viamchess.ChessModel})
}
