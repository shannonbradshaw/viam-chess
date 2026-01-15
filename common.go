package viamchess

import (
	"context"
	"fmt"
	"os"

	"go.opentelemetry.io/otel/exporters/otlp/otlptrace/otlptracegrpc"

	"go.viam.com/rdk/resource"
	"go.viam.com/utils/trace"
)

var family = resource.ModelNamespace("erh").WithFamily("viam-chess")

func enableTracing() {
	if os.Getenv("OTEL_SERVICE_NAME") == "" {
		fmt.Println("no OTEL_SERVICE_NAME, not enabling tracing")
	} else {
		exporter, err := otlptracegrpc.New(context.Background())
		if err != nil {
			fmt.Printf("can't enable tracing: %v", err)
		} else {
			err := trace.SetProvider(context.Background())
			fmt.Printf("error setting new trace provider: %v", err)
			trace.AddExporters(exporter)
		}
	}
}
