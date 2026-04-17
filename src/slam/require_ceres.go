//go:build !ceres

package slam

// Keep cgo active in !ceres builds so this package still handles the .cpp wrapper
// consistently and then fails with our explicit build-tag error below.
/*
#include "ceres_wrapper.h"
*/
import "C"

var _ C.CeresPose

// This file deliberately fails to compile when the "ceres" build tag is absent.
// Ceres Solver is required — there is no fallback optimizer.
//
// Always build/run this package with the tag:
//
//   go build -tags ceres ./...
//   go run   -tags ceres ./...
//   go test  -tags ceres ./...

var _ = ERROR_missing_build_tag__use_TAGS_ceres // undefined → compile error
