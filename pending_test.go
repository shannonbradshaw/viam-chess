package viamchess

import "testing"

func TestPendingMoveTracker(t *testing.T) {
	var p pendingMoveTracker

	// Zero value: nothing pending.
	if move, phase := p.get(); move != "" || phase != "" {
		t.Fatalf("zero value should be empty, got move=%q phase=%q", move, phase)
	}

	// Phase updates on an empty tracker are ignored.
	p.setPhase(phaseCaptureCleared)
	p.setPhaseIfDest("f6", phaseCaptureCleared)
	if move, phase := p.get(); move != "" || phase != "" {
		t.Fatalf("phase set without a pending move should be ignored, got move=%q phase=%q", move, phase)
	}

	// set starts a move at phasePlanned.
	p.set("d8f6", "f6")
	if move, phase := p.get(); move != "d8f6" || phase != phasePlanned {
		t.Fatalf("after set, got move=%q phase=%q", move, phase)
	}

	// setPhaseIfDest with the wrong destination is a no-op (the castle rook
	// leg and unrelated movePiece calls must not mislabel the pending move).
	p.setPhaseIfDest("f1", phaseCaptureCleared)
	if _, phase := p.get(); phase != phasePlanned {
		t.Fatalf("wrong-dest phase update should be ignored, got phase=%q", phase)
	}

	// Matching destination advances the phase.
	p.setPhaseIfDest("f6", phaseCaptureCleared)
	if _, phase := p.get(); phase != phaseCaptureCleared {
		t.Fatalf("matching-dest phase update should apply, got phase=%q", phase)
	}

	// Unconditional phase set applies while a move is pending.
	p.setPhase(phaseCastleRookMoved)
	if _, phase := p.get(); phase != phaseCastleRookMoved {
		t.Fatalf("setPhase should apply, got phase=%q", phase)
	}

	// clear empties everything.
	p.clear()
	if move, phase := p.get(); move != "" || phase != "" {
		t.Fatalf("after clear, got move=%q phase=%q", move, phase)
	}
}
