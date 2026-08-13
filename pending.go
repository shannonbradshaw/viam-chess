package viamchess

import "sync"

// Pending-move phases, advanced as makeAMove passes physical milestones. A
// non-empty move with any phase means execution started but never committed
// (game.Move/saveGame run only after all arm motion succeeds), so on an
// execution fault the tracker still holds what the robot was attempting.
const (
	phasePlanned           = "planned"              // move chosen, no arm motion yet committed
	phaseCastleRookMoved   = "castle-rook-moved"    // castle: rook repositioned, king not yet
	phaseEnPassantPawnGone = "enpassant-pawn-taken" // en passant: captured pawn graveyarded
	phaseCaptureCleared    = "capture-cleared"      // destination occupant graveyarded
)

// pendingMoveTracker records the engine move currently being executed
// physically so UIs (board-snapshot / mode-status) can report what the robot
// was attempting when execution faults into ERROR. It has its own lock because
// the board-snapshot and mode-status fast paths read it without doCommandLock.
type pendingMoveTracker struct {
	mu    sync.Mutex
	move  string // UCI, e.g. "d8f6"; empty = nothing pending
	to    string // destination square, for phase attribution by movePiece
	phase string
}

func (p *pendingMoveTracker) set(move, to string) {
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.move == move && p.to == to {
		// Retry of the same move: keep the recorded phase so already-completed
		// physical steps are skipped, not repeated.
		return
	}
	p.move = move
	p.to = to
	p.phase = phasePlanned
}

func (p *pendingMoveTracker) setPhase(phase string) {
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.move == "" {
		return
	}
	p.phase = phase
}

// setPhaseIfDest advances the phase only when a move is pending and its
// destination is dest. movePiece calls this unconditionally after clearing a
// destination square, so unrelated movePiece uses (undo, manual moves, the
// castle rook leg) never mislabel the pending move.
func (p *pendingMoveTracker) setPhaseIfDest(dest, phase string) {
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.move == "" || p.to != dest {
		return
	}
	p.phase = phase
}

func (p *pendingMoveTracker) clear() {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.move = ""
	p.to = ""
	p.phase = ""
}

func (p *pendingMoveTracker) get() (move, phase string) {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.move, p.phase
}

// captureCleared reports whether the pending move's destination is dest and a
// prior attempt already moved dest's occupant to the graveyard. movePiece uses
// this to skip the physical graveyard step when retrying a partially-completed
// capture.
func (p *pendingMoveTracker) captureCleared(dest string) bool {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.move != "" && p.to == dest && p.phase == phaseCaptureCleared
}
