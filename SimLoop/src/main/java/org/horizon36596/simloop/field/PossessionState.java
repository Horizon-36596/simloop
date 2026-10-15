package org.horizon36596.simloop.field;

/**
 * The complete, fixed set of possession states one game piece can be in. A piece is in exactly one of
 * these at all times, and it only ever changes through {@link GamePieceTracker}, which logs the change
 * (domain R5 — every transition is in the RLOG and identical on replay).
 *
 * <p><b>Every legal transition</b>, and the tracker call that makes it:
 * <pre>
 *   LOOSE     -> HELD       tryAcquire (a running mouth over the piece, with room)
 *   HELD      -> LOOSE      eject
 *   HELD      -> IN_FLIGHT  launch
 *   HELD      -> SCORED     score (label only, in no goal) or scoreInto (parked in a goal)
 *   IN_FLIGHT -> SCORED     advance (the piece entered a goal)
 *   IN_FLIGHT -> LOOSE      advance (the piece settled on the floor)
 *   SCORED    -> HELD       tryRetrieveBottomPiece
 *   SCORED    -> IN_FLIGHT  releaseScoredPiece
 * </pre>
 * No other change is possible: the tracker has no call that makes one. (A piece can also be created
 * straight into LOOSE, HELD or SCORED by the tracker's add methods.)
 *
 * <p><b>History of what this models.</b> On 2026-09-03 the human ruled that game pieces were possession
 * events plus trigger volumes and nothing else: no rolling, no bouncing, no flight, because domain R6
 * forbade contact physics. On 2026-09-20 the human reversed that for game pieces specifically — a
 * season's sim needs balls that get pushed, roll, and fly a real arc out of the launcher — so a piece now
 * also has a height, a velocity and one extra state, {@link #IN_FLIGHT}. The <em>possession</em> half is
 * unchanged: acquisition is still a trigger volume overlapping a loose piece while the mechanism runs
 * (a "teleport" into the robot), and nothing about the robot's own mechanisms is rigid-body.
 *
 * <p>Season-agnostic (domain R7): there is no state here that only one game's pieces could be in, and the
 * set is deliberately small — four states a student can hold in their head, not a graph of handoffs.
 */
public enum PossessionState {

    /**
     * On the field floor, available to be acquired. A loose piece rolls, slows to a stop, is pushed by
     * the robot's footprint, and bounces off walls, obstacles and other loose pieces
     * ({@link GamePieceTracker#advance}). This is where a piece starts and where a landed or ejected piece
     * returns to.
     */
    LOOSE,

    /**
     * Carried by the robot. A held piece's logged field position is the robot's field position — the model
     * does not track where inside the robot a piece sits. The count of held pieces is what the capacity
     * invariant bounds.
     */
    HELD,

    /**
     * Launched and airborne: a ballistic arc under gravity (plus optional air drag) from the exit point the
     * launcher stated ({@link GamePieceTracker#launch}), or from the point a goal let go of it
     * ({@link GamePieceTracker#releaseScoredPiece}). An in-flight piece cannot be acquired. It leaves
     * this state either by entering a {@link ScoringVolume} (becomes {@link #SCORED}) or by settling on the
     * floor after its last bounce (becomes {@link #LOOSE}).
     */
    IN_FLIGHT,

    /**
     * Inside a scoring location and out of the ordinary play: {@link GamePieceTracker#tryAcquire} never
     * takes a scored piece, so a trigger volume sweeping over one does nothing. A piece parked in one of
     * the world's goals keeps a field position — the resting slot the scoring volume gave it — so a viewer
     * can still draw it sitting in the goal; a piece scored with the label-only
     * {@link GamePieceTracker#score(int, String)} is in no goal and is not drawn at all.
     *
     * <p><b>Not terminal.</b> A parked piece can leave a goal two ways, both only through the tracker:
     * <ul>
     *   <li>{@code SCORED -> HELD} — {@link GamePieceTracker#tryRetrieveBottomPiece}, a mouth pulling the
     *       bottom piece out of a column open at its base;</li>
     *   <li>{@code SCORED -> IN_FLIGHT} — {@link GamePieceTracker#releaseScoredPiece}, a goal emptying
     *       itself (a tipping goal dumping its contents).</li>
     * </ul>
     */
    SCORED
}
