package org.horizon36596.simloop.vision;

import org.horizon36596.simloop.field.SeededRandom;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * The timing faults of a {@link VisionFaults}, for any kind of frame: extra latency and jitter, dropped
 * frames, repeated frames (the same capture stamp delivered twice) and frames delivered out of capture order.
 * The frame contents are whatever the caller hands in ({@code T}: a list of sightings, a {@link SimFrame},
 * a JSON string) and are never looked at; {@link VisionFaultInjector} changes contents.
 *
 * <p><b>How to drive it.</b> On each capture, {@link #offer} the frame with its capture time and the time the
 * caller's own model would deliver it (capture plus that model's latency; the capture time itself for none).
 * On each loop, {@link #pollDue} takes every frame whose delivery time has come, in delivery order. All times
 * are one clock, nanoseconds; sim time, never the wall clock.
 *
 * <p><b>The rules</b>, per offered frame, each fault drawn from its own seeded stream:
 * <ol>
 *   <li><b>Drop</b>: the frame is never delivered.</li>
 *   <li><b>Latency</b>: delivery is the caller's time plus {@link VisionFaults#getExtraLatencySeconds()} plus
 *       a uniform shift of up to {@link VisionFaults#getLatencyJitterSeconds()} either way, never before the
 *       capture. A frame never overtakes the one offered before it because of jitter: it waits for it.</li>
 *   <li><b>Reorder</b>: the frame is held back until the next frame offered has been delivered, and comes at
 *       the start of the next {@link #pollDue} call after that, so its capture stamp is older than the one
 *       delivered before it. Two held in a row come out newest first. A held frame with no next frame is
 *       never delivered.</li>
 *   <li><b>Repeat</b>: after delivery, the same frame (same stamp, same contents object) is delivered once
 *       more, at the start of the next {@link #pollDue} call that is at least one nanosecond later.</li>
 * </ol>
 *
 * <p>A late frame (reordered or repeated) comes at the start of a poll, before the frames newly due in it.
 * A consumer that keeps only the newest delivery of each poll sees it only on a poll with nothing newer.
 *
 * <p>Deterministic: the same faults and the same sequence of calls give the same deliveries. Not thread-safe.
 *
 * @param <T> what a frame carries
 */
public final class FaultedFrameStream<T> {

    /**
     * One delivered frame. Immutable apart from whatever {@code T} is.
     *
     * @param <T> what a frame carries
     */
    public static final class Delivery<T> {
        private final long captureNanos;
        private final long deliveredNanos;
        private final T frame;
        private final boolean repeat;
        private final boolean outOfOrder;

        Delivery(long captureNanos, long deliveredNanos, T frame, boolean repeat, boolean outOfOrder) {
            this.captureNanos = captureNanos;
            this.deliveredNanos = deliveredNanos;
            this.frame = frame;
            this.repeat = repeat;
            this.outOfOrder = outOfOrder;
        }

        /** {@return the frame's capture time, nanoseconds, as offered: the stamp a consumer is handed} */
        public long getCaptureNanos() {
            return captureNanos;
        }

        /** {@return when the frame became due, nanoseconds; at or before the poll that returned it} */
        public long getDeliveredNanos() {
            return deliveredNanos;
        }

        /** {@return what the frame carries, the object offered} */
        public T getFrame() {
            return frame;
        }

        /** {@return true for the second delivery of a repeated frame} */
        public boolean isRepeat() {
            return repeat;
        }

        /** {@return true when a frame captured later was delivered before this one} */
        public boolean isOutOfOrder() {
            return outOfOrder;
        }
    }

    private static final class Pending<T> {
        final long captureNanos;
        final long dueNanos;
        final T frame;
        final boolean outOfOrder;
        /** A held-back frame waiting for this one to be delivered; null for none. */
        Pending<T> follower;

        Pending(long captureNanos, long dueNanos, T frame, boolean outOfOrder) {
            this.captureNanos = captureNanos;
            this.dueNanos = dueNanos;
            this.frame = frame;
            this.outOfOrder = outOfOrder;
        }
    }

    private final VisionFaults faults;
    private final SeededRandom dropRandom;
    private final SeededRandom jitterRandom;
    private final SeededRandom reorderRandom;
    private final SeededRandom repeatRandom;
    private final List<Pending<T>> queue = new ArrayList<Pending<T>>();
    private final List<Pending<T>> repeats = new ArrayList<Pending<T>>();
    /** Held-back frames whose leader has been delivered: they come at the start of the next poll. */
    private final List<Pending<T>> late = new ArrayList<Pending<T>>();
    private Pending<T> heldBack;
    private long lastDueNanos = Long.MIN_VALUE;
    private long lastOfferedCaptureNanos = Long.MIN_VALUE;
    private long lastPollNanos = Long.MIN_VALUE;
    private long offered;
    private long dropped;
    private long delivered;
    private long repeated;
    private long reordered;

    /**
     * A stream for these faults, at the start of its random streams, with nothing in flight.
     *
     * @param faults the timing faults to apply; {@link VisionFaults#none()} delivers every frame at the
     *               caller's own time, in order, once
     */
    public FaultedFrameStream(VisionFaults faults) {
        if (faults == null) {
            throw new IllegalArgumentException("faults must not be null; use VisionFaults.none()");
        }
        this.faults = faults;
        this.dropRandom = VisionFaultInjector.stream(faults.getSeed(), 11);
        this.jitterRandom = VisionFaultInjector.stream(faults.getSeed(), 12);
        this.reorderRandom = VisionFaultInjector.stream(faults.getSeed(), 13);
        this.repeatRandom = VisionFaultInjector.stream(faults.getSeed(), 14);
    }

    /**
     * Hand in one captured frame.
     *
     * @param captureNanos the capture time, nanoseconds; not before the last frame offered
     * @param readyNanos   when the caller's own model would deliver it, nanoseconds; not before the capture
     * @param frame        what the frame carries; may be null if {@code T} allows it
     * @throws IllegalArgumentException when the capture time goes backwards or the frame would be ready
     *                                  before it was captured
     */
    public void offer(long captureNanos, long readyNanos, T frame) {
        if (captureNanos < lastOfferedCaptureNanos) {
            throw new IllegalArgumentException("frames are offered in capture order; " + captureNanos
                    + " ns follows " + lastOfferedCaptureNanos + " ns. Use reorderFrames to deliver out of order");
        }
        if (readyNanos < captureNanos) {
            throw new IllegalArgumentException("a frame cannot be ready (" + readyNanos
                    + " ns) before it is captured (" + captureNanos + " ns)");
        }
        lastOfferedCaptureNanos = captureNanos;
        offered++;
        if (faults.getDropProbability() > 0.0 && dropRandom.nextFraction() < faults.getDropProbability()) {
            dropped++;
            return;
        }
        long due = readyNanos + Math.round(faults.getExtraLatencySeconds() * 1e9);
        double jitter = faults.getLatencyJitterSeconds();
        if (jitter > 0.0) {
            due += Math.round(jitterRandom.nextBetween(-jitter, jitter) * 1e9);
        }
        due = Math.max(due, captureNanos);
        due = Math.max(due, lastDueNanos);      // in order: wait for the frame before
        lastDueNanos = due;

        boolean holdBack = faults.getReorderProbability() > 0.0
                && reorderRandom.nextFraction() < faults.getReorderProbability();
        Pending<T> previouslyHeld = heldBack;
        Pending<T> current = new Pending<T>(captureNanos, due, frame, holdBack);
        if (previouslyHeld != null) {
            current.follower = previouslyHeld;     // after this frame, whenever this frame comes out
            reordered++;
        }
        if (holdBack) {
            heldBack = current;
        } else {
            heldBack = null;
            queue.add(current);
        }
    }

    /**
     * Every frame due at {@code nowNanos}: first any repeats owed from an earlier poll, then the frames whose
     * delivery time has come, in delivery order.
     *
     * @param nowNanos the current time, nanoseconds, same clock as {@link #offer}
     * @return the deliveries, possibly empty, unmodifiable
     */
    public List<Delivery<T>> pollDue(long nowNanos) {
        List<Delivery<T>> out = new ArrayList<Delivery<T>>();
        if (nowNanos > lastPollNanos) {
            // Held-back frames first, then repeats. Delivering a held frame can release the one held behind
            // it; that one waits for the next poll, so each comes on its own poll.
            List<Pending<T>> lateNow = new ArrayList<Pending<T>>(late);
            late.clear();
            for (Pending<T> p : lateNow) {
                deliver(p, false, out);
            }
            for (Pending<T> p : repeats) {
                out.add(new Delivery<T>(p.captureNanos, p.dueNanos, p.frame, true, p.outOfOrder));
                delivered++;
                repeated++;
            }
            repeats.clear();
        }
        lastPollNanos = Math.max(lastPollNanos, nowNanos);
        // The queue is sorted by due time by construction: jitter never lets a frame overtake the one before.
        while (!queue.isEmpty() && queue.get(0).dueNanos <= nowNanos) {
            deliver(queue.remove(0), true, out);
        }
        return Collections.unmodifiableList(out);
    }

    private void deliver(Pending<T> p, boolean mayRepeat, List<Delivery<T>> out) {
        out.add(new Delivery<T>(p.captureNanos, p.dueNanos, p.frame, false, p.outOfOrder));
        delivered++;
        if (p.follower != null) {
            late.add(p.follower);
            p.follower = null;
        }
        if (mayRepeat && faults.getRepeatProbability() > 0.0
                && repeatRandom.nextFraction() < faults.getRepeatProbability()) {
            repeats.add(p);
        }
    }

    /** {@return frames waiting for their delivery time, a held-back frame included} */
    public int inFlight() {
        int held = 0;
        for (Pending<T> p : queue) {
            for (Pending<T> f = p.follower; f != null; f = f.follower) {
                held++;
            }
        }
        for (Pending<T> f = heldBack; f != null; f = f.follower) {
            held++;
        }
        return queue.size() + held + late.size() + repeats.size();
    }

    /** {@return frames offered so far} */
    public long getOffered() {
        return offered;
    }

    /** {@return frames dropped so far} */
    public long getDropped() {
        return dropped;
    }

    /** {@return deliveries so far, repeats included} */
    public long getDelivered() {
        return delivered;
    }

    /** {@return second deliveries of repeated frames so far} */
    public long getRepeated() {
        return repeated;
    }

    /** {@return frames queued behind a later frame so far} */
    public long getReordered() {
        return reordered;
    }

    /** {@return the faults this stream applies} */
    public VisionFaults getFaults() {
        return faults;
    }
}
