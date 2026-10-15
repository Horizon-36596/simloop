package org.horizon36596.simloop.sim.live;

/**
 * Passes another source's frames through unchanged and writes each one to a track as it goes. A live
 * session is a keyboard source wrapped in one of these, which is how every live session leaves a track
 * behind that replays it.
 *
 * <p>A frame is written BEFORE it is handed on, so the track holds every loop that ran, including the
 * last one of a session that then crashed.
 */
public final class RecordingGamepadInputSource implements GamepadInputSource {

    private final GamepadInputSource source;
    private final GamepadInputTrack.Writer track;

    /**
     * A source that passes on every frame of {@code source}, writing each one to {@code track} first.
     *
     * @param source where the frames really come from
     * @param track  where to record them; the caller closes it when the session ends
     */
    public RecordingGamepadInputSource(GamepadInputSource source, GamepadInputTrack.Writer track) {
        this.source = source;
        this.track = track;
    }

    @Override
    public GamepadInputTrack.Frame next(int tickIndex) {
        GamepadInputTrack.Frame frame = source.next(tickIndex);
        if (frame == null) {
            return null;
        }
        if (frame.tick != tickIndex) {
            throw new IllegalStateException("the source answered tick " + tickIndex + " with a frame for tick "
                    + frame.tick);
        }
        track.append(frame);
        return frame;
    }
}
