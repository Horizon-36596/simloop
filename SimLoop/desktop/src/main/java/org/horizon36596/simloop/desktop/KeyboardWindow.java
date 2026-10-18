package org.horizon36596.simloop.desktop;

import java.awt.BorderLayout;
import java.awt.Font;
import java.awt.GraphicsEnvironment;
import java.awt.KeyEventDispatcher;
import java.awt.KeyboardFocusManager;
import java.awt.event.KeyEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.lang.reflect.InvocationTargetException;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;

import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JTextArea;
import javax.swing.SwingUtilities;
import javax.swing.WindowConstants;

/**
 * A small desktop window that stands in for a gamepad: while it has focus, it knows which keys are held
 * down. Horizon (FTC 36596) SimLoop live-drive tooling.
 *
 * <p><b>Only the JDK.</b> AWT and Swing, nothing else, so it adds no dependency and needs no native
 * library. That is also its limit: it cannot read a real game controller, which needs a native input
 * library and is a separate decision.
 *
 * <p><b>Why keys-down and not key events.</b> A driver holds a stick; a sim loop wants to know what is
 * held NOW. So the window keeps a set of held keys, updated by the toolkit's own thread as keys go down
 * and up, and {@link #keysDown()} hands out a copy. The sim reads that copy once per loop, and what it read
 * is what gets recorded, so the recording is exactly what the sim saw.
 *
 * <p><b>Focus.</b> A key released while another window has focus never reaches this one, and would stay
 * "held" for ever. So losing focus lets go of every key, the way letting go of a gamepad would.
 *
 * <p><b>Known limit: key auto-repeat on Linux (X11).</b> On Windows and macOS a held key sends repeated
 * KEY_PRESSED events and one KEY_RELEASED when it is let go, which the held set absorbs. Under X11, Java
 * reports auto-repeat as a RELEASED/PRESSED pair for every repeat, so a held key can read as "up" for the
 * odd loop that samples between the two. The recording is still exactly what the sim saw, so a replay still
 * matches; only the driving feels a little choppy. Turning key repeat off ({@code xset r off}) while driving
 * avoids it. Not filtered here, because telling a repeat from a real quick tap needs a timing guess.
 *
 * <p>Key names are plain strings, listed in {@link #REPORTED_KEY_NAMES}; nothing in this class's public API is an
 * AWT type, so an Android module can call it without compiling against AWT.
 */
public final class KeyboardWindow {

    /** The name each key this window reports goes by. */
    private static final Map<Integer, String> NAME_BY_KEY_CODE = new HashMap<>();

    static {
        for (char letter = 'A'; letter <= 'Z'; letter++) {
            NAME_BY_KEY_CODE.put(KeyEvent.VK_A + (letter - 'A'), String.valueOf(letter));
        }
        for (char digit = '0'; digit <= '9'; digit++) {
            NAME_BY_KEY_CODE.put(KeyEvent.VK_0 + (digit - '0'), String.valueOf(digit));
        }
        NAME_BY_KEY_CODE.put(KeyEvent.VK_UP, "UP");
        NAME_BY_KEY_CODE.put(KeyEvent.VK_DOWN, "DOWN");
        NAME_BY_KEY_CODE.put(KeyEvent.VK_LEFT, "LEFT");
        NAME_BY_KEY_CODE.put(KeyEvent.VK_RIGHT, "RIGHT");
        NAME_BY_KEY_CODE.put(KeyEvent.VK_SHIFT, "SHIFT");
        NAME_BY_KEY_CODE.put(KeyEvent.VK_TAB, "TAB");
        NAME_BY_KEY_CODE.put(KeyEvent.VK_ESCAPE, "ESCAPE");
        NAME_BY_KEY_CODE.put(KeyEvent.VK_ENTER, "ENTER");
        NAME_BY_KEY_CODE.put(KeyEvent.VK_BACK_SPACE, "BACKSPACE");
        NAME_BY_KEY_CODE.put(KeyEvent.VK_SPACE, "SPACE");
    }

    /** Every key name {@link #keysDown()} can contain: {@code A}-{@code Z}, {@code 0}-{@code 9}, and the named keys. */
    public static final Set<String> REPORTED_KEY_NAMES =
            Collections.unmodifiableSet(new TreeSet<>(NAME_BY_KEY_CODE.values()));

    private final Set<String> held = new TreeSet<>();
    private volatile boolean open = true;
    private JFrame frame;
    private JLabel status;
    private KeyEventDispatcher dispatcher;

    private KeyboardWindow() {
    }

    /**
     * The name this window reports a key under.
     *
     * @param keyCode an AWT virtual key code ({@code KeyEvent.VK_...})
     * @return the name, or {@code null} for a key this window ignores
     */
    public static String keyName(int keyCode) {
        return NAME_BY_KEY_CODE.get(keyCode);
    }

    /**
     * Open the window and give it focus.
     *
     * @param title     the window title
     * @param helpLines shown in the window, one per line: the keymap, typically
     * @return the open window
     * <p>Works from a team's Android unit tests on Windows, macOS and Linux. Android's unit-test task
     * starts the test JVM with {@code java.awt.headless=true}, which would refuse every window, so when this
     * machine has a display and no AWT class has decided otherwise yet, this lifts that flag first (see
     * {@link #looksLikeADesktop}). It cannot lift it once something earlier in the same test JVM has
     * already asked AWT whether it is headless; run the live session in a test of its own.
     *
     * @param title     the window title
     * @param helpLines shown in the window, one per line: the keymap, typically
     * @return the open window
     * @throws IllegalStateException if this JVM has no display (a headless CI machine, or a remote shell)
     */
    public static KeyboardWindow open(final String title, final List<String> helpLines) {
        if ("true".equalsIgnoreCase(System.getProperty("java.awt.headless"))
                && looksLikeADesktop(System.getProperty("os.name"), System.getenv())) {
            System.setProperty("java.awt.headless", "false");
        }
        if (GraphicsEnvironment.isHeadless()) {
            throw new IllegalStateException("there is no display, so there is no keyboard window to drive from."
                    + " Run the live session on a desktop, or replay a recorded track headless instead.");
        }
        final KeyboardWindow window = new KeyboardWindow();
        try {
            SwingUtilities.invokeAndWait(new Runnable() {
                @Override
                public void run() {
                    window.build(title, helpLines);
                }
            });
        } catch (InterruptedException interrupted) {
            Thread.currentThread().interrupt();
            throw new IllegalStateException("interrupted while opening the keyboard window", interrupted);
        } catch (InvocationTargetException failed) {
            throw new IllegalStateException("could not open the keyboard window", failed.getCause());
        }
        return window;
    }

    /**
     * Whether this machine has a screen a window can open on, judged without touching AWT (asking AWT
     * would fix the headless decision for the rest of the JVM). A CI machine never does. Windows and macOS
     * do. Linux and the other Unixes do when an X11 or Wayland display is named in the environment.
     *
     * @param osName      the {@code os.name} system property
     * @param environment the process environment, {@code System.getenv()}
     * @return true if a window can be expected to open
     */
    static boolean looksLikeADesktop(String osName, Map<String, String> environment) {
        if (isSet(environment.get("CI"))) {
            return false;
        }
        String os = osName == null ? "" : osName.toLowerCase(Locale.ROOT);
        if (os.startsWith("windows") || os.startsWith("mac")) {
            return true;
        }
        return isSet(environment.get("DISPLAY")) || isSet(environment.get("WAYLAND_DISPLAY"));
    }

    private static boolean isSet(String value) {
        return value != null && !value.trim().isEmpty();
    }

    /** {@return a copy of the keys held down right now, by name} */
    public Set<String> keysDown() {
        synchronized (held) {
            return new TreeSet<>(held);
        }
    }

    /** {@return false once the person has closed the window} */
    public boolean isOpen() {
        return open;
    }

    /**
     * Replace the one-line status at the bottom of the window.
     *
     * @param line what to show, e.g. the sim time and which gamepad the keys are driving
     */
    public void showStatus(final String line) {
        SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
                if (status != null) {
                    status.setText(line);
                }
            }
        });
    }

    /** Close the window, if the person has not already. Safe to call more than once. */
    public void close() {
        open = false;
        SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
                if (dispatcher != null) {
                    KeyboardFocusManager.getCurrentKeyboardFocusManager().removeKeyEventDispatcher(dispatcher);
                    dispatcher = null;
                }
                if (frame != null) {
                    frame.dispose();
                }
            }
        });
    }

    /** Runs on the Swing thread. */
    private void build(String title, List<String> helpLines) {
        frame = new JFrame(title);
        frame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
        // TAB is a gamepad key here, not "move focus to the next control".
        frame.setFocusTraversalKeysEnabled(false);

        JTextArea help = new JTextArea(String.join("\n", helpLines));
        help.setEditable(false);
        help.setFocusable(false);
        help.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 13));
        status = new JLabel(" ");
        frame.getContentPane().add(help, BorderLayout.CENTER);
        frame.getContentPane().add(status, BorderLayout.SOUTH);

        frame.addWindowListener(new WindowAdapter() {
            @Override
            public void windowClosing(WindowEvent event) {
                open = false;
            }

            @Override
            public void windowClosed(WindowEvent event) {
                open = false;
            }

            @Override
            public void windowDeactivated(WindowEvent event) {
                releaseEveryKey();
            }
        });

        // A dispatcher rather than a listener on one component: it sees every key while this window is
        // focused, whichever part of it has focus, and it can swallow TAB before focus traversal does.
        dispatcher = new KeyEventDispatcher() {
            @Override
            public boolean dispatchKeyEvent(KeyEvent event) {
                if (!frame.isFocused()) {
                    return false;
                }
                String name = keyName(event.getKeyCode());
                if (name == null) {
                    return false;
                }
                synchronized (held) {
                    if (event.getID() == KeyEvent.KEY_PRESSED) {
                        held.add(name);
                    } else if (event.getID() == KeyEvent.KEY_RELEASED) {
                        held.remove(name);
                    }
                }
                return true;
            }
        };
        KeyboardFocusManager.getCurrentKeyboardFocusManager().addKeyEventDispatcher(dispatcher);

        frame.pack();
        frame.setLocationByPlatform(true);
        frame.setVisible(true);
        frame.toFront();
        frame.requestFocus();
    }

    private void releaseEveryKey() {
        synchronized (held) {
            held.clear();
        }
    }
}
