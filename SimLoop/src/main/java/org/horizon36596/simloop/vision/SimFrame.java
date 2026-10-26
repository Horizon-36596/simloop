package org.horizon36596.simloop.vision;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.charset.Charset;
import java.util.zip.CRC32;
import java.util.zip.Deflater;

/**
 * One synthetic camera frame from a {@link SimFrameRenderer}: 8-bit RGB pixels, plus a label per pixel
 * saying which piece or tag it shows. <b>It is a drawing, not a photograph</b>: flat colours, one light, no
 * shadows, no blur, no compression. Use it to drive a real vision pipeline's code paths and check its
 * geometry, never to tune a colour threshold. The PNG it writes says so in a text chunk.
 *
 * <p><b>Pixel layout.</b> Row-major from the top-left, {@code +x} right, {@code +y} down. {@link #getRgbBytes()}
 * is interleaved {@code R, G, B}, three bytes a pixel, which is exactly the buffer an OpenCV
 * {@code CV_8UC3} {@code Mat} of {@code height} rows and {@code width} columns holds:
 * <pre>
 *   Mat rgb = new Mat(frame.getHeightPx(), frame.getWidthPx(), CvType.CV_8UC3);
 *   rgb.put(0, 0, frame.getRgbBytes());
 *   processor.processFrame(rgb, captureTimeNanos);      // your real VisionProcessor
 * </pre>
 * {@link #getRgbaBytes()} gives the four-channel form, alpha 255, for a pipeline that expects
 * {@code CV_8UC4}. SimLoop itself never touches OpenCV.
 *
 * <p>Immutable: every accessor that returns an array returns a copy.
 */
public final class SimFrame {

    /** The text the PNG's {@code tEXt} chunk carries, so a saved frame cannot pass for a camera image. */
    public static final String PNG_COMMENT = "SimLoop synthetic frame, rendered from sim geometry. Not a camera"
            + " image: do not tune colour thresholds on it.";

    private static final byte[] PNG_SIGNATURE = {(byte) 0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'};
    private static final Charset LATIN_1 = Charset.forName("ISO-8859-1");

    private final int widthPx;
    private final int heightPx;
    private final long frameNumber;
    private final byte[] rgb;
    private final int[] pieceIds;
    private final int[] tagIds;

    SimFrame(int widthPx, int heightPx, long frameNumber, byte[] rgb, int[] pieceIds, int[] tagIds) {
        this.widthPx = widthPx;
        this.heightPx = heightPx;
        this.frameNumber = frameNumber;
        this.rgb = rgb;
        this.pieceIds = pieceIds;
        this.tagIds = tagIds;
    }

    /** {@return the image width, pixels} */
    public int getWidthPx() {
        return widthPx;
    }

    /** {@return the image height, pixels} */
    public int getHeightPx() {
        return heightPx;
    }

    /** {@return the renderer's count of frames before this one, from 0: the sensor-noise stream it used} */
    public long getFrameNumber() {
        return frameNumber;
    }

    /** {@return the pixels as interleaved R, G, B bytes, row-major from the top-left; a fresh copy} */
    public byte[] getRgbBytes() {
        return rgb.clone();
    }

    /** {@return the pixels as interleaved R, G, B, A bytes with A = 255, row-major; a fresh copy} */
    public byte[] getRgbaBytes() {
        byte[] rgba = new byte[widthPx * heightPx * 4];
        for (int i = 0, j = 0; i < rgb.length; i += 3, j += 4) {
            rgba[j] = rgb[i];
            rgba[j + 1] = rgb[i + 1];
            rgba[j + 2] = rgb[i + 2];
            rgba[j + 3] = (byte) 0xFF;
        }
        return rgba;
    }

    /**
     * One pixel's colour.
     *
     * @param x column, 0 to width - 1
     * @param y row, 0 to height - 1
     * @return {@code 0xRRGGBB}
     */
    public int getRgb(int x, int y) {
        int i = index(x, y) * 3;
        return ((rgb[i] & 0xFF) << 16) | ((rgb[i + 1] & 0xFF) << 8) | (rgb[i + 2] & 0xFF);
    }

    /**
     * Which piece a pixel shows: ground truth, from the ray through the pixel's centre.
     *
     * @param x column
     * @param y row
     * @return the piece id, or -1 when the pixel shows no piece
     */
    public int getPieceIdAt(int x, int y) {
        return pieceIds[index(x, y)];
    }

    /**
     * Which tag a pixel shows, white border included: ground truth, from the ray through the pixel's centre.
     *
     * @param x column
     * @param y row
     * @return the tag id, or -1 when the pixel shows no tag
     */
    public int getTagIdAt(int x, int y) {
        return tagIds[index(x, y)];
    }

    /**
     * How many pixels show a piece. 0 for a piece the camera cannot see, whether it is out of frame or
     * hidden behind something.
     *
     * @param pieceId the piece's id
     * @return the pixel count
     */
    public int pixelCountOfPiece(int pieceId) {
        int count = 0;
        for (int id : pieceIds) {
            if (id == pieceId) {
                count++;
            }
        }
        return count;
    }

    /**
     * The mean position of the pixels that show a piece: where a perfect blob detector would put its
     * centre, which moves toward the visible part when the piece is partly hidden or cut off by the edge.
     *
     * @param pieceId the piece's id
     * @return {@code {x, y}} in pixels, measured to pixel centres ({@code x + 0.5}), or {@code null} when no
     *         pixel shows the piece
     */
    public double[] centroidOfPiece(int pieceId) {
        double sumX = 0.0;
        double sumY = 0.0;
        int count = 0;
        for (int y = 0; y < heightPx; y++) {
            for (int x = 0; x < widthPx; x++) {
                if (pieceIds[y * widthPx + x] == pieceId) {
                    sumX += x + 0.5;
                    sumY += y + 0.5;
                    count++;
                }
            }
        }
        return count == 0 ? null : new double[] {sumX / count, sumY / count};
    }

    /**
     * The frame as a PNG: 8-bit RGB, no interlace, with a {@code tEXt} {@code Comment} chunk holding
     * {@link #PNG_COMMENT}. Written with {@code java.util.zip} only, so it works wherever SimLoop runs.
     *
     * @param out where to write; not closed
     * @throws IOException when {@code out} throws
     */
    public void writePng(OutputStream out) throws IOException {
        out.write(PNG_SIGNATURE);
        ByteArrayOutputStream header = new ByteArrayOutputStream();
        writeInt(header, widthPx);
        writeInt(header, heightPx);
        header.write(8);                                    // bit depth
        header.write(2);                                    // colour type: truecolour RGB
        header.write(0);                                    // deflate
        header.write(0);                                    // adaptive filtering, filter type per row
        header.write(0);                                    // no interlace
        writeChunk(out, "IHDR", header.toByteArray());

        ByteArrayOutputStream text = new ByteArrayOutputStream();
        text.write("Comment".getBytes(LATIN_1));
        text.write(0);
        text.write(PNG_COMMENT.getBytes(LATIN_1));
        writeChunk(out, "tEXt", text.toByteArray());

        int stride = widthPx * 3;
        byte[] raw = new byte[(stride + 1) * heightPx];
        for (int y = 0; y < heightPx; y++) {
            raw[y * (stride + 1)] = 0;                      // filter: none
            System.arraycopy(rgb, y * stride, raw, y * (stride + 1) + 1, stride);
        }
        Deflater deflater = new Deflater(Deflater.DEFAULT_COMPRESSION);
        try {
            deflater.setInput(raw);
            deflater.finish();
            ByteArrayOutputStream compressed = new ByteArrayOutputStream();
            byte[] buffer = new byte[64 * 1024];
            while (!deflater.finished()) {
                int n = deflater.deflate(buffer);
                compressed.write(buffer, 0, n);
            }
            writeChunk(out, "IDAT", compressed.toByteArray());
        } finally {
            deflater.end();
        }
        writeChunk(out, "IEND", new byte[0]);
    }

    /**
     * {@link #writePng} into a byte array.
     *
     * @return the PNG file's bytes
     */
    public byte[] toPng() {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        try {
            writePng(out);
        } catch (IOException impossible) {
            throw new IllegalStateException("a ByteArrayOutputStream does not throw", impossible);
        }
        return out.toByteArray();
    }

    private static void writeChunk(OutputStream out, String type, byte[] data) throws IOException {
        byte[] typeBytes = type.getBytes(LATIN_1);
        ByteArrayOutputStream length = new ByteArrayOutputStream(4);
        writeInt(length, data.length);
        out.write(length.toByteArray());
        out.write(typeBytes);
        out.write(data);
        CRC32 crc = new CRC32();
        crc.update(typeBytes);
        crc.update(data);
        ByteArrayOutputStream crcBytes = new ByteArrayOutputStream(4);
        writeInt(crcBytes, (int) crc.getValue());
        out.write(crcBytes.toByteArray());
    }

    private static void writeInt(ByteArrayOutputStream out, int value) {
        out.write((value >>> 24) & 0xFF);
        out.write((value >>> 16) & 0xFF);
        out.write((value >>> 8) & 0xFF);
        out.write(value & 0xFF);
    }

    private int index(int x, int y) {
        if (x < 0 || x >= widthPx || y < 0 || y >= heightPx) {
            throw new IndexOutOfBoundsException("pixel (" + x + ", " + y + ") is outside the " + widthPx + "x"
                    + heightPx + " frame");
        }
        return y * widthPx + x;
    }

    @Override
    public String toString() {
        return "SimFrame(" + widthPx + "x" + heightPx + ", frame " + frameNumber + ", synthetic)";
    }
}
