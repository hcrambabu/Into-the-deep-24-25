package org.firstinspires.ftc.teamcode.anime;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.Range;

import java.util.HashMap;
import java.util.Map;
import java.util.logging.Logger;

public class ColorUtility {

    private static Logger log = Logger.getLogger(ColorUtility.class.getName());

    public enum Color {
        UNKNOWN, RED, BLUE, YELLOW, GRAY
    }
    private static Map<Color, int[]> colors = new HashMap<>();
    static {
        colors.put(Color.RED, new int[]{255, 0, 0});
        colors.put(Color.BLUE, new int[]{0, 0, 255});
        colors.put(Color.YELLOW, new int[]{255, 255, 0});
        colors.put(Color.GRAY, new int[]{128, 128, 128});
    }

    public static Color getColorName(NormalizedRGBA colors) {
//        float scale = 256; int min = 0, max = 255;
//        return getColorName(
//                Range.clip((int)((colors.red   * scale * scale)), min, max),
//                Range.clip((int)((colors.red   * scale * scale)), min, max),
//                Range.clip((int)((colors.red   * scale * scale)), min, max)
//        );
        return getColorName_v2(colors.red, colors.green, colors.blue);
    }

    public static Color getColorName_v2(double r, double g, double b) {
        if(r > g && r > b) {
            return Color.RED;
        } else if(b > r && b > g) {
            return Color.BLUE;
        } else if(g > r && r > b) {
           return Color.YELLOW;
        } else {
            return Color.GRAY;
        }
    }

    public static Color getColorName(int r, int g, int b) {
        int threshold = 50; // Adjust as needed

        double minDistance = Double.MAX_VALUE;
        Color closestColor = Color.UNKNOWN;

        for (Map.Entry<Color, int[]> entry : colors.entrySet()) {
            Color name = entry.getKey();
            int[] rgb = entry.getValue();
            double distance = calculateDistance(r, g, b, rgb);
            if (distance < minDistance) {
                minDistance = distance;
                closestColor = name;
            }
        }

        if (minDistance <= threshold) {
            return closestColor;
        } else {
            return Color.UNKNOWN;
        }
    }

    private static double calculateDistance(int r1, int g1, int b1, int[] rgb) {
        int r2 = rgb[0];
        int g2 = rgb[1];
        int b2 = rgb[2];
        return Math.sqrt(Math.pow(r1 - r2, 2) + Math.pow(g1 - g2, 2) + Math.pow(b1 - b2, 2));
    }
}
