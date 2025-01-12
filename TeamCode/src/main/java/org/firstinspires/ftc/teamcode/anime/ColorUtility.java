package org.firstinspires.ftc.teamcode.anime;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import java.util.HashMap;
import java.util.Map;

public class ColorUtility {

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
        return getColorName(
                (int)colors.red*255,
                (int)colors.green*255,
                (int)colors.blue*255
        );
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
