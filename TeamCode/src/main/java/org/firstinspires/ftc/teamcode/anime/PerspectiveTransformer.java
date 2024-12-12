package org.firstinspires.ftc.teamcode.anime;

import android.graphics.Matrix;
import com.acmerobotics.roadrunner.Vector2d;

public class PerspectiveTransformer {

    // http://graphics.cs.cmu.edu/courses/15-463/2008_fall/Papers/proj.pdf
    // http://docs-hoffmann.de/persprect13052005.pdf
    // https://math.stackexchange.com/questions/13404/mapping-irregular-quadrilateral-to-a-rectangle
    // https://naadispeaks.blog/2021/08/31/perspective-transformation-of-coordinate-points-on-polygons/

    private Matrix matrix;
    public PerspectiveTransformer(float[] srcRect,
                                  float[] destQuad) {
        matrix = new Matrix();
        matrix.setPolyToPoly(
                srcRect, 0, destQuad, 0, 4);
    }

    public Vector2d transformPoint(float x, float y) {
        // Create a PointF object for the input point
        float[] srcPoint = new float[]{x, y};
        float[] destPoint = new float[]{0, 0};

        // Apply the transformation
        matrix.mapPoints(destPoint,srcPoint);

        return new Vector2d(destPoint[0],-destPoint[1]); // Add -ve sign to trans form the coordinate to +x, -y (4th quadrant)
    }

    public void main(String[] args) { // For testing purposes (not for Android)
        // Example usage:
        float x = 100;
        float y = 50;

        // Example source rectangle coordinates
        float[] srcRect = {0, 0, 200, 0, 200, 100, 0, 100};
        // Example destination quadrilateral coordinates
        float[] destQuad = {10, 20, 180, 30, 170, 120, 20, 110};

        PerspectiveTransformer perspectiveTransformer = new PerspectiveTransformer(srcRect, destQuad);

        Vector2d transformedPoint = perspectiveTransformer.transformPoint(x, y);

        System.out.println("Transformed Point: " + transformedPoint.x + ", " + transformedPoint.y);
    }
}

