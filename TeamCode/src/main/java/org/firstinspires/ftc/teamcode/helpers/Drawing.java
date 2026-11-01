package org.firstinspires.ftc.teamcode.helpers;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.math.Pose;

public class Drawing {

    private static final double ROBOT_WIDTH = 16.0;
    private static final double ROBOT_LENGTH = 18.0;
    private static final double HEADING_LENGTH = 12.0;
    private static final double TARGET_RADIUS = 3.0;
    private static final int TARGET_SEGMENTS = 24;
    private static final FieldManager panelsField =
            PanelsField.INSTANCE.getField();
    private static final Style robotStyle =
            new Style("", "#2196F3", 0.0);
    private static final Style targetStyle =
            new Style("", "#F44336", 0.0);
    private static final Style lineStyle =
            new Style("", "#4CAF50", 0.0);
    /**
     * Initialize Panels field.
     */
    public static void init() {
        panelsField.setOffsets(
                PanelsField.INSTANCE.getPresets().getPEDRO_PATHING()
        );
    }

    /**
     * Draw a circle using connected line segments.
     */
    public static void drawCircle(
            double x,
            double y,
            double radius,
            int segments
    ) {
        if (segments < 3 || radius <= 0) return;

        double angleStep = (2.0 * Math.PI) / segments;

        double previousX = x + radius;
        double previousY = y;

        for (int i = 1; i <= segments; i++) {
            double angle = i * angleStep;

            double nextX = x + radius * Math.cos(angle);
            double nextY = y + radius * Math.sin(angle);

            panelsField.moveCursor(previousX, previousY);
            panelsField.line(nextX, nextY);

            previousX = nextX;
            previousY = nextY;
        }
    }

    /**
     * Draw a rotated rectangle.
     */
    public static void drawRectangle(
            double x,
            double y,
            double width,
            double length,
            double heading
    ) {
        double halfWidth = width / 2.0;
        double halfLength = length / 2.0;

        // Local rectangle corners
        double[][] corners = {
                {-halfLength, -halfWidth},
                { halfLength, -halfWidth},
                { halfLength,  halfWidth},
                {-halfLength,  halfWidth}
        };

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double[] worldX = new double[4];
        double[] worldY = new double[4];

        // Rotate and translate corners
        for (int i = 0; i < 4; i++) {
            double localX = corners[i][0];
            double localY = corners[i][1];

            worldX[i] = x + localX * cos - localY * sin;
            worldY[i] = y + localX * sin + localY * cos;
        }

        // Connect corners
        for (int i = 0; i < 4; i++) {
            int next = (i + 1) % 4;

            panelsField.moveCursor(worldX[i], worldY[i]);
            panelsField.line(worldX[next], worldY[next]);
        }
    }

    /**
     * Draw robot position and heading.
     */
    public static void drawRobot(Pose pose) {
        double x = pose.x();
        double y = pose.y();
        double heading = pose.heading();

        panelsField.setStyle(robotStyle);

        // Robot outline
        drawRectangle(
                x,
                y,
                ROBOT_WIDTH,
                ROBOT_LENGTH,
                heading
        );

        // Robot heading
        double headingX =
                x + HEADING_LENGTH * Math.cos(heading);

        double headingY =
                y + HEADING_LENGTH * Math.sin(heading);

        panelsField.moveCursor(x, y);
        panelsField.line(headingX, headingY);
    }

    /**
     * Draw target position.
     */
    public static void drawTarget(Pose pose) {
        panelsField.setStyle(targetStyle);

        drawCircle(
                pose.x(),
                pose.y(),
                TARGET_RADIUS,
                TARGET_SEGMENTS
        );
    }

    /**
     * Draw a line between two poses.
     */
    public static void drawLine(Pose start, Pose end) {
        panelsField.setStyle(lineStyle);

        panelsField.moveCursor(start.x(), start.y());
        panelsField.line(end.x(), end.y());
    }

    /**
     * Update Panels.
     */
    public static void update() {
        panelsField.update();
    }

    /**
     * Draw robot and update Panels.
     */
    public static void draw(Pose pose) {
        drawRobot(pose);
        update();
    }
}