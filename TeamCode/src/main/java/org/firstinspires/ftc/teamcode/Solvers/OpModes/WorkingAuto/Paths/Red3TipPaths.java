
package org.firstinspires.ftc.teamcode.Solvers.Opmodes.WorkingAuto.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class Red3TipPaths {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(30.7116, 8.3142, -90);

    private final Pose toIntakePoint1 = poseFactory.of(6.6312, 11.1554, -90);
    private final Pose toIntakePoint1Control1 = poseFactory.of(28.6545, 43.1264, 0);

    private final Pose toShootingPoint1 = poseFactory.of(34.698, 125.0407, -90);
    private final Pose toShootingPoint1Control1 = poseFactory.of(20.5864, 76.2825, 0);
    private final Pose toShootingPoint1Control2 = poseFactory.of(32.2944, 37.4262, 0);

    private final Pose toIntakePoint2Start = poseFactory.of(34.698, 125.0407, -90);
    private final Pose toIntakePoint2 = poseFactory.of(10.2889, 106.5622, 180);
    private final Pose toIntakePoint2Control1 = poseFactory.of(35.4259, 105.8494, 0);

    private final Pose toShootPoint2 = poseFactory.of(52.0531, 117.9695, -164.723);
    private final Pose toPark = poseFactory.of(5.7777, 104.0834, -163.2968);

    public Pose getStartPose() {
        return start;
    }

    public Path toIntakePoint1() {
        return com.pedropathing.api.Paths
                .curve(start, toIntakePoint1Control1, toIntakePoint1)
                .constant(toIntakePoint1);
    }

    public Path toShootingPoint1() {
        return com.pedropathing.api.Paths
                .curve(toIntakePoint1, toShootingPoint1Control1,
                        toShootingPoint1Control2, toShootingPoint1)
                .constant(toShootingPoint1);
    }

    public Path toIntakePoint2() {
        return com.pedropathing.api.Paths
                .curve(toIntakePoint2Start, toIntakePoint2Control1, toIntakePoint2)
                .linear(toIntakePoint2Start, toIntakePoint2);
    }

    public Path toShootPoint2() {
        return com.pedropathing.api.Paths
                .line(toIntakePoint2, toShootPoint2)
                .reverseTangent();
    }

    public Path toPark() {
        return com.pedropathing.api.Paths
                .line(toShootPoint2, toPark)
                .tangent();
    }
}
