
package org.firstinspires.ftc.teamcode.Solvers.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Globals.Robot;
import org.firstinspires.ftc.teamcode.helpers.PoseFilter;

@Configurable
public class ShootingWhileMoving extends SubsystemBase {

    private final Robot robot = Robot.getInstance();

    Follower follower;
    PoseFilter velFilter;
    PoseFilter accelFilter;

    private Pose previousVelocity;
    private long previousTime;

    public static double offset = 0.4;
    public static double epsilonStopH = 15;

    public static double accelScale = 1.0;

    public ShootingWhileMoving() {
        velFilter = new PoseFilter(9);
        accelFilter = new PoseFilter(7);

        follower = robot.follower;

        previousTime = System.nanoTime();
    }

    private void updateFilteredVelocities() {
        Velocity velocity = follower.velocity();

        Pose currentVelocity = new Pose(
                velocity.vx,
                velocity.vy,
                velocity.omega
        );

        velFilter.updateFilteredPose(currentVelocity);

        // Calculate acceleration using velocity change over time
        long currentTime = System.nanoTime();

        double dt = (currentTime - previousTime) / 1e9;

        if (previousVelocity != null && dt > 0) {

            Pose acceleration = new Pose(
                    (currentVelocity.x() - previousVelocity.x()) / dt,
                    (currentVelocity.y() - previousVelocity.y()) / dt,
                    (currentVelocity.heading() - previousVelocity.heading()) / dt
            );

            accelFilter.updateFilteredPose(acceleration);
        }

        previousVelocity = currentVelocity;
        previousTime = currentTime;
    }

    public boolean isMoving() {
        double mag = follower.velocity().toVector().magnitude();

        robot.telemetry.addData("abs", mag);

        return mag > epsilonStopH;
    }

    public Pose getExpectedPose(Pose pose) {

        Pose velocity = velFilter.filteredPose;
        Pose acceleration = accelFilter.filteredPose;

        if (!isMoving()) {
            velocity = new Pose(0, 0, 0);
            acceleration = new Pose(0, 0, 0);
        }

        Pose goal = robot.getHive();

        double dx = goal.x() - pose.x();
        double dy = goal.y() - pose.y();

        double distance = Math.sqrt(dx * dx + dy * dy);

        double flyTime = getFlyTime(distance);

        // Velocity compensation
        double predictedX = velocity.x() * flyTime;
        double predictedY = velocity.y() * flyTime;

        // Acceleration compensation
        double accelX = 0.5 * acceleration.x()
                * flyTime * flyTime * accelScale;

        double accelY = 0.5 * acceleration.y()
                * flyTime * flyTime * accelScale;

        // Predict robot position when projectile reaches target
        return new Pose(
                pose.x() - predictedX - accelX,
                pose.y() - predictedY - accelY,
                pose.heading()
        );
    }

    public double getFlyTime(double x) {
        return 0.00513606 * x + 0.459651 + offset;
    }

    @Override
    public void periodic() {
        updateFilteredVelocities();
    }
}
