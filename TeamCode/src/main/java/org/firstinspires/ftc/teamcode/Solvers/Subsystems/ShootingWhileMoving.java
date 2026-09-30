package org.firstinspires.ftc.teamcode.Solvers.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.Globals.Robot;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

@Config
public class ShootingWhileMoving extends SubsystemBase {

    private final Robot robot = Robot.getInstance();

    Follower follower;
    PoseFilter velFilter;
    PoseFilter accelFilter;
    public static double offset = 0.4;
    public static double epsilonStopH = 15;
    public static double accelScale = 0.0;
    public ShootingWhileMoving() {
        velFilter = new PoseFilter(9);
        accelFilter = new PoseFilter(7);
        follower = robot.follower;
    }
    private void updateFilteredVelocities() {
        Vector vel = follower.getVelocity();
        double rawVelX = vel.getXComponent();
        double rawVelY = vel.getYComponent();
        velFilter.updateFilteredVelocities(new Pose(rawVelX,rawVelY));
    }
    private void updateFilteredAcceleration() {
        Vector accel = follower.getAcceleration();
        double rawAccelX = accel.getXComponent();
        double rawAccelY = accel.getYComponent();
        velFilter.updateFilteredVelocities(new Pose(rawAccelX,rawAccelY));
    }
    public boolean isMoving() {
        robot.telemetry.addData("abs", follower.getVelocity().getMagnitude());
        return follower.getVelocity().getMagnitude()> epsilonStopH;
    }
    public Pose getExpectedPose() {
        double velX = velFilter.filteredPose.getX();
        double velY = velFilter.filteredPose.getY();
        double accelX = velFilter.filteredPose.getX();
        double accelY = velFilter.filteredPose.getY();
        if(!isMoving())

            {
                velX = 0;
                velY = 0;
                accelX = 0;
                accelY = 0;
            }
        Pose g = robot.GoalPose;
        Pose currentP = robot.CurrentPose;
        double dx = g.getX() - currentP.getX();
        double dy = g.getY() - currentP.getY();
        double distance = Math.sqrt(dx * dx + dy * dy);
        double flyTime = getFlyTime(distance);
        Pose p1 = new Pose(
                g.getX() - velX * flyTime - 0.5*accelX*(flyTime*flyTime)*accelScale,
                g.getY() - velY * flyTime - 0.5*accelY*(flyTime*flyTime)*accelScale
        );
        double dx2 = p1.getX() - currentP.getX();
        double dy2 = p1.getY() - currentP.getY();
        double distance2 = Math.sqrt(dx2 * dx2 + dy2 * dy2);
        double flyTime2 = getFlyTime(distance2);
        return new Pose(
                g.getX() - velX * flyTime2- 0.5*accelX*(flyTime2*flyTime2)*accelScale,
                g.getY() - velY * flyTime2- 0.5*accelY*(flyTime2*flyTime2)*accelScale
        );
    }
    public double getFlyTime(double x) {
        return 0.00513606*x+0.459651 + offset;
    }

    @Override
    public void periodic(){
        updateFilteredVelocities();
        updateFilteredAcceleration();
        robot.profiler.start("Moving Loop");
        robot.PredictedGoalPose = getExpectedPose();
        robot.profiler.end("Moving Loop");
    }
}
