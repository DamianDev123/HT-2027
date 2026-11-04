
package org.firstinspires.ftc.teamcode.Solvers.Opmodes.WorkingAuto;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Globals.Robot;
import org.firstinspires.ftc.teamcode.Solvers.Opmodes.WorkingAuto.Paths.Red3TipPaths;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Storage;

@Autonomous(name = "Red 3 Tip Auto", group = "Red")
public class Red3TipAuto extends CommandOpMode {

    private final Robot robot = Robot.getInstance();

    private Follower follower;
    private Storage storage;
    private final Red3TipPaths paths = new Red3TipPaths();

    @Override
    public void initialize() {
        robot.init(hardwareMap, telemetry);

        follower = robot.follower;
        storage = new Storage();

        // Set starting position
        follower.setPose(paths.getStartPose());

        schedule(new SequentialCommandGroup(
                new WaitCommand(2500),
                // Shoot preload
                storage.shootCommand(),
                new WaitCommand(2500),
                storage.stopShootingCommand(),

                // First pickup
                storage.Intake,
                new FollowPathCommand(follower, paths.toIntakePoint1()),
                new WaitCommand(500),
                storage.Outake,

                // First shooting position
                new FollowPathCommand(follower, paths.toShootingPoint1()),
                storage.shootCommand(),
                new WaitCommand(2500),
                storage.stopShootingCommand(),

                // Second pickup
                storage.Intake,
                new FollowPathCommand(follower, paths.toIntakePoint2()),
                new WaitCommand(500),
                storage.Outake,

                // Second shooting position
                new FollowPathCommand(follower, paths.toShootPoint2()),
                storage.shootCommand(),
                new WaitCommand(2500),
                storage.stopShootingCommand(),

                // Park
                new FollowPathCommand(follower, paths.toPark())
        ));
    }

    @Override
    public void run() {
        super.run();
        follower.update();
        CommandScheduler.getInstance().run();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Storage Full", Storage.full);
        telemetry.update();
        robot.updateLoop();
    }

    @Override
    public void end() {
        robot.intakeMotor.set(0);
        robot.transferMotor.set(0);
    }
}
