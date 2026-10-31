package org.firstinspires.ftc.teamcode.Solvers.Opmodes.WorkingAuto;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Globals.Robot;

public class Red3TipAuto extends OpMode {
    Follower follower;
    private final Robot robot = Robot.getInstance();
    @Override
    public void init() {
        robot.init(hardwareMap);
        follower = robot.follower;
        robot.schedule(
                new SequentialCommandGroup(

                )
        );
    }

    @Override
    public void loop() {

    }
}
