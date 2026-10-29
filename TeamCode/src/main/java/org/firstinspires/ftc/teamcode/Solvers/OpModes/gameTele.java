package org.firstinspires.ftc.teamcode.Solvers.Opmodes;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Globals.Robot;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Storage;

public class gameTele extends OpMode {
    Follower follower;
    private final Robot robot = Robot.getInstance();
    GamepadEx gm1;
    @Override
    public void init() {
        robot.init(hardwareMap);
        gm1 = new GamepadEx(gamepad1);
        follower = Robot.follower;
    }

    @Override
    public void loop() {
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                follower.pose().heading()
        );
        ManualDrive.driveOrHold(follower, powers);
        if(gm1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.4) robot.schedule(robot.storage.Intake);
        if(gm1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)<0.4) robot.schedule(robot.storage.Outake);
        gm1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(robot.storage.Shoot());
        robot.updateLoop();
    }
}
