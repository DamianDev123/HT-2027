package org.firstinspires.ftc.teamcode.Solvers.Opmodes;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Globals.Robot;
import org.firstinspires.ftc.teamcode.Solvers.Subsystems.Storage;
@TeleOp
public class gameTele extends OpMode {
    Follower follower;
    private final Robot robot = Robot.getInstance();
    GamepadEx gm1;
    @Override
    public void init() {
        robot.init(hardwareMap,telemetry);
        gm1 = new GamepadEx(gamepad1);
        follower = robot.follower;
    }

    @Override
    public void loop() {

        telemetry.addData("turretAngle",robot.turret.currentAngle);
        if(gm1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.4) robot.schedule(robot.storage.Intake);
        if(gm1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)<0.4) robot.schedule(robot.storage.Outake);
        if(gm1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.4) robot.schedule(robot.storage.IntakeT);
        if(gm1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)<0.4) robot.schedule(robot.storage.OutakeT);
        gm1.getGamepadButton(GamepadKeys.Button.B).whenReleased(robot.turret.calibrate);
        gm1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(robot.storage.SingularShot);
        double forward = gm1.getLeftY();
        double lateral = -gm1.getLeftX();
        double turn = -gm1.getRightX();
        DrivePowers powers = ManualDrive.fieldCentric(
                forward, lateral,
                turn,
                follower.pose().heading()
        );
        follower.manual(powers);
        robot.updateLoop();
    }
}
