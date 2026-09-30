package org.firstinspires.ftc.teamcode.Solvers.Opmodes;

import static org.firstinspires.ftc.teamcode.Globals.Constants.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class AlliancePoseSelector extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addData("Cross / Triangle", "Blue");
        telemetry.addData("Circle / Square", "Red");
        telemetry.addData("Alliance Color", ALLIANCE_COLOR);

        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad1.cross || gamepad2.cross || gamepad1.triangle || gamepad2.triangle) {
                ALLIANCE_COLOR = "BLUE";
            } else if (gamepad1.circle || gamepad2.circle || gamepad1.square || gamepad2.square) {
                ALLIANCE_COLOR = "RED";
            }

            telemetry.addData("Cross / Triangle", "Blue");
            telemetry.addData("Circle / Square", "Red");
            telemetry.addData("Alliance Color", ALLIANCE_COLOR);

            telemetry.update();
        }
    }
}