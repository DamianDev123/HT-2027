package org.firstinspires.ftc.teamcode.Solvers.Opmodes;

import static org.firstinspires.ftc.teamcode.Globals.Constants.zoneType;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Globals.Constants.ZoneType;

@TeleOp
public class ZonePoseSelector extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addData("Cross / Triangle", "CloseZone");
        telemetry.addData("Circle / Square", "FarZone");
        telemetry.addData("Alliance Color", zoneType);

        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad1.cross || gamepad2.cross || gamepad1.triangle || gamepad2.triangle) {
                zoneType = ZoneType.Closezone;
            } else if (gamepad1.circle || gamepad2.circle || gamepad1.square || gamepad2.square) {
                zoneType = ZoneType.Farzone;
            }

            telemetry.addData("Cross / Triangle", "CloseZone");
            telemetry.addData("Circle / Square", "FarZone");
            telemetry.addData("Alliance Color", zoneType);

            telemetry.update();
        }
    }
}