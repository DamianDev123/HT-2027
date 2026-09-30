package org.firstinspires.ftc.teamcode.Solvers.Opmodes;

import static org.firstinspires.ftc.teamcode.Globals.Constants.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.Globals.Robot;

@TeleOp
public class ResetEncoders extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        DcMotor turretEnc = hardwareMap.dcMotor.get("Intake");

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            turretEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}