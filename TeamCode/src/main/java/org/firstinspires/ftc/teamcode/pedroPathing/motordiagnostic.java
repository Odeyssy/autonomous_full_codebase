package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Motor Encoder Diagnostic", group = "Test")
public class motordiagnostic extends LinearOpMode {

    @Override
    public void runOpMode() {
        // 1. Initialize Motors using your exact hardware names
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        // 2. Set Directions (Based on your last update)
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // 3. Reset Encoders to zero at start
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // 4. Set to run using encoders
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("--- MOTOR TEST (Hold Button) ---");
        telemetry.addLine("Y: LF | B: RF");
        telemetry.addLine("X: LR | A: RR");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Control motors with buttons (50% power)
            if (gamepad1.y) leftFront.setPower(0.5); else leftFront.setPower(0);
            if (gamepad1.x) leftRear.setPower(0.5); else leftRear.setPower(0);
            if (gamepad1.b) rightFront.setPower(0.5); else rightFront.setPower(0);
            if (gamepad1.a) rightRear.setPower(0.5); else rightRear.setPower(0);

            // Telemetry: Motor Power & Encoder Ticks
            telemetry.addLine("--- Motor Status ---");
            telemetry.addData("LF (Y) Ticks", leftFront.getCurrentPosition());
            telemetry.addData("LR (X) Ticks", leftRear.getCurrentPosition());
            telemetry.addData("RF (B) Ticks", rightFront.getCurrentPosition());
            telemetry.addData("RR (A) Ticks", rightRear.getCurrentPosition());

            // Helpful Direction Check
            telemetry.addLine("\n--- Logic Check ---");
            telemetry.addLine("All Ticks should INCREASE when wheel spins FORWARD.");

            telemetry.update();
        }
    }
}