package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Diagnostic: Ramp Motor", group = "Diagnostics")
public class ramp_diagnostic extends LinearOpMode {

    private DcMotorEx rampmotor;

    @Override
    public void runOpMode() {
        // --- HARDWARE MAPPING ---
        rampmotor = hardwareMap.get(DcMotorEx.class, "rampmotor");

        // Match your original configuration
        rampmotor.setDirection(DcMotorEx.Direction.REVERSE);
        rampmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoder for fresh diagnostic data
        rampmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rampmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized. Press Play to start test.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- CONTROL LOGIC ---
            // Left Stick Y: Raw Power control (Test mechanical movement)
            double manualPower = -gamepad1.left_stick_y;

            // Right Stick Y: Velocity control (Test encoder/PID)
            double targetVelocity = -gamepad1.right_stick_y * 1500; // Scaled to ~1500 ticks/sec

            if (Math.abs(manualPower) > 0.05) {
                rampmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rampmotor.setPower(manualPower);
            } else if (Math.abs(targetVelocity) > 50) {
                rampmotor.setVelocity(targetVelocity);
            } else {
                rampmotor.setPower(0);
            }

            // --- DIAGNOSTIC TELEMETRY ---
            telemetry.addLine("--- Ramp Motor Diagnostics ---");
            telemetry.addData("Controls", "L-Stick: Power | R-Stick: Velocity");
            telemetry.addLine("------------------------------");
            telemetry.addData("Power (Set)", "%.2f", rampmotor.getPower());
            telemetry.addData("Velocity (Actual)", "%.2f ticks/s", rampmotor.getVelocity());
            telemetry.addData("Position (Ticks)", rampmotor.getCurrentPosition());

            // Current draw is great for finding mechanical binding
            telemetry.addData("Current Draw", "%.2f Amps", rampmotor.getCurrent(CurrentUnit.AMPS));

            // Check for encoder health
            if (rampmotor.getPower() > 0.2 && Math.abs(rampmotor.getVelocity()) < 5) {
                telemetry.addLine("!!! WARNING: MOTOR POWERED BUT NO VELOCITY (Check Encoder/Cables) !!!");
            }

            telemetry.update();
        }
    }
}