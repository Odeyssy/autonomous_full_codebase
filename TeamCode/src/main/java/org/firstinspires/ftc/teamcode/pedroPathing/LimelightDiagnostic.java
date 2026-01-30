package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Limelight Aligner Diagnostic", group = "Diagnostics")
public class LimelightDiagnostic extends LinearOpMode {

    private LimelightAligner aligner;

    @Override
    public void runOpMode() {
        // Initialize the aligner
        aligner = new LimelightAligner(hardwareMap);

        telemetry.addLine("--- Limelight Diagnostic ---");
        telemetry.addLine("Point the robot at a Blue (20) or Red (24) tag.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // We pass 0.0 because we want to see what the aligner does without stick input
            double calculatedPower = aligner.calculateAlignPower(0.0);

            // Display Results
            telemetry.addData("Status", "Running");
            telemetry.addData("Calculated Turn Power", "%.3f", calculatedPower);

            if (calculatedPower == 0) {
                telemetry.addLine("RESULT: [ CENTERED ] or [ NO TAG FOUND ]");
            } else if (calculatedPower > 0) {
                telemetry.addLine("RESULT: [ TURNING LEFT ]");
            } else {
                telemetry.addLine("RESULT: [ TURNING RIGHT ]");
            }

            telemetry.addLine("\nCheck the Limelight Web View (IP: 192.168.43.11:5801)");
            telemetry.update();
        }

        aligner.stop();
    }
}