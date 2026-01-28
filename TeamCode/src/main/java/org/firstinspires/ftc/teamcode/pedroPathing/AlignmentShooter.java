package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

public class AlignmentShooter {

    // Hardware - Drivetrain
    private DcMotor rightFront, rightRear, leftFront, leftRear;

    // Hardware - Shooting System
    private DcMotorEx intakeMotor;
    private DcMotorEx shooterMotor1, shooterMotor2;
    private DcMotorEx rampMotor;
    private CRServo crServofrontR;
    private Servo gateServo;

    // Hardware - Vision
    private Limelight3A limelight;

    // Alignment constants
    private int targetTagID = 20;
    private double alignmentTolerance = 2.0;
    private double alignmentTurnPower = 0.3;
    private double alignmentSlowPower = 0.15;
    private double alignmentSlowZone = 10.0;

    // Shooting constants
    private double shooterVelocity = 1600.0;
    private double intakeVelocity = 2800.0;
    private double rampVelocity = 1500.0;
    private double gateOpenPosition = 0.5;
    private double gateClosedPosition = 0.1;

    // Status tracking
    private boolean aligned = false;
    private boolean tagFound = false;
    private boolean shooting = false;
    private double currentOffset = 0.0;
    private String status = "Idle";

    /**
     * Constructor
     */
    public AlignmentShooter(HardwareMap hardwareMap) {
        // Initialize drivetrain
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize shooting system
        intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "ShooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "ShooterMotor2");
        rampMotor = hardwareMap.get(DcMotorEx.class, "rampmotor");
        crServofrontR = hardwareMap.get(CRServo.class, "crServofrontR");
        gateServo = hardwareMap.get(Servo.class, "GateServo");

        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        rampMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rampMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        crServo.setDirection(DcMotorSimple.Direction.REVERSE);

        // Keep gate CLOSED initially
        gateServo.setPosition(gateClosedPosition);

        // START SHOOTER MOTORS IMMEDIATELY
        shooterMotor1.setVelocity(shooterVelocity);
        shooterMotor2.setVelocity(shooterVelocity);

        // Initialize limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    /**
     * Execute full align and shoot sequence
     * Call this in a loop - returns true when complete
     */
    public boolean executeAlignAndShoot() {
        // Step 1: Align to target
        if (!aligned) {
            aligned = alignToTarget();
            if (!aligned) {
                // Still aligning - status updated in alignToTarget()
                return false;
            }
        }

        // Step 2: Once aligned, shoot
        if (!shooting) {
            shoot();
            shooting = true;
        }

        return true;  // Sequence complete - aligned and shooting
    }

    /**
     * Align robot to AprilTag target
     * Returns true when aligned, false otherwise
     */
    private boolean alignToTarget() {
        tagFound = false;
        currentOffset = 0.0;

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            status = "No vision data";
            stopDrivetrain();
            return false;
        }

        List<FiducialResult> tags = result.getFiducialResults();

        for (FiducialResult tag : tags) {
            if (tag.getFiducialId() == targetTagID) {
                tagFound = true;
                currentOffset = tag.getTargetXDegrees();

                // Check if aligned
                if (Math.abs(currentOffset) < alignmentTolerance) {
                    stopDrivetrain();
                    status = "ALIGNED!";
                    return true;  // ALIGNED!
                }

                // Calculate turn power
                double turnPower;
                if (Math.abs(currentOffset) < alignmentSlowZone) {
                    turnPower = alignmentSlowPower;
                    status = "Fine tuning...";
                } else {
                    turnPower = alignmentTurnPower;
                    status = "Aligning...";
                }

                // Turn to align
                if (currentOffset > 0) {
                    turnRight(turnPower);
                } else {
                    turnLeft(turnPower);
                }

                return false;  // Not aligned yet
            }
        }

        // Tag not found
        status = "Tag #" + targetTagID + " not found";
        stopDrivetrain();
        return false;
    }

    /**
     * Execute shooting sequence
     * Shooter motors are already running - this opens gate and runs intake
     */
    public void shoot() {
        // Shooter motors already running from constructor

        // Open gate to release game elements
        gateServo.setPosition(gateOpenPosition);

        // Run intake system to feed game elements
        intakeMotor.setVelocity(intakeVelocity);
        rampMotor.setVelocity(rampVelocity);
        crServo.setPower(-1.0);

        status = "Shooting!";
    }

    /**
     * Stop shooting system completely
     */
    public void stopShooting() {
        shooterMotor1.setVelocity(0);
        shooterMotor2.setVelocity(0);
        intakeMotor.setVelocity(0);
        rampMotor.setVelocity(0);
        crServo.setPower(0);
        gateServo.setPosition(gateClosedPosition);
        shooting = false;
        status = "Stopped";
    }

    /**
     * Stop intake/ramp but keep shooters running
     */
    public void pauseFeeding() {
        intakeMotor.setVelocity(0);
        rampMotor.setVelocity(0);
        crServo.setPower(0);
        gateServo.setPosition(gateClosedPosition);
        shooting = false;
        status = "Shooters running - feeding paused";
    }

    /**
     * Stop drivetrain
     */
    private void stopDrivetrain() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

    /**
     * Turn right (clockwise)
     */
    private void turnRight(double power) {
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(-power);
        rightRear.setPower(-power);
    }

    /**
     * Turn left (counter-clockwise)
     */
    private void turnLeft(double power) {
        leftFront.setPower(-power);
        leftRear.setPower(-power);
        rightFront.setPower(power);
        rightRear.setPower(power);
    }

    /**
     * Stop all systems
     */
    public void stopAll() {
        stopDrivetrain();
        stopShooting();
    }

    /**
     * Reset alignment status (stops shooting, keeps shooter motors running)
     */
    public void reset() {
        aligned = false;
        tagFound = false;
        shooting = false;
        currentOffset = 0.0;
        stopDrivetrain();
        pauseFeeding();
        status = "Idle";
    }

    // Getters
    public boolean isAligned() { return aligned; }
    public boolean isTagFound() { return tagFound; }
    public boolean isShooting() { return shooting; }
    public double getCurrentOffset() { return currentOffset; }
    public String getStatus() { return status; }

    // Getters for telemetry
    public double getShooterVelocity() { return shooterMotor1.getVelocity(); }
    public double getIntakeVelocity() { return intakeMotor.getVelocity(); }
    public double getRampVelocity() { return rampMotor.getVelocity(); }

    // Setters for customization
    public void setTargetTagID(int id) { this.targetTagID = id; }
    public void setAlignmentTolerance(double tolerance) { this.alignmentTolerance = tolerance; }
    public void setShooterVelocity(double velocity) {
        this.shooterVelocity = velocity;
        // Update running shooter motors
        shooterMotor1.setVelocity(velocity);
        shooterMotor2.setVelocity(velocity);
    }
    public void setIntakeVelocity(double velocity) { this.intakeVelocity = velocity; }
    public void setRampVelocity(double velocity) { this.rampVelocity = velocity; }
    public void setAlignmentTurnPower(double power) { this.alignmentTurnPower = power; }
    public void setAlignmentSlowPower(double power) { this.alignmentSlowPower = power; }

    /**
     * Cleanup - stop limelight
     */
    public void shutdown() {
        limelight.stop();
        stopAll();
    }
}
