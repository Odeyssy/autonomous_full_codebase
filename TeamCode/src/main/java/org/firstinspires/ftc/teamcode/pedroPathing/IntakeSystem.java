package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSystem {

    // Hardware components
    private DcMotorEx intakeMotor;
    private DcMotorEx rampMotor;
    private CRServo crServofrontR;
    private CRServo crServobackR; // New Servo

    // Velocity constants
    private double intakeVelocity;
    private double rampVelocity;

    /**
     * Constructor with default velocities
     */
    public IntakeSystem(HardwareMap hardwareMap) {
        this(hardwareMap, 2800.0, 850.0);
    }

    /**
     * Constructor with custom velocities
     */
    public IntakeSystem(HardwareMap hardwareMap, double intakeVelocity, double rampVelocity) {
        this.intakeVelocity = intakeVelocity;
        this.rampVelocity = rampVelocity;

        // Initialize hardware
        intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        rampMotor = hardwareMap.get(DcMotorEx.class, "rampmotor");
        crServofrontR = hardwareMap.get(CRServo.class, "crServofrontR");
        crServobackR = hardwareMap.get(CRServo.class, "crServobackR"); // Initialize new servo

        // Set motor modes and directions
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rampMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rampMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Directions based on your TeleOp: Front is REVERSE, Back is FORWARD
        crServofrontR.setDirection(DcMotorSimple.Direction.REVERSE);
        crServobackR.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Run intake at full speed
     */
    public void runIntake() {
        intakeMotor.setVelocity(intakeVelocity);
        rampMotor.setVelocity(rampVelocity);
        // Syncing power based on your TeleOp logic
        crServofrontR.setPower(-1.0);
        crServobackR.setPower(1.0);
    }

    /**
     * Stop all intake motors and servos
     */
    public void stopAll() {
        intakeMotor.setVelocity(0);
        rampMotor.setVelocity(0);
        crServofrontR.setPower(0);
        crServobackR.setPower(0);
    }

    // Getters for telemetry
    public double getIntakeVelocity() {
        return intakeMotor.getVelocity();
    }

    public double getRampVelocity() {
        return rampMotor.getVelocity();
    }

    public double getFrontCRServoPower() {
        return crServofrontR.getPower();
    }

    public double getBackCRServoPower() {
        return crServobackR.getPower();
    }

    // Setters for customization
    public void setIntakeVelocity(double velocity) {
        this.intakeVelocity = velocity;
    }

    public void setRampVelocity(double velocity) {
        this.rampVelocity = velocity;
    }
}