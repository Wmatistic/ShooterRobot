package org.firstinspires.ftc.teamcode.util;


import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.Shooter;

public class RobotHardware {

    // ----- DRIVETRAIN -----
    public DcMotorEx leftFront, leftRear, rightFront, rightRear;
    public IMU imu;

    // ----- SHOOTER -----
    public DcMotorEx shooterMotorLeft, shooterMotorRight, turretMotor;
    public Servo shooterServoLeft, shooterServoRight;
    public AnalogInput turretServoInput;
    public PIDFController turretPID;

    public Shooter shooter;

    private HardwareMap hardwareMap;
    private static RobotHardware instance = null;
    private boolean enabled;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(final HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        // ******************* DRIVETRAIN *******************
        leftFront = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.leftFront);
        leftRear = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.leftRear);
        rightFront = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.rightFront);
        rightRear = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.rightRear);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters( new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);
        imu.resetYaw();

        // ******************* SHOOTER *******************
        shooterMotorLeft = hardwareMap.get(DcMotorEx.class, RobotConstants.Shooter.shooterMotorLeft);
        shooterMotorRight = hardwareMap.get(DcMotorEx.class, RobotConstants.Shooter.shooterMotorRight);

        shooterMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotorLeft.setVelocityPIDFCoefficients(RobotConstants.Shooter.flywheelP, RobotConstants.Shooter.flywheelI, RobotConstants.Shooter.flywheelD, RobotConstants.Shooter.flywheelF);

        shooterMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotorRight.setVelocityPIDFCoefficients(RobotConstants.Shooter.flywheelP, RobotConstants.Shooter.flywheelI, RobotConstants.Shooter.flywheelD, RobotConstants.Shooter.flywheelF);

        shooterMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterServoLeft = hardwareMap.servo.get(RobotConstants.Shooter.shooterServoLeft);
        shooterServoRight = hardwareMap.servo.get(RobotConstants.Shooter.shooterServoRight);

        turretMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.Shooter.turretMotor);
        turretServoInput = hardwareMap.get(AnalogInput.class, RobotConstants.Shooter.turretServoInput);
        turretPID = new PIDFController(RobotConstants.Shooter.turretP, RobotConstants.Shooter.turretI, RobotConstants.Shooter.turretD, RobotConstants.Shooter.turretF);

    }
}
