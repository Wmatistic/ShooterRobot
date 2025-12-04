package org.firstinspires.ftc.teamcode.util;


import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.Indexer;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
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

    // ----- INTAKE -----
    public DcMotorEx intakeMotor;

    // ----- INDEXER -----
    public RevColorSensorV3 frontLeftSensor, frontRightSensor, middleLeftSensor, middleRightSensor, rearLeftSensor, rearRightSensor;
    public Servo frontServo, middleServo, rearServo;

    // ----- LIMELIGHT -----
    public Limelight3A limelight;

    // ----- PINPOINT -----
    public PinpointDrive pinpointDrive;

    // ----- SUBSYSTEMS -----
    public Drivetrain drivetrain;
    public Shooter shooter;
    public RobotLocalization robotLocalization;
    public Intake intake;
    public Indexer indexer;

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

        // ******************* INTAKE *******************
        intakeMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.Intake.intakeMotor);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setPower(0);

        // ******************* INDEXER *******************
        frontLeftSensor = hardwareMap.get(RevColorSensorV3.class, RobotConstants.Indexer.frontLeftSensor);
        frontRightSensor = hardwareMap.get(RevColorSensorV3.class, RobotConstants.Indexer.frontRightSensor);
        middleLeftSensor = hardwareMap.get(RevColorSensorV3.class, RobotConstants.Indexer.middleLeftSensor);
        middleRightSensor = hardwareMap.get(RevColorSensorV3.class, RobotConstants.Indexer.middleRightSensor);
        rearLeftSensor = hardwareMap.get(RevColorSensorV3.class, RobotConstants.Indexer.rearLeftSensor);
        rearRightSensor = hardwareMap.get(RevColorSensorV3.class, RobotConstants.Indexer.rearRightSensor);

        frontServo = hardwareMap.servo.get(RobotConstants.Indexer.frontServo);
        frontServo.setPosition(RobotConstants.Indexer.indexerServoStowed);

        middleServo = hardwareMap.servo.get(RobotConstants.Indexer.middleServo);
        middleServo.setPosition(RobotConstants.Indexer.indexerServoStowed);

        rearServo = hardwareMap.servo.get(RobotConstants.Indexer.rearServo);
        rearServo.setPosition(RobotConstants.Indexer.indexerServoStowed);

        // ******************* LIMELIGHT / PINPOINT *******************
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);

        pinpointDrive = new PinpointDrive(hardwareMap, RobotConstants.RobotLocalization.start);

        // ******************* INITIALIZE SUBSYSTEMS *******************
        robotLocalization = new RobotLocalization();
        shooter = new Shooter();
        drivetrain = new Drivetrain();
        intake = new Intake();
        indexer = new Indexer();
    }

    public void periodic() {
        drivetrain.periodic();
        robotLocalization.periodic();
        shooter.periodic();
    }
}
