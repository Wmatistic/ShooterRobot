package org.firstinspires.ftc.teamcode.util;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;

public class RobotHardware {

    // ----- DRIVETRAIN -----
    public DcMotorEx leftFront, leftRear, rightFront, rightRear;
    public IMU imu;

    // ----- SHOOTER -----
    public DcMotorEx shooterMotorLeft, shooterMotorRight, turretMotor;
    public Servo shooterServoLeft, shooterServoRight;
    public AnalogInput turretServoInput;

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
        shooterMotorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(RobotConstants.Shooter.p, RobotConstants.Shooter.i, RobotConstants.Shooter.d, RobotConstants.Shooter.f));

        shooterMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(RobotConstants.Shooter.p, RobotConstants.Shooter.i, RobotConstants.Shooter.d, RobotConstants.Shooter.f));

        shooterMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterServoLeft = hardwareMap.servo.get(RobotConstants.Shooter.shooterServoLeft);
        shooterServoRight = hardwareMap.servo.get(RobotConstants.Shooter.shooterServoRight);

        turretMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.Shooter.turretMotor);
        turretServoInput = hardwareMap.get(AnalogInput.class, RobotConstants.Shooter.turretServoInput);

    }
}
