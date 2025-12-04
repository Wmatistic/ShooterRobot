package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class Intake implements Subsystem {
    private final RobotHardware robot;

    private IntakeState intakeState;
    public enum IntakeState {
        INTAKING, STOWED
    }

    public Intake() {
        this.robot = RobotHardware.getInstance();

        intakeState = IntakeState.STOWED;
    }

    public void setIntake(IntakeState intakeState) {
        setIntakeState(intakeState);

        switch (intakeState) {
            case INTAKING:
                setIntakeMotorPower(RobotConstants.Intake.intakePower);
                break;

            case STOWED:
                setIntakeMotorPower(RobotConstants.Intake.intakeOff);
                break;
        }
    }

    public void setIntakeMotorPower(double power) {
        robot.intakeMotor.setPower(power);
    }

    public void setIntakeState(IntakeState intakeState) {
        this.intakeState = intakeState;
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }
}
