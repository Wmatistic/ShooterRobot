package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.teleopcommand.TransferBallCommand;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystem.Indexer;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

@Autonomous
public class BlueAuto extends OpMode {

    PinpointDrive drive;
    TelemetryPacket tele;

    SequentialAction blueAutoPath;
    RobotHardware robot = RobotHardware.getInstance();

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();

        drive = new PinpointDrive(hardwareMap, Poses.BlueAuto.start);
        tele = new TelemetryPacket();
        robot.init(hardwareMap);

        robot.robotLocalization.setPinpointPose(Poses.BlueAuto.start);

        robot.shooter.setShooterState(Shooter.ShooterState.AIMING);

        blueAutoPath = scorePreload();

        robot.shooter.shooterControl.changeGoalXOffset(2);
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        robot.autoPeriodic();

        blueAutoPath.run(tele);
    }

    public SequentialAction scorePreload() {
        return new SequentialAction(
                drive.actionBuilder(Poses.BlueAuto.start)
                        .strafeToLinearHeading(new Vector2d(Poses.BlueAuto.firstShot.position.x, Poses.BlueAuto.firstShot.position.y), Math.toRadians(90))
                        .build(),

                telemetryPacket -> {
                    CommandScheduler.getInstance().schedule(
                            new SequentialCommandGroup(
                                    new TransferBallCommand(Indexer.SlotID.REAR),
                                    new WaitCommand(200),
                                    new TransferBallCommand(Indexer.SlotID.MIDDLE),
                                    new WaitCommand(200),
                                    new TransferBallCommand(Indexer.SlotID.FRONT)
                            )
                    );
                    return false;
                },

                new SleepAction(3.0),

                new ParallelAction(
                        drive.actionBuilder(Poses.BlueAuto.firstShot)
                                .setTangent(Math.toRadians(90))
                                .lineToYConstantHeading(Poses.BlueAuto.intakeFirstLine.position.y)
                                .build(),

                        telemetryPacket -> {
                            CommandScheduler.getInstance().schedule(new InstantCommand(() -> robot.intake.setIntake(Intake.IntakeState.INTAKING)));
                            return false;
                        }
                ),

                new SleepAction(0.2),

                new ParallelAction(
                        drive.actionBuilder(Poses.BlueAuto.intakeFirstLine)
                                .setTangent(Math.toRadians(90))
                                .lineToYConstantHeading(Poses.BlueAuto.shootFirstLine.position.y)
                                .build(),

                        telemetryPacket -> {
                            CommandScheduler.getInstance().schedule(new InstantCommand(() -> robot.intake.setIntake(Intake.IntakeState.STOWED)));
                            return false;
                        }
                ),

                telemetryPacket -> {
                    CommandScheduler.getInstance().schedule(
                            new SequentialCommandGroup(
                                    new TransferBallCommand(Indexer.SlotID.REAR),
                                    new WaitCommand(200),
                                    new TransferBallCommand(Indexer.SlotID.MIDDLE),
                                    new WaitCommand(200),
                                    new TransferBallCommand(Indexer.SlotID.FRONT)
                            )
                    );
                    return false;
                },

                new SleepAction(3.0),

                drive.actionBuilder(Poses.BlueAuto.shootFirstLine)
                        .splineToLinearHeading(Poses.BlueAuto.intakeSecondLineInitial, Math.toRadians(180))
                        .build(),

                new ParallelAction(
                        new ParallelAction(
                                drive.actionBuilder(Poses.BlueAuto.intakeSecondLineInitial)
                                        .setTangent(Math.toRadians(90))
                                        .lineToYConstantHeading(Poses.BlueAuto.intakeSecondLinePost.position.y)
                                        .build(),

                                telemetryPacket -> {
                                    CommandScheduler.getInstance().schedule(new InstantCommand(() -> robot.intake.setIntake(Intake.IntakeState.INTAKING)));
                                    return false;
                                }
                        )
                ),

                new ParallelAction(
                        drive.actionBuilder(Poses.BlueAuto.intakeFirstLine)
                                .setReversed(true)
                                .splineToLinearHeading(Poses.BlueAuto.shootSecondLine, Math.toRadians(0))
                                .build(),

                        telemetryPacket -> {
                            CommandScheduler.getInstance().schedule(new InstantCommand(() -> robot.intake.setIntake(Intake.IntakeState.STOWED)));
                            return false;
                        }
                ),

                telemetryPacket -> {
                    CommandScheduler.getInstance().schedule(
                            new SequentialCommandGroup(
                                    new TransferBallCommand(Indexer.SlotID.REAR),
                                    new WaitCommand(200),
                                    new TransferBallCommand(Indexer.SlotID.MIDDLE),
                                    new WaitCommand(200),
                                    new TransferBallCommand(Indexer.SlotID.FRONT)
                            )
                    );
                    return false;
                },

                new SleepAction(3.0),

                drive.actionBuilder(Poses.BlueAuto.shootSecondLine)
                        .setReversed(false)
                        .splineToLinearHeading(Poses.BlueAuto.intakeThirdLineInitial, Math.toRadians(180))
                        .build(),

                new ParallelAction(
                        drive.actionBuilder(Poses.BlueAuto.intakeThirdLineInitial)
                                .setTangent(90)
                                .lineToYConstantHeading(Poses.BlueAuto.intakeThirdLinePost.position.y)
                                .build(),

                        telemetryPacket -> {
                            CommandScheduler.getInstance().schedule(new InstantCommand(() -> robot.intake.setIntake(Intake.IntakeState.INTAKING)));
                            return false;
                        }
                ),

                new ParallelAction(
                        drive.actionBuilder(Poses.BlueAuto.intakeThirdLinePost)
                                .setReversed(true)
                                .splineToLinearHeading(Poses.BlueAuto.shootThirdLine, Math.toRadians(0))
                                .build(),

                        telemetryPacket -> {
                            CommandScheduler.getInstance().schedule(new InstantCommand(() -> robot.intake.setIntake(Intake.IntakeState.STOWED)));
                            return false;
                        }
                )
        );
    }
}
