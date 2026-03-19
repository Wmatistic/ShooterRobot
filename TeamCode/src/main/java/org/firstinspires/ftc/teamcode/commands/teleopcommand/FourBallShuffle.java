package org.firstinspires.ftc.teamcode.commands.teleopcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.IndexerArmCommand;
import org.firstinspires.ftc.teamcode.subsystem.Indexer;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class FourBallShuffle extends SequentialCommandGroup {
    public FourBallShuffle() {
        super(
                new IndexerArmCommand(Indexer.SlotID.REAR, RobotConstants.Indexer.rearServoShuffle),
                new WaitCommand(200),
                new IndexerArmCommand(Indexer.SlotID.REAR, RobotConstants.Indexer.rearIndexerServoStowed),
                new WaitCommand(400)
        );
    }
}
