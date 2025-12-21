package org.firstinspires.ftc.teamcode.commands.teleopcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.IndexerArmCommand;
import org.firstinspires.ftc.teamcode.subsystem.Indexer;
import org.firstinspires.ftc.teamcode.util.RobotConstants;

public class TransferBallCommand extends SequentialCommandGroup {
    public TransferBallCommand(Indexer.SlotID slotID) {
        super(
                new IndexerArmCommand(slotID, RobotConstants.Indexer.indexerServoUp),
                new WaitCommand(200),
                new IndexerArmCommand(slotID, RobotConstants.Indexer.indexerServoStowed)
        );
    }
}
