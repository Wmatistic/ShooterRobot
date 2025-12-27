package org.firstinspires.ftc.teamcode.commands.teleopcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.IndexerArmCommand;
import org.firstinspires.ftc.teamcode.subsystem.Indexer;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class TransferBallCommand extends SequentialCommandGroup {
    public TransferBallCommand(Indexer.SlotID slotID) {
        super(
                new IndexerArmCommand(slotID, RobotHardware.getInstance().indexer.getIndexerServoUpPosition(slotID)),
                new WaitCommand(300),
                new IndexerArmCommand(slotID, RobotHardware.getInstance().indexer.getIndexerServoStowedPosition(slotID))
        );
    }
}
