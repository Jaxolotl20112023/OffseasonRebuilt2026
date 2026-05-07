package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.States.IndexStates;
import frc.robot.subsystems.IndexSubsystem;

public class IndexCommand extends Command {
    // Store Index subsytem and current state
    private final IndexSubsystem s_indexSubsystem;
    private IndexStates indexState;

    // Initalize command outline 
    // Set requirment for shooter subsytem to avoid conflicting issues
    public IndexCommand(IndexSubsystem s_indexSubsystem) {
        this.s_indexSubsystem = s_indexSubsystem;
        addRequirements(s_indexSubsystem);
    }

    // Return current Index State
    public IndexStates getIndexState() {
        return indexState;
    }

    // Return an instant command that changes the state of the subsystem
    public InstantCommand setIndexState(IndexStates indexState) {
        this.indexState = indexState;
        return new InstantCommand(() -> s_indexSubsystem.setIndexState(indexState), s_indexSubsystem);
    }
}
