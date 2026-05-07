package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.States.ShooterStates;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
    // Store shooter subsytem and current state
    private final ShooterSubsystem s_ShooterSubsystem; 
    private ShooterStates shooterStates;
    
    // Initalize command outline 
    // Set requirment for shooter subsytem to avoid conflicting issues
    public ShooterCommand(ShooterSubsystem s_ShooterSubsystem) {
        this.s_ShooterSubsystem = s_ShooterSubsystem; 
        addRequirements(s_ShooterSubsystem);
    }

    // Get current shooter state
    public ShooterStates getShooterStates() {
        return shooterStates; 
    }

    // Return an instant command that changes the state of the subsystem
    public InstantCommand setShooterState(ShooterStates shooterStates) {
        this.shooterStates = shooterStates; 
        return new InstantCommand(() -> s_ShooterSubsystem.setShooterState(shooterStates),s_ShooterSubsystem); 
    }
}
