package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.States.IntakeStates;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command{
    // Store Intake subsytem and current state
    private final IntakeSubsystem s_intakesubsystem;
    private IntakeStates intakeState;

    // Initalize command outline 
    // Set requirment for shooter subsytem to avoid conflicting issues
    public IntakeCommand(IntakeSubsystem s_intakesubsystem) {
        this.s_intakesubsystem = s_intakesubsystem;
        addRequirements(s_intakesubsystem);
    }

    // return current set intake state
    public IntakeStates getIntakeState() {
        return intakeState;
    }

    // Return an instant command that changes the state of the subsystem
    public InstantCommand setIntakeState(IntakeStates intakeState) {
        this.intakeState = intakeState;
        return new InstantCommand(() -> s_intakesubsystem.setIntakeState(intakeState), s_intakesubsystem);
    }

    public RepeatCommand setPiviotShake() {
        Command piviotShake = Commands.runOnce(() -> {
            s_intakesubsystem.setIntakeState(IntakeStates.START);
            new WaitCommand(0.5);
            s_intakesubsystem.setIntakeState(IntakeStates.OUTAKE);
        });


        return piviotShake.repeatedly();
       
    }
}
