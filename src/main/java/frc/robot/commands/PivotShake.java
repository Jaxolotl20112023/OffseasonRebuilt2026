package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.IntakeStates;
import frc.robot.subsystems.IntakeSubsystem;

public class PivotShake extends Command {
    private final IntakeSubsystem s_intake;

    private IntakeStates state;

    public PivotShake(IntakeSubsystem intake) {
        s_intake = intake;
        state = IntakeStates.PUSH_IN;

        addRequirements(s_intake);
    }

    @Override
    public void initialize() {
        s_intake.setIntakeState(state);
    }

    @Override
    public void execute() {
        if(state.equals(IntakeStates.PUSH_IN) 
            && Math.abs(s_intake.getPiviotPosition()) > (Math.abs(IntakeStates.PUSH_IN.pivotAngle) - 20)) {
            state = IntakeStates.START;
            s_intake.setIntakeState(state);
        } else if (state.equals(IntakeStates.START) 
            && (Math.abs(s_intake.getPiviotPosition()) < (Math.abs(IntakeStates.START.pivotAngle) + 20))) {
            state = IntakeStates.PUSH_IN;
            s_intake.setIntakeState(state);
        }
    }
}
