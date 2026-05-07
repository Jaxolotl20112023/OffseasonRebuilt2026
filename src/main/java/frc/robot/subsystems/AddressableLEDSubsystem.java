package frc.robot.subsystems;

import java.lang.Thread.State;
import java.util.Optional;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.States;
import frc.robot.Constants.Shooter;
import frc.robot.States.AddressableLEDStates;
import frc.robot.States.ShooterStates;
import frc.robot.generated.TunerConstants;

public class AddressableLEDSubsystem extends SubsystemBase {
    private final Spark blinkin0;
    // private final Spark blinkin1; // set up both blinkin
    private Supplier<Boolean> isAtRps; // function for max. RPS for shooter motor
    private final Optional<DriverStation.Alliance> alliance; 
    private double pattern; 
    private AddressableLEDStates ledState; 
    private final ShooterSubsystem s_shooter; 
    private double tolerance; 

    // Simple pattern constants (values are placeholders for Blinkin output)

// Supplier<Boolean> isAtRps, ShooterSubsystem shooter
    public AddressableLEDSubsystem(ShooterSubsystem s_shooter) {
        blinkin0 = new Spark(Constants.AddressableLED.firstBlinkIn); 
        alliance = DriverStation.getAlliance(); 
        this.s_shooter = s_shooter; 
        // s_shooter = new ShooterSubsystem(x);
        isAtRps = () -> false; 

        // this.alliance = DriverStation.getAlliance().get(); 
    }

    @Override
    public void periodic() { // called every 20 ms

        double[] shooterSpeeds = s_shooter.getShooterRPS(); 

        if (s_shooter.getShooterState() == ShooterStates.CLIMB_TO_CENTER) 
            tolerance = 0.93; 
        else  
            tolerance = 0.95; 

        isAtRps = () -> {
            if (s_shooter.desiredBackSpinRPS != 0 
                && s_shooter.desiredShooterRPS != 0 
                && (Math.abs(Math.round(shooterSpeeds[0])) >= Math.round(s_shooter.desiredShooterRPS)*tolerance
                && Math.abs(Math.round(shooterSpeeds[1])) >= Math.round(s_shooter.desiredShooterRPS)*tolerance
                && Math.abs(Math.round(shooterSpeeds[2])) >= Math.round(s_shooter.desiredShooterRPS)*tolerance
                && Math.abs(Math.round(s_shooter.getBackSpinRPS())) >= Math.round(s_shooter.desiredBackSpinRPS)*tolerance)) {
                return true; 
            } 
            return false; 
        };

        if (alliance.isPresent()) {
            if (isAtRps.get() && alliance.get() == Alliance.Red) 
                pattern = setLEDState(AddressableLEDStates.HEARTBEAT_WHITE);
            else if (isAtRps.get() && alliance.get() == Alliance.Blue)
                pattern = setLEDState(AddressableLEDStates.HEARTBEAT_WHITE);
            else if (alliance.get() == Alliance.Red) 
                pattern = setLEDState(AddressableLEDStates.CHASE_RED); 
            else if (alliance.get() == Alliance.Blue) 
                pattern = setLEDState(AddressableLEDStates.CHASE_BLUE);
        }

        blinkin0.set(pattern);
        
    }

    public double setLEDState(AddressableLEDStates ledState) {
        this.ledState = ledState; 

        return ledState.ledID; 
    }
}                                 
