package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

public class States {
    // Indexer states with corresponding indexer and conveyor speeds
    // Both are in motor speed range of -1 to 1
    // Positive in Index in 
    public enum IndexStates {
        STOP(0, 0),
        INDEX(0.8, -0.8),
        AUTO_INDEX(0.5, -0.5),
        REVERSE(-1, 0.5);

        public final double indexerSpeed;
        public final double conveyorSpeed;

         IndexStates(double indexerSpeed, double conveyorSpeed) {
            this.indexerSpeed = indexerSpeed;
            this.conveyorSpeed = conveyorSpeed;
        }
    }

    // Shooter states with corresponding backspin and shooting speeds
    // Both are in Rotations Per Second (RPS) and need to set range
    // Positive for both is shoot outward 
    // Contains the backspin and shooterspin states for different distances
    public enum ShooterStates {

        // Set the shooter move
        STOP(0, 0, 0),
        FORWARD_SHOOT(60, 30, 0),
        VARIABLE_SHOOT(0, 0, 0),
        TINY_SHOOT(20,38.67,0), 

        // SETTING THE STATES FOR DIFFERENT DISTANCES
        // DISTANCE_0M(3.7, 37.7, 0),
        // DISTANCE_0_25M(0, 39.1, 0.25),
        // DISTANCE_0_5M(0, 38.6, 0.5),
        // DISTANCE_0_75M(0, 37.4, 0.5),
        // DISTANCE_1M(0.3, 40.2, 1),
        // DISTANCE_1_25M(0.6, 41.8, 1.25),
        // DISTANCE_1_5M(1.7, 44.9, 1.5),
        // DISTANCE_1_75M(2.7, 46.4, 1.75),
        // DISTANCE_2M(3.7, 45.6, 2),
        // DISTANCE_2_25M(4.7, 46.7, 2.25),
        // DISTANCE_2_5M(5.8, 49.2, 2.5),

       // IN_120(62.5,30,Units.inchesToMeters(120)),
        //IN_120(25, 42.5,Units.inchesToMeters(120)),
        IN_120(7.5 ,48, Units.inchesToMeters(120)),
        IN_89(32, 45,Units.inchesToMeters(89.5)),
        IN_60(22.5,37.5, Units.inchesToMeters(60)),
        IN_45(25, 35, Units.inchesToMeters(45)),
        IN_75(25, 40, Units.inchesToMeters(75)),
        IN_100(57.5, 30, Units.inchesToMeters(100)),
        CLIMB_TO_CENTER(15, 44.25, Units.inchesToMeters(115)),
        AUTO_IN(57.5,28.5, Units.inchesToMeters(67)),
        // New auto states for left auto
        SHOOT_FAR(15, 47.5, Units.inchesToMeters(6700)),
        // ah
        REVERSE_SHOOT(0,-100,0); 
        
        public final double backSpinRPS;
        public final double shootingRPS;
        public final double distance; 

        ShooterStates (double backSpinRPS, double shootingRPS, double distance) {
            this.backSpinRPS = backSpinRPS;
            this.shootingRPS = shootingRPS;
            this.distance = distance; 
        }
    }

    // Intake states with corresponding intake speeds and pivot angles
    // Intake Speed range is -1 to 1 motor speed
    // Pivot is in angles need to set range
    // Positive for intake speed is intaking, positive for pivot angle is pivoting up
    public enum IntakeStates {
        STOP(0, -304),

        // Set the speed of the intak-ing motor and the position of piviot 
        INTAKE(1, -220),
        AUTO_INTAKE(0.7, -219),
        OUTAKE(-0.5,-280),
    
        // Set the home values for piviot 
        GIGA_HOME(0, -310),
        PUSH_IN(0.7, -280),
        HALF(0.7, -280),
        START(0,-219),
        HOME(0,-300),
        pickle(0,-271),
        TESTING_INTAKE(1, -210);

        public final double intakeSpeed;
        public final double pivotAngle;

        IntakeStates(double intakeSpeed, double pivotAngle) {
            this.intakeSpeed = intakeSpeed;
            this.pivotAngle = pivotAngle;
        }
    }

    public enum AddressableLEDStates {

        SOLID_WHITE(0.91),
        SOLID_RED (0.61),
        CHASE_RED(-0.73), 
        CHASE_BLUE(-0.75),
        SOLID_BLUE (-0.73),
        HEARTBEAT_RED (-0.25),
        HEARTBEAT_BLUE (-0.23),
        HEARTBEAT_WHITE(-0.21);

        public final double ledID; 

        AddressableLEDStates(double ledID) {
            this.ledID = ledID; 
        }
    }
}
