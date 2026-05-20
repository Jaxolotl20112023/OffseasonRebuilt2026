package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    public class Swerve {
        // Amount of dead radius on the controller 
        public static final double kSwerveDeadband = 0.04;
        public static final double kSwerveP = 0.078;
        public static final double kSwerveI = 0;
        public static final double kSwerveD = 0; 

        public static final double kYawTranslationLimiter = 0.4; 
    }

    public class Shooter {
        // REV SparkFlex 
        public static final int kBackSpinID = 34;
        
        // Pheonix Krakens
        public static final int kRightShootingID = 20;
        public static final int kMiddleShootingID = 21;
        public static final int kLeftShootingID = 22;

        // PID constants for slot 0 on shooter
        public static final double kShooterP = 0.03;
        public static final double kShooterI = 0;
        public static final double kShooterD = 0.009; 
        public static final double kShooterS = 0;
        public static final double kShooterV = 0.13;

        // PID constants for backspin motors
        public static final double kBackSpinP = 0.0001;
        public static final double kBackSpinI = 0;
        public static final double kBackSpinD = 0;
        public static final double kBackSpinV = 0.000149537;
        
        // Configuration for backspin motors
        public static final boolean kBackSpinEnable = false;
        public static final boolean kBackSpinInvert = true;

        // IDS
        public static final int[] validIDs = {9,10,26,25};
    }

    public class Indexor {
        // REV SparkMax 
        public static final int kIndexorID = 33;
        public static final int kConveyorID = 31; // 30
    }

    public class Intake{
        // REV SparkFLex 
        public static final int kLeftIntakeID = 36;
        public static final int kRightIntakeID = 35; 

        // REV SparkMax
        public static final int kPivotID = 30; // 30

        // CTRE Through Bore Encoder 
        public static final int kEncoderID = 23;

        // Max and Min
        public static final int kMin = -322; 
        public static final int kMax = -208;

        // PID values
        public static final double kIntakeP = 0.09; // 0.03
        public static final double kIntakeI = 0;
        public static final double kIntakeD = 0;
    
    }

    public class Vision {
        public static final String leftLimelightName = "";
        public static final String rightLimelightName = ""; 

        // Vision threshold: 
        public static final double ambiguityThreshold = 0.9;

        // Vision distance to camera threshold: 
        public static final double distanceThreshold = 5; 
    }

    public class Field {
        public static final double kBlueXPos = Units.inchesToMeters(158.84);
        public static final double kBlueYPos = Units.inchesToMeters(182.11);

        public static final double kRedXPos = 0;
        public static final double kRedYPos = 0; 
    }

    public class AddressableLED {
        public static final int firstBlinkIn = 0; 
        
        
    }
}
//     public class LED {
//     private static final double SOLID_BLUE = 0.87;
//     private static final double SOLID_RED = 0.61;
//     private static final double SOLID_WHITE = 0.93;
//     private static final double HEARTBEAT_BLUE = -0.23;
//     private static final double HEARTBEAT_RED = -0.25;
//     }
// }
