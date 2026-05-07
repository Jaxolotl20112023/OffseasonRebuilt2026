// package frc.robot.subsystems;

// // ... other imports ...
// import edu.wpi.first.networktables.DoublePublisher;
// import edu.wpi.first.networktables.StringPublisher;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import frc.lib.util.Elastic; // Ensure you have ElasticLib in your project

// public class ShooterSubsystem extends SubsystemBase {
//     // ... motor and state variables ...

//     // NT4 Publishers (The 2027-safe way)
//     private final DoublePublisher rightRPSPub;
//     private final DoublePublisher middleRPSPub;
//     private final DoublePublisher leftRPSPub;
//     private final DoublePublisher backSpinRPSPub;
//     private final StringPublisher statePub;

//     public ShooterSubsystem(CommandSwerveDrivetrain x) { 
//         // ... motor initialization code ...

//         // Initialize NT4 Publishers
//         NetworkTable shooterTable = NetworkTableInstance.getDefault().getTable("Shooter");
        
//         rightRPSPub = shooterTable.getDoubleTopic("Right RPS").publish();
//         middleRPSPub = shooterTable.getDoubleTopic("Middle RPS").publish();
//         leftRPSPub = shooterTable.getDoubleTopic("Left RPS").publish();
//         backSpinRPSPub = shooterTable.getDoubleTopic("Backspin RPS").publish();
//         statePub = shooterTable.getStringTopic("Current State").publish();

//         // ... rest of constructor ...
//     }

//     @Override
//     public void periodic() { 
//         setDashboardData();
        
//         // Logical Check for Notification
//         if (m_leftShooter.getMotorVoltage().getValueAsDouble() > 11.5) {
//             sendHighVoltageWarning();
//         }
//     }

//     private void sendHighVoltageWarning() {
//         // Send a pop-up to the Elastic Dashboard
//         Elastic.sendNotification(new Elastic.Notification()
//             .withLevel(Elastic.NotificationLevel.WARNING)
//             .withTitle("Shooter Warning")
//             .withDescription("Shooter motors drawing high voltage!")
//             .withDisplaySeconds(3.0));
//     }

//     private void setDashboardData() {
//         // Use .set() on publishers instead of SmartDashboard.putNumber()
//         rightRPSPub.set(m_rightShooter.getVelocity().getValueAsDouble());
//         middleRPSPub.set(m_middleShooter.getVelocity().getValueAsDouble());
//         leftRPSPub.set(m_leftShooter.getVelocity().getValueAsDouble());
//         backSpinRPSPub.set(getBackSpinRPS());
        
//         statePub.set(s_state.toString());
//     }
// }