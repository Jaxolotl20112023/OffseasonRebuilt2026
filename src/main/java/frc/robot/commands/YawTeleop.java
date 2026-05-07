// package frc.robot.commands;

// import static edu.wpi.first.units.Units.MetersPerSecond;
// import static edu.wpi.first.units.Units.RadiansPerSecond;
// import static edu.wpi.first.units.Units.RotationsPerSecond;

// import java.util.ResourceBundle.Control;

// import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.Constants;
// import frc.lib.util.Utilities;

// // Ensure smooth acceleration with rapid decleration 
// public class YawTeleop extends Command {
//     private final CommandSwerveDrivetrain drivetrain;
//     private final CommandXboxController controller;

//     // Set max speeds for swerve driving and deadband
//     private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
//     private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
//     private final double deadband = Constants.Swerve.kSwerveDeadband;

//     // PID for auto tag yawing
//     private final PIDController c_yawPID; 

//     // Setting up bindings for necessary control of the swerve drive platform 
//     private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
//             .withDeadband(MaxSpeed * deadband) // Add a 10% deadband
//             .withRotationalDeadband(MaxAngularRate * deadband) // Add a 10% deadband
//             .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

//     // Store inputs and speeds
//     private double xInput, yInput; 
//     private double xSpeed, ySpeed, rSpeed;

//     // State of robot
//     private Pose2d swerveState; 

//     public YawTeleop(CommandSwerveDrivetrain drivetrain, CommandXboxController controller) {
//         // Initialize drivetrain and controller
//         this.drivetrain = drivetrain; 
//         this.controller = controller;

//         // Intialize controller inputs to 0
//         xInput = 0; 
//         yInput = 0;

//         // Intiatlize swerve speeds to 0 
//         xSpeed = 0;
//         ySpeed = 0;
//         rSpeed = 0;

//         // Configure PID        
//         c_yawPID = new PIDController(Constants.Swerve.kSwerveP, Constants.Swerve.kSwerveI, Constants.Swerve.kSwerveD);
//         c_yawPID.enableContinuousInput(0, 360);

//         // Set requiremnts for the drivetrain subsystem to ensure no conflicts with other commands
//         addRequirements(drivetrain);
//     }

//     @Override
//     public void execute() {
//         // Set contoller speeds 
//         xInput = -controller.getLeftY();
//         yInput = -controller.getLeftX();

        
//         // Set setpoint depending on desired yaw to center tag
//         swerveState = drivetrain.getState().Pose;
//         c_yawPID.setSetpoint(Utilities.calculateYawToCenterPiece(swerveState.getX(), swerveState.getY()));

//         // Apply polynomial acceleration
//         setSwerveSpeeds();

//         // Apply speeds to the swerve drive
//         drivetrain.applyRequest(() -> drive.withVelocityX(ySpeed)
//             .withVelocityY(xSpeed)
//             .withRotationalRate(rSpeed))
//             .execute();

//         setDashboardData();
//     }
    
//     // Apply a polynomial acceleration curve to the joystick inputs for smoother control
//     // Calculate speed for yaw align
//     private void setSwerveSpeeds() {
//         xSpeed = Utilities.polynomialAccleration(yInput) * MaxSpeed * 0.8;
//         ySpeed = Utilities.polynomialAccleration(xInput) * MaxSpeed * 0.8;
//         rSpeed = c_yawPID.calculate(Utilities.processYaw(drivetrain.getPigeon2().getYaw().getValueAsDouble()));
//     }

//     // Display important information for debugging
//     private void setDashboardData() {
//         // Display current odometry positioning of robot
//         SmartDashboard.putNumber(drivetrain.getName() + " pidgeon 2", Utilities.processYaw(drivetrain.getPigeon2().getYaw().getValueAsDouble()));
//         SmartDashboard.putNumber(drivetrain.getName() + " state x pos", drivetrain.getState().Pose.getMeasureX().baseUnitMagnitude());
//         SmartDashboard.putNumber(drivetrain.getName() + " state y pos", drivetrain.getState().Pose.getMeasureY().baseUnitMagnitude());
//         SmartDashboard.putNumber(drivetrain.getName() + " state rot pos", drivetrain.getState().Pose.getRotation().getDegrees());
        
//          // Controller Inputs
//         SmartDashboard.putNumber(drivetrain.getName() + "xInput", xInput);
//         SmartDashboard.putNumber(drivetrain.getName() + "yInput", yInput);

//         // Calculated Speeds
//         SmartDashboard.putNumber(drivetrain.getName() + "xSpeed", xSpeed);
//         SmartDashboard.putNumber(drivetrain.getName() + "ySpeed", ySpeed);
//         SmartDashboard.putNumber(drivetrain.getName() + "rSpeed", rSpeed);

//         // PID information
//         SmartDashboard.putNumber(drivetrain.getName() + "yawSetpoint", c_yawPID.getSetpoint());
//         SmartDashboard.putNumber(drivetrain.getName() + "yawError", c_yawPID.getPositionError());
//     }
// }