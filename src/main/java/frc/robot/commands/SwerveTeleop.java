package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.lib.util.Utilities;

// Ensure smooth acceleration with rapid decleration 
public class SwerveTeleop extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController controller;

    // Set max speeds for swerve driving and deaband
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private final double deadband = Constants.Swerve.kSwerveDeadband;

    // Setting up bindings for necessary control of the swerve drive platform 
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * deadband) 
            .withRotationalDeadband(MaxAngularRate * deadband) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    // Store inputs and speeds
    private double xInput, yInput, rInput;   
    private double xSpeed, ySpeed, rSpeed;

    public SwerveTeleop(CommandSwerveDrivetrain drivetrain, CommandXboxController controller) {
        // Initialize drivetrain and controller
        this.drivetrain = drivetrain; 
        this.controller = controller;

        // Intialize controller inputs to 0
        xInput = 0; 
        yInput = 0;
        rInput = 0; 

        // Intiatlize swerve speeds to 0 
        xSpeed = 0;
        ySpeed = 0;
        rSpeed = 0;

        // Set requiremnts for the drivetrain subsystem to ensure no conflicts with other commands
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // Set contoller speeds 
        xInput = controller.getLeftY();
        yInput = controller.getLeftX();
        rInput = -controller.getRightX();

        // Apply polynomial acceleration
        setPolynomialAcceleration();
        // setFastPolynomialAcceleration();

        // Apply speeds to the swerve drive
        drivetrain.applyRequest(() -> drive.withVelocityX(ySpeed)
            .withVelocityY(xSpeed)
            .withRotationalRate(rSpeed))
            .execute();

        // Display the dashboard data
        setDashboardData();
    }
    
    // Apply a polynomial acceleration curve to the joystick inputs for smoother control
    private void setPolynomialAcceleration() {
        xSpeed = Utilities.polynomialAccleration(yInput) * MaxSpeed;
        ySpeed = Utilities.polynomialAccleration(xInput) * MaxSpeed;
        rSpeed = rInput * MaxAngularRate; 
    }

    private void setFastPolynomialAcceleration() {
        xSpeed = Utilities.supaHotFireAcceleration(xInput) * MaxSpeed;
        ySpeed = Utilities.supaHotFireAcceleration(yInput) * MaxSpeed;
        rSpeed = rInput * MaxAngularRate; 
    }
    
    // // Limit the rate of change of the xSpeed to 0.8 m/s^2
    // private final SlewRateLimiter xLimiter = new SlewRateLimiter(3);
    // private final SlewRateLimiter yLimiter = new SlewRateLimiter(3);
    // private final SlewRateLimiter rLimiter = new SlewRateLimiter(3); 

    // //My methods of the code 
    // private void phatSpeed() {
    //     xSpeed = xLimiter.calculate(xSpeed); //Apply slew rate limiting to the xSpeed
    //     ySpeed = yLimiter.calculate(ySpeed);
    //     rSpeed = rLimiter.calculate(rSpeed);
    // }
 
    // Display important information for debugging
    private void setDashboardData() {
        // Display current odometry positioning of robot
        SmartDashboard.putNumber(drivetrain.getName() + " pidgeon 2", Utilities.processYaw(drivetrain.getPigeon2().getYaw().getValueAsDouble()));
        SmartDashboard.putNumber(drivetrain.getName() + " state x pos", drivetrain.getState().Pose.getMeasureX().baseUnitMagnitude());
        SmartDashboard.putNumber(drivetrain.getName() + " state y pos", drivetrain.getState().Pose.getMeasureY().baseUnitMagnitude());
        SmartDashboard.putNumber(drivetrain.getName() + " state rot pos", drivetrain.getState().Pose.getRotation().getDegrees());
        
         // Controller Inputs
        SmartDashboard.putNumber(drivetrain.getName() + "xInput", xInput);
        SmartDashboard.putNumber(drivetrain.getName() + "yInput", yInput);
        SmartDashboard.putNumber(drivetrain.getName() + "rInput", rInput);

        // Calculated Speeds
        SmartDashboard.putNumber(drivetrain.getName() + "xSpeed", xSpeed);
        SmartDashboard.putNumber(drivetrain.getName() + "ySpeed", ySpeed);
        SmartDashboard.putNumber(drivetrain.getName() + "rSpeed", rSpeed);


    }
}