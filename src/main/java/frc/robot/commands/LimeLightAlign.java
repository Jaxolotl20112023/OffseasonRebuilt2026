package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.lib.util.Utilities;

// Ensure smooth acceleration with rapid decleration 
public class LimeLightAlign extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController controller;
    private final VisionSubsystem limelight; 

    // Set max speeds for swerve driving and deadband
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private final double deadband = Constants.Swerve.kSwerveDeadband;

    // PID for auto tag yawing
    private final PIDController c_yawPID; 
    private double desiredYaw; 

    // Setting up bindings for necessary control of the swerve drive platform 
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * deadband) // Add a 10% deadband
            .withRotationalDeadband(MaxAngularRate * deadband) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    // Store inputs and speeds
    private double xInput, yInput; 
    private double xSpeed, ySpeed, rSpeed;
    private double yaw; 

    private double offset = 3; 

    // State of robot
    private Pose2d swerveState; 

    public LimeLightAlign(CommandSwerveDrivetrain drivetrain, CommandXboxController controller) {
        // Initialize drivetrain and controller
        this.drivetrain = drivetrain; 
        this.controller = controller;
        this.limelight = new VisionSubsystem(drivetrain, "limelight-four");

        // Intialize controller inputs to 0
        xInput = 0; 
        yInput = 0;

        // Intiatlize swerve speeds to 0 
        xSpeed = 0;
        ySpeed = 0;
        rSpeed = 0;

        // Configure PID        
        c_yawPID = new PIDController(Constants.Swerve.kSwerveP, Constants.Swerve.kSwerveI, Constants.Swerve.kSwerveD);
        c_yawPID.enableContinuousInput(0, 360);

        // Set requiremnts for the drivetrain subsystem to ensure no conflicts with other commands
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {

        // swerveState = drivetrain.getState().Pose; 

        try {
            yaw = limelight.getTagYaw();
        }
        catch (NullPointerException n) {
            SmartDashboard.putBoolean(getSubsystem()+" Tag Yaw Exception", true);
            return;
        }

        // if (swerveState.getX() > 2.46 ) {
        //     desiredYaw = yaw; //+ Utilities.processYaw(drivetrain.getPigeon2().getYaw().getValueAsDouble()); 
        // } else if (swerveState.getX() < 2.46 ){
        //     desiredYaw = yaw; //+ Utilities.processYaw(drivetrain.getPigeon2().getYaw().getValueAsDouble()); 
        // }

        // Set contoller speeds 
        xInput = -controller.getLeftY();
        yInput = -controller.getLeftX();

        
        // Set setpoint depending on desired yaw to center tag
        swerveState = drivetrain.getState().Pose;
        c_yawPID.setSetpoint(yaw);

        // Apply polynomial acceleration
        setSwerveSpeeds();

        // Apply speeds to the swerve drive
        drivetrain.applyRequest(() -> drive.withVelocityX(ySpeed)
            .withVelocityY(xSpeed)
            .withRotationalRate(rSpeed))
            .execute();

        setSmartDashboard();
    }
    
    // Apply a polynomial acceleration curve to the joystick inputs for smoother control
    // Calculate speed for yaw align
    private void setSwerveSpeeds() {
        xSpeed = Utilities.polynomialAccleration(yInput) * MaxSpeed * 0.8;
        ySpeed = Utilities.polynomialAccleration(xInput) * MaxSpeed * 0.8;
        rSpeed = c_yawPID.calculate(Utilities.processYaw(drivetrain.getPigeon2().getYaw().getValueAsDouble()));
    }

    public void setSmartDashboard() {
        SmartDashboard.putNumber("Swerve X Pose: ", swerveState.getX());
    }

    // Display important information for debugging
    
}