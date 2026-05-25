package frc.robot.commands;

// import javax.swing.text.Utilities;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.PhotonVisionVersoin2;
import frc.lib.util.Utilities;

public class PoseAlign extends Command{
    
    private Pose2d robotPose; 
    private final Rotation2d robotRotation; 
    private final Pose2d hubPose; 
    private final Rotation2d hubRotation; 
    private final CommandSwerveDrivetrain drivetrain; 
    private final CommandXboxController driver0; 
    private final PIDController c_yawPID; 

    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private final double deadband = Constants.Swerve.kSwerveDeadband;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * deadband) // Add a 10% deadband
            .withRotationalDeadband(MaxAngularRate * deadband) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Optional<Alliance> ally; 

    private double robot_hub_angle;
    private double x_input; 
    private double y_input; 
    private double x_speed;
    private double y_speed;
    private double r_speed;


    public PoseAlign(CommandSwerveDrivetrain drivetrain, CommandXboxController driver0) { 
        this.drivetrain = drivetrain;
        this.driver0 = driver0; 

        ally = DriverStation.getAlliance(); 
        robotPose = drivetrain.getStateCopy().Pose;
        robotRotation = robotPose.getRotation();

        hubPose = Constants.FieldPoseConstants.hubPose; 
        hubRotation = hubPose.getRotation(); 

        x_input = 0;
        y_input = 0;

        x_speed = 0; 
        y_speed = 0; 
        r_speed = 0; 

        c_yawPID = new PIDController(Constants.Swerve.kP, Constants.Swerve.kI, Constants.Swerve.kD); 
        c_yawPID.enableContinuousInput(0, 360);

        addRequirements(drivetrain);

    }

    @Override
    public void execute() {

        robotPose = drivetrain.getStateCopy().Pose; 
        robot_hub_angle = get_angle_robot_hub();

        x_input = -driver0.getLeftX();
        y_input = -driver0.getLeftY();

        c_yawPID.setSetpoint(robot_hub_angle);
        
        set_swerve_speeds();

        drivetrain.applyRequest(() -> drive.withVelocityX(y_speed)
            .withVelocityY(x_speed)
            .withRotationalRate(r_speed))
            .execute();

    }

    public double get_angle_robot_hub() { 
        double x_distance = robotPose.getX() - hubPose.getX(); 
        double y_distance = robotPose.getY() - hubPose.getY(); 

        return ally.get() == Alliance.Red ? Math.atan(y_distance/x_distance) : Math.atan(x_distance/y_distance); 
    }

    public void set_swerve_speeds() {
        x_speed = Utilities.polynomialAccleration(y_input) * MaxSpeed * 0.8; 
        y_speed = Utilities.polynomialAccleration(x_input) * MaxSpeed * 0.8; 
        r_speed = c_yawPID.calculate(Utilities.processYaw(drivetrain.getPigeon2().getYaw().getValueAsDouble())); 

    }
 }
