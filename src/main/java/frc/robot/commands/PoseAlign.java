package frc.robot.commands;

// import javax.swing.text.Utilities;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.derive;

import java.util.Optional;

import org.opencv.core.Mat;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.PhotonVisionVersoin2;
import frc.lib.util.Utilities;

public class PoseAlign extends Command{
    
    private Pose2d robotPose; 
    private double robotRotation; 
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
    private final boolean red_alliance;
    private boolean setPreSets = true; 

    private double robot_hub_angle;
    private double x_input; 
    private double y_input; 
    private double x_speed;
    private double y_speed;
    private double r_speed;
    private double x_distance; 
    private double y_distance; 


    public PoseAlign(CommandSwerveDrivetrain drivetrain, CommandXboxController driver0) { 
        this.drivetrain = drivetrain;
        this.driver0 = driver0; 

        ally = DriverStation.getAlliance(); 
        red_alliance = ally.get() == Alliance.Red ? true : false; 

        robotPose = drivetrain.getStateCopy().Pose;
        robotRotation = robotPose.getRotation().getDegrees();

        hubPose = Constants.FieldPoseConstants.hubPose; 
        hubRotation = hubPose.getRotation(); 

        x_input = 0;
        y_input = 0;

        x_speed = 0; 
        y_speed = 0; 
        r_speed = 0; 

        c_yawPID = new PIDController(Constants.Swerve.kSwerveP, Constants.Swerve.kSwerveI, Constants.Swerve.kSwerveD); 
        c_yawPID.enableContinuousInput(0, 360);

        addRequirements(drivetrain);

    }

    @Override
    public void initialize() {
        robotPose = drivetrain.getStateCopy().Pose; 
        robotRotation = robotPose.getRotation().getDegrees();
        robot_hub_angle = robotPose.getX() < Constants.FieldPoseConstants.hubPose.getX() 
            ? -get_angle_robot_hub()
            : get_angle_robot_hub();

        c_yawPID.setSetpoint(robot_hub_angle);
    }

    @Override
    public void execute() {
        robotPose = drivetrain.getStateCopy().Pose; 
        robotRotation = robotPose.getRotation().getDegrees();
        robot_hub_angle = get_angle_robot_hub();

        x_input = -driver0.getLeftX();
        y_input = -driver0.getLeftY();
        
        set_swerve_speeds();

        drivetrain.applyRequest(() -> drive.withVelocityX(y_speed)
            .withVelocityY(x_speed)
            .withRotationalRate(r_speed))
            .execute();

        setSmartDashboard();

    }

    public double get_angle_robot_hub() { 
        x_distance = Math.abs(robotPose.getX() - hubPose.getX()); 
        y_distance = Math.abs(robotPose.getY() - hubPose.getY()); 

        return red_alliance ? Units.radiansToDegrees(Math.atan(y_distance/x_distance)) : Units.radiansToDegrees(Math.atan(x_distance/y_distance)); 
    }

    public void set_swerve_speeds() {
        x_speed = Utilities.polynomialAccleration(y_input) * MaxSpeed * 0.8; 
        y_speed = Utilities.polynomialAccleration(x_input) * MaxSpeed * 0.8; 
        r_speed = c_yawPID.calculate(Utilities.processYaw(drivetrain.getPigeon2().getYaw().getValueAsDouble())); 

    }

    public void setSmartDashboard() {
        SmartDashboard.putNumber("Robot to hub angle: ", robot_hub_angle); 
        SmartDashboard.putNumber("Robot Yaw Setpoint: ", c_yawPID.getSetpoint());    
        SmartDashboard.putNumber("X-distance robot -> hub", x_distance);
        SmartDashboard.putNumber("Y-distance robot -> hub", y_distance);
        SmartDashboard.putNumber("Robot Pose X", robotPose.getX());
        SmartDashboard.putNumber("Robot Pose Y", robotPose.getY());

        SmartDashboard.putNumber("Rotation of Robot", robotRotation);
        SmartDashboard.putBoolean("Red-Alliance", red_alliance);
        SmartDashboard.putBoolean("Set Pre Sets", setPreSets);
    }
 }
