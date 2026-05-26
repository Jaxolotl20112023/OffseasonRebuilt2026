package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class Simulation extends SubsystemBase {

    private Field2d m_Field; 
    private CommandSwerveDrivetrain drivetrain; 
    private final SwerveDriveKinematics m_kinematics; 
    private final SwerveDriveOdometry m_odometry; 
    private final Pigeon2 m_pigeon; 
    private final Swerve swerve_properties; 

    
    public Simulation(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain; 
        
        swerve_properties = new Swerve(); 

        m_pigeon = new Pigeon2(Constants.Swerve.pigeon_id); 

        m_kinematics = new SwerveDriveKinematics(
            Constants.Swerve.m_front_left, Constants.Swerve.m_front_right, Constants.Swerve.m_back_left, Constants.Swerve.m_back_right
        ); 

        m_odometry = new SwerveDriveOdometry(
            m_kinematics, 
            m_pigeon.getRotation2d(), 
            swerve_properties.createModulePositions()
        );


        m_Field = new Field2d(); 
        SmartDashboard.putData("Field", m_Field);
    }

    @Override
    public void periodic() {
        m_odometry.update(m_pigeon.getRotation2d(), swerve_properties.createModulePositions()); 
        m_Field.setRobotPose(get_robot_pose());
    }

    public Pose2d get_robot_pose() {
        return m_odometry.getPoseMeters(); 
    }
}
