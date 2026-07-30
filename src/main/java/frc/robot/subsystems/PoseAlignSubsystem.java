package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseAlignSubsystem extends SubsystemBase{
    
    private Pose2d robotPose; 
    private CommandSwerveDrivetrain drivetrain;

    public PoseAlignSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain; 
        robotPose = drivetrain.getStateCopy().Pose; 
    }

    @Override 
    public void periodic() {
        robotPose = drivetrain.getStateCopy().Pose; 
        SmartDashboard.putNumber("Robot angle", robotPose.getRotation().getDegrees()); 
    }
}
