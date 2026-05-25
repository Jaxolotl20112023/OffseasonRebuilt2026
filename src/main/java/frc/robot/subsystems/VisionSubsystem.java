package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.util.Utilities;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    // private final CameraServer cameraServer;
    private final CommandSwerveDrivetrain drivetrain; 

    private static LimelightHelpers.PoseEstimate s_poseEstimate; 
    private String limelightName; 
    
    private double robotYaw;
    public static double tag_distance; 
    public static int leastAmbiguity;

    public static SwerveDrivePoseEstimator swerveDrivePoseEstimator; 

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain, String limelightName){
        this.drivetrain = drivetrain;
        this.limelightName = limelightName;         

        this.robotYaw = 0; 

        LimelightHelpers.setPipelineIndex(limelightName, 1); // pipeline 1 will be used for 3D Vision
    }

    // @Override
    // public void periodic() {

    //     s_poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiRed(limelightName);  // before was megatag 2 blue
    //     megaTag2();
    // }

    // public void megaTag1() {

    //     if(s_poseEstimate == null) {
    //         return; 
    //     }
        
    //     if (s_poseEstimate.tagCount == 0
    //         || getTagAmbiguity(0) > Constants.Vision.ambiguityThreshold
    //         || getTagDistance() > Constants.Vision.distanceThreshold) {

    //         return; 
    //     } else if (s_poseEstimate.tagCount > 0) {
    //         robotYaw = Utilities.convertYawReadings(drivetrain.getPigeon2().getYaw().getValueAsDouble()); 
    //         tag_distance = s_poseEstimate.rawFiducials[0].distToCamera;

    //         LimelightHelpers.SetRobotOrientation(limelightName, robotYaw, 0, 0, 0, 0, 0);
    //     }
    // }

    public void megaTag2 () { 
        if(s_poseEstimate == null) {
            return;
        }
        
        if(s_poseEstimate.tagCount == 0  
            || getTagAmbiguity(0) > Constants.Vision.ambiguityThreshold
            || getTagDistance() > Constants.Vision.distanceThreshold) {
            return;
        } else if (s_poseEstimate.tagCount > 0) {
            robotYaw = Utilities.convertYawReadings(drivetrain.getPigeon2().getYaw().getValueAsDouble());
            tag_distance = s_poseEstimate.rawFiducials[0].distToCamera;

            LimelightHelpers.SetRobotOrientation(limelightName, robotYaw, 0, 0, 0, 0, 0);     
   
            drivetrain.setVisionMeasurementStdDevs(calculateStdDevs(getTagDistance()));
            drivetrain.addVisionMeasurement(
                    s_poseEstimate.pose, 
                    s_poseEstimate.timestampSeconds,
                    calculateStdDevs(tag_distance)
            );

            drivetrain.addVisionMeasurement(s_poseEstimate.pose, s_poseEstimate.timestampSeconds);
        }            
        setSmartDashboard();
    }

    public double getTagYaw() {
        return LimelightHelpers.getTX(limelightName); 
    }

    public static double getTagDistance(){
        if(s_poseEstimate.tagCount != 0) {
            return s_poseEstimate.rawFiducials[0].distToCamera;
        }
        return 0; 
    }

    public static double getTagAmbiguity(int limelightIndex) {
        return s_poseEstimate.rawFiducials[limelightIndex].ambiguity; 
    }

    // public static int getLeastAmbiguity() {
    //     double[] max = {getTagAmbiguity(0),0};

    //     for (int i=1;i<s_poseEstimate.tagCount;i++){
    //         if (getTagAmbiguity(i) > max[0]) {
    //             max[0] = getTagAmbiguity(i);
    //             max[1] = i; 
    //         }
    //     }
    //     return (int)max[1]; 
    // }

    private Matrix<N3, N1> calculateStdDevs(double distance) {
        // 1. Define your base trust (error in meters/radians when very close)
        double baseStdDevTrans = 0.05; // 5 centimeters
        double baseStdDevRot = 0.1;    // ~5.7 degrees

        // Adjust these based on your specific camera mounting height and light levels

        // Calculate the dynamic std dev using a quadratic curve (distance^2)
        double calculatedTrans = baseStdDevTrans + (0.24 * Math.pow(distance, 2));
        double calculatedRot = baseStdDevRot + (0.43 * Math.pow(distance, 2));

        // 4. Return the matrix for the Pose Estimator
        return VecBuilder.fill(calculatedTrans, calculatedTrans, calculatedRot);
    }

    private void setSmartDashboard() {
        // Get the yaw of the robot and the yaw from tag
        SmartDashboard.putNumber(limelightName+" Yaw from Tag", getTagYaw());
        SmartDashboard.putNumber("Yaw of Robot", robotYaw); 

        // Get the number of apritags and the distance from the closest one
        SmartDashboard.putNumber(limelightName+" Number of AprilTags", s_poseEstimate.tagCount);
        SmartDashboard.putNumber(limelightName+" Distance", tag_distance);
    }

}
