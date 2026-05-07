package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.util.Utilities;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class VisionSubsystem1 extends SubsystemBase {
    private final CommandSwerveDrivetrain s_drivetrain;
    private final Pigeon2 s_pigeon; 

    private final String leftLimelight;
    private final String rightLimelight;

    private static double yawToCenterPiece; 
    private static double distanceToCenterTarget;

    private LimelightHelpers.PoseEstimate poseEstimate;

    public VisionSubsystem1(CommandSwerveDrivetrain drivetrain) {
        s_drivetrain = drivetrain;
        s_pigeon = drivetrain.getPigeon2();

        leftLimelight = Constants.Vision.leftLimelightName;
        rightLimelight = Constants.Vision.rightLimelightName;
    }

    @Override
    public void periodic() {
        runLimelight(leftLimelight);
        runLimelight(rightLimelight);
    }

    private void runLimelight(String name) {
        // Full LimelightHelpers set robot orientation maybe
        // LimelightHelpers.SetRobotOrientation(name, 
        //     s_pigeon.getYaw().getValueAsDouble(), 
        //     s_pigeon.getAngularVelocityZWorld().getValueAsDouble(), 
        //     s_pigeon.getPitch().getValueAsDouble(), 
        //     s_pigeon.getAngularVelocityYWorld().getValueAsDouble(),
        //     s_pigeon.getRoll().getValueAsDouble(), 
        //     s_pigeon.getAngularVelocityXWorld().getValueAsDouble());

        // Set robot orientation simple
        LimelightHelpers.SetRobotOrientation(name, 
            Utilities.convertYawReadings(s_pigeon.getYaw().getValueAsDouble()), 
            s_pigeon.getAngularVelocityZWorld().getValueAsDouble(), 
            0, 
            0,
            0, 
            0);

        // reject any measurements gathered when rotating too fast
        if(s_pigeon.getAngularVelocityZWorld().getValueAsDouble() > 360) {
            return;
        }

        // Estimate bot position relative to the filed using MT 2 
        poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);        
        
        // return if estimate not given or no tags detected
        if(poseEstimate == null || poseEstimate.tagCount == 0) {
            return; 
        }

        // Update robots odometry based on tag readings
        // Have a stv dev to calculate how much to use visions readings
        // TODO variable calculation on the stv dev
        s_drivetrain.addVisionMeasurement(
            poseEstimate.pose, 
            poseEstimate.timestampSeconds,
            VecBuilder.fill(.7,.7,9999999));

        distanceToCenterTarget = poseEstimate.rawFiducials[0].distToRobot;
        yawToCenterPiece = LimelightHelpers.getTX(name);
    }


    // Calculate how much to trust measurements based on distance
    private Matrix<N3, N1> calculateStdDevs(double distance) {
        double baseStdDevTrans = 0.05; // 5 centimeters
        double baseStdDevRot = 0.1;    // ~5.7 degrees

        // Calculate the dynamic std dev using a quadratic curve (distance^2)
        double calculatedTrans = baseStdDevTrans + (0.24 * Math.pow(distance, 2));
        double calculatedRot = baseStdDevRot + (0.43 * Math.pow(distance, 2));

        return VecBuilder.fill(calculatedTrans, calculatedTrans, calculatedRot);
    }

    public static double getTagDistance() {
        return distanceToCenterTarget;
    }

    public static double getTagYaw() {
        return yawToCenterPiece;
   }
}