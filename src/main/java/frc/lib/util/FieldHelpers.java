package frc.lib.util;

import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.Constants;

public class FieldHelpers {
    private final Supplier<Double> robotXPos;
    private final Supplier<Double> robotYPos;
    
    private final double hubXPosition;
    private final double hubYPosition;

    private double yawToCenterPiece; 
    private double distanceToCenterPiece; 

    public FieldHelpers(Supplier<Double> robotXSupplier, Supplier<Double> robotYSupplier) {
        this.robotXPos = robotXSupplier;
        this.robotYPos = robotYSupplier;

        if(DriverStation.getAlliance().get() == null || DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            hubXPosition = Constants.Field.kBlueXPos;
            hubYPosition = Constants.Field.kBlueYPos; 
        } else {
            hubXPosition = Constants.Field.kRedXPos;
            hubYPosition = Constants.Field.kRedYPos; 
        }

        yawToCenterPiece = 0; 
        distanceToCenterPiece = 0; 
    }

    public double getYawToCenterPiece() {
        yawToCenterPiece = Math.atan((hubYPosition - robotXPos.get()) / (hubXPosition - robotYPos.get()));
        return Units.radiansToDegrees(yawToCenterPiece);
    }

    public double getDistanceToCenterPiece() {
        distanceToCenterPiece = Math.pow(hubYPosition - robotYPos.get(), 2) + Math.pow(hubXPosition - robotXPos.get(), 2);
        distanceToCenterPiece = Math.pow(distanceToCenterPiece, 0.5);
        return distanceToCenterPiece; 
    }
}