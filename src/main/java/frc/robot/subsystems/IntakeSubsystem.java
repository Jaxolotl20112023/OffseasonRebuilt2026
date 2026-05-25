package frc.robot.subsystems;

import java.lang.Thread.State;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.util.SparkFlexUtils;
import frc.lib.util.SparkMaxUtils;
import frc.robot.Constants;
import frc.robot.States.IntakeStates;

public class IntakeSubsystem extends SubsystemBase {
    // Spark Flex controlling ball intake
    // Spark Max controlling pivot of intake
    private final TalonFX m_rightIntake;
    private final TalonFX m_leftIntake; 
    public final SparkMax m_pivot; 
    // public final SparkMax m_leftPivot; 

    // Absolute Encorder to track pivot angle
    private final CANcoder pivotEncoder;
    // private final CANcoder leftPivotEncoder; 

    // PID controller to maintain pivot angle
    private final PIDController c_pivotPID;
    // private final ArmFeedforward c_ArmFeedforward; 
    
    // Current intake state
    private IntakeStates i_state;

    // Calculated PID Speed for pivot
    private double pivotSpeed;
    // private double leftPivotSpeed; 

    // Enable or disable subsystem
    private final boolean disable;
    private boolean override; 

    public IntakeSubsystem() {
        // Initialize Spark Flex, Spark Max motors, and Throguh Bore Cancoder
        m_rightIntake = new TalonFX(Constants.Intake.kRightIntakeID); 
        m_leftIntake = new TalonFX(Constants.Intake.kLeftIntakeID); 

        
        m_pivot = new SparkMax(Constants.Intake.kPivotID, MotorType.kBrushless);
        pivotEncoder = new CANcoder(Constants.Intake.kEncoderID);
        
        // m_leftPivot = new SparkMax(Constants.Intake.kLeftPivotID, MotorType.kBrushless);
        // leftPivotEncoder = new CANcoder(Constants.Intake.kLeftEncoderID); 

        // Optimize BUS usage
        // SparkFlexUtils.setSparkFlexBusUsage(m_rightIntake, SparkFlexUtils.Usage.kMinimal, IdleMode.kCoast, false, true);
        SparkMaxUtils.setSparkMaxBusUsage(m_pivot, SparkMaxUtils.Usage.kAll, IdleMode.kBrake, false, false);

        // Initialize PID controller for pivot
        c_pivotPID = new PIDController(Constants.Intake.kIntakeP, Constants.Intake.kIntakeI, Constants.Intake.kIntakeD);
        // c_ArmFeedforward = new ArmFeedforward(0, 0.028, 0.25);

        // Start intake in STOP position
        i_state = IntakeStates.STOP;
        setIntakeState(i_state);

        c_pivotPID.setTolerance(0.5);
        override = false; 

        // Disable Subsystem if set to true 
        disable = false;
        if(disable) {
            disableSubsystem();
        }
    }

    @Override
    public void periodic() { 

        pivotSpeed = c_pivotPID.calculate(getPiviotPosition()) ;
            // + c_ArmFeedforward.calculate(Units.degreesToRadians(getPiviotPosition()), pivotEncoder.getVelocity().getValueAsDouble());

        // leftPivotSpeed = c_pivotPID.calculate(getLeftPiviotPosition()) 
            // + c_ArmFeedforward.calculate(Units.degreesToRadians(getLeftPiviotPosition()), leftPivotEncoder.getVelocity().getValueAsDouble())  

        if(pivotSpeed < 0) {
            pivotSpeed *= 0.65;
        } 

        // if (leftPivotSpeed < 0) {
        //     leftPivotSpeed *= 0.65; 
        // }

        if(override) {
            pivotSpeed = 0.2;
            // leftPivotSpeed = -0.2; 
        }

        // if (getPiviotPosition() < Constants.Intake.kMin || getPiviotPosition() > Constants.Intake.kMax) {
        //     m_pivot.set(0);
        //     SmartDashboard.putBoolean(getName() + " disabled", true);
        // } else {
        m_pivot.set(pivotSpeed);
            // SparkMaxUtils.setSparkMaxBusUsage(m_pivot, SparkMaxUtils.Usage.kAll, IdleMode.kCoast, false, false);
        // }

        // m_leftPivot.set(leftPivotSpeed);
        setDashboardData();
    }


    // Adjust Subsytem to desired Intake states 
    public void setIntakeState(IntakeStates state) {
        i_state = state;
        
        // Set PID setpoint on intake to calculate motor output during periodic
        c_pivotPID.setSetpoint(state.pivotAngle);
        m_rightIntake.set(state.intakeSpeed); // try to invert the motor internally. 
        m_leftIntake.set(state.intakeSpeed);
    }

    public String getName() {
        return "Intake Subsystem";
    }

    // Return position of encoder in degrees
    // For some reasons it negative
    // Increases as you go up 
    public double getPiviotPosition() {
        return Units.rotationsToDegrees(pivotEncoder.getPosition().getValueAsDouble()) * -1 -60 * 4;
    }

    // public double getLeftPiviotPosition() {
    //     // return Units.rotationsToDegrees(leftPivotEncoder.getPosition().getValueAsDouble()) * -1 - 60 * 5.6333333333333; 
    //   }

    // Disable susbystem if needed
    public void disableSubsystem() {
        m_rightIntake.disable();
        m_leftIntake.disable();
        m_pivot.disable();
        // m_leftPivot.disable();
    }
    
    public boolean isPivotAtSetpoint() {
        return c_pivotPID.atSetpoint();
    }

    public IntakeStates getState() {
        return i_state;
    }

    public void setOverride(boolean on) {
        override = on;
    }

    // Set dashboard data for testing and debugging purposes
    private void setDashboardData() {
        // Put current state on the dashboard
        SmartDashboard.putString(getName() + " current state", i_state.toString());

        // Put motor speeds and pid setpoints
        SmartDashboard.putNumber(getName() + " pivot setpoint", c_pivotPID.getSetpoint());
        SmartDashboard.putNumber(getName() + " right pivot speed", pivotSpeed);
        // SmartDashboard.putNumber(getName() + " left pivot speed", leftPivotSpeed);
        // SmartDashboard.putNumber(getName() + " left pivot position", getLeftPiviotPosition());
        SmartDashboard.putNumber(getName() + " right intake speed", m_rightIntake.get());
        SmartDashboard.putNumber(getName() + " left intake speed", m_leftIntake.get()); 
        SmartDashboard.putNumber(getName() + " right piviot position", getPiviotPosition());
        SmartDashboard.putData(getName() + "pivot pid", c_pivotPID);

        SmartDashboard.putNumber(getName() + "pivot internal motor", m_pivot.getAbsoluteEncoder().getPosition());
        
        SmartDashboard.putBoolean(getName() + "at set point", c_pivotPID.atSetpoint());
    }
}
