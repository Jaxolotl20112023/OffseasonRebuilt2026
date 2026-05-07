package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.FieldHelpers;
import frc.lib.util.SparkFlexUtils;
import frc.lib.util.TalonFxUtils;
import frc.lib.util.Utilities;

import frc.robot.Constants;
import frc.robot.States.ShooterStates;

public class ShooterSubsystem extends SubsystemBase {
    // Different motors for each channel on the robot 
    private final TalonFX m_rightShooter;
    private final TalonFX m_middleShooter;
    private final TalonFX m_leftShooter; 

    // One backspin motor for all channels 
    private final SparkFlex m_backSpin;

    // Get backspin motor encoders for constant RPS control
    private final RelativeEncoder e_backSpin;

    // PID controllers to maintain backspin RPS
    private final SparkClosedLoopController c_backSpinPID;

    // Enable or disable subsystem
    private final boolean disable; 

    // Current shooter state
    private ShooterStates s_state;

    private CommandSwerveDrivetrain x; 

    // Current Motor Speeds
    private double rightMotorSpeed;
    private double middleMotorSpeed;
    private double leftMotorSpeed;
    private double backSpinSpeed;

    public double desiredShooterRPS; 
    public double desiredBackSpinRPS; 

    //Boolean usePIDonShooter; // CHANGE ME
    public double[] optimalShotsResult;

    // Iterator and the HashMap for the checking the distance
    private double distance;

    private TalonFXConfiguration setConfigs;

    public ShooterSubsystem(CommandSwerveDrivetrain x) { 
        // Initialize Kraken Motors 
        m_rightShooter = new TalonFX(Constants.Shooter.kRightShootingID);
        m_middleShooter = new TalonFX(Constants.Shooter.kMiddleShootingID);
        m_leftShooter = new TalonFX(Constants.Shooter.kLeftShootingID);

        // Configure Kraken RPS PID controllers
        TalonFxUtils.configureSlot0(m_leftShooter, Constants.Shooter.kShooterP, 
            Constants.Shooter.kShooterI, Constants.Shooter.kShooterD, 
            Constants.Shooter.kShooterS, Constants.Shooter.kShooterV);

        TalonFxUtils.configureSlot0(m_middleShooter, Constants.Shooter.kShooterP, 
            Constants.Shooter.kShooterI, Constants.Shooter.kShooterD, 
            Constants.Shooter.kShooterS, Constants.Shooter.kShooterV);

        TalonFxUtils.configureSlot0(m_rightShooter, Constants.Shooter.kShooterP, 
            Constants.Shooter.kShooterI, Constants.Shooter.kShooterD, 
            Constants.Shooter.kShooterS, Constants.Shooter.kShooterV);

        // Initialize SparkFlex Motors and optimize CAN BUS usage
        m_backSpin = new SparkFlex(Constants.Shooter.kBackSpinID, MotorType.kBrushless);
        SparkFlexUtils.setSparkFlexBusUsage(m_backSpin, SparkFlexUtils.Usage.kVelocityOnly, IdleMode.kCoast, 
            Constants.Shooter.kBackSpinEnable, Constants.Shooter.kBackSpinInvert,
            Constants.Shooter.kBackSpinP,Constants.Shooter.kBackSpinI, 
            Constants.Shooter.kBackSpinD, Constants.Shooter.kBackSpinV);

        // Get encoders for backspin motors
        e_backSpin = m_backSpin.getEncoder();

        // Initialize PID Controllers
        c_backSpinPID = m_backSpin.getClosedLoopController();

        m_rightShooter.setVoltage(40);
        m_middleShooter.setVoltage(40);
        m_leftShooter.setVoltage(40);
        m_backSpin.setVoltage(40);

        // Get Field helpers for distance to center piece calculation 

        // Initialize shooter state to STOP 
        s_state = ShooterStates.STOP;
        setShooterState(s_state);

        desiredShooterRPS = 0;
        desiredBackSpinRPS = 0;
        
        setConfigs = new TalonFXConfiguration(); 

        this.x = x;

        // Disable Subsystem if set to true 
        disable = false; 
        if(disable) {
            disableSubsystem();
        }
    }

    @Override
    public void periodic() { 
        var swerveState = x.getState().Pose;
        SmartDashboard.putNumber("nsoetahutnsaeou n198213", Utilities.calculateYawToCenterPiece(swerveState.getX(), swerveState.getY()));

        setDashboardData();
    }

    public void setShooterState(ShooterStates state) {
        s_state = state; 

        if(state.equals(ShooterStates.VARIABLE_SHOOT)) {
            // TODO: 
            // distance = Units.metersToInches(VisionSubsystem.tag_distance);
            // // distance = fieldHelper.getDistanceToCenterPiece(); 

            // //distance = Units.metersToInches(Utilities.calculateDistanceToCenterPiece(x.getState().Pose.getX(), x.getState().Pose.getY()));
            // SmartDashboard.putNumber("help me sigma 267", distance);
            // // distance = Units.metersToInches(distance) + 13; 

            // var sigma = calculateMotorSpeeds(distance); 
           // setVelocitySetpoints(sigma[0], sigma[1]);

            return;
        } 

        setVelocitySetpoints(state.shootingRPS, state.backSpinRPS);
    }

    public String getName() {
        return "Shooter Subsystem";
    }

    public ShooterStates getShooterState() {
        return s_state; 
    }

    private double[] calculateMotorSpeeds(double distance) {
        double[] speeds = new double[2];

        // Calculate shooter speed for the given distance
        if(distance > Units.metersToInches(2.9718)) {
            speeds[0] = Utilities.calculateCubicShooterSpeed(distance);
        } else if (distance < 2 && distance > 1) { 
           speeds[0] = Utilities.calculateCubicShooterSpeed(distance) * 0.91267;
        } else {
            speeds[0] = Utilities.calculateCubicBackSpinSpeed(distance) *  0.93267;
        }
        
        // Calculate the backspin speed for the given distance
        if(distance > Units.metersToInches(2.9718)) {
            speeds[1] = Utilities.calculateCubicBackSpinSpeed(distance);
        } else if (distance < 2 && distance > 1) { 
            speeds[1] = Utilities.calculateCubicBackSpinSpeed(distance) *  0.91267;
        } else {
            speeds[1] = Utilities.calculateCubicBackSpinSpeed(distance) *  0.93267;
        }

        return speeds;
    }

    // Set the desired setpoints for all motors 
    // Will continue to move after this
    public void setVelocitySetpoints(double desiredShooterRPS, double desiredBackSpinRPS) {
        this.desiredShooterRPS = desiredShooterRPS; 
        this.desiredBackSpinRPS = desiredBackSpinRPS; 

        // Set control for desired shooter RPS
        if(desiredBackSpinRPS != 0 ) {
            c_backSpinPID.setSetpoint(Utilities.rpsToRpm(desiredBackSpinRPS), ControlType.kVelocity);
        } else {
            m_backSpin.set(0);
        }

        if(desiredShooterRPS != 0) {
            m_leftShooter.setControl(new VelocityVoltage(-desiredShooterRPS));
            m_middleShooter.setControl(new VelocityVoltage(-desiredShooterRPS));
            m_rightShooter.setControl(new VelocityVoltage(-desiredShooterRPS));
        } else {
            m_leftShooter.set(0);
            m_middleShooter.set(0);
            m_rightShooter.set(0);
        }
    }

    // Get backspin speed in RPS 
    public double getBackSpinRPS() {
        return Utilities.rpmToRps(e_backSpin.getVelocity());
    }

    public double[] getShooterRPS() {
        double[] shooterSpeeds = {
            m_leftShooter.getVelocity().getValueAsDouble(), 
            m_middleShooter.getVelocity().getValueAsDouble(),
            m_rightShooter.getVelocity().getValueAsDouble()
        }; 

        return shooterSpeeds;
    }

    // Fully disable subsystem for testing purposes
    private void disableSubsystem() {
        m_rightShooter.disable();
        m_middleShooter.disable();
        m_leftShooter.disable();
        m_backSpin.disable();
    }

    // Set the max voltage output for the shooter motors (USE WHEN HIGH POWER CONSUMPTION FROM SHOOTERS)
    private void setShooterConfigs(double voltageLimit) {
        setConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        setConfigs.CurrentLimits.SupplyCurrentLimit = voltageLimit; 
        setConfigs.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;

        m_leftShooter.getConfigurator().apply(setConfigs); 
        m_middleShooter.getConfigurator().apply(setConfigs);
        m_rightShooter.getConfigurator().apply(setConfigs); 
    }   


    // Set dashboard data for testing and debugging purposes
    private void setDashboardData() {
        // Motor speeds
        SmartDashboard.putNumber(getName() + " Right Shooter Motor Speed", rightMotorSpeed);
        SmartDashboard.putNumber(getName() + " Middle Shooter Motor Speed", middleMotorSpeed);
        SmartDashboard.putNumber(getName() + " Left Shooter Motor Speed", leftMotorSpeed);
        SmartDashboard.putNumber(getName() + " Back Spin Motor Speed", backSpinSpeed);

        // Motor RPS
        SmartDashboard.putNumber(getName() + " Right Shooter RPS", m_rightShooter.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber(getName() + " Middle Shooter RPS", m_middleShooter.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber(getName() + " Left Shooter RPS", m_leftShooter.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber(getName() + " Back Spin RPS", getBackSpinRPS());

        // PID Setpoint Values
        SmartDashboard.putNumber(getName() + " shooter PID setpoints", this.desiredShooterRPS);
        SmartDashboard.putNumber(getName() + " Back Spin PID Setpoint", this.desiredBackSpinRPS);

        // Distance to tag
        SmartDashboard.putNumber(getName() + " Distance to tag", distance);

        // Current Shooter State
        SmartDashboard.putString(getName() + " Shooter State", s_state.toString());                      
    }
}