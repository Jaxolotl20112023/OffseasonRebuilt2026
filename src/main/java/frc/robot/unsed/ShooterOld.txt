package frc.robot.subsystems;

import java.util.zip.ZipEntry;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.internal.DriverStationModeThread;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.util.SparkFlexUtils;
import frc.lib.util.TalonFxUtils;
import frc.lib.util.Utilities;
// import frc.robot.CalculateShooterSpeed;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Vision;
import frc.robot.States.ShooterStates;

import java.util.Arrays;
import java.util.function.Supplier;

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

    // Suppliers for current x and y of Robot
    // private final Supplier<Double> swerveStateXSupplier;
    // private final Supplier<Double> swerveStateYSupplier;

    // Enable or disable subsystem
    private final boolean disable; 

    // Current shooter state
    private ShooterStates s_state;

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
    private final CommandSwerveDrivetrain x;

    public ShooterSubsystem(CommandSwerveDrivetrain x) { //Supplier<Double> x, Supplier<Double> y) {
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

        // Intialize Suppliers for the swerve state
        // swerveStateXSupplier = x;
        // swerveStateYSupplier = y; 

        this.x = x; 

        // Initialize shooter state to STOP 
        s_state = ShooterStates.STOP;
        setShooterState(s_state);

        desiredShooterRPS = 0;
        desiredBackSpinRPS = 0;
        
        setConfigs = new TalonFXConfiguration(); 

        // setShooterConfigs(100); 



        // Disable Subsystem if set to true 
        disable = false; 
        if(disable) {
            disableSubsystem();
        }
    }
    @Override
    public void periodic() { 
        setDashboardData();
    }

    public void setShooterState(ShooterStates state) {
        //stateDistance = checkShooterRange();
        s_state = state; 

        if(state.equals(ShooterStates.VARIABLE_SHOOT)) {
            // TODO: 
            distance = Units.metersToInches(VisionSubsystem.tag_distance);

        //    distance = Utilities.calculateDistanceToCenterPiece(x.getState().Pose.getX(), x.getState().Pose.getY());
        //    distance = Units.metersToInches(distance) + 13; 
            //checkShooterRange(distance)

            var sigma = calculateMotorSpeeds(distance); 
            setVelocitySetpoints(sigma[0], sigma[1]);

            return;
        } 

        setVelocitySetpoints(state.shootingRPS, state.backSpinRPS);
    }

    public String getName() {
        return "Shooter Subsystem";
    }

    private double[] calculateMotorSpeeds(double distance) {
        double[] speeds = new double[2];

        // Shooter Equations
        // Logistic Shooter
        // speeds[0] = Utilities.calculateLogisticShooterSpeed(distance);
        if(distance > 2.9718) {
            speeds[0] = Utilities.calculateCubicShooterSpeed(distance);
        } else if (distance < 2 && distance > 1) { 
           speeds[0] = Utilities.calculateCubicShooterSpeed(distance) * 0.91267;
        } else {
            speeds[0] = Utilities.calculateCubicBackSpinSpeed(distance) *  0.93267;
        }
        // Quadratic Shooter
        //speeds[0] = Utilities.calculateQuadraticShooterSpeed(distance);

        // Linear Shooter
        // speeds[0] = Utilities.calculateLinearShooterSpeed(distance);

        // Quartic Shooter
        // speeds[0] = Utilities.calculateQuarticShooterSpeed(distance);

        // Backspin Equations
        // Sinusoidal Backspin
        // speeds[1] = Utilities.calculcateSinsoidalBackSpinSpeed(distance);

        // Logistic Backspin
        //peeds[1] = Utilities.calculateLogisticBackSpinSpeed(distance);

        // Quadratic Backspin
        // speeds[1] = Utilities.calculateQuadraticBackSpinSpeed(distance);

        // Cubic Backspin
            
        if(distance > 2.9718) {
            speeds[1] = Utilities.calculateCubicBackSpinSpeed(distance);
        } else if (distance < 2 && distance > 1) { 
            speeds[1] = Utilities.calculateCubicBackSpinSpeed(distance) *  0.91267;
        } else {
            speeds[1] = Utilities.calculateCubicBackSpinSpeed(distance) *  0.93267;
        }
        // Quartic Backspin
        // speeds[1] = Utilities.calculcateQuarticBackSpinSpeed(distance);

        return speeds;
    }

    private double[] checkShooterRange(double distance) {
        if (distance <= 0.125) {
            return new double[] {
                ShooterStates.DISTANCE_0M.shootingRPS,
                ShooterStates.DISTANCE_0M.backSpinRPS
            };

        } else if (distance <= 0.25) {
            return new double[] {
                ShooterStates.DISTANCE_0_25M.shootingRPS,
                ShooterStates.DISTANCE_0_25M.backSpinRPS
            };

        } else if (distance <= 0.375) {
            return new double[] {
                ShooterStates.DISTANCE_0_25M.shootingRPS,
                ShooterStates.DISTANCE_0_25M.backSpinRPS
            };

        } else if (distance <= 0.5) {
            return new double[] {
                ShooterStates.DISTANCE_0_5M.shootingRPS,
                ShooterStates.DISTANCE_0_5M.backSpinRPS
            };

        } else if (distance <= 0.625) {
            return new double[] {
                ShooterStates.DISTANCE_0_5M.shootingRPS,
                ShooterStates.DISTANCE_0_5M.backSpinRPS
            };

        } else if (distance <= 0.75) {
            return new double[] {
                ShooterStates.DISTANCE_0_75M.shootingRPS,
                ShooterStates.DISTANCE_0_75M.backSpinRPS
            };

        } else if (distance <= 0.875) {
            return new double[] {
                ShooterStates.DISTANCE_0_75M.shootingRPS,
                ShooterStates.DISTANCE_0_75M.backSpinRPS
            };

        } else if (distance <= 1.0) {
            return new double[] {
                ShooterStates.DISTANCE_1M.shootingRPS,
                ShooterStates.DISTANCE_1M.backSpinRPS
            };

        } else if (distance <= 1.125) {
            return new double[] {
                ShooterStates.DISTANCE_1M.shootingRPS,
                ShooterStates.DISTANCE_1M.backSpinRPS
            };

        }else if (distance <= 1.25) {
            return new double[] {
                ShooterStates.DISTANCE_1_25M.shootingRPS,
                ShooterStates.DISTANCE_1_25M.backSpinRPS
            };

        }else if (distance <= 1.375) {
            return new double[] {
                ShooterStates.DISTANCE_1_25M.shootingRPS,
                ShooterStates.DISTANCE_1_25M.backSpinRPS
            };

        } else if (distance <= 1.5) {
            return new double[] {
                ShooterStates.DISTANCE_1_5M.shootingRPS,
                ShooterStates.DISTANCE_1_5M.backSpinRPS
            };

        } else if (distance <= 1.625) {
            return new double[] {
                ShooterStates.DISTANCE_1_5M.shootingRPS,
                ShooterStates.DISTANCE_1_5M.backSpinRPS
            };

        } else if (distance <= 1.75) {
           return new double[] {
                ShooterStates.DISTANCE_1_75M.shootingRPS,
                ShooterStates.DISTANCE_1_75M.backSpinRPS
            };

        } else if (distance <= 1.875) {
            return new double[] {
                ShooterStates.DISTANCE_1_75M.shootingRPS,
                ShooterStates.DISTANCE_1_75M.backSpinRPS
            };

        }else if (distance <= 2.0) {
            return new double[] {
                ShooterStates.DISTANCE_2M.shootingRPS,
                ShooterStates.DISTANCE_2M.backSpinRPS
            };

        } else if (distance <= 2.125) {
            return new double[] {
                ShooterStates.DISTANCE_2M.shootingRPS,
                ShooterStates.DISTANCE_2M.backSpinRPS
            };

        }else if (distance <= 2.25) {
            return new double[] {
                ShooterStates.DISTANCE_2_25M.shootingRPS,
                ShooterStates.DISTANCE_2_25M.backSpinRPS
            };

        } else if (distance <= 2.375) {
            return new double[] {
                ShooterStates.DISTANCE_2_25M.shootingRPS,
                ShooterStates.DISTANCE_2_25M.backSpinRPS
            };

        }else if (distance <= 2.5) {
            return new double[] {
                ShooterStates.DISTANCE_2_5M.shootingRPS,
                ShooterStates.DISTANCE_2_5M.backSpinRPS
            };
        }

        // Default fallback
        return new double[] {
            ShooterStates.FORWARD_SHOOT.shootingRPS,
            ShooterStates.FORWARD_SHOOT.backSpinRPS
        };
    }

    // Set the desired setpoints for all motors 
    // Will continue to move after this
    public void setVelocitySetpoints(double desiredShooterRPS, double desiredBackSpinRPS) {
        this.desiredShooterRPS = desiredShooterRPS; 
        this.desiredBackSpinRPS = desiredBackSpinRPS; 

        // Set control for desired shooter RPS
        c_backSpinPID.setSetpoint(Utilities.rpsToRpm(desiredBackSpinRPS), ControlType.kVelocity);
        m_leftShooter.setControl(new VelocityVoltage(-desiredShooterRPS));
        m_middleShooter.setControl(new VelocityVoltage(-desiredShooterRPS));
        m_rightShooter.setControl(new VelocityVoltage(-desiredShooterRPS));
    }

    // Get backspin speed in RPS 
    private double getBackSpinRPS() {
        return Utilities.rpmToRps(e_backSpin.getVelocity());
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
                SmartDashboard.putNumber(getName() + " Distance", distance);


        // PID Setpoint Values
        SmartDashboard.putNumber(getName() + " shooter PID setpoints", this.desiredShooterRPS);
        SmartDashboard.putNumber(getName() + " Back Spin PID Setpoint", this.desiredBackSpinRPS);

        // Distance to tag
        SmartDashboard.putNumber(getName() + " Distance to tag", this.desiredBackSpinRPS);
        // SmartDashboard.putNumber(getName() + " Current x", swerveStateXSupplier.get());
        // SmartDashboard.putNumber(getName() + " Current y", swerveStateYSupplier.get());


        // Current Shooter State
        SmartDashboard.putString(getName() + " Shooter State", s_state.toString());                      
    }
}