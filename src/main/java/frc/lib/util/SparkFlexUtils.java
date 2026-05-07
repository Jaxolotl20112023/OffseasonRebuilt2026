package frc.lib.util;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

/** Sets motor usage for a Spark Max motor controller. */
public class SparkFlexUtils {
  public enum Usage {
    kAll,
    kAbsolutePositionOnly,
    kAnalogPositionOnly,
    kRelativePositionOnly,
    kVelocityOnly,
    kMinimal
  };

  /**
   * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic status
   * frame period of nonessential frames from 20ms to 500ms.
   *
   * <p>See
   * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
   * for a description of the status frames.
   *
   * @param motor The motor to adjust the status frame periods on.
   * @param usage The status frame feedack to enable. kAll is the default when a CANSparkMax is
   *     constructed.
   * @param enableFollowing Whether to enable motor following.
   */
  public static void setSparkFlexBusUsage(SparkFlex motor, SparkFlexUtils.Usage usage, SparkBaseConfig.IdleMode idleMode, boolean enableFollowing, boolean setInverted) {
    SparkFlexUtils.sparkConfigurationBase configuration = new SparkFlexUtils.sparkConfigurationBase();
    SparkFlexConfig motorConfiguration = new SparkFlexConfig();

    if(enableFollowing) {
      configuration.setBusVoltageOn();
      configuration.setOutputCurrentOn();
      configuration.setMotorTemperatureOn();
      configuration.setFaultsOn();
      configuration.setWarningsOn();
      configuration.setIAccumulationOn();
      configuration.setLimitsOn();
      configuration.setAppliedOutputOn();
      configuration.setOutputCurrentOn();
    }

    switch (usage) {
      case kAll:
        configuration.setAllOn();
        break;
      case kAbsolutePositionOnly:
        configuration.setAbsoluteEncoderOn();
        break;
      case kAnalogPositionOnly:
        configuration.setAnalogOn();
        break;
      case kRelativePositionOnly:
        configuration.setPrimaryEncoderOn();
        break;
      case kVelocityOnly:
        configuration.setPrimaryEncoderOn();
        break;
      case kMinimal:
        break;
    }

    motorConfiguration.apply(configuration.build());
    motorConfiguration.inverted(setInverted); 
    motorConfiguration.idleMode(idleMode);
    motor.configure(motorConfiguration, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  public static void setSparkFlexBusUsage(SparkFlex motor, SparkFlexUtils.Usage usage, 
    SparkBaseConfig.IdleMode idleMode, boolean enableFollowing, boolean setInverted,
    double kP, double kI, double kD, double kV) {
    SparkFlexUtils.sparkConfigurationBase configuration = new SparkFlexUtils.sparkConfigurationBase();
    SparkFlexConfig motorConfiguration = new SparkFlexConfig();

    motorConfiguration.closedLoop.p(kP, ClosedLoopSlot.kSlot0);
    motorConfiguration.closedLoop.i(kI, ClosedLoopSlot.kSlot0);
    motorConfiguration.closedLoop.d(kD, ClosedLoopSlot.kSlot0);
    motorConfiguration.closedLoop.feedForward.kV(kV, ClosedLoopSlot.kSlot0);
    motorConfiguration.closedLoop.allowedClosedLoopError(0.02, ClosedLoopSlot.kSlot0);

    if(enableFollowing) {
      configuration.setBusVoltageOn();
      configuration.setOutputCurrentOn();
      configuration.setMotorTemperatureOn();
      configuration.setFaultsOn();
      configuration.setWarningsOn();
      configuration.setIAccumulationOn();
      configuration.setLimitsOn();
      configuration.setAppliedOutputOn();
      configuration.setOutputCurrentOn();
    }

    switch (usage) {
      case kAll:
        configuration.setAllOn();
        break;
      case kAbsolutePositionOnly:
        configuration.setAbsoluteEncoderOn();
        break;
      case kAnalogPositionOnly:
        configuration.setAnalogOn();
        break;
      case kRelativePositionOnly:
        configuration.setPrimaryEncoderOn();
        break;
      case kVelocityOnly:
        configuration.setPrimaryEncoderOn();
        break;
      case kMinimal:
        break;
    }

    motorConfiguration.apply(configuration.build());
    motorConfiguration.inverted(setInverted); 
    motorConfiguration.idleMode(idleMode);
    motor.configure(motorConfiguration, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  public static class sparkConfigurationBase {
    private final SignalsConfig signalConfiguration;

    public sparkConfigurationBase() {
      signalConfiguration = new SignalsConfig();
      signalConfiguration.absoluteEncoderPositionAlwaysOn(false);
      signalConfiguration.absoluteEncoderVelocityAlwaysOn(false);
      signalConfiguration.analogPositionAlwaysOn(false);
      signalConfiguration.analogVelocityAlwaysOn(false);
      signalConfiguration.analogVoltageAlwaysOn(false);
      signalConfiguration.externalOrAltEncoderPositionAlwaysOn(false);
      signalConfiguration.externalOrAltEncoderVelocityAlwaysOn(false);
      signalConfiguration.faultsAlwaysOn(false);
      signalConfiguration.iAccumulationAlwaysOn(false);
      signalConfiguration.primaryEncoderPositionAlwaysOn(false);
      signalConfiguration.primaryEncoderVelocityAlwaysOn(false);
      signalConfiguration.warningsAlwaysOn(false);

      signalConfiguration.absoluteEncoderPositionPeriodMs(500);
      signalConfiguration.absoluteEncoderVelocityPeriodMs(500);
      signalConfiguration.analogPositionPeriodMs(500);
      signalConfiguration.analogVelocityPeriodMs(500);
      signalConfiguration.analogVoltagePeriodMs(500);
      signalConfiguration.appliedOutputPeriodMs(500);
      signalConfiguration.busVoltagePeriodMs(500);
      signalConfiguration.externalOrAltEncoderPosition(500);
      signalConfiguration.externalOrAltEncoderVelocity(500);
      signalConfiguration.faultsPeriodMs(500);
      signalConfiguration.iAccumulationPeriodMs(500);
      signalConfiguration.limitsPeriodMs(500);
      signalConfiguration.motorTemperaturePeriodMs(500);
      signalConfiguration.outputCurrentPeriodMs(500);
      signalConfiguration.primaryEncoderPositionPeriodMs(500);
      signalConfiguration.primaryEncoderVelocityPeriodMs(500);
      signalConfiguration.warningsPeriodMs(500);
    }

    public void setAllOn() {
      signalConfiguration.absoluteEncoderPositionAlwaysOn(true);
      signalConfiguration.absoluteEncoderVelocityAlwaysOn(true);
      signalConfiguration.analogPositionAlwaysOn(true);
      signalConfiguration.analogVelocityAlwaysOn(true);
      signalConfiguration.analogVoltageAlwaysOn(true);
      signalConfiguration.externalOrAltEncoderPositionAlwaysOn(true);
      signalConfiguration.externalOrAltEncoderVelocityAlwaysOn(true);
      signalConfiguration.faultsAlwaysOn(true);
      signalConfiguration.iAccumulationAlwaysOn(true);
      signalConfiguration.primaryEncoderPositionAlwaysOn(true);
      signalConfiguration.primaryEncoderVelocityAlwaysOn(true);
      signalConfiguration.warningsAlwaysOn(true);

      signalConfiguration.absoluteEncoderPositionPeriodMs(20);
      signalConfiguration.absoluteEncoderVelocityPeriodMs(20);
      signalConfiguration.analogPositionPeriodMs(20);
      signalConfiguration.analogVelocityPeriodMs(20);
      signalConfiguration.analogVoltagePeriodMs(20);
      signalConfiguration.appliedOutputPeriodMs(20);
      signalConfiguration.busVoltagePeriodMs(20);
      signalConfiguration.externalOrAltEncoderPosition(20);
      signalConfiguration.externalOrAltEncoderVelocity(20);
      signalConfiguration.faultsPeriodMs(20);
      signalConfiguration.iAccumulationPeriodMs(20);
      signalConfiguration.limitsPeriodMs(20);
      signalConfiguration.motorTemperaturePeriodMs(20);
      signalConfiguration.outputCurrentPeriodMs(20);
      signalConfiguration.primaryEncoderPositionPeriodMs(20);
      signalConfiguration.primaryEncoderVelocityPeriodMs(20);
      signalConfiguration.warningsPeriodMs(20);
    }

    public void setAbsoluteEncoderOn() {
      signalConfiguration.absoluteEncoderPositionAlwaysOn(true);
      signalConfiguration.absoluteEncoderVelocityAlwaysOn(true);
      signalConfiguration.absoluteEncoderPositionPeriodMs(20);
      signalConfiguration.absoluteEncoderVelocityPeriodMs(20);
    }

    public void setAnalogOn() {
      signalConfiguration.analogPositionAlwaysOn(true);
      signalConfiguration.analogVelocityAlwaysOn(true);
      signalConfiguration.analogVoltageAlwaysOn(true);
      signalConfiguration.analogPositionPeriodMs(20);
      signalConfiguration.analogVelocityPeriodMs(20);
      signalConfiguration.analogVoltagePeriodMs(20);
    }

    public void setExternalOrAltEncoderOn() {
      signalConfiguration.externalOrAltEncoderPositionAlwaysOn(true);
      signalConfiguration.externalOrAltEncoderVelocityAlwaysOn(true);
      signalConfiguration.externalOrAltEncoderPosition(20);
      signalConfiguration.externalOrAltEncoderVelocity(20);
    }

    public void setFaultsOn() {
      signalConfiguration.faultsAlwaysOn(true);
      signalConfiguration.faultsPeriodMs(20);
    }

    public void setIAccumulationOn() {
        signalConfiguration.iAccumulationAlwaysOn(true);
        signalConfiguration.iAccumulationPeriodMs(20);
    }
    
    public void setPrimaryEncoderOn() {
      signalConfiguration.primaryEncoderPositionAlwaysOn(true);
      signalConfiguration.primaryEncoderVelocityAlwaysOn(true);
      signalConfiguration.primaryEncoderPositionPeriodMs(20);
      signalConfiguration.primaryEncoderVelocityPeriodMs(20);
    }

    public void setWarningsOn() {
      signalConfiguration.warningsAlwaysOn(true);
      signalConfiguration.warningsPeriodMs(20);
    }

    public void setAppliedOutputOn() {
      signalConfiguration.appliedOutputPeriodMs(20);
    }

    public void setBusVoltageOn() {
      signalConfiguration.busVoltagePeriodMs(20);
    }

    public void setLimitsOn() {
      signalConfiguration.limitsPeriodMs(20);
    }

    public void setMotorTemperatureOn() {
      signalConfiguration.motorTemperaturePeriodMs(20);
    }

    public void setOutputCurrentOn() {
      signalConfiguration.outputCurrentPeriodMs(20);
    }

    public SignalsConfig build() {
      return signalConfiguration;
    }
  }
}