package frc.lib.util;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class TalonFxUtils {
    public static void configureSlot0(TalonFX motor, double kP, 
        double kI, double kD, double kS, double kV) {
            TalonFXConfiguration configs = new TalonFXConfiguration();
            Slot0Configs slot0 = configs.Slot0;

            slot0.kP = kP;
            slot0.kI = kI;
            slot0.kD = kD;

            slot0.kS = kS;
            slot0.kV = kV;

            motor.getConfigurator().apply(configs);
        }
}
