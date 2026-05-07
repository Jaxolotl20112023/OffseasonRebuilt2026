package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SparkFlexUtils;
import frc.lib.util.SparkMaxUtils;

import frc.robot.Constants;
import frc.robot.States.IndexStates;

public class IndexSubsystem extends SubsystemBase {
    // Intialize motors on the indexing subsystem
    // Conveyor is the front PVC that feeds into the indexor
    // Indexor feeds the balls into shooter 
    private final SparkMax m_conveyor;
    public final SparkFlex m_indexing; 
    
    // State of index subsystem
    private IndexStates i_state; 

    // Disable subsystem 
    private final boolean disable;


    public IndexSubsystem() {
        // Initalize Motors for Subsystem
        m_conveyor = new SparkMax(Constants.Indexor.kConveyorID, MotorType.kBrushless);
        m_indexing = new SparkFlex(Constants.Indexor.kIndexorID, MotorType.kBrushless);
        
        // Optimize BUS usage and set inversion
        SparkMaxUtils.setSparkMaxBusUsage(m_conveyor, SparkMaxUtils.Usage.kMinimal, IdleMode.kCoast, false, true);
        SparkFlexUtils.setSparkFlexBusUsage(m_indexing, SparkFlexUtils.Usage.kMinimal, IdleMode.kCoast, false, false);

        // Start indexor in STOP position
        i_state = IndexStates.STOP;
        setIndexState(i_state);

        // If needed disable subsytem 
        disable = false;
        if(disable) {
            disable();
        }


    }

    @Override
    public void periodic() {
        // Update dashboard data periodically
        setDashboardData();
    }

    public void setIndexState(IndexStates state) {
        // Update index state of subsytem 
        this.i_state = state; 
        
        // Set motor speeds according to the values in Index
        m_indexing.set(state.indexerSpeed);
        m_conveyor.set(state.conveyorSpeed);
    }

    // Get subsystem name
    public String getName() {
        return ("Index Subsystem");
    }

    // Return state
    public IndexStates getState() {
        return i_state;
    }

    // Disable the susbystem
    public void disable() {
        m_indexing.disable();
        m_conveyor.disable();
    }

    // Set dashboard data for testing and debugging purposes
    private void setDashboardData() {
        // Set current state of subsystem
        SmartDashboard.putString(getName() + " current state", i_state.toString());
        
        // Put motor speeds for all motors in the subsystem
        SmartDashboard.putNumber(getName() + " index speed", m_indexing.get());
        SmartDashboard.putNumber(getName() + " convyor speed", m_conveyor.get());
    }
}
