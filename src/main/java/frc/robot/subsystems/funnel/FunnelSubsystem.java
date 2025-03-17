package frc.robot.subsystems.funnel;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.system.plant.DCMotor;

import java.util.EnumMap;
import java.util.Map;

import frc.robot.Constants;

public class FunnelSubsystem extends SubsystemBase {
    public enum SubsystemState {
        UP,
        DOWN
    }

    private static final Map<SubsystemState, Double> stateConfigs = new EnumMap<>(SubsystemState.class);
    static {
        stateConfigs.put(SubsystemState.UP, Constants.FunnelConstants.CORAL_STATION_POSITION);
        stateConfigs.put(SubsystemState.DOWN, Constants.FunnelConstants.STOWED_POSITION);
    }

    private final TalonFX funnelMotor;
    private final MotionMagicVoltage positionRequest;
    private final VoltageOut brakeRequest;

    public FunnelSubsystem() {
        super();

        funnelMotor = new TalonFX(Constants.CanIDs.FUNNEL_TALON);

        TalonFXConfiguration funnelConfig = new TalonFXConfiguration();
        funnelConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        funnelConfig.primaryPID.sensorToMechanismRatio = Constants.FunnelConstants.GEAR_RATIO;
        funnelConfig.motorOutput.inverted = InvertType.Clockwise;
        funnelConfig.motorOutput.neutralMode = NeutralMode.Brake;
        funnelConfig.Slot0.k = Constants.FunnelConstants.GAINS;

        // TODO what is this? funnelSlot0.kF = Constants.FunnelConstants.GAINS.kF;
        funnelConfig.Slot0 = funnelSlot0;
        funnelConfig.motionMagicCruiseVelocity = Constants.FunnelConstants.CRUISE_VELOCITY;
        funnelConfig.motionMagicAcceleration = Constants.FunnelConstants.MM_ACCELERATION;
        funnelConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, Constants.FunnelConstants.SUPPLY_CURRENT, Constants.FunnelConstants.SUPPLY_CURRENT, 0);
        funnelConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, Constants.FunnelConstants.STATOR_CURRENT, Constants.FunnelConstants.STATOR_CURRENT, 0);

        funnelMotor.configAllSettings(funnelConfig);

        funnelMotor.setSelectedSensorPosition(0);

        DCMotorSim funnelSim = new DCMotorSim(DCMotor.getFalcon500(1), Constants.FunnelConstants.GEAR_RATIO, 1.0);
        addChild("FunnelSim", funnelSim);

        positionRequest = new MotionMagicVoltage(0);
        brakeRequest = new VoltageOut(0);
    }

    public void setDesiredState(SubsystemState desiredState) {
        if (desiredState == null) {
            return;
        }

        Double position = stateConfigs.getOrDefault(desiredState, Constants.FunnelConstants.STOWED_POSITION);
        positionRequest.setPosition(position);
        funnelMotor.set(ControlMode.MotionMagic, positionRequest.getPosition());
    }
}

