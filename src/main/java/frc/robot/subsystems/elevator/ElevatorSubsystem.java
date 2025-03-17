package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.motorcontrol.ControlMode;
import com.ctre.phoenix6.motorcontrol.NeutralMode;
import com.ctre.phoenix6.motorcontrol.FeedbackDevice;
import com.ctre.phoenix6.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix6.motorcontrol.LimitSwitchNormal;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.EnumMap;
import java.util.Map;

public class ElevatorSubsystem extends SubsystemBase {
    public enum SubsystemState {
        IDLE,
        DEFAULT,
        L1,
        L2,
        L3,
        L4,
        L2_ALGAE,
        L3_ALGAE,
        NET
    }

    private static final Map<SubsystemState, Double> stateConfigs = new EnumMap<>(SubsystemState.class);

    static {
        stateConfigs.put(SubsystemState.DEFAULT, Constants.ElevatorConstants.DEFAULT_POSITION);
        stateConfigs.put(SubsystemState.L1, Constants.ElevatorConstants.L1_SCORE_POSITION);
        stateConfigs.put(SubsystemState.L2, Constants.ElevatorConstants.L2_SCORE_POSITION);
        stateConfigs.put(SubsystemState.L3, Constants.ElevatorConstants.L3_SCORE_POSITION);
        stateConfigs.put(SubsystemState.L4, Constants.ElevatorConstants.L4_SCORE_POSITION);
        stateConfigs.put(SubsystemState.L2_ALGAE, Constants.ElevatorConstants.L2_ALGAE_POSITION);
        stateConfigs.put(SubsystemState.L3_ALGAE, Constants.ElevatorConstants.L3_ALGAE_POSITION);
        stateConfigs.put(SubsystemState.NET, Constants.ElevatorConstants.NET_SCORE_POSITION);
        stateConfigs.put(SubsystemState.IDLE, null);
    }

    private final TalonFX masterMotor;
    private final TalonFX followerMotor;
    private final Debouncer atSetpointDebounce;
    private boolean atSetpoint;
    private double positionRequest;

    public ElevatorSubsystem() {
        masterMotor = new TalonFX(Constants.CanIDs.LEFT_ELEVATOR_TALON);
        followerMotor = new TalonFX(Constants.CanIDs.RIGHT_ELEVATOR_TALON);

        configureMotor(masterMotor);
        configureMotor(followerMotor);

        followerMotor.follow(masterMotor);

        atSetpointDebounce = new Debouncer(0.1, Debouncer.DebounceType.kRising);
        atSetpoint = true;

        masterMotor.setSelectedSensorPosition(Constants.ElevatorConstants.DEFAULT_POSITION);
    }

    private void configureMotor(TalonFX motor) {
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        motor.config_kP(0, Constants.ElevatorConstants.GAINS.kP);
        motor.config_kI(0, Constants.ElevatorConstants.GAINS.kI);
        motor.config_kD(0, Constants.ElevatorConstants.GAINS.kD);
        motor.configMotionCruiseVelocity(Constants.ElevatorConstants.CRUISE_VELOCITY);
        motor.configMotionAcceleration(Constants.ElevatorConstants.MM_DOWNWARD_ACCELERATION);
    }

    @Override
    public void periodic() {
        double latencyCompensatedPosition = masterMotor.getSelectedSensorPosition() + masterMotor.getSelectedSensorVelocity() * Timer.getFPGATimestamp();
        atSetpoint = atSetpointDebounce.calculate(Math.abs(latencyCompensatedPosition - positionRequest) <= Constants.ElevatorConstants.SETPOINT_TOLERANCE);
        SmartDashboard.putBoolean("At Setpoint", atSetpoint);
    }

    public void setDesiredState(SubsystemState desiredState) {
        Double position = stateConfigs.get(desiredState);

        if (position == null) {
            masterMotor.set(ControlMode.MotionMagic, masterMotor.getSelectedSensorPosition());
        } else {
            if (masterMotor.getSelectedSensorPosition() < position) {
                masterMotor.configMotionAcceleration(Constants.ElevatorConstants.MM_UPWARD_ACCELERATION);
            } else {
                masterMotor.configMotionAcceleration(Constants.ElevatorConstants.MM_DOWNWARD_ACCELERATION);
            }

            positionRequest = position;
            masterMotor.set(ControlMode.MotionMagic, position);
        }
    }

    public boolean isAtSetpoint() {
        return atSetpoint;
    }

    public Command stop() {
        return new InstantCommand(() -> masterMotor.set(ControlMode.MotionMagic, masterMotor.getSelectedSensorPosition()));
    }

    public double getHeight() {
        return (masterMotor.getSelectedSensorPosition() / Constants.ElevatorConstants.GEAR_RATIO) * (2 * Math.PI * 0.508);
    }
}
