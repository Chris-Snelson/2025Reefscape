package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.motorcontrol.InvertType;
import com.ctre.phoenix6.motorcontrol.FeedbackDevice;
import com.ctre.phoenix6.motorcontrol.StatusFrameEnhanced;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class PivotSubsystem extends SubsystemBase {
    public enum SubsystemState {
        IDLE,
        AVOID_ELEVATOR,
        STOW,
        GROUND_INTAKE,
        FUNNEL_INTAKE,
        ALGAE_INTAKE,
        HIGH_SCORING,
        L3_CORAL,
        L2_CORAL,
        LOW_SCORING,
        NET_SCORING,
        PROCESSOR_SCORING,
        AVOID_CLIMBER
    }

    private final TalonFX masterMotor;
    private final TalonFX followerMotor;
    private final CANcoder encoder;
    private final Timer debounceTimer;
    private boolean atSetpoint;
    private double positionRequest;

    public PivotSubsystem() {
        masterMotor = new TalonFX(Constants.CanIDs.LEFT_PIVOT_TALON);
        followerMotor = new TalonFX(Constants.CanIDs.RIGHT_PIVOT_TALON);
        encoder = new CANcoder(Constants.CanIDs.PIVOT_CANCODER);

        masterMotor.configFactoryDefault();
        followerMotor.getBaseTalon().configFactoryDefault();
        encoder.configFactoryDefault();

        masterMotor.setNeutralMode(NeutralModeValue.Brake);
        followerMotor.setNeutralMode(NeutralModeValue.Brake);

        masterMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
        masterMotor.configRemoteFeedbackFilter(encoder, 0);
        masterMotor.setInverted(InvertType.Clockwise);
        followerMotor.setInverted(InvertType.Clockwise);
        followerMotor.follow(masterMotor);

        debounceTimer = new Timer();
        debounceTimer.start();
        atSetpoint = true;
        positionRequest = 0.0;
    }

    @Override
    public void periodic() {
        double currentPosition = masterMotor.getSelectedSensorPosition();
        atSetpoint = Math.abs(currentPosition - positionRequest) <= Constants.PivotConstants.SETPOINT_TOLERANCE;
        SmartDashboard.putBoolean("At Setpoint", atSetpoint);
        SmartDashboard.putBoolean("In Elevator", isInElevator());
    }

    public void setDesiredState(SubsystemState desiredState) {
        Double position = Constants.PivotConstants.STATE_CONFIGS.get(desiredState);
        if (position == null) {
            masterMotor.set(ControlModeValue.PercentOutput, 0);
            return;
        }
        positionRequest = position;
        masterMotor.set(ControlModeValue.MotionMagic, positionRequest);
    }

    public boolean isAtSetpoint() {
        return atSetpoint;
    }

    public boolean isInElevator() {
        double position = masterMotor.getSelectedSensorPosition();
        return position >= Constants.PivotConstants.INSIDE_ELEVATOR_ANGLE;
    }

    public double getSetpoint() {
        return positionRequest;
    }

    public CommandBase stop() {
        return new CommandBase() {
            @Override
            public void initialize() {
                masterMotor.set(ControlModeValue.PercentOutput, 0);
            }
        };
    }

    public CommandBase sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return new CommandBase() {
            @Override
            public void initialize() {
                // Implement SysIdRoutine logic here
            }

            @Override
            public void end(boolean interrupted) {
                stop().schedule();
            }
        };
    }

    public CommandBase sysIdDynamic(SysIdRoutine.Direction direction) {
        return new CommandBase() {
            @Override
            public void initialize() {
                // Implement SysIdRoutine logic here
            }

            @Override
            public void end(boolean interrupted) {
                stop().schedule();
            }
        };
    }

    public double getPosition() {
        return masterMotor.getSelectedSensorPosition();
    }
}