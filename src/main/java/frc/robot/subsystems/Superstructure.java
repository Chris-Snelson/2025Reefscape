package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.funnel.FunnelSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.EnumMap;
import java.util.Map;

public class Superstructure extends SubsystemBase {
    public enum Goal {
        DEFAULT,
        L4_CORAL,
        L3_CORAL,
        L2_CORAL,
        L1_CORAL,
        L2_ALGAE,
        L3_ALGAE,
        PROCESSOR,
        NET,
        FUNNEL,
        FLOOR,
        CLIMBING
    }

    private static final Map<Goal, StateTuple> goalToStates = new EnumMap<>(Goal.class);

    static {
        goalToStates.put(Goal.DEFAULT, new StateTuple(PivotSubsystem.SubsystemState.STOW, ElevatorSubsystem.SubsystemState.DEFAULT, FunnelSubsystem.SubsystemState.DOWN));
        goalToStates.put(Goal.L4_CORAL, new StateTuple(PivotSubsystem.SubsystemState.HIGH_SCORING, ElevatorSubsystem.SubsystemState.L4, FunnelSubsystem.SubsystemState.DOWN));
        goalToStates.put(Goal.L3_CORAL, new StateTuple(PivotSubsystem.SubsystemState.L3_CORAL, ElevatorSubsystem.SubsystemState.L3, FunnelSubsystem.SubsystemState.DOWN));
        goalToStates.put(Goal.L2_CORAL, new StateTuple(PivotSubsystem.SubsystemState.L2_CORAL, ElevatorSubsystem.SubsystemState.L2, FunnelSubsystem.SubsystemState.DOWN));
        goalToStates.put(Goal.L1_CORAL, new StateTuple(PivotSubsystem.SubsystemState.LOW_SCORING, ElevatorSubsystem.SubsystemState.L1, FunnelSubsystem.SubsystemState.DOWN));
        goalToStates.put(Goal.L2_ALGAE, new StateTuple(PivotSubsystem.SubsystemState.ALGAE_INTAKE, ElevatorSubsystem.SubsystemState.L2_ALGAE, FunnelSubsystem.SubsystemState.DOWN));
        goalToStates.put(Goal.L3_ALGAE, new StateTuple(PivotSubsystem.SubsystemState.ALGAE_INTAKE, ElevatorSubsystem.SubsystemState.L3_ALGAE, FunnelSubsystem.SubsystemState.DOWN));
        goalToStates.put(Goal.PROCESSOR, new StateTuple(PivotSubsystem.SubsystemState.PROCESSOR_SCORING, ElevatorSubsystem.SubsystemState.DEFAULT, FunnelSubsystem.SubsystemState.DOWN));
        goalToStates.put(Goal.NET, new StateTuple(PivotSubsystem.SubsystemState.NET_SCORING, ElevatorSubsystem.SubsystemState.NET, FunnelSubsystem.SubsystemState.DOWN));
        goalToStates.put(Goal.FUNNEL, new StateTuple(PivotSubsystem.SubsystemState.FUNNEL_INTAKE, ElevatorSubsystem.SubsystemState.DEFAULT, FunnelSubsystem.SubsystemState.UP));
        goalToStates.put(Goal.FLOOR, new StateTuple(PivotSubsystem.SubsystemState.GROUND_INTAKE, ElevatorSubsystem.SubsystemState.DEFAULT, FunnelSubsystem.SubsystemState.DOWN));
        goalToStates.put(Goal.CLIMBING, new StateTuple(PivotSubsystem.SubsystemState.AVOID_CLIMBER, ElevatorSubsystem.SubsystemState.DEFAULT, FunnelSubsystem.SubsystemState.DOWN));
    }

    private final SwerveSubsystem drivetrain;
    private final PivotSubsystem pivot;
    private final ElevatorSubsystem elevator;
    private final FunnelSubsystem funnel;
    private final VisionSubsystem vision;

    private Goal goal;
    private ElevatorSubsystem.SubsystemState elevatorOldState;
    private PivotSubsystem.SubsystemState pivotOldState;
    private double pivotOldSetpoint;

    private final NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    private final NetworkTableInstance.Entry currentGoalPub = networkTableInstance.getTable("Superstructure").getEntry("Current Goal");

    private Mechanism2d superstructureMechanism;
    private MechanismRoot2d superstructureRoot;
    private MechanismLigament2d elevatorMech;
    private MechanismLigament2d pivotMech;

    public Superstructure(SwerveSubsystem drivetrain, PivotSubsystem pivot, ElevatorSubsystem elevator, FunnelSubsystem funnel, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.pivot = pivot;
        this.elevator = elevator;
        this.funnel = funnel;
        this.vision = vision;

        this.goal = Goal.DEFAULT;
        setGoalCommand(this.goal);

        this.elevatorOldState = this.elevator.getCurrentState();
        this.pivotOldState = this.pivot.getCurrentState();
        this.pivotOldSetpoint = this.pivot.getSetpoint();

        if (isSimulation()) {
            this.superstructureMechanism = new Mechanism2d(1, 5, new Color8Bit(0, 0, 105));
            this.superstructureRoot = this.superstructureMechanism.getRoot("Root", 1 / 2.0, 0.125);
            this.elevatorMech = this.superstructureRoot.append(new MechanismLigament2d("Elevator", 0.2794, 90, 5, new Color8Bit(194, 194, 194)));
            this.pivotMech = this.elevatorMech.append(new MechanismLigament2d("Pivot", 0.635, 90, 4, new Color8Bit(19, 122, 127)));
        }
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            return;
        }

        PivotSubsystem.SubsystemState pivotState = this.pivot.getCurrentState();
        ElevatorSubsystem.SubsystemState elevatorState = this.elevator.getCurrentState();

        if (!this.elevator.isAtSetpoint()) {
            this.pivot.setDesiredState(PivotSubsystem.SubsystemState.AVOID_ELEVATOR);
            this.pivot.freeze();
            if (this.pivot.isInElevator()) {
                this.elevator.setDesiredState(ElevatorSubsystem.SubsystemState.IDLE);
                this.elevator.freeze();
            }
        }

        if (!this.pivot.isInElevator() && pivotState == PivotSubsystem.SubsystemState.AVOID_ELEVATOR && elevatorState == ElevatorSubsystem.SubsystemState.IDLE) {
            this.elevator.unfreeze();
            this.elevator.setDesiredState(this.elevatorOldState);
        }

        if (this.elevator.isAtSetpoint() && pivotState == PivotSubsystem.SubsystemState.AVOID_ELEVATOR) {
            this.pivot.unfreeze();
            this.pivot.setDesiredState(this.pivotOldState);
        }

        if (pivotState != PivotSubsystem.SubsystemState.AVOID_ELEVATOR) {
            this.pivotOldState = pivotState;
            this.pivotOldSetpoint = this.pivot.getSetpoint();
        }

        if (elevatorState != ElevatorSubsystem.SubsystemState.IDLE) {
            this.elevatorOldState = elevatorState;
        }
    }

    @Override
    public void simulationPeriodic() {
        this.elevatorMech.setLength(this.elevator.getHeight());
        this.pivotMech.setAngle(this.pivot.getPosition() * 360 - 90);
    }

    private void setGoal(Goal goal) {
        this.goal = goal;

        StateTuple states = goalToStates.getOrDefault(goal, new StateTuple(null, null, null));
        if (states.pivotState != null) {
            this.pivot.setDesiredState(states.pivotState);
        }
        if (states.elevatorState != null) {
            this.elevator.setDesiredState(states.elevatorState);
        }
        if (states.funnelState != null) {
            this.funnel.setDesiredState(states.funnelState);
        }

        this.currentGoalPub.setString(goal.name());
    }

    public Command setGoalCommand(Goal goal) {
        return new InstantCommand(() -> setGoal(goal), this);
    }

    private static class StateTuple {
        final PivotSubsystem.SubsystemState pivotState;
        final ElevatorSubsystem.SubsystemState elevatorState;
        final FunnelSubsystem.SubsystemState funnelState;

        StateTuple(PivotSubsystem.SubsystemState pivotState, ElevatorSubsystem.SubsystemState elevatorState, FunnelSubsystem.SubsystemState funnelState) {
            this.pivotState = pivotState;
            this.elevatorState = elevatorState;
            this.funnelState = funnelState;
        }
    }

    private boolean isSimulation() {
        // Implement your simulation check here
        return false;
    }
}
