package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import java.util.function.Consumer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.pivot.PivotSubsystem;

public class RobotContainer {
    private final double maxSpeed = TunerConstants.getSpeedAt12Volts();
    private final double maxAngularRate = Units.rotationsToRadians(1);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController functionController = new CommandXboxController(1);
    private final Drivetrain drivetrain = TunerConstants.createDrivetrain();

    private final PivotSubsystem pivot = new PivotSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final FunnelSubsystem funnel = new FunnelSubsystem();
    private final VisionSubsystem vision = new VisionSubsystem(
        drivetrain,
        Constants.VisionConstants.FRONT_RIGHT,
        Constants.VisionConstants.FRONT_CENTER,
        Constants.VisionConstants.FRONT_LEFT,
        Constants.VisionConstants.BACK_CENTER
    );

    private final Superstructure superstructure = new Superstructure(
        drivetrain, pivot, elevator, funnel, vision
    );

    public RobotContainer() {
        setupSwerveRequests();
        pathplannerSetup();
        setupControllerBindings();
    }

    private void pathplannerSetup() {
        // Register NamedCommands
        NamedCommands.registerCommand("Default", superstructure.setGoalCommand(Superstructure.Goal.DEFAULT));
        NamedCommands.registerCommand("L4 Coral", superstructure.setGoalCommand(Superstructure.Goal.L4_CORAL));
        NamedCommands.registerCommand("L3 Coral", superstructure.setGoalCommand(Superstructure.Goal.L3_CORAL));
        NamedCommands.registerCommand("L2 Coral", superstructure.setGoalCommand(Superstructure.Goal.L2_CORAL));
        NamedCommands.registerCommand("L1 Coral", superstructure.setGoalCommand(Superstructure.Goal.L1_CORAL));
        NamedCommands.registerCommand("L2 Algae", superstructure.setGoalCommand(Superstructure.Goal.L2_ALGAE));
        NamedCommands.registerCommand("L3 Algae", superstructure.setGoalCommand(Superstructure.Goal.L3_ALGAE));
        NamedCommands.registerCommand("Processor", superstructure.setGoalCommand(Superstructure.Goal.PROCESSOR));
        NamedCommands.registerCommand("Net", superstructure.setGoalCommand(Superstructure.Goal.NET));
        NamedCommands.registerCommand("Funnel", superstructure.setGoalCommand(Superstructure.Goal.FUNNEL));
        NamedCommands.registerCommand("Floor", superstructure.setGoalCommand(Superstructure.Goal.FLOOR));

        NamedCommands.registerCommand("Hold", intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.HOLD));
        NamedCommands.registerCommand("Coral Intake", intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.CORAL_INTAKE));
        NamedCommands.registerCommand("Coral Output", intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.CORAL_OUTPUT));
        NamedCommands.registerCommand("Algae Intake", intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.ALGAE_INTAKE));
        NamedCommands.registerCommand("Algae Output", intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.ALGAE_OUTPUT));

        // Build AutoChooser
        AutoBuilder autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.onChange(() -> setCorrectSwervePosition());
        autoChooser.addOption("Basic Leave", drivetrain.applyRequest(() -> robotCentric.withVelocityX(1)).withTimeout(1.0));
        SmartDashboard.putData("Selected Auto", autoChooser);
    }

    private void setCorrectSwervePosition() {
        Command selected = autoChooser.getSelected();
        try {
            drivetrain.resetPose(flipPoseIfNeeded(selected.getStartingPose()));
            drivetrain.resetRotation(selected.getStartingPose().getRotation().plus(drivetrain.getOperatorForwardDirection()));
        } catch (Exception e) {
            // Handle exception
        }
    }

    private Pose2d flipPoseIfNeeded(Pose2d pose) {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            double flippedX = Constants.FIELD_LAYOUT.getFieldLength() - pose.getX();
            double flippedY = Constants.FIELD_LAYOUT.getFieldWidth() - pose.getY();
            Rotation2d flippedRotation = pose.getRotation().plus(Rotation2d.fromDegrees(180));
            return new Pose2d(flippedX, flippedY, flippedRotation);
        }
        return pose;
    }

    private void setupSwerveRequests() {
        Consumer<SwerveRequest> commonSettings = req -> req.withDeadband(maxSpeed * 0.01)
                .withRotationalDeadband(maxAngularRate * 0.01)
                .withDriveRequestType(SwerveModule.DriveRequestType.VELOCITY)
                .withSteerRequestType(SwerveModule.SteerRequestType.MOTION_MAGIC_EXPO);
        fieldCentric = commonSettings.apply(new FieldCentric());
        robotCentric = commonSettings.apply(new RobotCentric());
        brake = new SwerveDriveBrake();
        point = new PointWheelsAt();
    }

    public static Command rumbleCommand(CommandXboxController controller, double duration, double intensity) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> controller.setRumble(XboxController.RumbleType.kBothRumble, intensity)),
            new WaitCommand(duration),
            new InstantCommand(() -> controller.setRumble(XboxController.RumbleType.kBothRumble, 0))
        );
    }

    private void setupControllerBindings() {
        XboxController hid = driverController.getHID();
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> fieldCentric
                .withVelocityX(-hid.getLeftY() * maxSpeed)
                .withVelocityY(-hid.getLeftX() * maxSpeed)
                .withRotationalRate(-driverController.getRightX() * maxAngularRate)
            )
        );

        driverController.rightBumper().whileHeld(
            drivetrain.applyRequest(() -> robotCentric
                .withVelocityX(-hid.getLeftY() * maxSpeed)
                .withVelocityY(-hid.getLeftX() * maxSpeed)
                .withRotationalRate(-driverController.getRightX() * maxAngularRate)
            )
        );

        new Trigger(() -> driverController.getRightTriggerAxis() > 0.75).whileHeld(
            intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.CORAL_OUTPUT)
        ).whenInactive(
            intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.HOLD)
        );

        driverController.a().whileHeld(drivetrain.applyRequest(() -> brake));
        driverController.b().whileHeld(
            drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-hid.getLeftY(), -hid.getLeftX())))
        );

        driverController.leftBumper().whenPressed(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        setupSysidBindings(driverController, drivetrain, driverController.y(), driverController.a());
        setupSysidBindings(functionController, elevator, functionController.y(), functionController.a());
        setupSysidBindings(functionController, pivot, functionController.b(), functionController.x());

        Map<Trigger, Superstructure.Goal> goalBindings = Map.of(
            functionController.y(), Superstructure.Goal.L4_CORAL,
            functionController.x(), Superstructure.Goal.L3_CORAL,
            functionController.b(), Superstructure.Goal.L2_CORAL,
            functionController.a(), Superstructure.Goal.DEFAULT,
            functionController.y().and(functionController.start()), Superstructure.Goal.NET,
            functionController.x().and(functionController.start()), Superstructure.Goal.L3_ALGAE,
            functionController.b().and(functionController.start()), Superstructure.Goal.L2_ALGAE,
            functionController.a().and(functionController.start()), Superstructure.Goal.PROCESSOR,
            functionController.leftStick(), Superstructure.Goal.L1_CORAL
        );

        for (Map.Entry<Trigger, Superstructure.Goal> entry : goalBindings.entrySet()) {
            Trigger button = entry.getKey();
            Superstructure.Goal goal = entry.getValue();
            if (goal == Superstructure.Goal.L3_ALGAE || goal == Superstructure.Goal.NET || goal == Superstructure.Goal.L2_ALGAE || goal == Superstructure.Goal.PROCESSOR) {
                button.whileHeld(
                    superstructure.setGoalCommand(goal)
                    .alongWith(intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.ALGAE_INTAKE))
                ).whenInactive(intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.HOLD));
            } else {
                button.whenPressed(superstructure.setGoalCommand(goal));
            }
        }

        functionController.leftBumper().whenPressed(
            new ParallelCommandGroup(
                superstructure.setGoalCommand(Superstructure.Goal.FUNNEL),
                intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.FUNNEL_INTAKE)
            )
        ).whenInactive(
            new ParallelCommandGroup(
                superstructure.setGoalCommand(Superstructure.Goal.DEFAULT),
                intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.HOLD)
            )
        );

        functionController.leftBumper().and(functionController.back()).whileHeld(
            new ParallelCommandGroup(
                superstructure.setGoalCommand(Superstructure.Goal.FLOOR),
                intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.CORAL_INTAKE)
            )
        ).whenInactive(
            new ParallelCommandGroup(
                superstructure.setGoalCommand(Superstructure.Goal.DEFAULT),
                intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.HOLD)
            )
        );

        functionController.rightBumper().whileHeld(
            intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.CORAL_OUTPUT)
        ).whenInactive(
            intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.HOLD)
        );

        functionController.rightBumper().and(functionController.start()).whenPressed(
            intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.ALGAE_OUTPUT)
        ).whenInactive(
            intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.HOLD)
        );
    }

    private void setupSysidBindings(CommandXboxController controller, Subsystem subsystem, Trigger forwardBtn, Trigger reverseBtn) {
        Command forwardDynamic = subsystem.sysIdDynamic(SysIdRoutine.Direction.kForward);
        Command reverseDynamic = subsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse);
        Command forwardQuasistatic = subsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward);
        Command reverseQuasistatic = subsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse);

        forwardBtn.whenPressed(new InstantCommand(SignalLogger::start)).whileHeld(forwardDynamic.onlyIf(() -> !DriverStation.isFMSAttached() && DriverStation.isTest()));
        reverseBtn.whenPressed(new InstantCommand(SignalLogger::start)).whileHeld(reverseDynamic.onlyIf(() -> !DriverStation.isFMSAttached() && DriverStation.isTest()));

        controller.back().and(forwardBtn).whenPressed(new InstantCommand(SignalLogger::start)).whileHeld(forwardQuasistatic.onlyIf(() -> !DriverStation.isFMSAttached() && DriverStation.isTest()));
        controller.back().and(reverseBtn).whenPressed(new InstantCommand(SignalLogger::start)).whileHeld(reverseQuasistatic.onlyIf(() -> !DriverStation.isFMSAttached() && DriverStation.isTest()));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
