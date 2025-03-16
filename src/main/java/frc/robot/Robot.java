package frc.robot;

import java.io.File;
import java.nio.file.Paths;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.Constants;
import frc.robot.lib.ElasticLib;
import frc.robot.lib.Notification;
import frc.robot.lib.NotificationLevel;
import frc.robot.subsystems.RobotContainer;

public class Robot extends TimedRobot {

    private RobotContainer container;
    private NetworkTableInstance dashboardNt;
    private NetworkTableInstance.FloatPublisher matchTimePub;

    public Robot() {
        super(0.02);

        DriverStation.silenceJoystickConnectionWarning(!DriverStation.isFMSAttached());
        container = new RobotContainer();

        SignalLogger.enableAutoLogging(DriverStation.isFMSAttached());
        DataLogManager.start(0.2);
        DriverStation.startDataLog(DataLogManager.getLog());

        WebServer.getInstance().start(5800, getDeployDirectory());
        PortForwarder portForwarder = PortForwarder.getInstance();
        for (int i = 0; i < 10; i++) { // Forward limelight ports for use when tethered at events.
            portForwarder.add(5800 + i, Constants.VisionConstants.FRONT_CENTER + ".local", 5800 + i);
            portForwarder.add(5800 + i + 10, Constants.VisionConstants.BACK_CENTER + ".local", 5800 + i);
        }

        DataLogManager.log("Robot initialized");

        dashboardNt = NetworkTableInstance.getDefault().getTable("Elastic");
        matchTimePub = dashboardNt.getFloatTopic("Match Time").publish();
    }

    private static String getDeployDirectory() {
        if (new File("/home/lvuser").exists()) {
            return "/home/lvuser/py/deploy";
        } else {
            return Paths.get(System.getProperty("user.dir"), "deploy").toString();
        }
    }

    @Override
    public void robotPeriodic() {
        matchTimePub.set(Timer.getMatchTime());
    }

    @Override
    public void simulationPeriodic() {
    }

    @Override
    public void autonomousInit() {
        DataLogManager.log("Autonomous period started");

        Command selectedAuto = container.getAutonomousCommand();
        if (selectedAuto != null) {
            DataLogManager.log("Selected Auto: " + selectedAuto.getName());
            selectedAuto.schedule();
        }

        ElasticLib.selectTab("Autonomous");
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        DataLogManager.log("Autonomous period ended");
        ElasticLib.selectTab("Teleop");
    }

    @Override
    public void teleopInit() {
        DataLogManager.log("Teleoperated period started");
    }

    @Override
    public void teleopExit() {
        DataLogManager.log("Teleoperated period ended");
        if (DriverStation.isFMSAttached()) {
            ElasticLib.sendNotification(
                new Notification(
                    NotificationLevel.INFO.value(),
                    "Good match!",
                    DriverStation.getReplayNumber() > 1 ? "(again)" : ""
                )
            );
        }
    }

    @Override
    public void testInit() {
        DataLogManager.log("Test period started");
        CommandScheduler.getInstance().cancelAll();
        ElasticLib.selectTab("Debug");
    }

    @Override
    public void disabledInit() {
        SignalLogger.stop();
    }

    @Override
    public void testExit() {
        DataLogManager.log("Test period ended");
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
    }
}