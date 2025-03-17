package frc.robot.generated;

import java.util.List;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.units.*;
import subsystems.swerve.SwerveSubsystem;
import wpimath.units.UnitConversions;

public class TunerConstants {
    // Both sets of gains need to be tuned to your individual robot

    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100)
        .withKI(0)
        .withKD(2.5)
        .withKS(0.1)
        .withKV(2.5811)
        .withKA(0.067726)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN);

    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(0.1)
        .withKI(0)
        .withKD(0)
        .withKS(0.18)
        .withKV(0.124);

    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.VOLTAGE;
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.VOLTAGE;

    private static final DriveMotorArrangement driveMotorType = DriveMotorArrangement.TALON_FX_INTEGRATED;
    private static final SteerMotorArrangement steerMotorType = SteerMotorArrangement.TALON_FX_INTEGRATED;

    private static final SteerFeedbackType steerFeedbackType = SteerFeedbackType.FUSED_CANCODER;

    private static final double slipCurrent = 90;

    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(60)
            .withStatorCurrentLimitEnable(true));
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    private static final Pigeon2Configuration pigeonConfigs = null;

    private static final CANBus canbus = new CANBus("Drivetrain", "./logs/drivetrain.hoot");

    private static final double speedAt12Volts = 4.2;
    public static double getSpeedAt12Volts() {
        return speedAt12Volts;
    }

    private static final double coupleRatio = 3.5714285714285716;

    private static final double driveGearRatio = 6.746031746031747;
    private static final double steerGearRatio = 21.428571428571427;
    private static final double wheelRadius = UnitConversions.inchesToMeters(2);

    private static final boolean invertLeftSide = false;
    private static final boolean invertRightSide = true;

    private static final int pigeonId = 9;

    private static final double steerInertia = 0.01;
    private static final double driveInertia = 0.01;
    private static final double steerFrictionVoltage = 0.2;
    private static final double driveFrictionVoltage = 0.2;

    private static final SwerveDrivetrainConstants drivetrainConstants = new SwerveDrivetrainConstants()
        .withCanBusName(canbus.getName())
        .withPigeon2Id(pigeonId)
        .withPigeon2Configs(pigeonConfigs);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constantsCreator = new SwerveModuleConstantsFactory<>()
        .withDriveMotorGearRatio(driveGearRatio)
        .withSteerMotorGearRatio(steerGearRatio)
        .withCouplingGearRatio(coupleRatio)
        .withWheelRadius(wheelRadius)
        .withSteerMotorGains(steerGains)
        .withDriveMotorGains(driveGains)
        .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
        .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
        .withSlipCurrent(slipCurrent)
        .withSpeedAt12Volts(speedAt12Volts)
        .withDriveMotorType(driveMotorType)
        .withSteerMotorType(steerMotorType)
        .withFeedbackSource(steerFeedbackType)
        .withDriveMotorInitialConfigs(driveInitialConfigs)
        .withSteerMotorInitialConfigs(steerInitialConfigs)
        .withEncoderInitialConfigs(encoderInitialConfigs)
        .withSteerInertia(steerInertia)
        .withDriveInertia(driveInertia)
        .withSteerFrictionVoltage(steerFrictionVoltage)
        .withDriveFrictionVoltage(driveFrictionVoltage);

    private static final int frontLeftDriveMotorId = 3;
    private static final int frontLeftSteerMotorId = 7;
    private static final int frontLeftEncoderId = 7;
    private static final double frontLeftEncoderOffset = 0.14990234375;
    private static final boolean frontLeftSteerMotorInverted = true;
    private static final boolean frontLeftEncoderInverted = false;

    private static final double frontLeftXPos = UnitConversions.inchesToMeters(12);
    private static final double frontLeftYPos = UnitConversions.inchesToMeters(12);

    private static final int frontRightDriveMotorId = 1;
    private static final int frontRightSteerMotorId = 5;
    private static final int frontRightEncoderId = 5;
    private static final double frontRightEncoderOffset = -0.28076171875;
    private static final boolean frontRightSteerMotorInverted = true;
    private static final boolean frontRightEncoderInverted = false;

    private static final double frontRightXPos = UnitConversions.inchesToMeters(12);
    private static final double frontRightYPos = UnitConversions.inchesToMeters(-12);

    private static final int backLeftDriveMotorId = 4;
    private static final int backLeftSteerMotorId = 8;
    private static final int backLeftEncoderId = 8;
    private static final double backLeftEncoderOffset = -0.173583984375;
    private static final boolean backLeftSteerMotorInverted = true;
    private static final boolean backLeftEncoderInverted = false;

    private static final double backLeftXPos = UnitConversions.inchesToMeters(-12);
    private static final double backLeftYPos = UnitConversions.inchesToMeters(12);

    private static final int backRightDriveMotorId = 2;
    private static final int backRightSteerMotorId = 6;
    private static final int backRightEncoderId = 6;
    private static final double backRightEncoderOffset = -0.06982421875;
    private static final boolean backRightSteerMotorInverted = true;
    private static final boolean backRightEncoderInverted = false;

    private static final double backRightXPos = UnitConversions.inchesToMeters(-12);
    private static final double backRightYPos = UnitConversions.inchesToMeters(-12);

    private static final SwerveModuleConstants frontLeft = constantsCreator.createModuleConstants(
        frontLeftSteerMotorId,
        frontLeftDriveMotorId,
        frontLeftEncoderId,
        frontLeftEncoderOffset,
        frontLeftXPos,
        frontLeftYPos,
        invertLeftSide,
        frontLeftSteerMotorInverted,
        frontLeftEncoderInverted
    );

    private static final SwerveModuleConstants frontRight = constantsCreator.createModuleConstants(
        frontRightSteerMotorId,
        frontRightDriveMotorId,
        frontRightEncoderId,
        frontRightEncoderOffset,
        frontRightXPos,
        frontRightYPos,
        invertRightSide,
        frontRightSteerMotorInverted,
        frontRightEncoderInverted
    );

    private static final SwerveModuleConstants backLeft = constantsCreator.createModuleConstants(
        backLeftSteerMotorId,
        backLeftDriveMotorId,
        backLeftEncoderId,
        backLeftEncoderOffset,
        backLeftXPos,
        backLeftYPos,
        invertLeftSide,
        backLeftSteerMotorInverted,
        backLeftEncoderInverted
    );

    private static final SwerveModuleConstants backRight = constantsCreator.createModuleConstants(
        backRightSteerMotorId,
        backRightDriveMotorId,
        backRightEncoderId,
        backRightEncoderOffset,
        backRightXPos,
        backRightYPos,
        invertRightSide,
        backRightSteerMotorInverted,
        backRightEncoderInverted
    );

    public static SwerveSubsystem createDrivetrain() {
        return new SwerveSubsystem(
            TalonFX.class,
            TalonFX.class,
            CANcoder.class,
            drivetrainConstants,
            List.of(frontLeft, frontRight, backLeft, backRight)
        );
    }
}