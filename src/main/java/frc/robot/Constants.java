package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

public final class Constants {

    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout
            .loadField(AprilTagFields.k2025ReefscapeWelded);

    public static class CanIDs {
        public static final int LEFT_ELEVATOR_TALON = 10;
        public static final int RIGHT_ELEVATOR_TALON = 11;
        public static final int INTAKE_TALON = 12;
        public static final int LEFT_PIVOT_TALON = 13;
        public static final int RIGHT_PIVOT_TALON = 14;
        public static final int FUNNEL_TALON = 22;

        public static final int ELEVATOR_CANDI = 20;
        public static final int PIVOT_CANCODER = 21;
    }

    public static class ClimberConstants {
        public static final double GEAR_RATIO = 61504.0 / 189;
        public static final Slot0Configs GAINS = new Slot0Configs()
                .withKP(1.0)
                .withKI(0.0)
                .withKD(0.0)
                .withKS(0.0)
                .withKV(0.0)
                .withKA(0.0);

        public static final double VOLTAGE_INWARDS = 16;
        public static final double VOLTAGE_OUTWARDS = -4;

        public static final int SERVO_PORT = 0;
        public static final double SERVO_ENGAGED_ANGLE = 0;
        public static final double SERVO_DISENGAGED_ANGLE = 90;
    }

    public static class ElevatorConstants {
        public static final double L1_SCORE_POSITION = 2.208;
        public static final double L2_SCORE_POSITION = 1.841;
        public static final double L3_SCORE_POSITION = 3.576;
        public static final double L4_SCORE_POSITION = 6.087158;
        public static final double L2_ALGAE_POSITION = 3.198;
        public static final double L3_ALGAE_POSITION = 5;
        public static final double NET_SCORE_POSITION = 6.052246;
        public static final double ELEVATOR_MAX = 6.096924;

        public static final double DEFAULT_POSITION = 0;

        public static final double CRUISE_VELOCITY = 8;
        public static final double MM_UPWARD_ACCELERATION = 48;
        public static final double MM_BRAKE_ACCELERATION = 24;
        public static final double MM_DOWNWARD_ACCELERATION = 12;
        public static final double EXPO_K_V = 10;
        public static final double EXPO_K_A = 4;

        public static final double GEAR_RATIO = 31.0 / 4;
        public static final Slot0Configs GAINS = new Slot0Configs()
                .withKG(0.36)
                .withKP(40)
                .withKI(0.0)
                .withKD(0.0)
                .withKS(0.11)
                .withKV(0.0)
                .withKA(0.0)
                .withGravityType(GravityTypeValue.Elevator_Static);

        public static final double SETPOINT_TOLERANCE = 0.1;
    }

    public static class PivotConstants {
        public static final double INSIDE_ELEVATOR_ANGLE = 0.2;
        public static final double ELEVATOR_PRIORITY_ANGLE = 0.168;
        public static final double STOW_ANGLE = 0.188;
        public static final double GROUND_INTAKE_ANGLE = -0.081543;
        public static final double FUNNEL_INTAKE_ANGLE = 0.299;
        public static final double ALGAE_INTAKE_ANGLE = -0.05;
        public static final double HIGH_SCORING_ANGLE = 0.22;
        public static final double MID_SCORING_ANGLE = 0.22;
        public static final double LOW_SCORING_ANGLE = -0.081543;
        public static final double NET_SCORING_ANGLE = 0.123535;
        public static final double PROCESSOR_SCORING_ANGLE = 0.004639;
        public static final double CLIMBER_PRIORITY_ANGLE = 0.201943;

        public static final double MINIMUM_ANGLE = -0.091;
        public static final double MAXIMUM_ANGLE = 0.392822;

        public static final double CRUISE_VELOCITY = 3;
        public static final double MM_ACCELERATION = 3;

        public static final double GEAR_RATIO = 961.0 / 36;
        public static final Slot0Configs GAINS = new Slot0Configs()
                .withKG(0.27)
                .withKP(30)
                .withKI(0.0)
                .withKD(0.6343)
                .withKS(0.19)
                .withKV(0)
                .withKA(0)
                .withGravityType(GravityTypeValue.Arm_Cosine);

        public static final double CANCODER_DISCONTINUITY = 0.5;
        public static final double CANCODER_OFFSET = 0.380126953125;

        public static final double SETPOINT_TOLERANCE = 0.03125;
    }

    public static class IntakeConstants {
        public static final double CORAL_INTAKE_SPEED = 0.4 * 1.2 * 1.1;
        public static final double FUNNEL_INTAKE_SPEED = 0.8 * 0.75;
        public static final double CORAL_OUTPUT_SPEED = 0.425;

        public static final double ALGAE_INTAKE_SPEED = 1;
        public static final double ALGAE_OUTPUT_SPEED = -1;

        public static final double GEAR_RATIO = 4;
        public static final Slot0Configs GAINS = new Slot0Configs()
                .withKP(1.0)
                .withKI(0.0)
                .withKD(0.0)
                .withKS(0.0)
                .withKV(0.0)
                .withKA(0.0);
    }

    public static class VisionConstants {
        public static final String FRONT_LEFT = "limelight-fl";
        public static final String FRONT_RIGHT = "limelight-fr";
        public static final String FRONT_CENTER = "limelight-front";
        public static final String BACK_CENTER = "limelight-back";
    }

    public static class FunnelConstants {
        public static final double CORAL_STATION_POSITION = 0.098;
        public static final double STOWED_POSITION = 0;

        public static final double GEAR_RATIO = 192.0 / 7;

        public static final double CRUISE_VELOCITY = 1;

        public static final double SETPOINT_TOLERANCE = 0.01;

        public static final double MM_ACCELERATION = 3.5;

        public static final Slot0Configs GAINS = new Slot0Configs()
                .withKP(35)
                .withKI(0.0)
                .withKD(0.0)
                .withKS(0.0)
                .withKV(0.0)
                .withKA(0.0)
                .withKG(0.25)
                .withGravityType(GravityTypeValue.Arm_Cosine);

        public static final int SUPPLY_CURRENT = 20;
        public static final int STATOR_CURRENT = 50;
    }

    public static class AutoAlignConstants {
        public static final double MAX_DISTANCE = 3.6343;

        public static final double TRANSLATION_P = 12;
        public static final double TRANSLATION_I = 0;
        public static final double TRANSLATION_D = 0.1;

        public static final double HEADING_P = 2;
        public static final double HEADING_I = 0;
        public static final double HEADING_D = 0.2;

        public static final double HEADING_TOLERANCE = 2;

        public static final double VELOCITY_DEADBAND = 0.1;
        public static final double ROTATIONAL_DEADBAND = 0.02;
    }
}
