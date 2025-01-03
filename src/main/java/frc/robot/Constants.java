package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {

    public static final class SwerveConstants {
        public static final double WHEELBASE = 0.5; // meters
        public static final double TRACKWIDTH = 0.5; // meters
        public static final double MAX_SPEED = 4.0; // meters per second
        
        public static final int PIGEON_ID = 13; // Replace 13 with your actual CAN ID

        public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(WHEELBASE / 2, TRACKWIDTH / 2);
        public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(WHEELBASE / 2, -TRACKWIDTH / 2);
        public static final Translation2d BACK_LEFT_LOCATION = new Translation2d(-WHEELBASE / 2, TRACKWIDTH / 2);
        public static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(-WHEELBASE / 2, -TRACKWIDTH / 2);

        public static final int FRONT_LEFT_DRIVE_ID = 1;
        public static final int FRONT_LEFT_TURN_ID = 2;
        public static final int FRONT_LEFT_ENCODER_ID = 3;
        public static final double FRONT_LEFT_ENCODER_OFFSET = 0.0;

        public static final int FRONT_RIGHT_DRIVE_ID = 4;
        public static final int FRONT_RIGHT_TURN_ID = 5;
        public static final int FRONT_RIGHT_ENCODER_ID = 6;
        public static final double FRONT_RIGHT_ENCODER_OFFSET = 0.0;

        public static final int BACK_LEFT_DRIVE_ID = 7;
        public static final int BACK_LEFT_TURN_ID = 8;
        public static final int BACK_LEFT_ENCODER_ID = 9;
        public static final double BACK_LEFT_ENCODER_OFFSET = 0.0;

        public static final int BACK_RIGHT_DRIVE_ID = 10;
        public static final int BACK_RIGHT_TURN_ID = 11;
        public static final int BACK_RIGHT_ENCODER_ID = 12;
        public static final double BACK_RIGHT_ENCODER_OFFSET = 0.0;
    }
}
