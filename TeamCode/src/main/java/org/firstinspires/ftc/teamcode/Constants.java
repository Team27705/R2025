package org.firstinspires.ftc.teamcode;

public final class Constants {
    public static final class DriveConstants {
        // Motor names
        public static final String LEFT_FRONT_MOTOR = "leftFront";
        public static final String RIGHT_FRONT_MOTOR = "rightFront";

        public static final String LEFT_BACK_MOTOR = "leftBack";
        public static final String RIGHT_BACK_MOTOR = "rightBack";

        // Drive parameters
        public static final double MAX_SPEED = 1.0;
        public static final double MIN_SPEED = -1.0;

        public static double SPEED_MULTIPLIER = 1.0;
    }

    public static final class PIDConstants{
        public static double Kp = 0.0;        //Proportion
        public static double Ki = 0.0;        //Integral
        public static double Kd = 0.0;        //Derivative
    }

    public static final class ControllerConstants {
        public static final double STICK_DEADBAND = 0.1; // Ignore small stick movements
    }

    public static final class VisionConstants {
        public static final String WEBCAM_NAME = "Vision";
        public static final int RESOLUTION_WIDTH = 640;
        public static final int RESOLUTION_HEIGHT = 480;
        public static final double FX = 578.272;
        public static final double FY = 578.272;
        public static final double CX = 402.145;
        public static final double CY = 221.506;
    }
} 