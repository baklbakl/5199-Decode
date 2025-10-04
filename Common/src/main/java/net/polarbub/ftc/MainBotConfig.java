package net.polarbub.ftc;

// Initialize the hardware variables. Note that the strings used here as parameters
// to 'get' must correspond to the names assigned during the robot configuration
// step (using the FTC Robot Controller app on the phone).
public class MainBotConfig {
    public static final double robotChassisWidth = 17;
    public static final double robotChassisLength = 14;

    public static final int leftFrontDriveID = 0;
    public static final int rightFrontDriveID = 1;
    public static final int leftRearDriveID = 2;
    public static final int rightRearDriveID = 3;
    public static final int pivotDriveID = 5;
    public static final int slideDriveID = 4;

    public static final int paraEncoder0ID = 3;
    public static final int paraEncoder1ID = 0;
    public static final int perpEncoderID = 1;

    public static final int intakeServoID = 0;
    public static final int intakeServo2ID = 1;
    public static final int blinkinServoID = 2;

    public static final int colorSensorID = 3;
    public static final int leftFrontDistID = 2;
    public static final int rightFrontDistID = 1;
    public static final int rightRearDistID = 0;
    public static double distSensorThresh = 0.8;

    public static class ArmConfig {
        public static int hangPivotCounts = -2400;

        public static int basketSlideCounts = 2400;
        public static int basketPivotCounts = -2350;

        public static int farIntakeSlideCounts = 2200;
        public static int farIntakePivotCounts = -700;

        public static int closeIntakeSlideCounts = 760;
        public static int closeIntakePivotCounts = -430;

        public static int hoverPivotCounts = -900;

        public static int specimenApproachSlideCounts = 540;
        public static int specimenAttachSlideCounts = 20;
        public static int specimenPivotCounts = -2410;

        public static int restingSlideCounts = 100;
        public static int restingPivotCounts = -110;

        public static double pivotHomingPower = 0.2;
        public static double slideHomingPower = -0.225;

        public static double restingServoPos = 0.5;
        public static double grabServoPos = 0;
        public static double expelServoPos = 1;
        public static double dropServoPos = 0.62;

        public static final double pivotPower = 0.8;
        public static final double slidePower = 1;
    }

    public static final int motorPositionTolerance = 20;

    public static final String opmodeGroupName = "1 - TeleOp";
    public static final String demoGroupName = "2 - DemoBots";
}