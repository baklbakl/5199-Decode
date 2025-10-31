package net.polarbub.ftc;

public class MainBotConfig {
    public static final double robotChassisWidth = 17;
    public static final double robotChassisLength = 14;

    public static final int leftFrontDriveID = 0;
    public static final int rightFrontDriveID = 1;
    public static final int leftRearDriveID = 2;
    public static final int rightRearDriveID = 3;
    public static final int intakeDriveID = 0;
    public static final int launchDriveID = 3;

    public static final int paraEncoder0ID = 3;
    public static final int paraEncoder1ID = 0;
    public static final int perpEncoderID = 1;

    public static final int moverServoID = 4;
    public static final int moverServo2ID = 5;
    public static final int blinkinServoID = 2;

    public static class MoveConfig {
        public static int motorPositionTolerance = 20;

        public static double restingServoPos = 0.5;
        public static double launchTicksPerSec = 1500;
        public static double intakeDrivePower = 1;
    }

    public static final String opmodeGroupName = "1 - TeleOp";
    public static final String demoGroupName = "2 - DemoBots";

    //The address of the expansion hub when attached via USB or RS485
    public static final int expansionHubAddress = 1;
}