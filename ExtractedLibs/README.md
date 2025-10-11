<!--ADD: Explanation of how to update and use extracted libs and how the build system differs from stock in general-->

## Patches
### RobotCore
`assets/java` moved to OnBotJava
`AndroidManifest.xml` OnBotJava service removed

### FTCRobotController
#### `org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity`
Removed references to OnBotJava and Blocks
Patched to make it work with extracted libs <!--How?-->

### FTCCommon
Deleted most of `src/main/res/raw/*.wav`. Some are left as they are needed directly