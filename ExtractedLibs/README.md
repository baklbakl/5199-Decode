# Extracted Libraries
This directory contains the source code to all the FTC libraries that are provided by FIRST as .aar files. The source code is posted by FIRST alongside the .aar files. The code has been extracted and repackaged in a way that can be used in place of the .aar libraries. To easily browse the extracted code online visit https://github.com/OpenFTC/Extracted-RC. This allows them to be easily inspected
and modified, but modifications are not allowed to be used in comp, so please don't modify anything in here without a good reason as it will break our code when we try to switch back to aar libs for comp

To be able to use the extracted libraries instead of the aar ones the TeamCode and FTCRobotController subprojects have an aarLibs and an extractedLibs flavor. The build flavor can be changed by changing the build variant by going to Build -> Select build variant. The debug build type should be used along with whatever flavor.

OnBotJava and Blocks were disabled to save space. As well, all the unnecessary .wav files have been removed to save space. 
In total the extracted libs flavor is about 19.7 MB while the aar one is about 56 MB. The aar one without 32-bit libraries is about 46 MB.
This cuts upload time in at least half. See the main `README.md` for how to make a 32-bit build

## How to update 
1) Rebase in [Extracted RC](https://github.com/OpenFTC/Extracted-RC) to [our fork](https://github.com/DuckTapeAndAPrayer/Extracted-RC) of it.
2) Update the Extracted RC git submodule in this repo.
3) Update `SDK_VERSION_CODE` in gradle.properties with the version code from the pre-extracted code in `FtcRobotContoller/src/main/AndroidManifest.xml`

## Patches
These are the patches that were applied to the extracted libs

<!--FIX: Update extracted libs readme update section to talk about new extracted rc changes-->
### All files called `AndroidManifest.xml`
`package="com.qualcomm.ftccommon"` removed from `<manifest>` tag. This is now done with Gradle

### RobotCore
`assets/java` moved to OnBotJava
#### `AndroidManifest.xml`
OnBotJava service removed
`android:versionCode="114" and android:versionName="23.0"` removed from `manifest` tag
`<uses-sdk android:minSdkVersion="24" />` removed

### FTCRobotController
Removed `FTCRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/internal` because they were annoying to maintain and can be found online.  
Patched `FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/internal/FtcRobotControllerActivity.java` to use reflection when dealing with OnBotJava or Blocks to be able to handle being used without either present  
Removed `gold.wav` and `silver.wav` because they aren't used and take up space

#### `AndroidManifest.xml`
`android:versionCode="57" android:versionName="11.0"` removed from `manifest` tag. This is now done in Gradle in the TeamCode subproject

### FTCCommon
Deleted most of `src/main/res/raw/*.wav`. Some are left as they are needed