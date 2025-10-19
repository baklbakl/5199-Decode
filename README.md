# Duct-Tape-and-a-Prayer repo
polarbub=Bryn  
BotDevPanda (Mr. Panda)=Kavish  
aruneoz=Haasini  
Annakhg=Anna  
cj-saez=CJ


### Folder Layout
`TeamCode` is our code  
`FtcRobotController` is the stub that combines all the 'closed source' FTC libraries and a bit of glue code into the robot controller app 
`ExtractedLibs` contains an extracted version of the 'closed source' FTC libraries. This allows them to be easily inspected
and modified, but modifications are not allowed to be used in comp. See `ExtractedLibs/README.md` for more info   
`buildSrc` contains plugins that have common build logic for multiple subprojects

### Hardware
The FTC Control system is based around two Android devices and a motor controller. One of the Android devices is for the driver called the Driver Station (DS) and one for the robot called the Robot Controller (RC).
There are very strict rules for electronics for FTC. The only part of the system that is allowed to have its code changed is the RC App.
For example teams can't take a device based on a Raspberry Pi and put their own custom code or even install their own packages. 
Initially the Android devices were actually phones, but the list of allowable devices was very limited as they all had to have the same chipset to make it so all teams had the same amount of processing power.
Eventually REV Robotics, the manufacturer of the motor controller made their own versions of both of the devices with better performance and convenience than the phones. Even though phones are still allowed most teams use the REV devices.
To connect between the two devices WIFI is used.

To control the robot itself a RC is used. The two main components of an RC are the Motor Controller, called by REV an Hardware Interface Board (HIB), and the Android Device.
The RC can either consist of a Phone and an Expansion hub or a Control Hub which integrates both together. Either setup may have a extra Expansion Hub added to be able to control more actuators and sensors.
The Control Hub is an Android TV devices not an Android Phone. The Expansion Hub is communicated with over RS485. When using a phone a FTDI chip is used to convert the Phone's USB to RS485, but when using a Control Hub there is USB or converter needed and the connection is more direct and faster. 
The direct connection results in a performance increase because the main bottleneck in the RC is communication between the Android Device and the Expansion Hub. It can take 1 ms to set a motor power! 
The second Expansion Hub is connected to the RS485 bus via the header on the first one. 
Many phones only support 2.4 GHz WIFI, but the Control Hub supports 5 GHz as well.
2.4 GHz is prone to interference problems due to its lack of channels so 5 GHz is preferred.

The DS runs an app that connects via WiFi to the RC. The DS app allows for the OpMode to be selected, started, and stopped and for controller inputs to be sent to the RC. 
Controllers must be wired to prevent interference on 2.4 GHz and are connected to phones with USB-OTG dongles. These dongles make the phones unable to be charged while in use and also are fragile due to the USB Micro-B connector.
If a second controller was to be used a USB hub also had to be used. 
To solve these problems REV made their Driver Hub. It has 3 built in USB-As for the controllers and a separate USB-C for charging.

### WiFi
The WiFi modem in the Control Hub is able to act as an access point, so the DS connects to it as if it were a normal WiFi network,
but the WiFi modems in the Android phones that we use as the RC for the demobots don't have the abiliy to be an access point.
They work in wifi direct mode. To connect a DS to a phone RC go to Settings -> Pairing Method -> WiFi Direct. 
Then with the RC app open on the RC phone on the DS go to Pair with Robot Controller, turn off Filter WiFi devices, and select the name of the RC that you are trying to connect to.
You may have to wait a few seconds for it to show up and/or toggle Filter WiFi Devices on and off a few times 

#### Credentials
Control Hub WiFi Password `verygoodpassword`
The phone doesn't have a password as it uses WiFi Direct mode

### How the build system differs from the FTC SDK
#### App size reductions
32-bit libraries have been removed because it reduces the size of the app. This causes issues with the phones as they are 32-bit and can't use the 64-bit ones that the Control Hub can. See `ExtractedLibs/README.md` for more details on space savings that can be gained from using the extracted libs. <!--ADD: how to make a 32 bit build-->

#### AGP and minSDK
The stock FTC SDK uses version 8.7.0 of the Android Gradle Plugin (AGP). This codebase has been updated to AGP version 8.12.x to keep up with the latest patches and bug fixes. The minSDK has been updated to SDK 25 as the Control Hub runs Android 7.1. SDK 25 is the newest one that works on Android 7.1.  

#### Buildscripts
We have replaced the Groovy buildscripts with easier to maintain Kotlin ones. (Note the extension is `.gradle.kts` not `.gradle`). As well, we have taken the FTC libraries in tree (See `ExtractedLibs/README.md`) which adds a lot of duplicated build logic. Much of the duplicated logic is extracted out into plugins in the `buildSrc` directory.

#### How to update the FTC libraries
To update the AAR libraries
  1) Edit `gradle.properties` and update `SDK_MAJOR_VERSION`, `SDK_MINOR_VERSION`, and `SDK_POINT_VERSION` to the current version.
  2) Copy and paste `FTCRobotController/src/main/res/`, `FTCRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/internal/FtcOpModeRegister` and `FTCRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/internal/PermissionValidatorWrapper` from the [FIRST GitHub repo](https://github.com/FIRST-Tech-Challenge/FtcRobotController) to here
  3) In the repo check to see if there were any changes to `FTCRobotController/src/main/AndroidManifest.xml` or `FTCRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/internal/FtcRobotControllerActivity` and apply them here. If there are any conflicts try to figure out if our version has the changed code and if so how our version differs from stock. `ExtractedLibs/README.md` details the changes made and why. They can't be copied and pasted because there are changes in them from stock that would be lost by copying and pasting. 
4) You're done!

See `ExtractedLibs/README.md` for how to update the Extracted Libs

### Hardware map
A critical part of controlling a robot is controlling the actuators. Typically, to get the actuator objects a Hardware Map is used.
This is set up on the RC and maps a string name to the physical ports on the robot. Teams will normally name their actuators based on what they do. Using a hardware map causes issues if we change our RC device out because then we have to reenter the map in which can take quite some time. This happens quite often as we have two demo bots plus a main bot. 

In the past we solved this by naming each of the entries in the hardware map a letter based on their type (M for motors, S for servos, etc.) and then a number, but now we bypass the hardware map by manually searching for hubs on startup and then instantiating all the control classes we need ourselves in MainBotHardware.

### Roadrunner
Roadrunner is the motion pathing library that we use. The tuning process is largely automatic, but it can be inaccurate, so Bryn has devised a process to manually tune it which he is yet to write down ...  
#### Documentation
https://rr.brott.dev/docs/v1-0/guides/centerstage-auto/
https://cookbook.dairy.foundation/roadrunner_10/complete_trajectorybuilder_reference.html
https://learnroadrunner.com/
https://www.ctrlaltftc.com/

### Code Style
Camel case most things. Class, Interface, etc names are Capital Camel case, everything else for the most part is lower Camel Case. 
Brackets should be on the same line as their statement that starts them. See [this](https://en.wikipedia.org/wiki/Indentation_style#One_True_Brace) for more details

#### TODO Style
The todo regex can be changed in Android Studio in the settings under Editor -> TODO  
`ADD:` A feature needs to be added.  
`TEST:` A new feature needs to be tested.  
`FIX:` A bug needs to be fixed.  


### Software Tools
- [Android Studio](https://developer.android.com/studio) will make your life easier by installing and setting up the SDK which you need to get started.
- [REV Hardware Client](https://docs.revrobotics.com/rev-hardware-client/) can be used to configure update the apps on various Android devices and on the Expansion Hub.
It can also set the WiFi password of the control hub as it doesn't have a screen. You probably won't need this and it's Windows only.
- [git](https://git-scm.com/) is the tool that we use for version control.

## How to git
If you don't want to deal with using git in the terminal then you can use [GitHub Desktop](https://desktop.github.com/download/) instead

### Prerequisites
1) Install git
2) Make sure you have access to the GitHub repo (how are you reading this if you don't)
3) [Add you ssh public key to GitHub](https://docs.github.com/en/authentication/connecting-to-github-with-ssh). Keep in mind that the school's network blocks ssh, so you need to be on a personal hotspot or at home to use it.
4) Open a terminal and navigate to the folder that you want the repo folder it be in.
5) Run `git clone git@github.com:polarbub/FTC.git`
6) The `FTC` folder now has a local copy of the code

### Usage
To push your changes for everyone else to see do the following
1) Run `git status` in the terminal. This is optional but shows you what changes have been made to the code and what files are added
2) Run `git add <files to add>`. This tells git to keep track of the files you passed it. This must be ran for every new file. If you want to add all new files run `git add -A`
3) Run `git commit -a -m "<your commit message>"`. This tells git to take a snapshot of the code at this point in time. The message should contain a one to two sentence summery what was changed in this commit.
4) Run `git push` This tells git to send all the commits you have made to GitHub.

To get others changes run `git pull`

While doing either of those things you may run into an issue where git tells you that it cannot push/pull because the upstream has new commits that conflict with yours. This is because someone else has committed while you were working. To resolve this run: `git pull`. This gets any new commits from GitHub and syncs them to your copy.
You may have git tell you that there are conflicts. If you get in this state and don't know how to fix it feel free to ask Bryn.

If rebaseing is possible please try that before making a merge commit as it makes the commit history cleaner

There are many other things that can done with git and other ways to use it, but this is just a foundation to get you started.

## How to Android Studio
Open the folder in Android Studio to get started

### Building & Uploading
1) Press the play button
2) Press start the opmode on the driver station

### Where does println() go?
If you're running via Android Studio then it goes into the Logcat tab at the bottom.

### Debugging
Press the debug icon (looks like a bug) to the right of the run button. Sometimes it will time out. If it does launch the app 
normally and then to Run -> Attach Debugger to Android Process. 

## Extra Credit
For extra credit you can take the 'Bryn needs to know how it works' approach and use the actual tools that Android Studio uses. Don't do this it will take a bunch more time that it needs to.

### Software
- Newish JDK (at least 17)
- Gradle 8
- Android SDK and sources version 34 
- Android studio  

### Building
The Android app must be built with Gradle 8. 
The whole thing can be built with `gradle build` (or `./gradlew build` if you don't have the right version installed. 
It just grabs an old version of gradle). Most of the time you don't want this. All you want is to build the debug version
which can be done with the `gradle assembleDebug`. We won't be using release mode as that is for uploading to the play
store which we don't plan on doing. [Source](https://developer.android.com/build/building-cmdline)

### Uploading
Make sure you have debug mode enabled on the device you want to send it to. Then run `adb -d install TeamCode/build/outputs/apk/debug/TeamCode-debug.apk`
It should get automatically started by the driver station when you connect. The same procedure can be used with the [Driver Station APK](https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases) to update the Driver Station app without using the REV Hardware Client.

If you want gradle to install the Robot Controller App for you, you can use `gradle installDebug` to build (if your apk is out of date from your code)
and upload it to the device.

### Running
Sometimes the app won't start on its own, so you need to start it manually. Run `adb shell monkey -p com.qualcomm.ftcrobotcontroller 1` 

### Listing Devices
`adb devices`

### Connecting wirelessly
Initial Setup
1. Connect via USB
2. Run `adb tcpip 5555`  

Each Time
1. Connect to the `5199-RC` WiFi network
2. Run `adb connect 192.168.43.1:5555`
3. Click on the device drop down in Android Studio and it will show up

### Debugging
IDK
