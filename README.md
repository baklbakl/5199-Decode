# Duct-Tape-and-a-Prayer repo
polarbub=Bryn  
BotDevPanda (Mr. Panda)=Kavish  
aruneoz=Haasini  
Annakhg=Anna  
CJ = cj-saez

### Old History
We are now on the `fixed_history` branch because it has the git history of the [FTC SDK](https://github.com/FIRST-Tech-Challenge/FtcRobotController), so it is much easier to update the SDK. All the old commits can still be found in the `Main` branch

### Folder Layout
`TeamCode` is our code  
`FtcRobotController` is the stub that combines all the 'closed source' FTC libraries into a robot controller app   
The FTC libraries aren't hosted by FIRST on GitHub, but they are on Maven Central as a sources jar. They have been unpacked in an easy to observe way [here](https://github.com/OpenFTC/Extracted-RC)

### Hardware
The FTC Control system is based around two Android devices and a motor controller. One of the Android devices is for the driver called the Driver Station (DS) and one for the robot called the Robot Controller (RC).
There are very strict rules for electronics for FTC. The only part of the system that is allowed to have its code changed is the RC App.
For example teams can't take a device based on a Raspberry Pi and put their own custom code or even install their own packages. 
Initially the Android devices were actually phones, but the list of allowable devices was very limited as they all had to have the same chipset to make it so all teams had the same amount of processing power.
Eventually REV Robotics, the manufacturer of the motor controller made their own versions of both of the devices with better performance and convenience than the phones, even though phones are still allowed most teams use the REV devices.

To control the robot itself a RC is used. The two main components of an RC are the Motor Controller, called by REV an Expansion hub, and the Android Device.
The RC can either consist of a Phone and an Expansion hub or a Control Hub which integrates both together. Either setup may have a extra Expansion Hub added to be able to interface more actuators and sensors.
The Control Hub is an Android TV devices not an Android Phone. The Expansion Hub is communicated with over RS485. When using a phone a FTDI chip is used to convert the Phone's USB to RS485, but when using a Control Hub the converter is more directly. 
The direct connection results in a performance increase because the main bottleneck in the RC is communication between the Android Device and the Expansion Hub. It can take 1 ms to set a motor power! 
The second Expansion Hub is connected to the RS485 bus via the header on the first one. Many phones only support 2.4 GHz, but the Control Hub supports 5 GHz as well.
2.4 GHz is prone to interference problems due to its lack of channels so 5 GHz is preferred

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
The stock FTC SDK uses version 7.2.0 of the Android Gradle Plugin (AGP). The mandates the use of Gradle 7.
This codebase has been updated to AGP version 8.6.1 as that is the latest that IntelliJ supports at the time of writing. 
This allows for latest release of Gradle, Gradle 8, to be used and for the compileSDK to be a modern SDK version.
The minSDK has been updated to SDK 25 as the Control Hub runs Android 7.1. SDK 25 is the newest one that works on Android 7.1.

### Hardware map
A critical part of controlling a robot is controlling the actuators. To get the actuator objects a Hardware Map is used.
This is set up on the RC and maps a string name to the physical ports on the robot. Some teams will name their actuators based on what they do, but this causes issues when the port that an actuator is plugged into changes.
The hardware map then has to be changed which can be finicky. To solve this issue our hardware map names motors M0-M7, servos S0-11, etc. and we have config class that has all of the HardwareMap name to use mappings.
[//]: # (ADD: How we do the hardwaremap now)

### Roadrunner
Roadrunner is the motion pathing library that we use.
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
