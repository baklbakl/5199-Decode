val extractedLibs = arrayOf("Blocks", "RobotCore", "FtcCommon", "Hardware", "Inspection", "OnBotJava", "RobotServer", "Vision")

include(":FtcRobotController")
include(":TeamCode")
include(":MeepMeepTesting")
include(":Common")

for(libName in extractedLibs) {
    include(libName)
    project(":" + libName).projectDir = file("ExtractedLibs/" + libName)
}

include("RoadRunner:core")
include("RoadRunner:ftc")
//project(":RoadRunner_core").projectDir = file("RoadRunner/core")
