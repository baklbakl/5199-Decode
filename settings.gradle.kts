val extractedLibs = arrayOf("Blocks", "RobotCore", "FtcCommon", "Hardware", "Inspection", "OnBotJava", "RobotServer", "Vision")

include(":FtcRobotController")
include(":TeamCode")
include(":MeepMeepTesting")
include(":Common")

for(libName in extractedLibs) {
    include(libName)
    project(":" + libName).projectDir = file("ExtractedLibs/" + libName)
}
/*
include(":Blocks")
include(":RobotCore")
include(":FtcCommon")
include(":Hardware")
include(":Inspection")
include(":OnBotJava")
include(":RobotServer")
include(":Vision")*/
