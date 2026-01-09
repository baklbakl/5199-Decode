package org.ducktapeandaprayer.ftc

//import com.acmerobotics.roadrunner.Pose2d
//import com.acmerobotics.roadrunner.*
import com.acmerobotics.roadrunner.*
import kotlin.math.PI

//@JvmOverloads
//fun actionBuilder(beginPose: Pose2d, assumeTangentHeading: Boolean) : TrajectoryActionBuilder{
//    TrajectoryActionBuilder(
//        ::TurnAction,
//        ::TrajectoryAction,
//        beginPose,
////        eps = 1e-6,
//        beginEndVel = 0.0,
//        TurnConstraints(PI / 2, -PI / 2, PI / 2),
//        MinVelConstraint(
//            listOf(
//                TranslationalVelConstraint(40.0),
//                if (assumeTangentHeading) {
//                    object : VelConstraint {
//                        val base = AngularVelConstraint(PI / 2)
//                        override fun maxRobotVel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double) = path[s, 3].let {
//                            base.maxRobotVel(Pose2dDual(it.position, it.position.drop(1).angleCast()), path, s)
//                        }
//                    }
//                } else {
//                    AngularVelConstraint(PI / 2)
//                },
//            )
//        ),
//        ProfileAccelConstraint(-30.0, 50.0),
//        dispResolution = 0.25,
//        angResolution = 0.1,
//    )
//}


fun fillBuilder(builder: TrajectoryActionBuilder, startPos: Pose2d) :TrajectoryActionBuilder {
//    constructor(
//        turnActionFactory: TurnActionFactory,
//        trajectoryActionFactory: TrajectoryActionFactory,
//        trajectoryBuilderParams: TrajectoryBuilderParams,
//        beginPose: Pose2d,
//        beginEndVel: Double,
//        baseTurnConstraints: TurnConstraints,
//        baseVelConstraint: VelConstraint,
//        baseAccelConstraint: AccelConstraint,
//        poseMap: PoseMap = IdentityPoseMap(),
//    ) :

    val assumeTangentHeading = true

    val newBuilder = TrajectoryActionBuilder(
        builder.turnActionFactory,
        builder.trajectoryActionFactory,
        builder.trajectoryBuilderParams,
        startPos,
        0.0,
        builder.baseTurnConstraints,
        MinVelConstraint(
            listOf(
                TranslationalVelConstraint(40.0),
                object : VelConstraint {
                    val base = AngularVelConstraint(PI / 2)
                    override fun maxRobotVel(robotPose: Pose2dDual<Arclength>, path: PosePath, s: Double) = path[s, 3].let {
                        base.maxRobotVel(Pose2dDual(it.position, it.position.drop(1).angleCast()), path, s)
                    }
                }
               ,
            )
        ),
        builder.baseAccelConstraint,
    )

    return newBuilder


//    val original = (Pose2d(-30.0, 0.0, 0.0), assumeTangentHeading = true)
//        .splineToConstantHeading(Vector2d(10.0, 0.0), 0.0)
//        .splineToConstantHeading(Vector2d(15.0, 5.0), PI / 2)
//        .splineToConstantHeading(Vector2d(10.0, 10.0), PI)
//        .splineToConstantHeading(Vector2d(-30.0, 10.0), PI)
//        .build()
//
//    val tangent = actionBuilder(Pose2d(-30.0, 0.0, 0.0), assumeTangentHeading = false)
//        .splineTo(Vector2d(10.0, 0.0), 0.0)
//        .splineTo(Vector2d(15.0, 5.0), PI / 2)
//        .splineTo(Vector2d(10.0, 10.0), PI)
//        .splineTo(Vector2d(-30.0, 10.0), PI)
//        .build()

//    showActionVideo(original, fps=30)
    // showActionVideo(tangent, fps=30)
}
