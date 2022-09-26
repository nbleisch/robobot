package com.gnabblobots

import robocode.AdvancedRobot
import robocode.Rules.getBulletSpeed
import robocode.ScannedRobotEvent
import robocode.StatusEvent
import robocode.util.Utils.*
import java.awt.Color.GREEN
import java.awt.Color.RED
import java.awt.geom.Point2D
import java.awt.geom.Point2D.distance
import java.lang.Math.toDegrees
import java.util.concurrent.ConcurrentHashMap
import kotlin.Double.Companion.MAX_VALUE
import kotlin.math.*


class GnabbloBot : AdvancedRobot() {

    companion object {
        private const val MAX_TARGET_SCAN_AGE_MS = 7
        private const val LOCATION_PROXIMITY_DELTA_UNITS = 10.0
        private const val SIZE_OF_ROBOT = 36
    }

    var targetLocationX: Double = 0.0
    var targetLocationY: Double = 0.0
    var acquiredTarget: ScannedRobotEventContainer? = null
    var scannedTargets: ConcurrentHashMap<String, ScannedRobotEventContainer> = ConcurrentHashMap()

    private fun initializeBot() {
        targetLocationX = (battleFieldWidth / 2) - width / 2
        targetLocationY = (battleFieldHeight / 2) - height / 2
        isAdjustGunForRobotTurn = true
        isAdjustRadarForGunTurn = true
        setAllColors(GREEN)
    }

    override fun run() {
        initializeBot()
        moveToStartLocation()
        while (true) {

            execute()
            revalidateTargets()
            acquiredTarget = (acquiredTarget?.let { scannedTargets[it.target.name] } ?: scannedTargets.values.minByOrNull { it.target.distance })?.also {
                setAllColors(RED)
                setTurnRadarRight(normalRelativeAngle(it.headingDuringScannedRadians + it.target.bearingRadians - radarHeading))
                aimAndShoot(it)
            }
            if (acquiredTarget == null) {
                setTurnRadarRight(MAX_VALUE)
            }
            doTheMovement()
        }
    }

    private fun doTheMovement() {
        setAhead(200.0)
        setTurnRight(200.0)
        setTurnRight(MAX_VALUE)
    }


    private fun aimAndShoot(target: ScannedRobotEventContainer) {
        val maxFirePower = 3.0
        val selectedFirePower = maxFirePower //TODO: Some distance based fire power solution
        val angleToShootAt = getTrueLinearTargetingAngle(target, getBulletSpeed(selectedFirePower))
        val normalizedGunAngleAdjustments = normalRelativeAngle(angleToShootAt - gunHeadingRadians)
        printStatus()
        setTurnGunRightRadians(normalizedGunAngleAdjustments)
        setFire(selectedFirePower)
        //scan()
        execute()
    }

    private fun printStatus() {
        println(
            """Heading: $heading
            |Heading gun current: $gunHeading
            |Heading gun set: ${normalRelativeAngleDegrees(gunHeading + gunTurnRemaining)}  
        """.trimMargin()
        )
    }


    private fun getTrueLinearTargetingAngle(scan: ScannedRobotEventContainer, bulletSpeed: Double): Double {

        //Aiming is off
        val passedTimeTicks = 1 + time - scan.target.time
        var predictedX = scan.positionX + sin(scan.target.headingRadians) * (scan.target.velocity * passedTimeTicks)
        var predictedY = scan.positionY + cos(scan.target.headingRadians) * (scan.target.velocity * passedTimeTicks)
        val distance = distance(x, y, predictedX, predictedY);
        val bulletTravelTime = (distance / bulletSpeed) * 1.2 //1.3 because of game engine
        println("Distance $distance")
        println("Bulletspeed $bulletSpeed")
        println("Bullettraveltime $bulletTravelTime")
        predictedX = scan.positionX + sin(scan.target.headingRadians) * (scan.target.velocity * (passedTimeTicks + bulletTravelTime))
        predictedY = scan.positionY + cos(scan.target.headingRadians) * (scan.target.velocity * (passedTimeTicks + bulletTravelTime))

        //Check if prediction is outer boundaries
        val consideredBattleFieldStart = 0.0 + SIZE_OF_ROBOT / 2.0
        val consideredBattleFieldWidth = battleFieldWidth - SIZE_OF_ROBOT / 2.0
        val consideredBattleFieldHeight = battleFieldHeight - SIZE_OF_ROBOT / 2.0
        if (predictedX < consideredBattleFieldStart || predictedY < consideredBattleFieldStart || predictedX > consideredBattleFieldWidth || predictedY > consideredBattleFieldHeight) {
            predictedX = min(max(consideredBattleFieldStart, predictedX), consideredBattleFieldWidth)
            predictedY = min(max(consideredBattleFieldStart, predictedY), consideredBattleFieldHeight)
        }
        val predictedAngle = atan2(predictedX - x, predictedY - y)
        return normalRelativeAngle(predictedAngle)
    }


    private fun revalidateTargets() {
        scannedTargets.keys.forEach {
            if (time - scannedTargets[it]?.target?.time!! > MAX_TARGET_SCAN_AGE_MS) scannedTargets.remove(it)
        }

    }


    private fun moveToStartLocation() {
        do {
            turnRightRadians(normalRelativeAngle(-headingRadians))
            val xd = targetLocationX - x
            val yd = targetLocationY - y
            val targetDistance = hypot(xd, yd)
            val targetAngle = atan2(xd, yd)
            turnRightRadians(normalRelativeAngle(targetAngle))
            ahead(targetDistance)
            turnRightRadians(normalRelativeAngle(-headingRadians))
        } while (!isCloseToStartLocation())
    }

    private fun isCloseToStartLocation(): Boolean = abs(x - targetLocationX) < LOCATION_PROXIMITY_DELTA_UNITS && abs(y - targetLocationY) < LOCATION_PROXIMITY_DELTA_UNITS

    override fun onStatus(e: StatusEvent) {

    }

    override fun onScannedRobot(event: ScannedRobotEvent) {
        val absoluteBearing: Double = headingRadians + event.bearingRadians
        scannedTargets[event.name] = ScannedRobotEventContainer(
            target = event,
            headingDuringScannedRadians = headingRadians,
            positionX = x + event.distance * sin(absoluteBearing),
            positionY = y + event.distance * cos(absoluteBearing)
        )
        if (acquiredTarget == null) {
            setTurnRadarRightRadians(normalRelativeAngle(absoluteBearing - radarHeadingRadians))
            setTurnGunRightRadians(normalRelativeAngle(absoluteBearing - gunHeadingRadians))
        }
    }


    class ScannedRobotEventContainer(
        val target: ScannedRobotEvent,
        val headingDuringScannedRadians: Double,
        val positionX: Double,
        val positionY: Double
    )
}

