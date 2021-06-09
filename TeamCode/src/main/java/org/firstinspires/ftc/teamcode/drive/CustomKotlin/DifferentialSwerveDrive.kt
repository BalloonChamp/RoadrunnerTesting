package org.firstinspires.ftc.teamcode.drive.CustomKotlin
import com.acmerobotics.roadrunner.drive.Drive
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import org.firstinspires.ftc.teamcode.drive.CustomKotlin.DifferentialSwerveKinematics
import com.acmerobotics.roadrunner.localization.Localizer
import com.acmerobotics.roadrunner.util.Angle

/**
 * This class provides the basic functionality of a swerve drive using [SwerveKinematics].
 *
 * @param kV velocity feedforward
 * @param kA acceleration feedforward
 * @param kStatic additive constant feedforward
 * @param trackWidth lateral distance between pairs of wheels on different sides of the robot
 * @param wheelBase distance between pairs of wheels on the same side of the robot
 */
abstract class DifferentialSwerveDrive @JvmOverloads constructor(
        private val kV: Double,
        private val kA: Double,
        private val kStatic: Double,
        private val trackWidth: Double,
        private val wheelBase: Double = trackWidth
) : Drive() {

    /**
     * Default localizer for swerve drives based on the drive encoder positions, module orientations, and (optionally) a
     * heading sensor.
     *
     * @param drive drive
     * @param useExternalHeading use external heading provided by an external sensor (e.g., IMU, gyroscope)
     */
    class SwerveLocalizer @JvmOverloads constructor(
            private val drive: DifferentialSwerveDrive,
            private val useExternalHeading: Boolean = true
    ) : Localizer {
        private var _poseEstimate = Pose2d()
        override var poseEstimate: Pose2d
            get() = _poseEstimate
            set(value) {
                lastWheelPositions = emptyList()
                lastExtHeading = Double.NaN
                if (useExternalHeading) drive.externalHeading = value.heading
                _poseEstimate = value
            }
        override var poseVelocity: Pose2d? = null
            private set
        private var lastWheelPositions = emptyList<Double>()
        private var lastExtHeading = Double.NaN

        override fun update() {
            val wheelPositions = drive.getWheelPositions()
            val moduleOrientations = drive.getModuleOrientations()
            val extHeading = if (useExternalHeading) drive.externalHeading else Double.NaN
            if (lastWheelPositions.isNotEmpty()) {
                val wheelDeltas = wheelPositions
                        .zip(lastWheelPositions)
                        .map { it.first - it.second }
                val robotPoseDelta = DifferentialSwerveKinematics.wheelToRobotVelocities(
                        wheelDeltas,
                        moduleOrientations,
                        drive.wheelBase,
                        drive.trackWidth
                )
                val finalHeadingDelta = if (useExternalHeading) {
                    Angle.normDelta(extHeading - lastExtHeading)
                } else {
                    robotPoseDelta.heading
                }
                _poseEstimate = Kinematics.relativeOdometryUpdate(
                        _poseEstimate,
                        Pose2d(robotPoseDelta.vec(), finalHeadingDelta)
                )
            }

            val wheelVelocities = drive.getWheelVelocities()
            val extHeadingVel = drive.getExternalHeadingVelocity()
            if (wheelVelocities != null) {
                poseVelocity = DifferentialSwerveKinematics.wheelToRobotVelocities(
                        wheelVelocities,
                        moduleOrientations,
                        drive.wheelBase,
                        drive.trackWidth
                )
                if (useExternalHeading && extHeadingVel != null) {
                    poseVelocity = Pose2d(poseVelocity!!.vec(), extHeadingVel)
                }
            }

            lastWheelPositions = wheelPositions
            lastExtHeading = extHeading
        }
    }

    override var localizer: Localizer = SwerveLocalizer(this)

    override fun setDriveSignal(driveSignal: DriveSignal) {
        val velocities = DifferentialSwerveKinematics.robotToWheelVelocities(
                driveSignal.vel,
                trackWidth,
                wheelBase
        )
        val accelerations = DifferentialSwerveKinematics.robotToWheelAccelerations(
                driveSignal.vel,
                driveSignal.accel,
                trackWidth,
                wheelBase
        )
        val powers = Kinematics.calculateMotorFeedforward(velocities, accelerations, kV, kA, kStatic)
        val orientations = DifferentialSwerveKinematics.robotToModuleOrientations(
                driveSignal.vel,
                trackWidth,
                wheelBase
        )
        setMotorPowers(powers[0], powers[1], powers[2], powers[3])
        setModuleOrientations(orientations[0], orientations[1])
    }

    override fun setDrivePower(drivePower: Pose2d) {
        val avg = (trackWidth + wheelBase) / 2.0
        val powers = DifferentialSwerveKinematics.robotToWheelVelocities(drivePower, trackWidth / avg, wheelBase / avg)
        val orientations = DifferentialSwerveKinematics.robotToModuleOrientations(drivePower, trackWidth / avg, wheelBase / avg)
        setMotorPowers(powers[0], powers[1], powers[2], powers[3])
        setModuleOrientations(orientations[0], orientations[1])
    }

    /**
     * Sets the following motor powers (normalized voltages). All arguments are on the interval `[-1.0, 1.0]`.
     */
    abstract fun setMotorPowers(leftTop: Double, leftBottom: Double, rightTop: Double, rightBottom: Double)

    /**
     * Sets the module orientations. All values are in radians.
     */
    abstract fun setModuleOrientations(left: Double, right: Double)

    /**
     * Returns the positions of the wheels in linear distance units. Positions should exactly match the ordering in
     * [setMotorPowers].
     */
    abstract fun getWheelPositions(): List<Double>

    /**
     * Returns the velocities of the wheels in linear distance units. Positions should exactly match the ordering in
     * [setMotorPowers].
     */
    open fun getWheelVelocities(): List<Double>? = null

    /**
     * Returns the current module orientations in radians. Orientations should exactly match the order in
     * [setModuleOrientations].
     */
    abstract fun getModuleOrientations(): List<Double>
}