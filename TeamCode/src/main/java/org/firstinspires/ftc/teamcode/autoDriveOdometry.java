package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name="Robot: Auto Drive By Pinpoint", group="Robot")
public class autoDriveOdometry extends LinearOpMode {
    // TODO configure the motors
    private DcMotor frontLeft = ;
    private DcMotor frontRight = ;
    private DcMotor backLeft = ;
    private DcMotor backRight = ;

    private goBildaPinpointDriver odo; // Pinpoint odometry computer
    private ElapsedTime runtime = new ElapsedTime();

    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double position_buffer = 10.0; // mm tolerance for position
    static final double heading_buffer = Math.toRadians(2); // 2 degree tolerance

    @Override
    public void runOpMode() {
        // Initialize the drive variables
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        odo = hardwareMap.get(goBildaPinpointDriver.class, "odo");

        // Set motor directions for mecanum drive
        //TODO check if these are right
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Configure odometry computer
        odo.setOffsets(157, 72, DistanceUnit.MM);
        odo.setEncoderResolution(goBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(goBildaPinpointDriver.EncoderDirection.FORWARD,
                goBildaPinpointDriver.EncoderDirection.FORWARD);

        // Reset position
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.update();

        waitForStart();

        // Update odometry
        odo.update();

        // Step through each leg of the path
        // Forward 48 inches (1219.2 mm)
        driveToPosition(254, 0, 0, DRIVE_SPEED, 5.0);

        // Turn right 90 degrees
        turnToHeading(Math.toRadians(-90), TURN_SPEED, 4.0);

        // Reverse 24 inches (609.6 mm) - move backward from current position
        Pose2D currentPos = odo.getPosition();
        double targetX = currentPos.getX(DistanceUnit.MM) - 609.6 * Math.cos(currentPos.getHeading(AngleUnit.RADIANS));
        double targetY = currentPos.getY(DistanceUnit.MM) - 609.6 * Math.sin(currentPos.getHeading(AngleUnit.RADIANS));
        driveToPosition(targetX, targetY, Math.toRadians(-90), DRIVE_SPEED, 4.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    // function driveToPosition
    // takes params targetX, targetY, targetHeading, speed, timeouts
    public void driveToPosition(double targetX, double targetY,
                                double targetHeading, double speed, double timeoutS) {
        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < timeoutS) {
            odo.update();

            // pretty sure this creates a class for robot's (x,y,heading)
            // https://acmerobotics.github.io/road-runner/core/0.4.5/javadoc/com/acmerobotics/roadrunner/geometry/Pose2d.html
            Pose2D pos = odo.getPosition();
            double currentX = pos.getX(DistanceUnit.MM);
            double currentY = pos.getY(DistanceUnit.MM);
            double currentHeading = pos.getHeading(AngleUnit.RADIANS);

            // Calculate distance and angle to target (where we want it to be)
            double distance_X = targetX - currentX;
            double distance_Y = targetY - currentY;
            // distance formula
            double distance_Target = Math.sqrt(distance_X * distance_X + distance_Y * distance_Y);

            // stop when reached both targets
            if (distance_Target < position_buffer &&
                    Math.abs(angleError(currentHeading, targetHeading)) < heading_buffer) {
                break;
            }

            // convert calculated target to actual driving coordinates for robot
            double angleToTarget = Math.atan2(distance_Y, distance_X);
            double relativeAngle = angleToTarget - currentHeading;

            // calculate robot drive components; cos for x, sine for y
            double driveX = Math.cos(relativeAngle) * distance_Target * 0.01;
            double driveY = Math.sin(relativeAngle) * distance_Target * 0.01;
            // calculate heading
            double headingError = angleError(currentHeading, targetHeading);
            double turn = headingError * 0.5; // Proportional control for heading

            // limits for drive power
            double maxDrive = Math.sqrt(driveX * driveX + driveY * driveY);
            if (maxDrive > speed) {
                driveX = (driveX / maxDrive) * speed;
                driveY = (driveY / maxDrive) * speed;
            }

            // mecanum wheel powers
            double frontLeftPower = driveY + driveX + turn;
            double frontRightPower = driveY - driveX - turn;
            double backLeftPower = driveY - driveX + turn;
            double backRightPower = driveY + driveX - turn;

            // normalize powers if any exceed 1
            double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            // telemetry display
            telemetry.addData("Target", "X: %.1f, Y: %.1f, H: %.1f°",
                    targetX, targetY, Math.toDegrees(targetHeading));
            telemetry.addData("Current", "X: %.1f, Y: %.1f, H: %.1f°",
                    currentX, currentY, Math.toDegrees(currentHeading));
            telemetry.addData("Distance to target", "%.1f mm", distance_Target);
            telemetry.addData("Powers", "FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.update();
        }

        // Stop motors
        stopMotors();
        sleep(250);
    }

    /**
     * Turn to a specific heading
     * @param targetHeading target heading in radians
     * @param speed turn speed (0-1)
     * @param timeoutS timeout in seconds
     */
    public void turnToHeading(double targetHeading, double speed, double timeoutS) {
        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < timeoutS) {
            odo.update();

            double currentHeading = odo.getHeading(AngleUnit.RADIANS);
            double headingError = angleError(currentHeading, targetHeading);

            // Check if reached the target heading
            if (Math.abs(headingError) < heading_buffer) {
                break;
            }

            // control for turning
            double turnPower = headingError * 0.5 * speed;
            turnPower = Math.max(-speed, Math.min(speed, turnPower));

            // mecanum turn without displacing
            frontLeft.setPower(-turnPower);
            frontRight.setPower(turnPower);
            backLeft.setPower(-turnPower);
            backRight.setPower(turnPower);

            // telemetry display
            telemetry.addData("Target Heading", "%.1f°", Math.toDegrees(targetHeading));
            telemetry.addData("Current Heading", "%.1f°", Math.toDegrees(currentHeading));
            telemetry.addData("Heading Error", "%.1f°", Math.toDegrees(headingError));
            telemetry.addData("Turn Power", "%.2f", turnPower);
            telemetry.update();
        }

        // stop
        stopMotors();
        sleep(250);
    }

// stop all motors
    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    /**
     * Calculate the smallest angle error between current and target angles
     * @param current current angle in radians
     * @param target target angle in radians
     * @return angle error in radians (-PI to PI)
     */
    private double angleError(double current, double target) {
        double error = target - current;
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;
        return error;
    }
}