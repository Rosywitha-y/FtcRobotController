package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

// this file contains actual autonomous driving :)
// it's disabled because i'm still working on it


@Autonomous
public class AUTO_2025 extends LinearOpMode {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    private GoBildaPinpointDriver odo; // Pinpoint odometry computer
    private final ElapsedTime runtime = new ElapsedTime();

    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double position_buffer = 10.0; // mm tolerance for position
    static final double heading_buffer = Math.toRadians(2); // 2 degree tolerance


    // import functions
    autoDriveOdometry driveFunctions = new autoDriveOdometry(); // creates a new autoDriveOdometry object
    ballDetector ballDetector = new ballDetector(); // // creates a new ballDetector object




    @Override
    public void runOpMode() {
        // Initialize the drive variables
        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right");

        // TODO: check hardware map for this
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        // Set motor directions for mecanum drive
        //TODO check if these are right
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Configure odometry computer
        odo.setOffsets(157, 72, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Reset position
        odo.resetPosAndIMU();

        // for now target's green but we will need to adjust it after sensing the apriltags
        ballDetector.init(hardwareMap, ColorRange.ARTIFACT_GREEN);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.update();

        waitForStart();

        // Update odometry
        odo.update();

        // drive towards ball
        Pose2D currentPose = odo.getPosition();
        double ball_target_X = ballDetector.detectBall(currentPose).absoluteX;
        double ball_target_Y = ballDetector.detectBall(currentPose).absoluteY;
        double ball_target_HEADING = ballDetector.detectBall(currentPose).angleFromRobot;

        driveToPosition(ball_target_X, ball_target_Y, ball_target_HEADING, DRIVE_SPEED, 4.0);


        // this is actually moving the robot with the functions we created
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

            // Convert field-relative to robot-relative coordinates
            double robotX = Math.cos(-currentHeading) * driveX - Math.sin(-currentHeading) * driveY;
            double robotY = Math.sin(-currentHeading) * driveX + Math.cos(-currentHeading) * driveY;

            // mecanum wheel powers (using robot-relative coordinates)
            double frontLeftPower = robotY + robotX + turn;
            double frontRightPower = robotY - robotX - turn;
            double backLeftPower = robotY - robotX + turn;
            double backRightPower = robotY + robotX - turn;

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

    // function to turn towards wanted angle
    // takes in targetHeading, speed, timeoutS (in seconds)
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

    // func to stop all motors
    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    // func that calculates the angle error
// takes in current and target angle in radians
    private double angleError(double current, double target) {
        double error = target - current;
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;
        return error;
    }
}