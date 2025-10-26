package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous
public class simpleAuto extends LinearOpMode {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    private GoBildaPinpointDriver odo; // Pinpoint odometry computer

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right");

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        //TODO check if these are right
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        // Configure odometry
        odo.setOffsets(157, 72, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Reset position
        odo.resetPosAndIMU(); // (0, 0) right now

        drive(100, 100, 0.6, 4.0);

    }

// BELOW ARE ALL UTILITY FUNCTIONS
    public void drive(double targetX, double targetY, double speed, double time) {
        double startTime = getRuntime();

        while (opModeIsActive() && (getRuntime() - startTime) < time) {
            odo.update();
            Pose2D pos = odo.getPosition();
            double currentX = pos.getX(DistanceUnit.MM);
            double offByX = targetX - currentX;

            if (Math.abs(offByX) < 10) {
                break;
            }

            double motorSpeed;
            if (offByX > 0) { motorSpeed = speed; } else { motorSpeed = -speed; }

            frontLeft.setPower(motorSpeed);
            frontRight.setPower(-motorSpeed);
            backLeft.setPower(motorSpeed);
            backRight.setPower(-motorSpeed);
        }

        // the time and speed do not need to align with the speed and time in the drive functions
        turn(targetX, targetY, speed, 3.0);

        while (opModeIsActive() && (getRuntime() - startTime) < time) {
            odo.update();
            Pose2D pos = odo.getPosition();
            double currentY = pos.getY(DistanceUnit.MM);
            double offByY = targetY - currentY;

            if (Math.abs(offByY) < 10) {
                break;
            }

            double motorSpeed;
            if (offByY > 0) { motorSpeed = speed; } else { motorSpeed = -speed; }

            frontLeft.setPower(motorSpeed);
            frontRight.setPower(-motorSpeed);
            backLeft.setPower(motorSpeed);
            backRight.setPower(-motorSpeed);
        }
    }
    public void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    // func that makes robot turn
    public void turn(double targetX, double targetY, double speed, double time) {
        double startTime = getRuntime();

        while (opModeIsActive() && (getRuntime() - startTime) < time) {
            odo.update();
            Pose2D pos = odo.getPosition();
            double currentX = pos.getX(DistanceUnit.MM);
            double offByX = targetX - currentX;
            double currentY = pos.getY(DistanceUnit.MM);
            double offByY = targetY - currentY;

            double offByAngleRadians = Math.atan2(offByY, offByX); // gets the angle between two points
            double offByAngle = Math.toDegrees(offByAngleRadians); // no one likes radians :)
            double currentAngle = odo.getHeading(AngleUnit.DEGREES);
            double angleError = offByAngle - currentAngle;

            while (angleError > 180) angleError -= 360; // think about -180 and 180
            while (angleError < -180) angleError += 360;

            if (Math.abs(angleError) < 10) {
                stopMotors();
                break;
            }

            if (angleError > 0) {
                // turn left
                frontLeft.setPower(-speed);
                frontRight.setPower(-speed);
                backLeft.setPower(speed);
                backRight.setPower(speed);
            } else {
                // turn right
                frontLeft.setPower(speed);
                frontRight.setPower(speed);
                backLeft.setPower(-speed);
                backRight.setPower(-speed);
            }
            stopMotors();
        }
    }
}
