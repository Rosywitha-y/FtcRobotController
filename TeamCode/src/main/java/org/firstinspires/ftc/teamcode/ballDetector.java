package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

// this file contains reusable ball detecting functions :)

public class ballDetector {
    private ColorBlobLocatorProcessor colorLocator;
    private VisionPortal portal;

    // TODO: ADJUST THESE FOR CAMERA
    private static final double CAMERA_HEIGHT_MM = 200; // Height of camera above ground
    private static final double BALL_RADIUS_MM = 38; // Radius of ball in mm
    private static final int CAMERA_WIDTH = 320;
    private static final int CAMERA_HEIGHT = 240;
    private static final double CAMERA_FOV_HORIZONTAL = 60; // degrees

    // some ball info
    public static class BallInfo {
        public boolean found;
        public double distanceFromRobot; // mm from robot center
        public double angleFromRobot; // radians from robot's forward direction
        public double absoluteX; // absolute field X position (mm)
        public double absoluteY; // absolute field Y position (mm)
        public double pixelX; // X position in camera frame
        public double pixelY; // Y position in camera frame
        public double radius; // radius in pixels

        public BallInfo() {
            this.found = false;
        }
    }


    // Initialize vision system
    public void init(HardwareMap hardwareMap, ColorRange targetColor) {
        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(targetColor)  // purple and green in this case
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(Color.rgb(255, 255, 0))
                .setBlurSize(5)
                .setDilateSize(15)
                .setErodeSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();
    }

    // function that detects the ball and returns info about it
    // takes in currentPose
    public BallInfo detectBall(Pose2D currentPose) {
        BallInfo info = new BallInfo();

        // Get blobs from camera
        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

        // Filter blobs
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                50, 20000, blobs);

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                0.6, 1.0, blobs);

        // If no ball found, return
        if (blobs.isEmpty()) {
            info.found = false;
            return info;
        }

        // Get the closest blob, the first after filtering
        ColorBlobLocatorProcessor.Blob targetBlob = blobs.get(0);
        Circle circleFit = targetBlob.getCircle();

        info.found = true;
        info.pixelX = circleFit.getX();
        info.pixelY = circleFit.getY();
        info.radius = circleFit.getRadius();

        // calculates distance to ball using radius (simple approximation)
        // uses similar triangles: distance ≈ (actual_size * focal_length) / pixel_size
        double focalLengthPixels = (CAMERA_WIDTH / 2.0) / Math.tan(Math.toRadians(CAMERA_FOV_HORIZONTAL / 2.0));
        info.distanceFromRobot = (BALL_RADIUS_MM * focalLengthPixels) / info.radius;

        // Calculate angle to ball from camera center
        double pixelOffsetFromCenter = info.pixelX - (CAMERA_WIDTH / 2.0);
        double anglePerPixel = Math.toRadians(CAMERA_FOV_HORIZONTAL) / CAMERA_WIDTH;
        info.angleFromRobot = pixelOffsetFromCenter * anglePerPixel;

        // Calculate absolute field position of ball
        double currentX = currentPose.getX(DistanceUnit.MM);
        double currentY = currentPose.getY(DistanceUnit.MM);
        double currentHeading = currentPose.getHeading(AngleUnit.RADIANS);

        // Ball angle in field coordinates
        double ballAbsoluteAngle = currentHeading + info.angleFromRobot;

        // Ball position in field coordinates
        info.absoluteX = currentX + info.distanceFromRobot * Math.cos(ballAbsoluteAngle);
        info.absoluteY = currentY + info.distanceFromRobot * Math.sin(ballAbsoluteAngle);

        return info;
    }



    // func that gets distance to the detected ball
    // returns the distance in mm or -1 when no ball found
    public double getDistanceToBall(Pose2D currentPose) {
        BallInfo info = detectBall(currentPose);
        return info.found ? info.distanceFromRobot : -1;
    }

    // func that checks if there a ball
    public boolean isBallDetected() {
        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                50, 20000, blobs);

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                0.6, 1.0, blobs);

        return !blobs.isEmpty();
    }

    // cleanup!
    public void close() {
        if (portal != null) {
            portal.close();
        }
    }
}