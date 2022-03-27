package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.DetectionPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.teamcode.drive.advanced.SamplePipeline;
import org.firstinspires.ftc.teamcode.drive.advanced.DetectionPipeline;

@Autonomous(name = "AUTONOMOUS_bluext")
public class auto_bluext extends LinearOpMode {
    OpenCvCamera webcam;
    SamplePipeline pipeline;
    DetectionPipeline detectionPipeline;
    private DcMotorEx slider;
    private DcMotorEx intake;
    private DcMotor carusel;
    private Servo cuva;
    private DcMotorEx rotire;
    boolean bCameraOpened = false;
    private ColorSensor color;

    //unfinished

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        slider = hardwareMap.get(DcMotorEx.class, "slider");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        carusel = hardwareMap.get(DcMotor.class, "carusel");
        cuva = hardwareMap.get(Servo.class,"cuva");
        rotire = hardwareMap.get(DcMotorEx.class,"rotire");
        color = hardwareMap.get(ColorSensor.class, "color");

        carusel.setDirection(DcMotorSimple.Direction.REVERSE);
        rotire.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        pipeline = new SamplePipeline();
        detectionPipeline = new DetectionPipeline();
        webcam.setPipeline(detectionPipeline);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ElapsedTime runtime2 = new ElapsedTime(0);



        //----------------------------------------------------------------------------------------------

        //traiectorii blueside extern
        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        Trajectory turnToShipLevel3 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-9,0,Math.toRadians(215)))
                .build();

        Trajectory turnToShipLevel2 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-15,-10,Math.toRadians(220)))
                .addTemporalMarker(1,()->{



                    rotire.setTargetPosition(2100);
                    rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                    rotire.setPower(-0.3);
                    if(rotire.isBusy()) {
                        if (rotire.getCurrentPosition() > 1900) {
                            rotire.setPower(0);
                            rotire.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


                        }
                    }
                })
                .build();

        Trajectory turnToShipLevel1 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-15,-10,Math.toRadians(220)))
                .addTemporalMarker(1,()->{





                    rotire.setTargetPosition(2400);
                    rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                    rotire.setPower(-0.3);
                    if(rotire.isBusy()) {
                        if (rotire.getCurrentPosition() > 2100) {
                            rotire.setPower(0);
                            rotire.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                        }
                    }})
                .build();
        Trajectory turnDuck = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-1,20,Math.toRadians(90)))
                .build();


        //x -9  y 7

        // x -1 y 20 pt duck

        // x -19 y 24 pt duck scan

        // x -37 y12 perpedincular cu shiphubu pt punerea duck

        // x -23 y 29 parking

        //----------------------------------------------------------------------------------------------


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                bCameraOpened = true;
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        detectionPipeline.setGridSize(2);

        double left_avg,right_avg;

        int zone = 0;
        sleep(5000);
        while (!opModeIsActive() && !isStopRequested()) {
            //telemetry.addData("Zona", pipeline.getZone());
            cuva.setPosition(0.09);



                left_avg = (detectionPipeline.getZoneLuminosity(1) + detectionPipeline.getZoneLuminosity(2)) / 2;
                right_avg = (detectionPipeline.getZoneLuminosity(3) + detectionPipeline.getZoneLuminosity(4)) / 2;

                if (left_avg <= 125)
                    zone = 1;
                else if (right_avg <= 125)
                    zone = 2;
                else
                    zone = 3;

                telemetry.addData("Zone", zone);
                telemetry.addData("Left", left_avg);
                telemetry.addData("Right", right_avg);

            telemetry.update();
        }
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Pose2d endPose = turnToShipLevel1.end();

        if (opModeIsActive()) {

            drive.followTrajectory(turnDuck);

        }
    }
}