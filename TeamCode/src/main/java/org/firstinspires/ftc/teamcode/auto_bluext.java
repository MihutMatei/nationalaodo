package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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
    private Servo rotire;


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
        rotire = hardwareMap.get(Servo.class,"rotire");


        carusel.setDirection(DcMotorSimple.Direction.REVERSE);



        pipeline = new SamplePipeline();
        detectionPipeline = new DetectionPipeline();
        webcam.setPipeline(pipeline);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ElapsedTime runtime2 = new ElapsedTime(0);



        //----------------------------------------------------------------------------------------------

        //traiectorii blueside extern
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        Trajectory turnToShip = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-9,-1,Math.toRadians(220)))
                .build();


        Trajectory turnDuck = drive.trajectoryBuilder(turnToShip.end())
                .lineToSplineHeading(new Pose2d(-4,20,Math.toRadians(-90)))
                .build();
        Trajectory toStorage = drive.trajectoryBuilder(turnDuck.end())
                .lineToSplineHeading(new Pose2d(-27,24,Math.toRadians(-90)))
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
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Zona", pipeline.getZone());
            cuva.setPosition(0.09);
            rotire.setPosition(1);
            telemetry.update();
        }
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int zone = pipeline.getZone();

        drive.followTrajectory(turnToShip);
        slider.setTargetPosition(-1200);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.7);

        rotire.setPosition(0.2);

        while(slider.isBusy())
        {

        }
        sleep(1000);
        cuva.setPosition(0.5);
        sleep(2000);

        slider.setTargetPosition(-100);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(-0.7);
        rotire.setPosition(1);
       sleep(1000);
        drive.followTrajectory(turnDuck);
        if (opModeIsActive()) {
            if (zone == 1 || zone == 0) {


            }
            if (zone == 2) {

            }
            if (zone == 3) {
            }



            runtime2.reset();
            while (runtime2.time() < 4) {

                carusel.setPower(0.3);

            }

            webcam.setPipeline(detectionPipeline);
            detectionPipeline.setGridSize(7);

            sleep(1000);
            drive.followTrajectory(toStorage);

            final double degrees = 4; // should be 3.5 but put it to 4 due to wheel error

            int duckZone = detectionPipeline.getDuckZone();
            int column = detectionPipeline.getColumn(duckZone);

            int degreesMult = column - 3;



            telemetry.addData("Degree", degrees * degreesMult);
            telemetry.addData("Zone",duckZone);
            telemetry.update();
            sleep(1000);



        }
    }
}
