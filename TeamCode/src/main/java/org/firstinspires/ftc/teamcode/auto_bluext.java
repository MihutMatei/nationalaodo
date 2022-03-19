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
    private DcMotorEx cremaliera;
    private DcMotorEx cascade;
    private Servo intake_servo;
    private DcMotorEx intake;
    private DcMotor carusel;
    private CRServo ruleta;
    private CRServo ruleta_x;
    private CRServo ruleta_z;
    //unfinished

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        cremaliera = hardwareMap.get(DcMotorEx.class, "cremaliera");
        cascade = hardwareMap.get(DcMotorEx.class, "cascade");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        carusel = hardwareMap.get(DcMotor.class, "carusel");


        carusel.setDirection(DcMotorSimple.Direction.REVERSE);
        cascade.setDirection(DcMotorSimple.Direction.REVERSE);


        pipeline = new SamplePipeline();
        detectionPipeline = new DetectionPipeline();
        webcam.setPipeline(pipeline);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ElapsedTime runtime1 = new ElapsedTime(0);
        ElapsedTime runtime2 = new ElapsedTime(0);
        ElapsedTime runtime3 = new ElapsedTime(0);
        ElapsedTime runtime4 = new ElapsedTime(0);


        //----------------------------------------------------------------------------------------------

        //traiectorii blueside extern
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        Trajectory f1 = drive.trajectoryBuilder(startPose)

                //  .splineTo(new Vector2d(24,18),0)
                .lineToSplineHeading(new Pose2d(26.5, 17.5, Math.toRadians(17)))


                .build();


        Trajectory duck = drive.trajectoryBuilder(f1.end())
                .lineToSplineHeading(new Pose2d(0.5, -31, Math.toRadians(140)))
                .addTemporalMarker(0.7, () -> {
                    cremaliera.setTargetPosition(-20);

                    cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cremaliera.setVelocity(3000);
                })
                .build();

        Trajectory pickupDuck = drive.trajectoryBuilder(duck.end())
                .lineToSplineHeading(new Pose2d(20, -30, Math.toRadians(180)))
                .build();


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

            telemetry.update();
        }
        int zone = pipeline.getZone();

        if (opModeIsActive()) {
            if (zone == 1 || zone == 0) {
                cremaliera.setTargetPosition(-1100);
                cremaliera.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                cremaliera.setVelocity(3000);
                while (cremaliera.isBusy()) {

                }
                drive.followTrajectory(f1);

             /*   cascade.setTargetPosition(-50);
                cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                cascade.setPower(0.6);
                while (cascade.isBusy())
                {

                }*/
                sleep(500);
                intake.setDirection(DcMotorSimple.Direction.FORWARD);
                intake.setPower(0.35);
                sleep(800);
                cascade.setTargetPosition(0);
                cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                cascade.setPower(0.4);


            }
            if (zone == 2) {  // intake.setDirection(DcMotorSimple.Direction.REVERSE);
                //intake.setPower(0.4);
                cremaliera.setTargetPosition(-2300);
                cremaliera.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                cremaliera.setVelocity(3000);
                while (cremaliera.isBusy()) {

                }
                drive.followTrajectory(f1);
                sleep(500);
                cascade.setTargetPosition(-600);
                cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                cascade.setPower(0.6);
                while (cascade.isBusy()) {

                }

                sleep(500);
                intake.setDirection(DcMotorSimple.Direction.FORWARD);
                intake.setPower(0.3);
                sleep(1000);
                cascade.setTargetPosition(0);
                cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                cascade.setPower(0.6);


            }
            if (zone == 3) {      // intake.setDirection(DcMotorSimple.Direction.REVERSE);
                // intake.setPower(0.4);
                cremaliera.setTargetPosition(-3000);
                cremaliera.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                cremaliera.setVelocity(3000);
                while (cremaliera.isBusy()) {

                }
                drive.followTrajectory(f1);
                sleep(500);
                cascade.setTargetPosition(-700);
                cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                cascade.setPower(0.6);
                sleep(500);
                while (cascade.isBusy()) {

                }
                sleep(500);
                intake.setDirection(DcMotorSimple.Direction.FORWARD);
                intake.setPower(0.8);
                sleep(1000);
                cascade.setTargetPosition(0);
                cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                cascade.setPower(0.6);
            }


            sleep(500);
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            intake.setPower(0);
            drive.followTrajectory(duck);
            cascade.setVelocity(0);
            runtime2.reset();
            while (runtime2.time() < 4) {
                //if(runtime2.time()<1.5)
                carusel.setPower(0.3);
                // else  carusel.setPower(0.7);
            }
            carusel.setPower(0);
            //drive.followTrajectory(parked);

            drive.followTrajectory(pickupDuck);

            webcam.setPipeline(detectionPipeline);
            detectionPipeline.setGridSize(7);

            sleep(1000);

            final double degrees = 4; // should be 3.5 but put it to 4 due to wheel error

            int duckZone = detectionPipeline.getDuckZone();
            int column = detectionPipeline.getColumn(duckZone);

            int degreesMult = column - 3;

            Trajectory DuckScan = drive.trajectoryBuilder(pickupDuck.end())
                    .lineToLinearHeading(new Pose2d(20.5, -30, Math.toRadians(180 -degrees * degreesMult)))
                    .build();


            telemetry.addData("Degree", degrees * degreesMult);
            telemetry.addData("Zone",duckZone);
            telemetry.update();
            sleep(1000);

            drive.followTrajectory(DuckScan);
            sleep(1000);
            Trajectory forward2 = drive.trajectoryBuilder(DuckScan.end())
                    .forward(23)
                    .addTemporalMarker(0.1,()->{
                        intake.setPower(1);
                    })
                    .build();
            drive.followTrajectory(forward2);
            sleep(1000);
            Trajectory LinetoShipp = drive.trajectoryBuilder(forward2.end())
                    .lineToLinearHeading(new Pose2d(33, -30, Math.toRadians(90)))

                    .build();
            Trajectory gotoshipp = drive.trajectoryBuilder(LinetoShipp.end())
                    .forward(30)
                    .addTemporalMarker(0.1,()->{

                        /*while(cremaliera.isBusy())
                        {

                        }*/
                        cascade.setTargetPosition(-700);
                        cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        cascade.setPower(0.6);
                    })

                    .build();
            Trajectory parked = drive.trajectoryBuilder(gotoshipp.end())
                    .lineToSplineHeading(new Pose2d(22, -33, Math.toRadians(90)))
                    .addTemporalMarker(0.1,()->{
                        cascade.setTargetPosition(0);
                        cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        cascade.setPower(0.6);
                        cremaliera.setTargetPosition(0);

                        cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        cremaliera.setVelocity(3000);

                    })
                    .build();
            drive.followTrajectory(LinetoShipp);
            sleep(1000);
            cremaliera.setTargetPosition(-3000);

            cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cremaliera.setVelocity(3000);
            sleep(1000);
            drive.followTrajectory(gotoshipp);
            intake.setPower(-0.7);
            sleep(1000);
            drive.followTrajectory(parked);

            //
            //            sleep(500);
            //
            //            int bestZone = detectionPipeline.getDuckZone(); // dam ca argument zona preferata pe langa cea din centru
            //            // adica daca suntem in stanga preferam sa luam din dreapta daca nu gasim nimic in centru ca sa nu ne bagam in perete si invers
            //            DetectionPipeline.ZoneType zoneType = detectionPipeline.getZoneType(bestZone);
            //            telemetry.addData("BestZone", bestZone);
            //            telemetry.addData("ZoneType", zoneType);
            //
            //            if(bestZone==0)
            //            {
            //                drive.followTrajectory(parked);
            //                break;
            //            }
            //            if(zoneType == DetectionPipeline.ZoneType.E_CENTER)
            //            {
            //                drive.followTrajectory(forward);
            //                sleep(2000);
            //                intake.setDirection(DcMotorSimple.Direction.REVERSE);
            //                intake.setPower(0.8);
            //
            //                telemetry.addData("BestZone", bestZone);
            //                telemetry.addData("ZoneType", zoneType);
            //                telemetry.update();
            //
            //                drive.followTrajectory(f2);
            //                sleep(1500);
            //            }
            //            else if(zoneType == DetectionPipeline.ZoneType.E_LEFT)
            //            {
            //                drive.followTrajectory(left);
            //            }
            //            else
            //            {
            //                drive.followTrajectory(right);
            //            }
            //shipp


            //drive.followTrajectory(parked);


        }
    }
}
