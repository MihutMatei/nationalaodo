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
@Autonomous(name = "AUTONOMOUS_redint")
public class auto_redint extends LinearOpMode {
    OpenCvCamera webcam;
    SamplePipeline pipeline;
    private DcMotorEx cremaliera;
    private DcMotorEx cascade;
    private Servo intake_servo;
    private DcMotorEx intake;
    private DcMotor carusel;
    private CRServo ruleta;
    private CRServo ruleta_x;
    private CRServo ruleta_z;
    static int zona=0;


    @Override
    public void runOpMode() {
        SampleMecanumDrive drive =new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        cremaliera =hardwareMap.get(DcMotorEx.class,"cremaliera");
        cascade =hardwareMap.get(DcMotorEx.class,"cascade");
/*
        intake_servo =hardwareMap.get(Servo.class,"intake_servo");
*/
        intake =hardwareMap.get(DcMotorEx.class,"intake");
        carusel=hardwareMap.get(DcMotor.class, "carusel");
        //ruleta =hardwareMap.get(CRServo.class,"ruleta");
        // ruleta_x =hardwareMap.get(CRServo.class,"ruleta_x");
        //   ruleta_z =hardwareMap.get(CRServo.class,"ruleta_z");


        carusel.setDirection(DcMotorSimple.Direction.REVERSE);
        cascade.setDirection(DcMotorSimple.Direction.REVERSE);



        pipeline = new SamplePipeline();
        webcam.setPipeline(pipeline);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ElapsedTime runtime1 = new ElapsedTime(0);
        ElapsedTime runtime2 = new ElapsedTime(0);
        ElapsedTime runtime3 = new ElapsedTime(0);
        ElapsedTime runtime4 = new ElapsedTime(0);


        //----------------------------------------------------------------------------------------------

        //traiectorii redside intern
        Pose2d startPose= new Pose2d(0,0,0);
        drive.setPoseEstimate(startPose);
        Trajectory f1 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(25.5,18,Math.toRadians(14)))
                .build();
        Trajectory warehouse=drive.trajectoryBuilder(f1.end())
                .lineToSplineHeading(new Pose2d(0,0,Math.toRadians(-90)))
                .addTemporalMarker(0.1,()->
                {   cremaliera.setTargetPosition(-20);
                    cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cremaliera.setVelocity(3000);
                    intake.setDirection(DcMotorSimple.Direction.REVERSE);
                    intake.setPower(0.7);

                })
                .build();
        Trajectory f2=drive.trajectoryBuilder(warehouse.end())
                .forward(40)
                .build();
        Trajectory ia_bila_cub = drive.trajectoryBuilder(f2.end())
                .back(40)
                .build();
        Trajectory revers_card = drive.trajectoryBuilder(ia_bila_cub.end())
                .lineToLinearHeading(new Pose2d(24.5, 18, Math.toRadians(0)))
                .addTemporalMarker(0.1,()->
                {
                    cremaliera.setTargetPosition(-1800);
                    cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cremaliera.setVelocity(3000);


                })
                .build();
        Trajectory ionutz = drive.trajectoryBuilder(revers_card.end())
                .lineToLinearHeading(new Pose2d(-0, 0, Math.toRadians(-90)))
                .addTemporalMarker(0.1,()->
                {
//                    cascade.setTargetPosition(-20);
//                    cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    cascade.setPower(0.4);
                    cremaliera.setTargetPosition(-20);
                    cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cremaliera.setVelocity(3000);
                    intake.setDirection(DcMotorSimple.Direction.REVERSE);
                    intake.setPower(0.7);
                })
                .build();
        //----------------------------------------------------------------------------------------------

        Trajectory inapoi=drive.trajectoryBuilder(ionutz.end())
                .forward(40)
                .build();

        Trajectory ia_bila_cub2 = drive.trajectoryBuilder(inapoi.end())
                .back(40)
                .build();
        Trajectory revers_card2 = drive.trajectoryBuilder(ia_bila_cub2.end())
                .lineToLinearHeading(new Pose2d(24.5, 16, Math.toRadians(0)))
                .addTemporalMarker(0.1,()->
                {
                    cremaliera.setTargetPosition(-1800);
                    cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cremaliera.setVelocity(3000);


                })
                .build();
        Trajectory ionutz2 = drive.trajectoryBuilder(revers_card2.end())
                .lineToLinearHeading(new Pose2d(-1, 7, Math.toRadians(-90)))
                .addTemporalMarker(0.1,()->
                {
//                    cascade.setTargetPosition(-20);
//                    cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    cascade.setPower(0.4);
                    cremaliera.setTargetPosition(-20);
                    cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cremaliera.setVelocity(3000);
                    intake.setDirection(DcMotorSimple.Direction.REVERSE);
                    intake.setPower(0.7);
                })
                .build();

        Trajectory inapoi2=drive.trajectoryBuilder(ionutz2.end())
                .forward(50)
                .build();
        //----------------------------------------------------------------------------------------------

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });


        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Zona", pipeline.getZone());

            telemetry.update();
        }

        int zone = pipeline.getZone();


        if(zone == 1 || zone == 0)
        {   cremaliera.setTargetPosition(-1400);
            cremaliera.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cremaliera.setVelocity(3000);
            while(cremaliera.isBusy())
            {

            }


            cascade.setTargetPosition(-400);
            cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cascade.setPower(1);
            while (cascade.isBusy())
            {

            }
            sleep(300);
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            intake.setPower(0.3);
            sleep(600);
            cascade.setTargetPosition(0);
            cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cascade.setPower(0.4);


        }
        if(zone == 2)
        {
            cremaliera.setTargetPosition(-2300);
            cremaliera.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cremaliera.setVelocity(3000);
            while(cremaliera.isBusy()){

            }
            sleep(300);
            cascade.setTargetPosition(-600);
            cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cascade.setPower(1);
            while (cascade.isBusy())
            {

            }

            sleep(300);
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            intake.setPower(0.35);
            sleep(600);
            cascade.setTargetPosition(0);
            cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cascade.setPower(0.4);


        }
        if(zone == 3)
        {
            cremaliera.setTargetPosition(-3000);
            cremaliera.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cremaliera.setVelocity(3000);
            while(cremaliera.isBusy()){

            }
            sleep(300);
            cascade.setTargetPosition(-700);
            cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cascade.setPower(1);
            sleep(300);
            while(cascade.isBusy())
            {

            }
            sleep(300);
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            intake.setPower(0.7);
            sleep(600);
            cascade.setTargetPosition(0);
            cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cascade.setPower(0.4);
        }


        sleep(400);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setPower(0);
        drive.followTrajectory(warehouse);
        sleep(300);
        drive.followTrajectory(f2);
        sleep(300);
        drive.followTrajectory(ia_bila_cub);
        sleep(400);
        drive.followTrajectory(revers_card);
        cascade.setTargetPosition(-600);
        cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cascade.setPower(1);
        while(cascade.isBusy())
        {

        }
        sleep(400);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setPower(0.6);
        sleep(600);
        intake.setPower(0);
        sleep(400);
        cascade.setTargetPosition(-20);
        cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cascade.setPower(0.4);
        sleep(200);
        drive.followTrajectory(ionutz);
        drive.followTrajectory(inapoi);
        //---------------------------------------------------------------------
        sleep(300);
        drive.followTrajectory(ia_bila_cub2);
        sleep(400);
        drive.followTrajectory(revers_card2);
        cascade.setTargetPosition(-600);
        cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cascade.setPower(1);
        while(cascade.isBusy())

        {

        }
        sleep(400);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setPower(0.4);
        sleep(600);
        intake.setPower(0);
        sleep(400);
        cascade.setTargetPosition(-20);
        cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cascade.setPower(0.4);
        sleep(200);
        drive.followTrajectory(ionutz2);

        drive.followTrajectory(inapoi2);


    }
}