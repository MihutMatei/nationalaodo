 /* Copyright (c) 2017 FIRST. All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted (subject to the limitations in the disclaimer below) provided that
  * the following conditions are met:
  *
  * Redistributions of source code must retain the above copyright notice, this list
  * of conditions and the following disclaimer.
  *
  * Redistributions in binary form must reproduce the above copyright notice, this
  * list of conditions and the following disclaimer in the documentation and/or
  * other materials provided with the distribution.
  *
  * Neither the name of FIRST nor the names of its contributors may be used to endorse or
  * promote products derived from this software without specific prior written permission.
  *
  * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
  * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  */

 package org.firstinspires.ftc.teamcode;

 import com.acmerobotics.roadrunner.geometry.Pose2d;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 import com.qualcomm.robotcore.hardware.ColorSensor;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.DcMotorEx;
 import com.qualcomm.robotcore.hardware.DcMotorSimple;
 import com.qualcomm.robotcore.hardware.Servo;
 import com.qualcomm.robotcore.util.ElapsedTime;

 import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

 @TeleOp(name="NEWDRIVE", group="Linear Opmode")

 public class NewDrive extends LinearOpMode {



     private DcMotorEx intake;
     private Servo cuva;
     private Servo rotire;
     private DcMotorEx slider;
     ColorSensor color;


     @Override
     public void runOpMode() throws InterruptedException {



         intake =hardwareMap.get(DcMotorEx.class,"intake");
         color = hardwareMap.get(ColorSensor.class, "color");
         cuva = hardwareMap.get(Servo.class,"cuva");
         rotire = hardwareMap.get(Servo.class,"rotire");
         slider = hardwareMap.get(DcMotorEx.class,"slider");

         slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

         boolean ok=true;






         SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


         waitForStart();
         drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




         while (opModeIsActive()) {

             ElapsedTime runtime1 = new ElapsedTime(0);



             drive.setWeightedDrivePower(
                     new Pose2d(
                             -gamepad1.left_stick_y,
                             -gamepad1.left_stick_x ,
                             -gamepad1.right_stick_x
                     )

             );
             drive.update();








            /*if(gamepad1.left_stick_y>0)
                intake.setPower(-0.3);
            else
            if(gamepad1.left_stick_y<0)
                intake.setPower(0.3);
            else intake.setPower(0);*/

            //outtake
             if(gamepad1.dpad_left) {
                 intake.setDirection(DcMotorSimple.Direction.FORWARD);
                 intake.setPower(0.5);

             }
             //intake
            // TODO:change intake power ; differebt mototor 1150 -> 1600
             if(gamepad1.dpad_right) {
                 intake.setDirection(DcMotorSimple.Direction.REVERSE);
                 intake.setPower(0.6);

             }
             if(gamepad1.dpad_down) {
                 intake.setPower(0);
             }

             //slider movement
             if(gamepad2.left_stick_y>0) {
                 slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                 slider.setPower(1);
             }
             else if(gamepad2.left_stick_y<0)
             {   slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                 slider.setPower(-1);
             }

             else
             {            slider.setPower(0);
             }


             //rotation
             /*if(gamepad2.dpad_left)rotire.setPosition(0.2);
             if(gamepad2.dpad_right)rotire.setPosition(0.97);*/

             //cuva
             if(color.red()>30&&color.green()>30&&ok) {
                 cuva.setPosition(0.09);
                 intake.setPower(0);


             }
             if(gamepad2.dpad_down) {
                 ok=false;   cuva.setPosition(0.5);

             }
             if(gamepad2.dpad_up)ok=true;


             //slider auto buttons
             if(gamepad2.cross)
             {
                 slider.setTargetPosition(-1500);
                 slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 slider.setPower(1);
                 intake.setDirection(DcMotorSimple.Direction.FORWARD);
                 intake.setPower(0.3);
                 rotire.setPosition(0.2);
             }
             if(gamepad2.square)
             {
                 slider.setTargetPosition(-1300);
                 slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 slider.setPower(1);
                 intake.setDirection(DcMotorSimple.Direction.FORWARD);
                 intake.setPower(0.3);
                 rotire.setPosition(0);
             }
             if(gamepad2.circle)
             {
                 slider.setTargetPosition(-100);
                 slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 slider.setPower(1);
                 intake.setDirection(DcMotorSimple.Direction.FORWARD);
                 intake.setPower(0);
                 rotire.setPosition(0.15);
             }  if(gamepad2.triangle)
             {  intake.setDirection(DcMotorSimple.Direction.FORWARD);
                 intake.setPower(0);
                 slider.setTargetPosition(0);
                 slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 slider.setPower(-1);
                 rotire.setPosition(0.97);

             }


             //telemetry


             telemetry.addData("Red", color.red());
             telemetry.addData("Green", color.green());
             telemetry.addData("Blue", color.blue());
             telemetry.addData("cuva",cuva.getPosition());
             telemetry.addData("slider",slider.getCurrentPosition());
             telemetry.update();





         }
     }
 }
