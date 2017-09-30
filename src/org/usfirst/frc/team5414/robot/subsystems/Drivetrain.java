package org.usfirst.frc.team5414.robot.subsystems;

import org.usfirst.frc.team5414.robot.Robot;
import org.usfirst.frc.team5414.robot.RobotMap;
import org.usfirst.frc.team5414.robot.commands.DrivewithJoystick;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends Subsystem {

	private SpeedController rightf_motor, rightb_motor, leftf_motor, leftb_motor;
//	private CANTalon leftf_motor, leftb_motor;
    private RobotDrive drive;
    private Encoder encoderFR; 
    private Encoder encoderBR; 
    private Encoder encoderBL; 
    private Encoder encoderFL; 
    PIDController motorController;
    private Encoder swerveFrontRight;
	Solenoid LightSol;
	public static DoubleSolenoid FL,FR,BL,BR; 
    
    public Drivetrain()
    {
    	//encoder for the angle of the swerve motor
    	swerveFrontRight = new Encoder(5, 6, false, Encoder.EncodingType.k4X);
    	
//    	encoderFR = new Encoder(RobotMap.DIOencoderFRa, RobotMap.DIOencoderFRb, false, Encoder.EncodingType.k4X);
//    	encoderBR = new Encoder(RobotMap.DIOencoderBRa, RobotMap.DIOencoderBRb, false, Encoder.EncodingType.k4X);
//    	encoderBL = new Encoder(RobotMap.DIOencoderBLa, RobotMap.DIOencoderBLb, false, Encoder.EncodingType.k4X);
//    	encoderFL = new Encoder(RobotMap.DIOencoderFLa, RobotMap.DIOencoderFLb, false, Encoder.EncodingType.k4X);
    	
//    	encoderFR.reset();
//    	encoderBR.reset();
//    	encoderBL.reset();
//    	encoderFL.reset();
    	
    	FL = new DoubleSolenoid(RobotMap.SolenoidFLa , RobotMap.SolenoidFLb );
    	FR = new DoubleSolenoid(RobotMap.SolenoidFRa,RobotMap.SolenoidFRb);
    	BL = new DoubleSolenoid(RobotMap.SolenoidBLa,RobotMap.SolenoidBLb);
    	BR = new DoubleSolenoid(RobotMap.SolenoidBRa,RobotMap.SolenoidBRb);

    	FL.set(DoubleSolenoid.Value.kForward);
    	FR.set(DoubleSolenoid.Value.kForward);
    	BL.set(DoubleSolenoid.Value.kForward);
    	BR.set(DoubleSolenoid.Value.kForward);

		leftb_motor = new Victor(RobotMap.PWMLeftBackMotor);
		leftf_motor = new Victor(RobotMap.PWMLeftFrontMotor);
		rightb_motor = new Victor(RobotMap.PWMRightBackMotor);
		rightf_motor = new Victor(RobotMap.PWMRightFrontMotor);
		leftf_motor.setInverted(Boolean.FALSE);
		leftb_motor.setInverted(Boolean.FALSE);
		rightf_motor.setInverted(Boolean.TRUE);
		rightb_motor.setInverted(Boolean.TRUE);
		drive = new RobotDrive(leftf_motor, leftb_motor, rightf_motor, rightb_motor);
		
//		LightSol = new Solenoid(RobotMap.LightSolenoid);
    }
    
    public double getSpeed()
    {
    	return 0;
//    	return (encoderFR.getRate() + encoderFL.getRate() + encoderBR.getRate() + encoderBL.getRate()) / 4. * RobotMap.LengthPerTick; 
    }
    
    public int getEncoderBL()
    {
//    	return encoderBL.get();
    	return 0;
    }
    
    public int getEncoderBR()
    {
//    	return -encoderBR.get();
    	return 0;
    }
    
    public void zeroEncoderBL()
    {
    	encoderBL.reset();
    }

    public void initDefaultCommand() {
    	
    	
    	setDefaultCommand(new DrivewithJoystick());
    	
    	
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
  
    
    public void arcadeDrive(Joystick stick){
    	SmartDashboard.putString("Drivemode", "Arcade"); //Displays the driving mode
    	double ax1; 	//X-axis of motion for robot
    	double ax2;		//Y-axis of motion for robot
    	
    	if(Math.abs(stick.getRawAxis(2)) < .087)	//Setting deadzone for the x-axis
    	{
    		ax1 = 0;
    	}
    	else
    	{
    		ax1 = stick.getRawAxis(2);
    	}
    	if(Math.abs(stick.getRawAxis(1)) < .18)		//Setting deazone for the y-axis
    	{
    		ax2 = 0;
    	}
    	else
    	{
    		ax2 = stick.getRawAxis(1);
    	}
    	
    	boolean ax1n = false, ax2n = false;
    	if(ax1 < 0) ax1n = true;
    	if(ax2 < 0) ax2n = true;
    	ax1 *= ax1; ax2 *= ax2; //scaling the output of the joystick to fine tune the end result
    	if(ax1n) ax1 *= -1;
    	if(ax2n) ax2 *= -1;
    	SmartDashboard.putNumber("EncoderBR", getEncoderBR());
    	SmartDashboard.putNumber("EncoderFR", -encoderFR.get());
    	SmartDashboard.putNumber("EncoderFL", getEncoderBL());
    	SmartDashboard.putNumber("EncoderBL", -encoderBL.get());
    	drive.arcadeDrive(-ax1,-ax2);
    }
    
    public void toggleLight()
    {
    	LightSol.set(!LightSol.get());
    }
    
    public void enableLight()
    {
    	LightSol.set(true);
    }
    
    public void disableLight()
    {
    	LightSol.set(false);
    }
    public void FullTraction(){

    	FL.set(DoubleSolenoid.Value.kReverse);		//this sets all of the solonoids to reverse so that tractions are engaged
    	FR.set(DoubleSolenoid.Value.kReverse);
    	BL.set(DoubleSolenoid.Value.kReverse);
    	BR.set(DoubleSolenoid.Value.kReverse);
    	
    } 
    public void FullButterfly(){

    	FL.set(DoubleSolenoid.Value.kForward);		//this sets all of the solonoids to full thrust, engaging the mecanum 
    	FR.set(DoubleSolenoid.Value.kForward);
    	BL.set(DoubleSolenoid.Value.kForward);
    	BR.set(DoubleSolenoid.Value.kForward);
    }
    public void HalfButterfly(){
    	FL.set(DoubleSolenoid.Value.kForward);		//this code isnt used for mechanums, but for omnis, sets half of the solonoids to engage omni's
    	FR.set(DoubleSolenoid.Value.kForward);
    	BL.set(DoubleSolenoid.Value.kReverse);
    	BR.set(DoubleSolenoid.Value.kReverse);
    } 
    public void drive(double left, double right) {
    	drive.tankDrive(left, -right);
    	
    }
    
    public void arcadeDrive(double throttle, double twist){
    	drive.arcadeDrive(throttle,twist);
    }
    
    public double getDistanceL()
	{
		return encoderBL.getDistance();
	}
	public double getDistanceR()
	{
		return encoderBR.getDistance();
	}
	public void reset()
	{
		encoderBL.reset();
		encoderBR.reset();
	}
	
    public void stop(){
    		drive.tankDrive(0,0);
    }
    
    public void swerveDrive(Joystick stick)
    {
    	//these three commands get input from the joysticks
    	double rotate = stick.getRawAxis(2);  	
    	double strafe = stick.getRawAxis(1);
    	double forward = stick.getRawAxis(0);
    	
    	//gets the angle that the encoder is currently on
    	double theta = getFrontRightAngle();
    	double temp = forward * Math.cos(theta) + strafe*Math.sin(theta);
    	strafe = -forward * Math.sin(theta) + strafe * Math.cos(theta);
    	forward = temp;
    	
    	//the ratio of the base of the robot
    	double length = 1;
    	double width = 1;
    	double radius = Math.sqrt(length * length + width * width);
    	
    	//
    	double A = strafe - (rotate * (length / radius));
    	double B = strafe + (rotate * (length / radius));
    	double C = forward - (rotate * (length / radius));
    	double D = forward + (rotate * (length / radius));
    	
    	//calculating the wheel speed
    	double ws1 = Math.sqrt(B*B + C*C);
    	double ws2 = Math.sqrt(B*B + D*D);
    	double ws3 = Math.sqrt(A*A + D*D);
    	double ws4 = Math.sqrt(A*A + C*C);
    
    	//calculating the angles of the wheels in degress
    	double wa1 = Math.atan(B/C) * 180/Math.PI;
    	double wa2 = Math.atan(B/D) * 180/Math.PI;
    	double wa3 = Math.atan(A/D) * 180/Math.PI;
    	double wa4 = Math.atan(A/C) * 180/Math.PI;
    	
    	double max = ws1;
    	
    	if(ws2 > max)
    		max=ws2;
    	if(ws3 > max)
    		max=ws3;
    	if(ws4 > max)
    		max=ws4;
    	
    	if(max>1) {
    		ws1/=max;
    		ws2/=max;
    		ws3/=max;
    		ws4/=max;
    	}
    	SmartDashboard.putNumber("Encoder Swerve", swerveFrontRight.);
    	
    	rightf_motor.set(ws1);
    	leftf_motor.set(ws2);
    	rightb_motor.set(ws3);
    	leftb_motor.set(ws4);
    	
    }
    
    
    public double getFrontRightAngle()
    {
    	double ticks = swerveFrontRight.get() % 1024;
    	double ticksPerRotation = 1024;
    	return ticks / ticksPerRotation * 360;
    }
    
    public void mecanumDrive(Joystick stick) {
    	SmartDashboard.putString("Drivemode", "Mecanum");  //publishing the drive mode to smartdashboard
    	double moveY; 	//X-axis of motion for robot
    	double Rotate;		//Rotation of motion for robot
    	
    	if(Math.abs(stick.getRawAxis(2)) < .37)	//Setting deadzone for the x-axis
    	{
    		Rotate = 0;
    	}
    	else
    	{
    		Rotate = stick.getRawAxis(2);
    	}
    	if(Math.abs(stick.getRawAxis(1)) < .18)		//Setting deazone for the y-axis
    	{
    		moveY = 0;
    	}
    	else
    	{
    		moveY = stick.getRawAxis(1);
    	}
    	
    	boolean moveYNegative = false, RotateNegative = false;
    	if(moveY < 0) moveYNegative = true;
    	if(Rotate < 0) RotateNegative = true;
    	moveY *= moveY; Rotate *= Rotate; //scaling the output of the joystick to fine tune the end result
    	if(moveYNegative) moveY *= -1;
    	if(RotateNegative) Rotate *= -1;
    	
		double moveX = stick.getRawAxis(0);					//setting joystick values to the axis'
		
    	moveY = -moveY;
    	Rotate = -Rotate;
    	
    	int StrafeRight = 90;							//Sets POV values to variables 
    	int StrafeLeft = 270;
    	int MoveForward = 0;
    	int MoveBackward = 180;
    	
    	double FBFactor = 1;							//setting scaling factors for the POV, to change the speeds of each direction
    	double strafeFactor = 1;
    	double rotateFactor = 1.25;
    	
    	double countIterations = 0;						
    	double desiredHeading = 0;
    	
    	double moveSpeedPOV = .3;						//Speed of the POV directions 
    	
    	
    	
    	if(stick.getPOV() == StrafeRight){				//Checks POV direction of joystick. Sets movement values if true
    		moveX = moveSpeedPOV;
    		moveY = 0.0;
    		Rotate = 0.0;	
    	}
    	else if(stick.getPOV() == StrafeLeft){
    		moveX = -moveSpeedPOV;
    		moveY = 0.0;
    		Rotate = 0.0;	
    	}
    	else if(stick.getPOV() == MoveForward){
    		moveX = 0.0;
    		moveY = moveSpeedPOV;
    		Rotate = 0.0;	
    	}
    	else if(stick.getPOV() == MoveBackward){
    		moveX = 0.0;
    		moveY = -moveSpeedPOV;
    		Rotate = 0.0;	
    	}
    	
    	
    	
    	
    	if ((-0.2 < moveX) && (moveX < 0.2)) {			//declaring deadzones for mecanum 
			moveX = 0.0;
		}
		if ((-0.2 < moveY) && (moveY < 0.2)) {
			moveY = 0.0;
		}
		if ((-0.2 < Rotate) && (Rotate < 0.07)) {
			Rotate = 0.0;
		}
		
		
		
		moveX *= strafeFactor;							//scaling the movement factors 
		moveY *= FBFactor;
		Rotate *= rotateFactor;
		
		
		
		if ((-0.01 < Rotate) && (Rotate < 0.01)) {
			// Rotate is practically zero, so just set it to zero and
			// increment iterations
			Rotate = 0.0;
			countIterations++;
		} else {
			// Rotate is being commanded, so clear iteration counter
			countIterations = 0;
		}
		
		// preserve heading when recently stopped commanding rotations
				if (countIterations == 5) {
					desiredHeading = Robot.gyro.getYaw();
				} else if (countIterations > 5) {
					Rotate = (desiredHeading - Robot.gyro.getYaw()) / 40.0;
				}
//				SmartDashboard.putNumber("EncoderBR", encoderBR.get());
//		    	SmartDashboard.putNumber("EncoderFR", encoderFR.get());
//		    	SmartDashboard.putNumber("EncoderFL", encoderFL.get());
//		    	SmartDashboard.putNumber("EncoderBL", encoderBL.get());
				drive.mecanumDrive_Cartesian(moveX, moveY, Rotate,0);
    }
    
    public void mecanumDrive(double strafeSpeed, double straightSpeed)
    {
    	drive.mecanumDrive_Cartesian(strafeSpeed, straightSpeed, 0, 0);
    }
    
    public void mecanumDrive(double strafeSpeed, double straightSpeed, double yaw)
    {
    	drive.mecanumDrive_Cartesian(strafeSpeed, straightSpeed, 0, yaw);
    }
}