package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;
import org.usfirst.frc.team5414.robot.RobotMap;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveEncDist extends Command{
	double initDistance;
	double distance;
	double originalAngle;
	double targetFeet;
	double drivep = .15;
	double Inputspeed;
	static final double CRUISE_VELOCITY = 14;
	static final double ACCELERATION = 3;
	double initialTime = 0;
	double initialEncoderTicks = 0;
	double lastError = 0;
	double lastTime = 0;
	
	final double Kv = 1 / CRUISE_VELOCITY;
	final double Ka = .06;
	final double Kp = 3.5;
	final double Kd = 200;

    public DriveEncDist(double d) {
    	targetFeet = d;
    	distance = d / RobotMap.LengthPerTick;
    	distance *= .92;
    	setTimeout(4);
    	requires(Robot.drivetrain);
	}
    
    public DriveEncDist(double d, double s) {}
    public DriveEncDist(double d, double s, double t) {}

    protected void initialize() {
    	
    	initialTime = Timer.getFPGATimestamp();
    	lastTime = Timer.getFPGATimestamp();
    	initialEncoderTicks = Robot.drivetrain.getEncoderBL();
    	Robot.drivetrain.FullTraction();
    	Robot.gyro.zeroYaw();
    	originalAngle = Robot.gyro.getYaw();
    	initDistance = Robot.drivetrain.getEncoderBL();
    }

    protected void execute() {
    	double changeInAngle = Robot.gyro.getYaw()-originalAngle;
    	Trajectory trajectory = getTrajectory(Math.abs(targetFeet), Timer.getFPGATimestamp() - initialTime);

    	if(Robot.drivetrain.getEncoderBL() < (initDistance + distance) && targetFeet >= 0) //positve
    	{
    		double error = trajectory.distance - (Robot.drivetrain.getEncoderBL() - initialEncoderTicks) * RobotMap.LengthPerTick;
    		double errorDeriv = (error - lastError) / lastTime;
    		double motorSpeed = Kv * trajectory.speed + Ka * trajectory.acceleration + Kp * error + Kd * errorDeriv;
    		Robot.drivetrain.arcadeDrive(changeInAngle * drivep, motorSpeed);
    		lastError = error;
    		lastTime = Timer.getFPGATimestamp();
    	}
    	else if(Robot.drivetrain.getEncoderBL() > (initDistance + distance) && targetFeet >= 0)
    	{
    		Robot.drivetrain.stop();
    	}
    	
    	else if(Robot.drivetrain.getEncoderBL() > (distance + initDistance) && targetFeet < 0) //negative
    	{
    		double error = (Robot.drivetrain.getEncoderBL() - initialEncoderTicks) * RobotMap.LengthPerTick - trajectory.distance;
    		double errorDeriv = (error - lastError) / lastTime;
    		double motorSpeed = Kv * trajectory.speed + Ka * trajectory.acceleration + Kp * error + Kd * errorDeriv;
    		Robot.drivetrain.arcadeDrive(changeInAngle * drivep, -motorSpeed);
    		lastError = error;
    		lastTime = Timer.getFPGATimestamp();
    	}
    	else if(Robot.drivetrain.getEncoderBL() < (distance + initDistance) && targetFeet < 0)
    	{
    		Robot.drivetrain.stop();
    	}

    	SmartDashboard.putNumber("Difference In Angle", Robot.gyro.getYaw() - originalAngle);
    	SmartDashboard.putNumber("Desired EncDist", initDistance + distance);
    	SmartDashboard.putNumber("Current EncDist", Robot.drivetrain.getEncoderBL());
    }

    protected boolean isFinished() {
    	if(isTimedOut()) return true;
    	if(Robot.drivetrain.getEncoderBL() >= initDistance + distance && targetFeet > 0){
    		return true;
    	}
    	else if((Robot.drivetrain.getEncoderBL() <= (initDistance + distance)) && targetFeet < 0)
    	{
    		return true;
    	}
    	else if(targetFeet == 0) return true;
    	
    	return false;
    }

    protected void end() {
    	Robot.drivetrain.stop();
    }

    protected void interrupted() {
    	end();
    }
    
    public static Trajectory getTrajectory(double distance, double time)
    {
        double accelTime = CRUISE_VELOCITY / ACCELERATION;
        double accelDistance = .5 * ACCELERATION * accelTime * accelTime;

        if(accelDistance * 2 <= distance) //trapezoidal
        {
            double rectangleDistance = distance - accelDistance*2;
            double cruiseTime = rectangleDistance / CRUISE_VELOCITY;

            if(time < accelTime)
            {
                double s = ACCELERATION * time;
                double a = ACCELERATION;
                double d = .5 * s * time;
                return new Trajectory(s, d, a);
            }

            if(cruiseTime + accelTime <= time)
            {
                double timeAfterDecelerationStarted = time - cruiseTime - accelTime;
                double toReturn = CRUISE_VELOCITY - ACCELERATION * timeAfterDecelerationStarted;
                if(toReturn < 0) toReturn = 0;
                double s = toReturn;
                double a = -ACCELERATION;
                if(toReturn == 0) a = 0;
                double d = distance - (accelTime - timeAfterDecelerationStarted )*s/2;
                return new Trajectory(s, d, a);
            }
            double s = CRUISE_VELOCITY;
            double a = 0;
            double d = accelDistance + (time - accelTime)*CRUISE_VELOCITY;
            return new Trajectory(s, d, a);
        }


        else //triangular
        {
            accelDistance = distance / 2.;
            accelTime = Math.sqrt(2 * accelDistance / ACCELERATION);
            if(time <= accelTime)
            {
                double s = ACCELERATION * time;
                double a = ACCELERATION;
                double d = .5 * s * time;
                return new Trajectory(s, d, a);
            }
            else
            {
                double peakVelocity = ACCELERATION * accelTime;
                double toReturn = peakVelocity - ACCELERATION * (time - accelTime);
                if(toReturn < 0) toReturn = 0;
                double s = toReturn;
                double a = -ACCELERATION;
                if(toReturn == 0) a = 0;
                double d = distance - (accelTime * 2 - time) * s / 2;
                return new Trajectory(s, d, a);
            }
        }
    }
    
}

class Trajectory
{

    public static double speed = 0;
    public static double distance = 0;
    public static double acceleration = 0;

    public Trajectory(double spd, double dist, double accel)
    {
        speed = spd;
        distance = dist;
        acceleration = accel;
    }

    @Override
    public String toString()
    {
        return String.format("%.2f", speed) + "Ft/s | " + String.format("%.2f", distance)+ " Ft | " + acceleration + " Ft/s^2";
    }
}
