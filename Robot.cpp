/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>

#include "WPILib.h"
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

class Robot : public frc::IterativeRobot {
public:

	// left side of drivetrain
	frc::PWMVictorSPX left0Mot{0}, left1Mot{1}, left2Mot{2};
	frc::SpeedControllerGroup lMots{ left0Mot, left1Mot, left2Mot };

	// right side of drivetrain
	frc::PWMVictorSPX right0Mot{3}, right1Mot{4}, right2Mot{5};
	frc::SpeedControllerGroup rMots{ right0Mot, right1Mot, right2Mot };

	// drive train object
	frc::DifferentialDrive drive{lMots, rMots};

	// xbox ctlr for main driver
	frc::Joystick driveCtl{0};
	frc::Joystick fxnCtl{1};

	frc::Talon armCtl{7};

	// gyro
	frc::ADXRS450_Gyro gyro;

	// pneumatics

	// claw to grab boxes
	frc::DoubleSolenoid grabber{7, 6};
	// launch cube from front
	frc::DoubleSolenoid flipper{0, 1};


	// autonomous chooser on DS
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoDriveStraight = "Drive Straight (no dump)";	  // drive forwards
	const std::string kAutoDriveStraightLeft = "drive straight (left)";   // starting on left side of field
	const std::string kAutoDriveStraightRight = "drive straight (right)"; // starting on right side of field
	std::string m_autoSelected;




	void RobotInit(){

		gyro.Calibrate();

		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoDriveStraight, kAutoDriveStraight);
		m_chooser.AddObject(kAutoDriveStraightLeft, kAutoDriveStraightLeft);
		m_chooser.AddObject(kAutoDriveStraightRight, kAutoDriveStraightRight);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to
	 * select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to
	 * the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as
	 * well.
	 */
	void AutonomousInit() override {

		// get chosen auto
		m_autoSelected = m_chooser.GetSelected();
		std::cout << "Auto selected: " << m_autoSelected << std::endl;

		// drive straight dont dump
		if (m_autoSelected == kAutoDriveStraight) {
			double center = gyro.GetAngle();
			// drive in direction of base angle 1000 times
			for(unsigned i = 0; i < 1000; i++){
				// redirect towards base angle
				drive.ArcadeDrive(.5, (gyro.GetAngle() - center) / 45);
				// drive straight 2 seconds / 1000 times
				Wait(2 / 1000);

			}

		} else if(m_autoSelected == kAutoDriveStraightLeft) {
			double center = gyro.GetAngle();
			// drive in direction of base angle 1000 times
			for(unsigned i = 0; i < 1000; i++){
				// redirect towards base angle
				drive.ArcadeDrive(.5, (gyro.GetAngle() - center) / 45);
				// drive straight 2 seconds / 1000 times
				Wait(2 / 1000);
			}
			if(startLeft()){
				flipper.Set(frc::DoubleSolenoid::kForward);
			}

		}else if(m_autoSelected == kAutoDriveStraightRight) {
			double center = gyro.GetAngle();
			// drive in direction of base angle 1000 times
			for(unsigned i = 0; i < 1000; i++){
				// redirect towards base angle
				drive.ArcadeDrive(.5, (gyro.GetAngle() - center) / 45);
				// drive straight 2 seconds / 1000 times
				Wait(2 / 1000);
			}
			if(!startLeft()){
				flipper.Set(frc::DoubleSolenoid::kForward);
			}
		}
		// Stop
		drive.ArcadeDrive(0, 0);
	}

	bool startLeft(){
		std::string msg = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		while(msg.length() == 0){
			msg = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		}
		return msg[0] == 'L';
	}

	void AutonomousPeriodic(){
		if (m_autoSelected == kAutoDriveStraight) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}



	void TeleopInit(){}
	void TeleopPeriodic(){


		static bool switchable = true;
		static float dir = 1;
		if (switchable && driveCtl.GetRawButton(1)) {
			dir = -dir;
		} else if (!switchable && !driveCtl.GetRawButton(2)) {
			switchable = true;
		}

		//reverse elevator
		// axis 3

		// arcade drive with 2 sticks & 80% turn speed
		drive.ArcadeDrive(dir * driveCtl.GetRawAxis(1), driveCtl.GetRawAxis(4) * 0.8);

		// flipper control
		if (fxnCtl.GetRawAxis(3) > 0.75) {
			flipper.Set(frc::DoubleSolenoid::kForward);
		} else {
			flipper.Set(frc::DoubleSolenoid::kReverse);
		}

		// grabber control
		if (fxnCtl.GetRawButton(3)) {
			grabber.Set(frc::DoubleSolenoid::kForward);

		} else if (fxnCtl.GetRawButton(4)) {
			grabber.Set(frc::DoubleSolenoid::kReverse);
		}

		// elevator
		if (fxnCtl.GetRawButton(6)) {
			armCtl.Set(.5);
		} else if (fxnCtl.GetRawButton(5)) {
			armCtl.Set(-.5);
		} else {
			armCtl.Set(0);
		}




	}

	void TestPeriodic() {}

};

START_ROBOT_CLASS(Robot)
