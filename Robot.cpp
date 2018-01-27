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

	// drive train
	frc::Spark frontLeftMot{2};
	frc::Spark rearLeftMot{3};
	frc::SpeedControllerGroup lMots{ frontLeftMot, rearLeftMot };

	frc::Spark frontRightMot{0};
	frc::Spark rearRightMot{1};
	frc::SpeedControllerGroup rMots{ frontRightMot, rearRightMot };
	frc::DifferentialDrive drive{lMots, rMots};

	// gyro
	ADXRS450_Gyro gyro;		 // get angles and stuff

	// xbox controller
	frc::Joystick xbox{0};

	// pneumatics
	frc::DoubleSolenoid dumper{0, 1};
	frc::DoubleSolenoid grabber{2, 3};


	// autonomous chooser code
	frc::LiveWindow& lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> autoChooser;
	const std::string autoNameDefault = "Default";
	const std::string autoDriveStraight = "drive straight";
	std::string autoSelected;

	void RobotInit() {
		autoChooser.AddDefault(autoNameDefault, autoNameDefault);
		autoChooser.AddObject(autoDriveStraight, autoDriveStraight);
		frc::SmartDashboard::PutData("Auto Modes", &autoChooser);
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
		autoSelected = autoChooser.GetSelected();
		// m_autoSelected = SmartDashboard::GetString(
		// 		"Auto Selector", kAutoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if (autoSelected == autoDriveStraight) {
			const double TURNINGCONST = 0.03;
			gyro.Reset();
			double offset = gyro.GetAngle();
			for (int i = 0; i < 160; i++){
				Wait (0.05 / 4);
				drive.ArcadeDrive(-.5, (offset - gyro.GetAngle()) * TURNINGCONST);
			}
			drive.ArcadeDrive(0,0);
		} else {
			// do nothing
		}
	}

	void AutonomousPeriodic() {
		if (autoSelected == autoDriveStraight) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void TeleopInit() {}

	void TeleopPeriodic() {

		// drive based on controller input
		drive.ArcadeDrive(-xbox.GetRawAxis(1), xbox.GetRawAxis(4));

		// control pneumatics
		if (xbox.GetRawButton(1)) {
			dumper.Set(frc::DoubleSolenoid::kForward);
		} else if (xbox.GetRawButton(2)) {
			dumper.Set(frc::DoubleSolenoid::kReverse);
		} else {
			dumper.Set(frc::DoubleSolenoid::kOff);
		}
		if (xbox.GetRawButton(3)) {
			grabber.Set(frc::DoubleSolenoid::kForward);
		} else if (xbox.GetRawButton(4)) {
			grabber.Set(frc::DoubleSolenoid::kReverse);
		} else {
			grabber.Set(frc::DoubleSolenoid::kOff);
		}
	}

	void TestPeriodic() {}

};

START_ROBOT_CLASS(Robot)
