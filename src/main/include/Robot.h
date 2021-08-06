// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include "rev/CANSparkMax.h"
#include <frc/XboxController.h>
#include <rev/CANEncoder.h>
#include <frc/drive/DifferentialDrive.h>
#include <math.h>

class Robot : public frc::TimedRobot {
 public:
  static const int rightLeadDeviceID = 1, rightFollowDeviceID = 2, leftLeadDeviceID = 3, leftFollowDeviceID = 4;
  rev::CANSparkMax* m_leftLeadMotor = nullptr; 
  rev::CANSparkMax* m_rightLeadMotor = nullptr; 
  rev::CANSparkMax* m_leftFollowMotor = nullptr; 
  rev::CANSparkMax* m_rightFollowMotor = nullptr;

  double left_y = 0.0, right_x = 0.0;
  frc::XboxController* controller = nullptr;
  frc::GenericHID::JoystickHand left_analog {frc::GenericHID::kLeftHand};
  frc::GenericHID::JoystickHand right_analog {frc::GenericHID::kRightHand};
  /*
  * 5.7 inch diameter wheels
  * 14 teeth on driver gear
  * 40 teeth on driven gear
  */
  const double kGearRatio = (40 / 14), kWheelCircum = (5.7 * M_PI);
  const double kDriveRots2Feet = kGearRatio * kWheelCircum * (1/12);
  double setpoint = 0.0, kP = 0.0, lDistance, rDistance, lError = 0.0, rError = 0.0, lSpeed = 0.0, rSpeed = 0.0;

  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

};
