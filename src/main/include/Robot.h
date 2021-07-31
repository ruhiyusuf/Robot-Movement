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
  static const int leftLeadDeviceID = 1, rightLeadDeviceID = 2, leftFollowDeviceID = 3, rightFollowDeviceID = 4; // replace with actual motor IDs
  rev::CANSparkMax* m_leftLeadMotor = nullptr; 
  rev::CANSparkMax* m_rightLeadMotor = nullptr; 
  rev::CANSparkMax* m_leftFollowMotor = nullptr; 
  rev::CANSparkMax* m_rightFollowMotor = nullptr; 
  
  frc::DifferentialDrive* m_robotDrive = nullptr;

  double left_y = 0.0, right_x = 0.0;
  frc::XboxController* controller = nullptr;
  frc::GenericHID::JoystickHand left_analog {frc::GenericHID::kLeftHand};
  frc::GenericHID::JoystickHand right_analog {frc::GenericHID::kRightHand};
  /*
  * 5.7 inch diameter wheels
  * 14 teeth on driver gear
  * 40 teeth on driven gear
  */
  const double wheel2GearR = ((5.7 * M_PI) / 12) / (40 / 14);
  double setpoint = 0.0, lError = 0.0, rError = 0.0, kP = 0.0, lSpeed = 0.0, rSpeed = 0.0;
  const double wheel_circum = (5.7 * M_PI) / 12; 
  double l_wheel_rots = 0.0, r_wheel_rots = 0.0, l_wheel_dist = 0.0, r_wheel_dist = 0.0;

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
