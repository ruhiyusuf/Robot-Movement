// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include "rev/CANSparkMax.h"
#include <frc/XboxController.h>

class Robot : public frc::TimedRobot {
 public:
  static const int leftLeadDeviceID = 1, rightLeadDeviceID = 2, leftFollowDeviceID = 3, rightFollowDeviceID = 4; // replace with actual motor IDs
  rev::CANSparkMax* m_leftLeadMotor; 
  rev::CANSparkMax* m_rightLeadMotor; 
  rev::CANSparkMax* m_leftFollowMotor; 
  rev::CANSparkMax* m_rightFollowMotor; 

  frc::XboxController* controller;
  double left_y{0.0};

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
