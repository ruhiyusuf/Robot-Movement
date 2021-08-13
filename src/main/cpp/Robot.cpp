// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_leftLeadMotor = new rev::CANSparkMax(leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  m_rightLeadMotor = new rev::CANSparkMax(rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  m_leftFollowMotor = new rev::CANSparkMax(leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  m_rightFollowMotor = new rev::CANSparkMax(rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless);

  m_leftLeadMotor->RestoreFactoryDefaults();
  m_rightLeadMotor->RestoreFactoryDefaults();
  m_leftFollowMotor->RestoreFactoryDefaults();
  m_rightFollowMotor->RestoreFactoryDefaults();

  m_leftLeadMotor->GetEncoder().SetPosition(0);
  m_rightLeadMotor->GetEncoder().SetPosition(0);

  m_leftLeadMotor->SetInverted(true);
  m_leftFollowMotor->Follow(*m_leftLeadMotor, false);
  m_rightLeadMotor->SetInverted(false);
  m_rightFollowMotor->Follow(*m_rightLeadMotor, false);

  controller = new frc::XboxController{0}; // replace with USB port number on driver station
}
void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("left y: ", -(controller->GetY(left_analog)));
  frc::SmartDashboard::PutNumber("right y: ", -(controller->GetY(right_analog)));
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  m_leftLeadMotor->GetEncoder().SetPosition(0);
  m_rightLeadMotor->GetEncoder().SetPosition(0);
}
void Robot::TeleopPeriodic() {
  left_y = controller->GetY(left_analog);
  right_y = controller->GetY(right_analog);

  if (abs(left_y) < 0.08) 
    left_y = 0;

  if (abs(right_y) < 0.08)
    right_y = 0;

  m_leftLeadMotor->Set(-left_y);
  m_rightLeadMotor->Set(-right_y);
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
