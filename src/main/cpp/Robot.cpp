// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_leftLeadMotor->RestoreFactoryDefaults();
  m_rightLeadMotor->RestoreFactoryDefaults();
  m_leftFollowMotor->RestoreFactoryDefaults();
  m_rightFollowMotor->RestoreFactoryDefaults();

  m_leftEncoder.SetPosition(0);
  m_rightEncoder.SetPosition(0);

  m_leftLeadMotor->SetInverted(true);
  m_leftFollowMotor->Follow(*m_leftLeadMotor, false);
  m_rightLeadMotor->SetInverted(false);
  m_rightFollowMotor->Follow(*m_rightLeadMotor, false);
}
void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("left y: ", -(m_stick->GetRawAxis(1)));
  frc::SmartDashboard::PutNumber("right x: ", m_stick->GetRawAxis(4));
  frc::SmartDashboard::PutNumber("left error", errorL);
  frc::SmartDashboard::PutNumber("right error", errorR);
}

void Robot::AutonomousInit() {
  m_P = 0.5, m_I = 0.25, m_D = 0.1, kMinOutput = -0.5, kMaxOutput = 0.5, setpoint = 5;

  m_leftPIDController.SetP(m_P);
  m_leftPIDController.SetP(m_I);
  m_leftPIDController.SetP(m_D);
  m_leftPIDController.SetOutputRange(kMinOutput, kMaxOutput);

  m_rightPIDController.SetP(m_P);
  m_rightPIDController.SetP(m_I);
  m_rightPIDController.SetP(m_D);
  m_rightPIDController.SetOutputRange(kMinOutput, kMaxOutput);

  m_leftEncoder.SetPosition(0);
  m_rightEncoder.SetPosition(0);
}
void Robot::AutonomousPeriodic() {
  errorL = m_robotDrive->feet2Rots(setpoint) - m_leftEncoder.GetPosition();
  errorR = m_robotDrive->feet2Rots(setpoint) - m_rightEncoder.GetPosition();

  m_leftPIDController.SetReference(errorL, rev::ControlType::kPosition);
  m_rightPIDController.SetReference(errorR, rev::ControlType::kPosition);
}

void Robot::TeleopInit() {
  m_leftEncoder.SetPosition(0);
  m_rightEncoder.SetPosition(0);
}
void Robot::TeleopPeriodic() {
  left_y = m_stick->GetRawAxis(1);
  right_x = m_stick->GetRawAxis(4);

  m_robotDrive->ArcadeDrive(-left_y, right_x);
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