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

  m_leftLeadMotor->SetInverted(true);
  m_leftFollowMotor->Follow(*m_leftLeadMotor);
  m_rightLeadMotor->SetInverted(false);
  m_rightFollowMotor->Follow(*m_rightLeadMotor);

  m_leftLeadMotor->SetSmartCurrentLimit(40);
  m_rightLeadMotor->SetSmartCurrentLimit(40);
  m_leftFollowMotor->SetSmartCurrentLimit(40);
  m_rightFollowMotor->SetSmartCurrentLimit(40);

  m_leftLeadMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rightLeadMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_leftFollowMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rightFollowMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  m_leftLeadMotor->GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyOpen).EnableLimitSwitch(false);
  m_rightLeadMotor->GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyOpen).EnableLimitSwitch(false);
  m_leftFollowMotor->GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyOpen).EnableLimitSwitch(false);
  m_rightFollowMotor->GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyOpen).EnableLimitSwitch(false);

  m_leftLeadMotor->GetReverseLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyOpen).EnableLimitSwitch(false);
  m_rightLeadMotor->GetReverseLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyOpen).EnableLimitSwitch(false);
  m_leftFollowMotor->GetReverseLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyOpen).EnableLimitSwitch(false);
  m_rightFollowMotor->GetReverseLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyOpen).EnableLimitSwitch(false);

  controller = new frc::XboxController{0}; // replace with USB port number on driver station
}
void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("left y: ", controller->GetY(left_analog));
  frc::SmartDashboard::PutNumber("right x: ", controller->GetX(right_analog));
  frc::SmartDashboard::PutNumber("left wheel distance: ", m_leftLeadMotor->GetEncoder().GetPosition() * kDriveRots2Feet);
  frc::SmartDashboard::PutNumber("right wheel distance: ", m_rightLeadMotor->GetEncoder().GetPosition() * kDriveRots2Feet);
}

void Robot::AutonomousInit() {
  kP = 0.0;
  m_leftLeadMotor->GetEncoder().SetPosition(0);
  m_rightLeadMotor->GetEncoder().SetPosition(0);
  setpoint = lDistance = rDistance = lSpeed = rSpeed = lError = rError = 0.0;
}
void Robot::AutonomousPeriodic() {
  if (controller->GetAButton()) {
    kP = 0.1;                 // needs tuning
    setpoint = 5.0;
  }
  else if (controller->GetBButton()) {
    kP = 0.0;
  }

  lDistance = m_leftLeadMotor->GetEncoder().GetPosition() * kDriveRots2Feet;
  rDistance = m_rightLeadMotor->GetEncoder().GetPosition() * kDriveRots2Feet;

  lError = setpoint - lDistance;
  rError = setpoint - rDistance;

  lSpeed = kP * lError;
  rSpeed = kP * rError;
  
  m_leftLeadMotor->Set(lSpeed);
  m_rightLeadMotor->Set(rSpeed);
}

void Robot::TeleopInit() {
  m_leftLeadMotor->GetEncoder().SetPosition(0);
  m_rightLeadMotor->GetEncoder().SetPosition(0);
}
void Robot::TeleopPeriodic() {
  left_y = controller->GetY(left_analog);
  right_x = controller->GetX(right_analog);

  m_leftLeadMotor->Set(-left_y);
  m_rightLeadMotor->Set(-left_y);
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
