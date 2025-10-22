g++ EV3_Localization.c ./EV3_RobotControl/btcomm.c -lbluetooth

g++ calibration.c EV3_RobotControl/btcomm.c -o color_calibration -lm -I. -I./EV3_RobotControl -lbluetooth