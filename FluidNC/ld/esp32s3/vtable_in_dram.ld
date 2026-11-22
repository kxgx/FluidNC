/* List of files and sections to place in RAM instead of FLASH */
/* See README.md in this directory for a complete explanation */

    *Laser.cpp.o(.rodata .rodata.*)

    /* All files whose name ends with Spindle.cpp */
    **Spindle.cpp.o(.rodata .rodata.*)

    /* Motors */
    *Motor.cpp.o(.rodata .rodata.*)
    *Dynamixel2.cpp.o(.rodata .rodata.*)
    *MotorDriver.cpp.o(.rodata .rodata.*)
    *NullMotor.cpp.o(.rodata .rodata.*)
    *RcServo.cpp.o(.rodata .rodata.*)
    *Servo.cpp.o(.rodata .rodata.*)
    *Solenoid.cpp.o(.rodata .rodata.*)
    *StandardStepper.cpp.o(.rodata .rodata.*)
    *StepStick.cpp.o(.rodata .rodata.*)
    *TMC*Driver.cpp.o(.rodata .rodata.*)
    *TrinamicBase.cpp.o(.rodata .rodata.*)
    *TrinamicSpiDriver.cpp.o(.rodata .rodata.*)
    *TrinamicUartDriver.cpp.o(.rodata .rodata.*)
    *UnipolarMotor.cpp.o(.rodata .rodata.*)

    /* Pin Details */
    **Detail.cpp.o(.rodata .rodata.*)
    *PwmPin.cpp.o(.rodata .rodata.*)
    *Pin.cpp.o(.rodata .rodata.*)
	*PinExtenderDriver.cpp.o(.rodata .rodata.*)
	*Extender.cpp.o(.rodata .rodata.*)
