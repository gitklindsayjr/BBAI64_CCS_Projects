This project file was created to control the velocity of a DC brush motor with an attached quadrature encoder using the Beaglebone AI-64 PRU0.  The encoder ouputs are connected to pins P8-33 & 35 that is connected to the BBAI-64's system eQEP.  A velocity error signal sampled at 1 Khzis calculated and fed through a PID controller and then to the PRU's APWM 20 Khz output P8-15.  The PWM is combined with directional logic, prgo_pru0_gpo16 P8-12, and then fed to a FULL bridge motor amp.  Additional outputs are used for debug diagnostics to aid in code development.  Scope output on P8-16 is currently being used for monitoring the error signal.

Control of the PRU0 is accomplished through the Linux "/dev/rpmsg_pru30" interface.  A tokenized command set is defined in the "motor_cntrl_msg.h" file.  Their is a companion file "motor-cntrl-msg" that run's natively on the BBAI-64.  This project was built with Eclipse and Cross Compiled on a Debian 11 (Bullseye) Desktop PC.  Refer to the "motor_control_msg.tar.xz' Eclipse project.

This Project was built with CCS12.4 using the pru_support-package-20.2 running on the BBAI-64.

Linux BeagleBone 5.10.168-ti-ar,m64-r103 Bullseye

Steps required to use this example
1.  Download CCS12.4 "CCS12.4.0.00007_linux-x64.tar.xz" and extract
2.  Run extracted installer "CCS12.4.0.00007_linux-x64-installer.run"
2.  Install in your home directory as "~/ti/ccs1240"
3.  Check the proper device support during your install
4.  Launch CCS and install support for the PRU compiler.
5.  Download TI's pru software support package "pru-software-support-package-6.2.0.tar.xz"
6.  Extract support package in your home directory "~/ti"
7.  Download and extract "pru0_motor_control.tar.xz"
8.  Launch CCS and "Import CCS Project" Browse to location of extracted project and select "MotorControlPru0"
9.  Check Copy projects into workspace assuming you created one other then the default.
10. In addition "Import CCS Project" Browse to location of extracted project and select "PruLibrary"
11. Change device paths's select menu item Window->Preferences->Code Composer Studio->Environment" and edit "/home/pops" to your home directory. Apply and Close.
12. Right click the MotorControlPru0 project and select "Properties->CCS Build->PRU Compiler->Include Options edit anything without changing it then if APPLY and CLOSEW.  This step will update the indexer to find your header files.  If successful the Includes folder shown in your project tree will go from greyed out to active.
13. Repeat the above process with the PruLibrary project.
14. Hopefully everything will go well and you can successfully build these 2 projects.

Note: I use the Remote Systems SSH connection to my BBAI-64 to drage and drop my "Binaries/MotorControlPru0.out" file on my Beaglebone My Home directory.

