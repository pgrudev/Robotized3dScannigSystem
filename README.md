# Robotized3dScannigSystem
This aplication was part of my Master's Thesis. It is a robotic system designed to scan 3d objects where computer is main control unit. 

System contains multiple modules which are responsible for creating cloud points based on readings from laser proximity sensor mounted on Fanuc M10ia robot.
System is capable of manipulating said point clouds:
- Denoising them
- Finding sharp edges
- Proposing points which are suspected to be scanned poorly

GUI and all algorithms responsible for calculations (point clouds manipulation, estimating robot path and comunication with it, files handling) were implemented in Matlab.
Communication from robot perspective as well as its part of steering logic was implemented in Fanuc Karel language. Source code can be seen in [this file](karel_kod_zrodlowy.txt)

Code is in poor shape as I didn't plan to publish it at a time. It also had exclusive purpose to be functional - implementation of individual algorithms and testing it in laboratory environment was my main target.
I don't have access to robot and proximity sensor anymore, so any changes that may influence their behaviour is highly discouraged.
