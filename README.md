# CAD files and code for kinematic, dynamic modeling, and path tracking
![Abstract Gráfic - Journal Article - IEEE](https://github.com/cparedes23/PaperID_8272/assets/134640332/43448aa1-b740-43a0-989f-4d2693ab98f8)

Additional resources and supporting documentation for paper **ID 8272** titled " **Mechatronics Design and Robotic Simulation of Serial Manipulators to Perform Automation Tasks in the Avocado Industry**" are available. CAD files for the Epson Scara T6 robot and the Universal UR10 robot have been created using SolidWorks. The programming codes for kinematics, dynamics, and trajectory tracking calculations have been developed in Matlab. In addition, the actual simulation is carried out in CoppeliaSim, following the following organization: `Matlab -> CoppeliaSim`. The project hierarchy and a description of the corresponding folders are shown below:

**1. Morphological Selection and Requirements:** The manufacturer's datasheet of the Scara T6 and UR10 robots is provided, whose characteristics have been used as a reference for the CAD drawings and the corresponding programming files.

**2. Solidworks Mechanical Modeling:** Precise information on the lengths between each joint was extracted from the CAD files made in SolidWorks to perform the kinematics calculation. In addition, the physical properties of the robots were used in the dynamics calculation. For trajectory tracking, the SolidWorks Motion configuration was used.

i. `Epson SCARA T6 -> Robot ScaraT6`. Scara T6 robot assembly is presented.

ii. `Universal UR10 Robot -> UR10`. UR10 robot assembly is presented.

**3. Matlab Simulation:** 
- **Epson SCARA T6 robot**
- 
`Scara T6 kinematics -> ejm2`. Scara T6 robot animation software test program

`Scara T6 dynamics -> ejm`. Scara T6 robot dynamic test program
