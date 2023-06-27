# CAD files and code for kinematic, dynamic modeling, and path tracking
![Abstract Gráfic - Journal Article - IEEE](https://github.com/cparedes23/PaperID_8272/assets/134640332/43448aa1-b740-43a0-989f-4d2693ab98f8)

Additional resources and supporting documentation for paper **ID 8272** titled " **Mechatronics Design and Robotic Simulation of Serial Manipulators to Perform Automation Tasks in the Avocado Industry**" are available. CAD files for the Epson Scara T6 robot and the Universal UR10 robot have been created using SolidWorks. The programming codes for kinematics, dynamics, and trajectory tracking calculations have been developed in Matlab. In addition, the actual simulation is carried out in CoppeliaSim, following the following organization: `Matlab -> CoppeliaSim`. The project hierarchy and a description of the corresponding folders are shown below:

**1. Morphological Selection and Requirements:** The manufacturer's datasheet of the Scara T6 and UR10 robots is provided, whose characteristics have been used as a reference for the CAD drawings and the corresponding programming files.

**2. Solidworks Mechanical Modeling:** Precise information on the lengths between each joint was extracted from the CAD files made in SolidWorks to perform the kinematics calculation. In addition, the physical properties of the robots were used in the dynamics calculation. For trajectory tracking, the SolidWorks Motion configuration was used.

i. `Epson SCARA T6 -> Robot ScaraT6`. Scara T6 robot assembly is presented.

ii. `Universal UR10 Robot -> UR10`. UR10 robot assembly is presented.

**3. Matlab Simulation:** 
- **Epson SCARA T6 robot**
  
  i. `Scara T6 kinematics -> ejm2`. Scara T6 robot animation software test program

  ii. `Scara T6 dynamics -> ejm`. Scara T6 robot dynamic test program

  iii. `Path Tracking Simulation in Matlab Scara T6 -> ejm3`. Scara T6 robot Path Tracking Simulation

- **Universal UR10 robot**
  
  i. `UR10 kinematics -> ejm2`. UR10 robot animation software test program
  
  ii. `UR10 dynamics -> ejm`. UR10 robot dynamic test program
  
  iii. `Path Tracking Simulation in Matlab UR10 -> ejemplo3`. UR10 robot Path Tracking Simulation

**4. Waypoint Tracking**
- **Waypoint Tracking ScaraT6**
  
  `urdf -> Waypoint_Tracking_ScaraT6`. Scara T6 waypoint tracking calculated screenshot

- **Waypoint Tracking UR10**
  
  `urdf -> Waypoint_Tracking_UR10`. UR10 waypoint tracking calculated screenshot

**5. CoppeliaSim simulation**

i. `Avocado Robotic Cell`.  Avocado pick and place and palletizing cell simulation environment

ii. `Robotic cell code in Matlab -> Main`. Avocado pick and place and palletizing cell simulation program

# Requirements
- Matlab R2019a or a later version is required. All necessary additional packages have been uploaded to this repository.
	- [Robotics System Toolbox](https://es.mathworks.com/products/robotics.html) provides a library of robotics algorithms and tools for designing, simulating, and testing robotics application

- Solidworks 2022 SP0 or a later version is required. All necessary additional packages have been uploaded to this repository.
	- [SolidWorks to URDF Exporter](http://wiki.ros.org/sw_urdf_exporter) is a SolidWorks add-in that allows for the convenient export of SW Parts and Assemblies into a URDF file.

- CoppeliaSim v4.5.1 or a later version is required. All necessary files have been uploaded to this repository.

# Screenshots
  
![Scara T6 Path Tracking Gif](https://github.com/cparedes23/PaperID_8272/assets/134640332/ae13d81d-6951-49fe-8194-bd8a573fb29a)

