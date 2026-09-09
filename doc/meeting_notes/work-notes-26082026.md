


### Περιορισμοί
Υπάρχον dwal: δύο διαφορετικούς περιορισμούς:

1. Οι πεζοί αντιμετωπίζονται σαν στιγμιαία στατικά εμπόδια.
2. Η οικογένεια υποψήφιων τροχιών είναι περιορισμένη: ένα τόξο και μετά ενδεχομένως μία ευθεία.




----------------------
Θέλουμε να φτιάξουμε προσομοίωση του iwalk σε Rviz και σε Gazebo για να δοκιμάσουμε τις επεκτάσεις μας.


iwalk πληροφορίες:
https://www.i-walk.gr/THE-PROJECT
https://robotics.ntua.gr/i-walk/#tab-e2b39d79ab4f592c4c6
iwalk.stl file

iral εργαστήριο NTUA.

Η πρόταση του επιβλέποντά μου είναι επειδή δεν υπάρχει έτοιμο το μοντέλο του να πάρω το stl που μου ανέβασε για το iwalk και να χρησιμοποιήσω το έτοιμο ρομποτ TurtleBot 3 Burger

Από ότι καταλαβαίνω ίσχυει ότι:

TurtleBot 3 Burger
1. compatible with NVIDIA Jetson Nano
2. differencial robot
3. ROS 2 και Nav2 compatible.

Επειδή λοιπόν και για το iwalk ισχύουν τα 1 και 2 (δύο servomotors στους πίσω τροχούς και encoders στους τροχούς) επιθυμούμε με το update να ισχυει το 3 οποτε επιλέγουμε το TurtleBot 3 Burger ως βάση μας.



-------------
Sensors

1 3D lidar μπροστά
Κάμερα?? Δες τι έχει βάλει ο βατίστας

Δεν με αφορούν ->1 2D lidar + RGBD camera πίσω


------------------
Cluster generation:
- Πως να υπολογίσω ένα μεταβλητό R ιδανικό για να απορρίπτω paths πχ ταχύτητα και γεωμετρια-διαστάσεις ρομποτ?
- Αξίζει να:
dubins curves



--------

Σε πρώτο στάδιο πρέπει να κάνω αυτά να τρέξουν για το iwalk simulation

Dwal planner
https://github.com/gmoustri/dwal_planner

Shared controller
https://github.com/gmoustri/bayesian_shared_control


Μετά θα πρέπει να ενσωματώσω:
- GitHub - andvatistas/kiro_nav: Social-Aware Navigation reusable module of the KIRO experiment
- GitHub - andvatistas/pmb2_hunav_simulation

Και ένα από τα παρακάτω:
2D simulators
- CrowdNav
- Extension with static obstacles, SFM human agents, and Stable Baselines 3 support: CrowdSimPlus.
- SocNav1: A Dataset to Benchmark and Learn Social Navigation Conventions, MDPI 2019.
- SocNav2
- SocNavBench: A Grounded Simulation Testing Framework for Evaluating Social Navigation, THRI 2022.
- ir-sim: A python based light weight robot simulator for the algorithm development of robotics navigation, control, and learning.\
3D simulators
- Pedsim-ROS
- pedsim_ros_with_gazebo
- AI Habitat 3.0
- iGibson
- gym_ped_sim
- Social Environment for Autonomous Navigation (SEAN) 2.0
- HuNavSim: A ROS 2 Human Navigation Simulator for Benchmarking Human-Aware Robot Navigation, RAL 2023.
- Arena-Rosnav 3.0: A Comprehensive Development and Benchmarking Platform for Navigation, RSS 2024.
- Arena 4.0: A Comprehensive ROS2 Development and Benchmarking Platform for Human-centric Navigatio…, ICRA 2025.


-------------------

Iwalk simulation creation
| Τμήμα     | Προσαρμογή στο i‑Walk              |
| --------- | ---------------------------------- |
| Visual    | STL του i‑Walk                     |
| Collision | Απλοποιημένη πραγματική γεωμετρία  |
| Inertial  | Μάζα, κέντρο μάζας, inertia        |
| Wheels    | Πραγματικές ακτίνες και πλάτη      |
| Joints    | Πραγματικές θέσεις και άξονες      |
| Sensors   | Πραγματικές θέσεις LiDAR/cameras   |
| Drive     | Differential drive δύο πίσω τροχών |
-------------
Χρησιμοποιούμε το αρχείο $check\_stl.py$

το οποίο τρέχουμε με την εντολή:
```python
python3   ~/crowd_aware_ws/src/iwalk_description/   ~/crowd_aware_ws/src/iwalk_description/meshes/iwalk_body.STL
```
Και μας δίνει τις εξής πληροφορίες
```bash
marina-fragkouli@marina-fragkouli-Cyborg-15-A13VF:~/crowd_aware_ws/src/iwalk_description$ python3   ~/crowd_aware_ws/src/iwalk_description/check_stl.py   ~/crowd_aware_ws/src/iwalk_description/meshes/iwalk_body.STL
File:       /home/marina-fragkouli/crowd_aware_ws/src/iwalk_description/meshes/iwalk_body.STL
STL type:   binary
Vertices:   14880

Minimum:    x=0.000000, y=0.000000, z=0.114396
Maximum:    x=653.000000, y=784.734741, z=1046.608887
Dimensions: x=653.000000, y=784.734741, z=1046.494491
Box center: x=326.500000, y=392.367371, z=523.361641
marina-fragkouli@marina-fragkouli-Cyborg-15-A13VF:~/crowd_aware_ws/src/iwalk_description$
```

---------------
Έλεγχος μετατροπής Xacro → URDF
```bash
xacro \
  src/iwalk_description/urdf/iwalk.urdf.xacro \
  > /tmp/iwalk.urdf
```
και
```bash
check_urdf /tmp/iwalk.urdf
```

Κάνε build
```bash
cd ~/crowd_aware_ws
```
```bash
colcon build \
  --symlink-install \
  --packages-select iwalk_description
```
```bash
source ~/crowd_aware_ws/install/setup.bash
```
Εκκίνησε το rviz:
```bash
ros2 launch iwalk_description display.launch.py
```
------
## FOR MEETING
Επίσης
-> η μάζα του κύριου σώματος <xacro:property name="body_mass" value="25.
-> πραγματικές inertias ή καλύτερη προσέγγισή τους,

Fix sensors
Burger

imu_joint
imu_link
scan_joint
Base_scan

Iwalk
Web says
2 Realsense camera RBGD
IMU
Hokuyuo LIDAR sensor
RP LIDARsensor


I will model one camera and 1 3d lidar on front IMU
-> που και ποια front camera and 1 3d lidar on front IMU
—----------------------------------


Πλήρως δυναμικό burger ή δυναμική σε σημείο του burger

contacts σιτς ροδες
dswb
kinodynamic planner
προσεξε αλλαξε μόνο ενα νοδε στον ντουαλ to ψλθστερινγ


1. Crowd simulation + Στατικό εμπόδια
2.  2d lidar αρκεί burger-iwalk με οδομετρία και λειζερ
3.  ντουαλ

Προσοοχη
footprint παράμετρος στο ναβ 2 ο dwal το παινει πρέπει να του το δώσω
Αν εμφανιζτεί θόρυβος φτιαξε φίλτρο να το κόψεις
ros2 run rosback replay


Το δίνω σε Nav2 αντί για καρλα hunav

πρώτα clusterring
μετα social navigation

Πάντα να έχεις στο νου σου τι υπάρχει ήδη από nav 2 και dwal μην δημιουργήσουμε nodes και topics που ήδη υπάρχουν

Αυξάνουμε το ελάχιστο cluster span

προσοχή στον συγχρονισμό

το dwal το τρέχει ςμε launch







-------------------------

NEW


https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html?utm_source=chatgpt.com

Ένα rosbag καταγράφει timestamped ROS messages από επιλεγμένα topics.


Χρήσιμα topics για το robot σας:

/cmd_vel: ζητούμενα \(v,\omega\).
/wheel_velocities: commands προς τους τροχούς.
/joint_states: wheel positions/velocities.
/odom: encoder odometry.
/scan: lidar.
Camera topics.

Μετά:

ros2 bag info <bag_directory>
ros2 bag play <bag_directory>


--------


## Ερωτήσεις


Εξηγησε αυτό

Ο incremental encoder παράγει παλμούς καθώς περιστρέφεται ο άξονας. Κάθε παλμός/edge αντιμετωπίζεται σαν ένα tick.

Αν ο συνολικός αριθμός είναι \(N\) counts ανά περιστροφή:

$$ \Delta\theta = 2\pi\frac{\Delta ticks}{N} $$

Η γωνιακή ταχύτητα είναι:

$$ \omega= \frac{\Delta\theta}{\Delta t} $$

Η απόσταση που διένυσε ο τροχός είναι:

$$ \Delta s=r\Delta\theta $$

“Integrating the encoder ticks” σημαίνει ότι προσθέτουμε διαδοχικά τις μικρές μεταβολές:

$$ ticks_{\text{total}} = ticks_{\text{total}}+\Delta ticks $$

Έτσι βρίσκουμε τη συνολική περιστροφή και την απόσταση, όχι μόνο την στιγμιαία ταχύτητα.

Για differential drive:

$$ \Delta s= \frac{\Delta s_R+\Delta s_L}{2} $$ $$ \Delta\theta_{\text{robot}} = \frac{\Delta s_R-\Delta s_L}{L} $$

Από αυτά υπολογίζεται το wheel odometry /odom.






-----------------

4. Γιατί χρησιμοποιούμε FOC

Για assistive mobile robot πιθανότατα θέλεις FOC — Field-Oriented Control.

Σε σύγκριση με απλό BLDC commutation προσφέρει συνήθως:

πιο ομαλή κίνηση,
λιγότερο θόρυβο,
καλύτερο έλεγχο ροπής,
καλύτερη συμπεριφορά σε μικρές ταχύτητες,
ομαλότερο ξεκίνημα και φρενάρισμα.



$I_{max} =min(I_{VESC} ,I_{motor},I_{battery},I_{BMS}, I_{wiring})$


----
120 Ohms in canbus

---
Τα παρακάτω είναι είδη encoders
