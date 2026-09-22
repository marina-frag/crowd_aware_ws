
#### nav2 concepts



| Data | Publisher | Consumer |
|---|---|---|
| `/clock` | Gazebo ROS | Simulation nodes |
| `/cmd_vel` | Optional existing teleop | Gazebo planar plugin |
| `/odom` | Gazebo planar plugin | DWAL |
| `/scan` | Gazebo GPU-ray ROS plugin | Nav2 obstacle layer |
| `/local_costmap/costmap` | Nav2 standalone costmap | DWAL |
| `/dwal_planner/sampled_paths` | DWAL generator | DWAL clustering |
| `/dwal_planner/clusters_near`, `clusters_far` | DWAL clustering | Future selector |
| DWAL marker topics | DWAL nodes | RViz |


nav2 costmap

DWAL
dwal has 2 nodes
`dwal_generator`
`dwal_clustering`
3 topics
`/dwal_planner/sampled_pathMarkers`

`/dwal_planner/cluster_markers_near`
`/dwal_planner/cluster_markers_far`

Υπάρχουν αλλα dwal topics?? ισως που να μην χρησιμοποιώ ακόμη

no servers
no actions


Plugins
nav2_costmap_2d::ObstacleLayer
++++

Υπάρχει lifecycle_manager, bt_navigator, waypoint_follower, map_server, amcl server?, collision_manager??, sensor_nodes???
δεν χρησιμοπιούμε global planner ούτε local planner χρησιμοποιούμε keyboard teleop
-> /cmd_vel
αρα δεν χρησιμοποιούμε (global) planner server του nav 2
ούτε controller server του nav2  
χρησιμοποιούμε το dwal ως plugin?? στον local  planner-controller server
υπάρχει smoother server?
route server δεν υπάρχει και δνε χρειάζεται
behavior server???

map->odom δεν γινεται από amcl?
odom->baselink δεν γίνεται από odometry?
baselink to lidarlink από  urdf
baselink to motor link από urdf??



#### Lidar
Εξήγησέ μου πως σεταριστηκε το lidar εδώ και οδηγιες ώστε αφού μάθω που πραγματικά είνια να μπορώ να το μετακινήσω στην σωστή θέση
έχεις σημειώσει ότι το Nav2 obstacle layer μετατρέπει τις μετρήσεις του lidar σε costmap.

#### Footprint
 footprint centered at the rear axle.
#### TF tree

```
odom -> sim_base -> base_link -> original wheels and base_footprint
                -> laser_frame
```

φτιαχτηκε όντως δεντρο?
πως κολλάνε με sim_base?
#### Εμπόδια
αφού δεν έχω βάλει shared controller 
γιατι σταματάει και δεν με αφήνει να ρίξω το ρομπότ πάνω σε εμπόδια?
Μπορώ να το απενεργοποιήσω? νομίζω είναι θέμα φυσικής

#### Docker
Συμφνωείς ότι συνεχίζουμε το developmenet ως έχει έτσι στο τέλος θα βάλουμε ενα docker σωστά??|
Τρέχει κάποιο docker ηδη?? Εξήγησε
νομίζω τρέχιε docker για humble?? αυτο είναι λόγω του coffee simulation???


####  Όλα τα BTs του project
τρέχει BT navigator?? 
Εχει κάτι για συμπεριφορά??
Ελάχιστη συμπεριφορά δεν θα έπρεπε να ειναι να τσεκα´ρει μπαταρία τύπου


import rclpy

from rclpy.node import Node
from sensor_msgs.msg import BatteryState


```python


import rclpy

from rclpy.node import Node
from sensor_msgs.msg import BatteryState



class MissionNode(Node):
    def __init__(self):
        super().__init__("mission_bt")
        
		self.latest_battery_percent = None 
		self.battery_ok = False
		self.last_status = None


        self.root = Sequence([
            BatteryOKCondition(self),
            NavigateAction(self)
        ])

        
        self.subscription = self.create_subscription(
            String,
            '/battery_state',
            self.battery_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.timer = self.create_timer(0.1, self.tick_tree)
        
    def battery_callback(self, msg):
    if not msg.present: 
	    self.get_logger().warn("Η μπαταρία δεν είναι παρούσα!") 
	    self.battery_ok = False
	    return
	    
	    current_percent = msg.percentage
    
	    new_battery_ok = (current_percent > 0.20)
	    

	
	if self.latest_battery_percent is None or self.latest_battery_percent != current_percent: 
		if new_battery_ok: 
			self.get_logger().info(f"Battery OK: {current_percent * 100:.2f}%") 
		else: 
		self.get_logger().warn(f"Battery LOW: {current_percent * 100:.2f}%")
		
		self.latest_battery_percent = current_percent self.battery_ok = new_battery_ok

   
    def tick_tree(self):
    if self.latest_battery_percent is None: 
	    self.get_logger().warn("Δεν έχει ληφθεί ακόμη μήνυμα μπαταρίας!", throttle_duration_sec=2.0)
	    
	    
        status = self.root.tick()
        self.get_logger().info(f"BT status: {status}")
        
        
def main(args=None): 
	rclpy.init(args=args) node = MissionNode() 
	try: 
		rclpy.spin(node) 
	except KeyboardInterrupt: 
		pass 
	finally: 
	node.destroy_node() 
	rclpy.shutdown() 
	
	if __name__ == '__main__': 
		main()
```

Πρέπει να ξέρω για το BMS πράγματα για να το κάνω??


#### Αξονες
navigation εωρήτικά το καλύψαμε
sensors and drivers ( lidar sensor θα μου εξηγήσεεςι πανω αν τρέχει σε node δικό του και motors -> iwalk_hardare είναι ενα node που καλέι σαν pluggin τον diff_driver??)
Task planning & behavior trees υποψιαζομαι ότι ειναι χαζό και δεν έχει
application task specific λειτουργια πάλι υποψιαζομαι ότι ειναι χαζό και δεν έχει


#### EKF vs diff_driver?
τρ χρησιμοποιούμε το 2ο αλλα πότε θα θέλαμε να δοκιμάζαμε το πρώτο??




#### HeadLess gazibo run
Πως το υλοποιώ δεδομένου ότι τρ τρέχω


#### Crowd aware
μην υλοπιήσεις τίποτα εδώ αλλά πες μου εγώ σκέφτομαι να κάνω 3 διαφορετικά πειράματα και να συγκρίνω αποτελέσαμτα
1) server ή node με server  που να κάνει perception- context node
Αυτό θα το χρειαστώ νομίζω και στις 3 περιπτώσεις 
	1) Σκέφοτμαι να φτιάξω έναν  συγκεκριμένα με κάμερα predict intent θα παίρνει όλα τα εμπόδια και θα αναγνωρίζει ποια είναι ανρθωποι ???? ( lidar δεν αρκεί νομίζω)
	2) Θα κάνει 
	   Context mode recognition για τα usecases που θέλουμε να καλύψουμε : πχ αναγνωρίζει DENSE_CROWD
		και θα επιλέγει το σωστό Application policy/BT: πχ επιλέγει αργό profile
			πχ 
			
```python
from enum import Enum, auto


class Context(Enum):
    OPEN_AREA = auto()
    NARROW_CORRIDOR = auto()
    DOORWAY = auto()
    DENSE_CROWD = auto()
    OCCLUSION = auto()


def navigation_parameters(context, robot_max_velocity=1.0):
    match context:
        case Context.OPEN_AREA:
            return {
                "max_velocity": robot_max_velocity,
                "clearance_weight": 0.4,
                "lateral_weight": 0.2,
            }

        case Context.NARROW_CORRIDOR:
            return {
                "max_velocity": 0.50 * robot_max_velocity,
                "clearance_weight": 0.7,
                "lateral_weight": 0.9,
            }

        case Context.DOORWAY:
            return {
                "max_velocity": 0.30 * robot_max_velocity,
                "clearance_weight": 0.9,
                "lateral_weight": 0.9,
            }

        case Context.DENSE_CROWD:
            return {
                "max_velocity": 0.25 * robot_max_velocity,
                "clearance_weight": 1.0,
                "lateral_weight": 0.4,
            }

        case Context.OCCLUSION:
            return {
                "max_velocity": 0.35 * robot_max_velocity,
                "clearance_weight": 0.9,
                "lateral_weight": 0.5,
            }

        case _:
            raise ValueError(f"Unsupported context: {context}")
```

Στο πλαίσιο του αιτιολογημένου development πρεπει αυτη η λειτουργια να μπορούμε να δοκιμάζουμε με και χωρις και κάθε ένα από τα τρια πειράματά μας 
  

###### Context-Dependent Use Cases

  

| Context | Desired behavior |

|---|---|

| Open area | Maintain the nominal forward distance. |

| Narrow corridor | Align with the corridor and reduce lateral offset. |

| Doorway | Restrict overtaking and wait when passage is unsafe. |

| Junction | Increase emphasis on short-horizon user-intention prediction. |

| Dense crowd | Reduce speed and shorten formation distance within safe bounds. |

| Occlusion zone | Increase caution and rely on short-term target prediction. |


2) τα 3 controller scenario developements
	1)  DWB is the Nav2 critic-based evolution of DWA: trajectory generators propose motions and configurable critic plugins score them.
	2) HATEB
	3) Γεωμετρικά αυστηρά ορισμένη εναλλακτική για την παραγωγή τόξων του dwal 
	   εδώ σκε´φοτμαι δυναμικό R αντι για cluster kinhshw 0->R σταθερό να φτιαξουμε μια εξισωσούλα για ενα R_dynamic με βαση context mode διαστάσεων του robot και max_robot_vel για να 

Συνολικά νομίζω πρέπει να δοκιμάσουμε 
vanilla dwal με crowd_perception 
vanilla dwal χωρίς crowd_perception 
dwb με crowd_perception 
dwb χωρίς crowd_perception
HATEB με crowd_perception
HATEB χωρίς crowd_prediction
dwal with dynamic R με crowd_perception 
dwal with dynamic R χωρις crowd_perception 

Επίσης θέλω να μου  πεις για το kiro_nav και pmb2_hunamv_simulation μου είανι χρησιμα??
να τεστάρω και αυτά?

Θέλω για την παρουσιαση της εβδομαάδας να τα σετάρουμε και να τρέξουμε και χοντρικά να δούμε ποιο τα πάει κλτρ.
Να προτίνουμε μετρικές αλλά να μην σετάρουμε τρ όντως τεστ με αριθμητικά αποτελε´σματα.

Να θυμάσαι πάντα θα δούμε αν κάτι υπάρχιε σε επίσημη υλοποίηση του ROS2 του Nav2 στα repos του εργαστηρίου 
δηλαδή dwal, shared controller,  kiro_nav και pmb2_hunamv_simulation
και μετά κάνουμε δική μας υλοποίηση

Που μπορεί να γίνει πραγματική ερευνητική επέκταση ? Προς το παρόν νομίζω ότι η δουλειά είναι πιο πρακτική και λγτρ ερευνητική συμφωνείς?

