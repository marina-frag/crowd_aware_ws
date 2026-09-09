

1. Προσθέσαμε simulated 2D lidar

2. Βλέπουμε τα τόξα που συνεχίζουν σε ευθύγραμμα τμήματα.

3. Έλεγχος κίνησης με teleop

Αυτή τη στιγμή έχουμε δύο παράλληλες λειτουργίες:

| Λειτουργία         | Ροή                                                                               |
| ------------------ | --------------------------------------------------------------------------------- |
| Χειροκίνητη κίνηση | Teleop → `/cmd_vel` → έτοιμο Gazebo planar-move plugin → κίνηση και `/odom`       |
| Υπολογισμός DWAL   | Lidar → `/scan` → Nav2 local costmap → DWAL, μαζί με `/odom` → paths και clusters |

**Το lidar δεν διαβάζεται απευθείας από τον DWAL.** Το χρησιμοποιεί το έτοιμο obstacle layer του Nav2 για να ενημερώνει τον χάρτη εμποδίων. Ο DWAL διαβάζει αυτόν τον χάρτη και ελέγχει τις διαδρομές με το footprint του i-WALK.

Επαναχρησιμοποιήσαμε τα υπάρχοντα Gazebo plugins, το Nav2 costmap και τα δύο nodes του DWAL. **Δεν γράψαμε νέο planner, νέο lidar driver ή νέο σύστημα σχεδίασης τόξων.** Τα αρχεία που πρόσθεσα συνδέουν και ρυθμίζουν αυτά τα κομμάτια.



| Display στο RViz | Υπάρχον topic                        |
| ---------------- | ------------------------------------ |
| `DWAL paths`     | `/dwal_planner/sampled_pathMarkers`  |
| `Near clusters`  | `/dwal_planner/cluster_markers_near` |
| `Far clusters`   | `/dwal_planner/cluster_markers_far`  |

## Tutorial

```bash
cd ~/crowd_aware_ws
bash scripts/run_dwal_cafe.sh build && bash scripts/run_dwal_cafe.sh run
```


Για χειροκίνητη κίνηση:

```bash
bash scripts/run_dwal_cafe.sh teleop

```
