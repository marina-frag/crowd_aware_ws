# Crowd Awareness project
## Τι έκανα
- Διάβασα το dwal paper
- Κοίταξα το dwal repo
- 3 εναλλακτικά σενάρια για το πως να προχωρήσω

## Για συζήτηση

### Περιορισμοί
Υπάρχον dwal: δύο διαφορετικούς περιορισμούς:

1. Οι πεζοί αντιμετωπίζονται σαν στιγμιαία στατικά εμπόδια.
2. Η οικογένεια υποψήφιων τροχιών είναι περιορισμένη: ένα τόξο και μετά ενδεχομένως μία ευθεία.

### Λύσεις
1. Tracking για crowd instance segmentation και
time-parameterized collision checking

    Κρατάς αρχικά τις υπάρχουσες τροχιές, αλλά κάθε σημείο τους αποκτά χρόνο:

    $τ_k={(x_0 ,y_0,t_0),…,(x_N,y_N,t_N)}$.


    Για κάθε tracked pedestrian j, ξεκινάς με απλή πρόβλεψη σταθερής ταχύτητας:

    $ \hat  p_j (t)=p_j (0)+v_j t$


    Στη συνέχεια συγκρίνεις, για κάθε χρονική στιγμή:

    $d_{j,k}(t_i)= ||p_{robot,k}(t_i)-\hat p _j (t_i)||$

2. Πιο περίπλοκες τροχές

    Εναλλακτικές:
    - DWB + social critics: baseline/fallback.
    - Dynamic multi-segment DWAL
    - HATEB/CoHAN: ισχυρό crowd-aware comparison αν δεν βγει κάτι με τα παραπάνω


    #### Dynamic multi-segment DWAL:
    Ας δοκιμάσουμε μια αλλαγή καμπυρότητας.
    Αντί για δυο μέρη σε μια τροχιά ας βάλουμε 3.

    Επιλογές: straight, left, right
    Κανόνες: μας νοιάζει η σειρά και μόνο το straight μπορεί να επαλαληφθεί

    Για left, right, straight: A, B, C αντίστοιχα:

    ABC, ACB, BAC, BCA, CAB, CBA2, ACC, CAC, CCA, BCC, CBC, CCB, CCC

    Συνολικό Άθροισμα: $6 + 3 + 3 + 1 = \mathbf{13}$ ακολουθίες.

    Τι μπορεί να μείνει ίδιο
    - Human Interaction Zone: μπορεί να παραμείνει με τον ίδιο γεωμετρικό ορισμό.
    - Ο μηχανισμός εμφάνισης του Undecidability: μπορεί να μείνει ίδιος, εφόσον υπολογίζεται από την κατάσταση ανθρώπου–ρομπότ και όχι από τη μορφή των υποψήφιων τροχιών.
    - Decay implementation: μπορεί να διατηρηθεί.
    - Hard και soft persistence: μπορούν να διατηρήσουν την ίδια λογική.
    - FSM: μπορούν να παραμείνουν οι ίδιες καταστάσεις και η ίδια βασική δομή μεταβάσεων.



## Ερωτήσεις
1. Πότε θα ξεκινήσουμε την αλλαγή από ROS 1-> ROS 2 για να αρχίσουμε να γράφουμε κώδικα- πριν τον Σεπτεμβρη ώστε τον σεπτεμβρη να κάνουμε μόνο hardware updates?
2. Θα προσθέσουμε ακριβέστερη εκτίμηση της ταχύτητας? (εκτίμηση πλήρους orientation ή velocity του χρήστη.)
3. Θα ο θανασης υποθέτω θα αλλάξει το Undecidability policy.
4. Τι προσομοιώσεις υπάρχουν για το dwal-iwalker?
5. Δεν υπάρχουν collaborative tasks.
Front following is independant ή assistive?
Η ερώτησή μου προκείπτει επειδή στο paper λέει
(a) Independent: Tasks where the robot performs  socially aware navigation and is not tightly bound to any human (e.g., crowd navigation and delivery). The pedestrians are treated as social dynamic obstacles, but no interaction occurs. (b) Assistive: The navigation task where a robot or a  vehicle provides assistance or support to one or more people. Assistance can be provided in several ways, like following or accompanying a person, or taking the shape of transportation services (e.g., pushing a wheelchair or running a shuttle).

-> assistive not independent

6. Shared controller repo πότε θα ανεβάσετε?
7. Θα χρειαστούμε και εμείς carla?







## Παράρτημα
### Απόδειξη ότι Συνολικό Άθροισμα: $6 + 3 + 3 + 1 = \mathbf{13}$ ακολουθίες.



i. Καμία επανάληψη (Όλα διαφορετικά)

Χρησιμοποιούμε ακριβώς από μία φορά τα $A, B$ και $C$.Οι πιθανές διατάξεις είναι $3! = 6$.

Ακολουθίες (6): ABC, ACB, BAC, BCA, CAB, CBA2.


ii. Το C εμφανίζεται ακριβώς 2 φορές

Αφού έχουμε 3 θέσεις και οι 2 καταλαμβάνονται από το $C$, η τρίτη θέση πρέπει να καλυφθεί είτε από το $A$ είτε από το $B$.Αν η τρίτη θέση είναι το $A$ (στοιχεία $C, C, A$):

Μπορούμε να τοποθετήσουμε το $A$ σε 3 διαφορετικές θέσεις. Ο μαθηματικός τύπος είναι $\frac{3!}{2!} = 3$.


Ακολουθίες (3): ACC, CAC, CCA

Αν η τρίτη θέση είναι το $B$ (στοιχεία $C, C, B$):Ομοίως, μπορούμε να τοποθετήσουμε το $B$ σε 3 διαφορετικές θέσεις.

Ακολουθίες (3): BCC, CBC, CCB.



iii. Το C εμφανίζεται 3 φορές


Αφού το $C$ είναι το μόνο που μπορεί να επαναληφθεί, μπορεί να καταλάβει και τις 3 θέσεις της ακολουθίας.Ακολουθία (1): CCC



Συνολικό Άθροισμα: $6 + 3 + 3 + 1 = \mathbf{13}$ ακολουθίες.


---
1 3D lidar μπροστά


1 2D lidar + RGBD camera πίσω





dubins curves


οχι αλλαγή των generated paths
αλλά αλλαγή του clustering
εντελώς local
path bundle
αντι για να φτάσεις μέχρι R
να κάνεις μέχρι να φτάνει κάπου που μπορει να φτάσει σε επόμενα frames

dwal clustering scoring

lidar

laser odemtry and costmap -> 2D
ros compile dwal

maybe social layer in costmap

Rviz not Gazebo
 dummy robot or stl walk differencial robot form  burger robot to stl of iwalker

