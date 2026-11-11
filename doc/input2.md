# Πρωτόκολλο πειραμάτων (input2)

Καθαρή και αυστηρή εκδοχή του `input1.md` για τα **πειράματα**. Το `input1.md` μένει ως τεκμηρίωση των bugs και του κώδικα. Ό,τι γράφεται εδώ αντιστοιχεί σε κώδικα που υπάρχει. Κάθε μετρική έχει το όνομά της στο `metrics.json` και το αρχείο όπου υπολογίζεται. Οι αποφάσεις του §6 είναι **οριστικές** (απαντήσεις 2026-11-10).

Σήμανση: ✅ υλοποιημένο και ελεγμένο σε run, 🟡 υλοποιημένο, δεν έχει τρέξει ακόμα το πλήρες βήμα.

---

## 1. Τι συγκρίνουμε

| Μέθοδος | Clustering | Αντιστοίχιση IDs | Ρύθμιση runtime | Πού τρέχει |
|---|---|---|---|---|
| **fixed** (baseline) | σταθερά levels near/far, κώδικας upstream (με τα bugs B1, B2) | upstream (greedy) | `fixed_dwal` | όλα τα βήματα |
| **dynamic tracking** | μεταβλητό r + split με άλμα μήκους τ | tracking: IoU, gate 0.2, σειρά διατηρείται, IDs δεν ξαναχρησιμοποιούνται | `dynamic_dwal`, `CORRESPONDENCE=tracking`, `JUMP=τ` | όλα τα βήματα |
| dynamic upstream | μεταβλητό r + split με άλμα μήκους τ | upstream, με διόρθωση B2 | `dynamic_dwal`, `CORRESPONDENCE=upstream`, `JUMP=τ` | μόνο SCAND (βήμα 4). Στο sim συγκρίνεται **offline** (H3) |

Τρεις υποθέσεις, μία για κάθε μηχανισμό:
- **H1 (μεταβλητό r):** ο dynamic κόβει λιγότερα ασφαλή paths, άρα ακινητοποιείται λιγότερο, χωρίς χειρότερη ασφάλεια.
- **H2 (split με τ):** το split χωρίζει διαφορετικές επιλογές (λιγότερα mixed clusters) χωρίς να τις θρυμματίζει (λίγο over-split).
- **H3 (tracking):** η αντιστοίχιση tracking κρατά πιο σταθερά IDs από την upstream.

---

## 2. Σενάρια, πυκνότητα, διαδρομή, seed

### 2.1 Επιλογές runtime ✅

| Μεταβλητή | Τιμές | Default | Τι αλλάζει |
|---|---|---|---|
| `SCENARIO` | 9 σενάρια στο `config/scenarios/` | `dense_crowd` | κόσμος και agents |
| `DENSITY` | `sparse`, `medium`, `dense` | `medium` | πλήθος agents (μόνο dense_crowd, dense_static) |
| `PATH_SHAPE` | `line`, `s_curve`, `hard_curve` | `line` | επιθυμητή διαδρομή (dense_crowd, dense_static, open_area) |
| `JUMP` | θετικός αριθμός [m] | από το `experiment.yaml` (0.3) | `length_jump_threshold` του dynamic |
| `CORRESPONDENCE` | `upstream`, `tracking` | από το `experiment.yaml` | αντιστοίχιση IDs του dynamic |
| `REFERENCE_MODE` | `autonomous`, `teleop` | `teleop` | ποιος δίνει την «εντολή χρήστη» |

Παράδειγμα (ένα run με RViz):
```bash
SCENARIO=dense_static DENSITY=dense PATH_SHAPE=hard_curve REFERENCE_MODE=autonomous RVIZ=true CORRESPONDENCE=tracking JUMP=0.3 bash ~/crowd_aware_ws/scripts/run_dwal_cafe.sh run dynamic_dwal off 2
```

Όλες οι επιλογές γράφονται στο `evaluation.json` κάθε run και στο όνομα του bag, π.χ. `dense_static_dense_hard_curve_dynamic_split_tracking_j0.3_off_seed2_<ώρα>`. Το `experiment.yaml` δεν αλλάζει κατά τα sim πειράματα (μόνο στο SCAND, βήμα 4).

### 2.2 Διαδρομές ✅

Ο autonomous reference (Regulated Pure Pursuit) **ακολουθεί** τη διαδρομή με ανάδραση από το odom. Δεν πηγαίνει απλώς ευθεία. Όλες οι διαδρομές ξεκινούν από (0,0) με μηδενική κλίση και έχουν μήκος 8 m κατά x (`config/scenarios/*.yaml: robot.path_shapes`, `prepare_scene.py: build_path`).

| Διαδρομή | Σχήμα | Μήκος | Ελάχιστη ακτίνα |
|---|---|---|---|
| `line` | ευθεία | 8.0 m | ∞ |
| `s_curve` (εύκολη) | ομαλή αλλαγή λωρίδας 1.5 m, μηδενική κλίση στα δύο άκρα | 8.2 m | 7.1 m |
| `hard_curve` (δύσκολη) | slalom: ημίτονο πλάτους 0.4 m, μήκος κύματος 5 m, ομαλή εκκίνηση 3 m | 8.6 m | 1.5 m (διαδοχικές στροφές ±27°) |

Το πλάτος του hard_curve είναι 0.4 m (όχι 0.5), ώστε η ακτίνα να μένει πάνω από την πλευρική απόσταση των εμποδίων (1.3 m + 0.15 m μετατόπιση seed). Αλλιώς ένα εμπόδιο στο εσωτερικό στροφής θα έπεφτε πάνω στη διαδρομή.

### 2.3 Τοποθέτηση agents ✅

Οι agents τοποθετούνται **σε σχέση με τη διαδρομή** (`prepare_scene.py: layout_agents`):
- **dense_static**: στατικοί άνθρωποι ανά `spacing_m` κατά μήκος της διαδρομής, εναλλάξ αριστερά/δεξιά, σε πλευρική απόσταση **1.0 m στην ευθεία, 1.3 m στις καμπύλες**. Ελέγχεται ότι κανένα δεν είναι πιο κοντά από (απόσταση − 0.05 m) σε οποιοδήποτε σημείο της διαδρομής. Στο εσωτερικό στροφής, αν δύο απέχουν < 0.9 m, το εμπόδιο μπαίνει στην άλλη πλευρά, ή παραλείπεται.
- **dense_crowd**: πεζοί που διασχίζουν τη διαδρομή ανά `crossing_spacing_m`, όλοι παράλληλα (κάθετα στη συνολική κατεύθυνση), ±2.5 m. Επιπλέον πεζοί κινούνται κατά μήκος της, σε πλευρική απόσταση +0.7 m (αντίθετα) και −0.8 m (ομόρροπα).
- **open_area**: χωρίς εμπόδια κοντά στη διαδρομή. Ένας πεζός περπατά στα 4.5 m μακριά.

| | sparse | medium | dense |
|---|---|---|---|
| dense_static: απόσταση εμποδίων | 1.8 m | 0.9 m | 0.6 m |
| dense_static: πλήθος (line / s_curve / hard_curve) | 4 / 4 / 4 | 8 / 8 / 8 | 10–11 / 9–11 / 8–10 |
| dense_crowd: διασχίζοντες ανά | 2.0 m | 1.0 m | 0.5 m |
| dense_crowd: διασχίζοντες + κατά μήκος | 3 + 1 = 4 | 6 + 2 = 8 | 11 + 2 = 13 |

Στο dense το πλήθος αλλάζει λίγο με το seed (εμπόδια που δεν χωρούν παραλείπονται). Όλοι οι συνδυασμοί για το seed 1: [scenario_layouts.png](scenario_layouts.png) (σχεδιασμένο πριν την αλλαγή στα 1.3 m).

### 2.4 Τι αλλάζει με το seed ✅

**Seed 1 = το σενάριο χωρίς μεταβολές** (το medium πάνω στην ευθεία αναπαράγει ακριβώς τα παλιά χειρόγραφα σενάρια). Κάθε άλλο seed (`agent_layout.seed_variation`):

| Σενάριο | Τι αλλάζει | Τιμές |
|---|---|---|
| dense_static | **μόνο η θέση των εμποδίων** | ±0.2 m κατά μήκος, ±0.15 m πλευρικά (ομοιόμορφα) |
| dense_crowd | **αρχική θέση** κάθε πεζού | τυχαίο σημείο στο πρώτο 80% της διαδρομής του, ≥ 0.75 m από τους άλλους, ≥ 1.5 m από το robot |
| | **τυχαία κίνηση** | δυνάμεις HuNav τυχαίες (configuration 3: goal / obstacle / social force από τα εύρη του `simulation.yaml`), ταχύτητα × U(0.85, 1.15) |
| | στόχοι | ±0.3 m κάθετα στη διεύθυνση βαδίσματος (`goal_variation`, όπως πριν) |
| cafe, crossing, doorway, junction, narrow_corridor, open_area (χειρόγραφα) | πεζοί που κινούνται: **αρχική θέση** + **τυχαία κίνηση** | τυχαίο σημείο στο πρώτο 80% του πρώτου σκέλους της διαδρομής τους, ≥ 1.5 m από το robot· δυνάμεις HuNav τυχαίες (configuration 3), ταχύτητα × U(0.85, 1.15) |
| group_blocking (σχεδόν ακίνητη ομάδα) | **θέση της ομάδας** (όλα τα μέλη μαζί) | ±0.3 m κατά x, ±0.2 m κατά y |

Τα χειρόγραφα σενάρια διαβάζουν το `seed_variation` στο επίπεδο του σεναρίου (`prepare_scene.py: vary_agents`), τα dense_* στο `agent_layout.seed_variation`. Πριν, στα 6 από τα 7 χειρόγραφα το seed δεν άλλαζε τίποτα, και στο cafe άλλαζαν μόνο οι δυνάμεις 3 agents. Το `prepare_scene` περνά για όλα τα 9 σενάρια με seeds 1–3, και σε όλα το seed 2 διαφέρει από το 1.

Ελέγχθηκε offline για όλους τους συνδυασμούς και seeds 1–50: απόσταση εμποδίων από τη διαδρομή ≥ 0.85 m (ευθεία) και ≥ 1.15 m (καμπύλες), πεζοί ≥ 0.75 m μεταξύ τους στην εκκίνηση.

### 2.5 Καταγραφή ✅

- Στα πειράματα ο autonomous reference **περιμένει να συνδεθεί ο recorder** πριν στείλει τη διαδρομή (`WAIT_FOR_RECORDER=true`). Πριν, η καταγραφή ξεκινούσε όταν το robot είχε ήδη διανύσει ~2 m. Τα runs της 2026-11-04 δεν έχουν την αρχή της διαδρομής.
- Καταγράφονται η επιθυμητή (`/experiment/path`) και η εκτελεσμένη διαδρομή (`/experiment/actual_path`). Στο RViz φαίνονται ως κυανή και πορτοκαλί γραμμή, ζωντανά (`VIEW=1`) και στο replay (`scripts/replay_rviz.sh`, που τις φτιάχνει μόνο του για τα παλιά bags).
- **Συμπίεση (2026-11-10).** Τα bags γράφονται με zstd ανά μήνυμα (`experiment.py`), χωρίς απώλειες: ~50 MB ανά run αντί για ~160 MB. Τα μισά bytes ήταν markers του RViz. Η συμπίεση ανά αρχείο θα έδινε 13 MB, αλλά στο Humble κάθε ανάγνωση αποσυμπιέζει ένα πλήρες αντίγραφο δίπλα στο bag και το αφήνει εκεί. Η αξιολόγηση (`dwal_bag.py`) και το replay διαβάζουν και τα παλιά, ασυμπίεστα bags. Χώρος: βήμα 2 ≈ 9 GB, βήμα 3 ≈ 18 GB, βήμα 6 ≈ 45 GB.
- **Αποτυχία εκκίνησης → επανάληψη.** Σπάνια το DDS χάνει μια απάντηση lifecycle στην εκκίνηση. Τότε ο `controller_server` του reference δεν ενεργοποιείται, κάθε στόχος απορρίπτεται και το robot δεν κινείται καθόλου. Το είδαμε μία φορά σε ~25 runs. Δεν είναι αποτέλεσμα του controller, οπότε το `run_experiments.sh` το εντοπίζει (> 25 «Reference goal rejected» στο log ή καθόλου bag), μετακινεί το run στο `rosbags_failed/` και το ξανατρέχει, έως 3 φορές. Κάθε αποτυχία γράφεται στο `experiment_logs/failed_runs.log` και αναφέρεται στο paper ως πλήθος αποτυχιών υποδομής.
- **Έλεγχοι εγκυρότητας** (μεταφέρθηκαν από τον παλιό `run_dwal_batch.py`, που σβήστηκε στις 2026-11-10):
  - **Ζεύγη.** Το `experiment.py` γράφει στο `evaluation.json` το `agents_sha256`, το hash των agents του run. Το `metrics_summary.py` ελέγχει δύο πράγματα στην ενότητα «Έλεγχος ζευγών»: ότι κάθε ζεύγος, δηλαδή ίδιο σενάριο, πυκνότητα, διαδρομή, seed και semantics, έχει τους ίδιους agents σε όλες τις μεθόδους, και ότι διαφορετικά seeds δίνουν διαφορετικούς agents. Πάνω σε αυτό στηρίζεται το paired Wilcoxon.
  - **Recorder.** Ελέγχεται ότι κάθε recorder τερμάτισε κανονικά (`recording_result.json`).
  - **Image.** Το `run_experiments.sh` γράφει το image κάθε batch στο `experiment_logs/batches.log` και σταματά αν το image αλλάξει στη μέση. Έτσι δεν ανακατεύονται εκδόσεις κώδικα.

---

## 3. Βήματα

Χρόνος ανά simulation run ≈ 2.6 min (110 s καταγραφή + εκκίνηση/τερματισμός). Με `VIEW=1` κάθε run ανοίγει RViz.

### Βήμα 1 — Απόκλιση διαδρομής (autonomous, χωρίς εμπόδια) ✅

**Ερώτηση:** ακολουθεί το robot την επιθυμητή διαδρομή, όταν δεν υπάρχει λόγος να αποκλίνει; Ελέγχει τον reference και την αλυσίδα εντολών πριν από όλα τα υπόλοιπα.

**Πρώτο τεστ (2026-11-10, seed 1):** και τα 6 runs του open_area (3 διαδρομές × fixed / tracking) περνούν το κριτήριο: 0 συγκρούσεις, στόχος σε 27–31 s, cross-track p95 0.001 m (line), 0.017 m (s_curve), 0.087 m (hard_curve). Με τα εμπόδια στα 1.3 m περνούν πια και οι καμπύλες με εμπόδια: dense_static s_curve και hard_curve, dense_crowd s_curve (seed 2) φτάνουν στον στόχο (98%), ενώ με 1.0 m το s_curve σταματούσε στο 7%.

**Κλείσιμο (2026-11-10).** Το βήμα 1 είναι έλεγχος της αλυσίδας (reference → controller → robot), όχι σύγκριση μεθόδων. Περνά με μεγάλο περιθώριο: p95 ≤ 0.087 m έναντι ορίου 0.30 m. Το επιβεβαίωσε και οπτικός έλεγχος σε RViz (open_area, hard_curve, seed 2). Τα 6 runs του seed 1 αρκούν ως έλεγχος. Η απόκλιση μετριέται ούτως ή άλλως σε κάθε autonomous run των βημάτων 2, 3 και 6, άρα η στατιστική της προκύπτει από εκεί. Τα πλήρη 18 runs (`deviation`) τρέχουν μόνο αν χρειαστεί για το paper πίνακας του βήματος 1 από την τελική έκδοση του κώδικα.

**Ο reference δεν εγκαταλείπει ποτέ (απόφαση Ι).** Παλιά, όταν ο DWAL δεν έβρισκε δρόμο, ο reference (Regulated Pure Pursuit) εγκατέλειπε τον στόχο και έστελνε 0. Συνέβαινε για δύο λόγους: το δικό του collision check («detected collision ahead») και το progress checker («Failed to make progress» μετά από 8 s). Το `standstill_while_requested_s` μετρά ακινησία **μόνο όσο ζητείται κίνηση**, οπότε ένα robot που κόλλησε μετρούσε σχεδόν 0 s: το run με εμπόδια στα 1.0 m σταμάτησε στο 7% και μέτρησε 0.2 s.

Τώρα (`config/controllers/reference.yaml`): `use_collision_detection: false`, `movement_time_allowance` 10⁶ s, `failure_tolerance: −1`. Ο reference αναπαριστά τον **χρήστη**, που συνεχίζει να ζητά κίνηση όσο το robot είναι κολλημένο, όπως θα γινόταν και στην πραγματική εφαρμογή, όπου δεν υπάρχει reference path. Την αποφυγή εμποδίων την κάνουν μόνο ο DWAL και ο shared controller. Έτσι το `standstill_while_requested_s` μένει η κύρια μετρική του H1, και μετριέται με τον ίδιο τρόπο σε teleop, autonomous και πραγματικά πειράματα.

|                       |                                                                                                                                                                                                  |
| --------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Runs                  | **open_area**, 3 διαδρομές × {fixed, tracking} × seeds 1–3 = **18 runs ≈ 47 min**                                                                                                                |
| Εντολή                | `bash ~/crowd_aware_ws/scripts/run_experiments.sh deviation`                                                                                                                                     |
| Μετρικές              | `python3 ~/crowd_aware_ws/scripts/metrics_summary.py --metrics path_deviation --evaluate` (μόνο αυτή, γρήγορα)                                                                                   |
| Έλεγχος με το μάτι    | `<run>/analysis_v3/path_deviation.png` (επιθυμητή έναντι εκτελεσμένης) και RViz                                                                                                                  |
| **Κριτήριο αποδοχής** | σε **κάθε** run: στόχος εντός `goal_tolerance` (0.2 m), 0 συγκρούσεις, cross-track p95 ≤ 0.30 m (περίπου το μισό πλάτος του iWalk, 0.33 m). Αν αποτύχει, διορθώνεται ο reference πριν το βήμα 2. |

Μετρικές (`path_deviation.py`, κλειδί `path_deviation` στο `metrics.json`, ή `path_deviation.json` με `--metrics path_deviation`):
- **cross-track** (`cross_track_mean_m`, `_rms_m`, `_p95_m`, `_max_m`): απόσταση της εκτελεσμένης τροχιάς από την επιθυμητή. Η τροχιά δειγματίζεται ανά 5 cm διανυθείσας απόστασης, ώστε η ακινησία να μη μετρά.
- **ADE / FDE** (`ade_m`, `fde_m`): μέση / τελική απόσταση **σε ίσο μήκος τόξου** από την πρώτη πόζα. **Ίδιος ορισμός με το ADE/FDE του SCAND**, όπου τη θέση της επιθυμητής διαδρομής παίρνει η διαδρομή του οδηγού.
- **πρόοδος** (`progress_fraction`), **στόχος** (`goal_reached`, `time_to_goal_s`).

Το ίδιο μπλοκ υπολογίζεται και μέσα στο πλήρες `metrics.json` κάθε autonomous run, άρα και στα βήματα 2–3. Για teleop και SCAND γράφεται `available: false` με την αιτία.

**Γιατί χωρίς εμπόδια.** Στο πρώτο τεστ (dense_static, εμπόδια στα 1.0 m) ο fixed κόλλησε στις καμπύλες: 7% πρόοδος στο s_curve, 35% στο hard_curve, ενώ στην ευθεία έφτασε στον στόχο ακόμα και με dense. Ο reference ζητούσε συνεχώς 0.3 m/s, αλλά ο DWAL δεν έβρισκε ελεύθερο τόξο: τα arc-line paths έχουν σταθερή καμπυλότητα ως τα 1.2–1.8 m και δεν χωρούν σε καμπύλο διάδρομο. Με εμπόδια, το βήμα 1 θα μετρούσε τον DWAL και όχι τον reference. Γι' αυτό τρέχει στο open_area, και στο βήμα 3 τα εμπόδια μπαίνουν στα 1.3 m στις καμπύλες.

### Βήμα 2 — Επιλογή τ (μόνο dynamic tracking) 🟡

**Ερώτηση:** ποιο κατώφλι άλματος μήκους τ χωρίζει καλύτερα τις επιλογές;

| | |
|---|---|
| Runs | τ ∈ {0.1, 0.2, 0.3, 0.5, 0.7, 1.0} × {dense_crowd, dense_static} × 3 πυκνότητες × line × seeds **101–105** = **180 runs ≈ 7.8 h** |
| Εντολή | `bash ~/crowd_aware_ws/scripts/run_experiments.sh jump` |
| Ανάλυση | `python3 ~/crowd_aware_ws/scripts/metrics_summary.py --evaluate` → πίνακας «Επιλογή jump» |
| Χώρος | ~9 GB (≈ 50 MB/run, συμπιεσμένα) |
| Πιλοτικό (2026-11-10) | dense_crowd / dense / τ = 0.3 / seed 999 (εκτός πρωτοκόλλου): 0 συγκρούσεις, στόχος σε 33 s, ακινησία 1.7 s, mixed 27%, over-split 6%, ID switches 0/min. Η αλυσίδα του βήματος 2 δουλεύει από άκρη σε άκρη: συμπιεσμένο bag → αξιολόγηση → replay. |

**Κύρια μετρική** (`metrics_summary.py: jump_selection`):

$$\mathrm{Loss}(\tau)=\frac{U(\tau)}{\max_\tau U}+\frac{O(\tau)}{\max_\tau O}+\frac{S(\tau)}{\max_\tau S}\qquad(\lambda_{under}=\lambda_{over}=\lambda_{switch}=1)$$

| Όρος | Μετρική στον κώδικα | Τι μετρά |
|---|---|---|
| U = under_split | `offline.clustering.far.mixed_cluster_fraction` | % clusters με paths που καταλήγουν σε ≥ 2 διαφορετικά εμπόδια |
| O = over_split | `offline.clustering.far.oversplit_run_fraction` | % συνεχόμενων paths με τον ίδιο στόχο που κόβονται σε ≥ 2 clusters |
| **S** = cluster_instability | `offline.clustering.far.ids_recorded.id_switches_per_min` | **ID switches ανά λεπτό**: πόσες φορές ένα cluster που είναι ουσιαστικά το ίδιο πέρασμα με το προηγούμενο frame (γωνιακή επικάλυψη IoU ≥ 0.5) εμφανίζεται με **άλλο ID** |

Κανόνας επιλογής, με τη σειρά:
1. U, O, S = μέσος όρος όλων των runs με αυτό το τ. Κάθε όρος διαιρείται με τη μέγιστη τιμή του στα τ, ώστε και οι τρεις να είναι στο [0, 1].
2. **Αποδεκτό** τ = όλα τα runs του χωρίς σύγκρουση και με στόχο (το κριτήριο του βήματος 1). Τα μη αποδεκτά τ αποκλείονται, όσο μικρό κι αν είναι το Loss τους.
3. Από τα αποδεκτά επιλέγεται το τ με το **ελάχιστο Loss**. Αν δύο απέχουν < 0.02, νικά αυτό με το μικρότερο S.
- **Τα seeds 101–105 δεν ξαναχρησιμοποιούνται** στο βήμα 3. Αλλιώς το τ θα ρυθμιζόταν πάνω στα δεδομένα αξιολόγησης.

### Βήμα 3 — Σύγκριση fixed έναντι dynamic tracking, simulation 🟡

| | |
|---|---|
| Runs | {fixed, tracking} × {dense_crowd, dense_static} × 3 πυκνότητες × 3 διαδρομές × seeds **1–10** = **360 runs ≈ 15.6 h** |
| Εντολή | `JUMP=<τ βήματος 2> VIEW=1 bash ~/crowd_aware_ws/scripts/run_experiments.sh compare` |
| Ανάλυση | `python3 ~/crowd_aware_ws/scripts/metrics_summary.py --evaluate --markdown summary.md --csv summary.csv` |

**Κύριες μετρικές (δηλωμένες πριν τρέξουμε)**, πίνακας «Κύριες υποθέσεις» (`metrics_summary.py: primary_hypotheses`):

| Υπόθεση | Σύγκριση | Ζεύγη | Κύρια μετρική | Βελτίωση |
|---|---|---|---|---|
| H1 | tracking − fixed | ίδιο (σενάριο, πυκνότητα, διαδρομή, seed): 180 | `standstill_while_requested_s` | αρνητική διαφορά |
| H2 | tracking − fixed | όπως H1: 180 | `offline.clustering.far.mixed_cluster_fraction` | αρνητική, με `oversplit_run_fraction` μικρό |
| H3 | replay tracking − replay upstream | κάθε run tracking, **ίδια clusters**: 180 | `ids_replay_*.id_switches_per_min` | αρνητική, με `duplicate_id_frames` = 0 για το tracking |

Το H3 κρίνεται offline: και οι δύο τρόποι αντιστοίχισης ξανατρέχουν πάνω στα **ίδια** καταγεγραμμένα clusters κάθε run (`offline_metrics.py: UpstreamMatcher`, `TrackingMatcher`, ίδιοι με τη C++). Η σύγκριση είναι πιο ελεγχόμενη από δύο ξεχωριστά runs, γιατί η είσοδος είναι ακριβώς η ίδια. Γι' αυτό το dynamic upstream δεν χρειάζεται δικά του runs στο sim.

Όλες οι υπόλοιπες μετρικές του §4 είναι **δευτερεύουσες**: αναφέρονται στον πίνακα «Στατιστική», αλλά δεν κρίνουν την υπόθεση. Από αυτές, οι μετρικές ασφάλειας (συγκρούσεις, ελάχιστη απόσταση από άνθρωπο και εμπόδιο) λειτουργούν ως **συνθήκη** για το H1.

**Στατιστική:**
- Ανά υπόθεση: **Wilcoxon signed-rank** (ακριβές p), **rank-biserial r** (μέγεθος επίδρασης), **bootstrap 95% CI** της διάμεσης διαφοράς.
- **Holm** στις τρεις κύριες υποθέσεις (οικογένεια 3 τεστ).
- **Κύρια ανάλυση: όλα τα ζεύγη μαζί. Δευτερεύουσα: ανά σενάριο.** Η κύρια ανάλυση δίνει το συμπέρασμα, γιατί έχει τα περισσότερα ζεύγη και δηλώθηκε πρώτη. Η ανάλυση ανά σενάριο (crowd χωριστά από static) ελέγχει αν η συνολική εικόνα κρύβει αντίθετες τάσεις. Για παράδειγμα, ο dynamic μπορεί να βοηθά πολύ στα στατικά εμπόδια και καθόλου στους πεζούς, και το συνολικό αποτέλεσμα να είναι «μέτρια βελτίωση». Η δευτερεύουσα **δεν** αλλάζει το συμπέρασμα της κύριας. Εξηγεί πού οφείλεται και γράφεται ως εύρημα.

**Γιατί 10 seeds αρκούν για ένα πρώτο συμπέρασμα:**
1. Με n ζεύγη, το μικρότερο δυνατό two-sided p του Wilcoxon είναι 2/2ⁿ. Με n = 6 είναι 0.031, που μετά τη διόρθωση Holm για 3 τεστ γίνεται 0.094 > 0.05. **Με 6 seeds είναι αδύνατο να βγει σημαντικό αποτέλεσμα σε ένα κελί.** Με n = 10 το ελάχιστο είναι 0.002 (0.006 μετά το Holm), και αντέχει ακόμα και 1 ζεύγος αντίθετης κατεύθυνσης (≈ 0.012 μετά το Holm).
2. Η κύρια ανάλυση ενώνει τα κελιά: 10 seeds × 9 κελιά (3 πυκνότητες × 3 διαδρομές) = **90 ζεύγη ανά σενάριο**, 180 συνολικά. Με 180 ζεύγη, α = 0.05/3 και ισχύ 80% ανιχνεύεται επίδραση περίπου **d ≈ 0.25** (μικρή). Ανά σενάριο (90) περίπου d ≈ 0.35. Ανά κελί (10) μόνο μεγάλες, d ≈ 1.
3. Τα 10 seeds δίνουν λοιπόν αξιόπιστο **συνολικό** συμπέρασμα και ανά σενάριο, και ανά κελί μόνο για μεγάλες διαφορές. Το βήμα 6 υπάρχει για τα λεπτότερα συμπεράσματα.
- Περιορισμός: δεν είναι τυπική ανάλυση ισχύος, γιατί δεν έχουμε εκτίμηση του πραγματικού μεγέθους επίδρασης. Η προϋπόθεση ότι τα seeds διαφέρουν πραγματικά ικανοποιείται πια (§2.4).

### Βήμα 4 — Σύγκριση 3 μεθόδων, SCAND 🟡

| | |
|---|---|
| Runs | fixed, dynamic upstream, dynamic tracking × 1 καταγραφή |
| Εντολή | `JUMP=<τ> bash ~/crowd_aware_ws/scripts/run_experiments.sh scand` |

Το SCAND διαβάζει το `experiment.yaml`, οπότε το script γράφει εκεί προσωρινά τα `correspondence` και `length_jump_threshold` και τα επαναφέρει στο τέλος. Είναι open loop: όλες οι μέθοδοι βλέπουν ακριβώς τα ίδια δεδομένα, οπότε δεν υπάρχουν seeds. Η σύγκριση είναι περιγραφική ανά frame.

Μετρικές με νόημα στο SCAND: clustering (H2), IDs (H3), επιλογή path (Β5), διαγνωστικά r (H1 SCAND). Οι μετρικές ασφάλειας **δεν** υπολογίζονται, γιατί η θέση είναι η καταγραφή του Jackal και όχι εφικτή κίνηση του iWalk.

### Βήμα 5 — Αξιολόγηση 🟡

**Κανόνας «εμφανούς βελτίωσης»**, για κάθε υπόθεση χωριστά:
1. **Holm p < 0.05**: η διαφορά δεν εξηγείται από τύχη, ακόμα και αφού λάβουμε υπόψη ότι κάνουμε 3 τεστ. Χωρίς διόρθωση, με 3 τεστ η πιθανότητα να βγει τουλάχιστον ένα «σημαντικό» κατά τύχη είναι ~14% αντί για 5%.
2. **Το bootstrap 95% CI δεν περιέχει το 0**: δεύτερη, ανεξάρτητη από υποθέσεις επιβεβαίωση της κατεύθυνσης. Δείχνει επίσης **πόσο** μεγάλη είναι η διαφορά, σε φυσικές μονάδες (π.χ. «−3 s ακινησία»). Ένα πολύ μικρό p με πολύ μικρή διαφορά είναι σημαντικό στατιστικά, αλλά όχι πρακτικά.
3. **Καμία μετρική ασφάλειας χειρότερη**: η διάμεση διαφορά σε συγκρούσεις και ελάχιστες αποστάσεις δεν είναι σημαντικά χειρότερη για τη νέα μέθοδο (Holm p < 0.05 προς τη λάθος κατεύθυνση). Αλλιώς η «βελτίωση» μπορεί απλώς να σημαίνει ότι η μέθοδος παίρνει περισσότερα ρίσκα.

Αν μια υπόθεση δεν επιβεβαιωθεί, γράφεται ως αποτέλεσμα, όχι ως αποτυχία. Αν επιβεβαιωθεί τουλάχιστον μία, προχωράμε σε βήμα 6 και paper.

### Βήμα 6 — Ευρεία επιβεβαίωση: 50 seeds, όλα τα σενάρια (μόνο αν το βήμα 5 είναι θετικό) 🟡

| | |
|---|---|
| Runs | {fixed, tracking} × 9 σενάρια × medium × line × seeds 1–50 = **900 runs ≈ 39 h (≈ 1.6 μέρες)** |
| Εντολή | `JUMP=<τ> bash ~/crowd_aware_ws/scripts/run_experiments.sh broad` |

Μόνο τα dense_crowd και dense_static έχουν πυκνότητα και διαδρομές. Τα υπόλοιπα 7 τρέχουν με τη χειρόγραφη διάταξή τους, και όλα αλλάζουν πια με το seed (§2.4).

### Βήμα 7 — Paper / slides. Βήμα 8 — Demos

Αν ο κανόνας του βήματος 5 ικανοποιηθεί για τουλάχιστον μία υπόθεση, υπάρχει αποτέλεσμα για δημοσίευση. Τα demos μπορούν να γίνουν από τα bags (`replay_rviz.sh`) χωρίς νέα runs.

---

## 4. Μετρικές: πίνακας αντιστοίχισης με τον κώδικα

Όλα τα κλειδιά είναι μέσα στο `<run>/analysis_v3/metrics.json` (schema 7). Τα γράφει το `evaluation.py`, με τις offline μετρικές από το `offline_metrics.py`.

| Βήμα / Υπόθεση | Ερώτηση | Πείραμα | Μετρική (κλειδί) | Επιτυχία | Κατάσταση |
|---|---|---|---|---|---|
| Βήμα 1 | Ακολουθεί τη διαδρομή; | autonomous, open_area, 3 διαδρομές | `path_deviation.cross_track_*`, `ade_m`, `fde_m`, `goal_reached` | κριτήριο βήματος 1 | ✅ ελεγμένο σε runs |
| **H1** μεταβλητό r | Κόβει λιγότερα ασφαλή paths; | HuNav, fixed έναντι tracking, ίδια seeds | **κύρια:** `standstill_while_requested_s`· ασφάλεια: `collision_episodes` (`obstacle_` / `human_`), `additional_metrics.minimum_human_center_distance_m`, `minimum_obstacle_distance_m`, `personal_zone_time_s` | standstill ↓, ασφάλεια όχι χειρότερη | ✅ |
| | | SCAND, 3 μέθοδοι | (α) `dwal_diagnostics.*.mean_path_fraction.clustered` (αποδεκτά paths), (β) `no_cluster_message_fraction`, (γ) `offline.intent.false_stop.fraction`: προτεινόμενη v < 0.01 m/s ενώ το Jackal κινούνταν > 0.1 m/s | (α) ↑ (β) ↓ (γ) ↓ | ✅ |
| **H2** άλμα μήκους | Χωρίζει επιλογές χωρίς να θρυμματίζει; | βήμα 2 (σάρωση τ), βήμα 3 | **κύρια:** `offline.clustering.<group>.mixed_cluster_fraction` (under)· `oversplit_run_fraction` (over) | mixed ↓, over-split μικρό | ✅ |
| | Τρεμοπαίζει; | HuNav, SCAND | `flicker_per_min`: αλλαγή πλήθους clusters που αναιρείται σε < 0.5 s | όχι χειρότερο από fixed | ✅ |
| | Είναι σταθερό σε θόρυβο; | offline, ίδιο frame | `stability_ari_mean`, `stability_ari_p05`: ARI ανάμεσα στα labels των ίδιων paths από το καθαρό scan και από scan με Gaussian θόρυβο απόστασης **σ = 0.03 m** | κοντά στο 1 | ✅ |
| | Αλλάζει τη συμπεριφορά; | HuNav | standstill, συγκρούσεις, αποστάσεις (όπως H1) | όχι χειρότερα | ✅ |
| **H3** αντιστοίχιση | Κρατά σταθερά IDs; | offline replay upstream / tracking στα ίδια clusters (sim) + online (SCAND) | **κύρια:** `ids_replay_tracking.id_switches_per_min` έναντι `ids_replay_upstream`· επίσης `fragmentations_per_min`, `crossings_per_min`, `duplicate_id_frames`, `unsupported_inheritance_per_min` (J < 0.1), και `ids_recorded` (τα δημοσιευμένα IDs) | όλα ↓, διπλά = 0 | ✅ |
| **Β5** επιλογή path | Πόσο κοντά στην πραγματική διαδρομή; | SCAND | `phi_error_mean_rad` (\|φ* − φ_true\| στα 1.8 m), `ade_mean_m`, `fde_mean_m` (ίσο μήκος τόξου) | καλύτερα από `baseline_ade_mean_m` («μόνο χρήστης»), κυρίως `by_obstacle_distance` < 1 m | ✅ |
| Καθυστέρηση | Πόσο είναι το T στην πράξη; | κάθε run | `offline.latency.total_mean_s`, `total_p95_sum_s` (ηλικία scan → /cmd_vel) | — | ✅ |
| Αλυσίδα εντολών | Πού χάνεται η ταχύτητα; | κάθε run | `offline.command_chain.mean_speed_mps`, `drops` | — | ✅ |
| Στατιστική | Είναι η διαφορά αξιόπιστη; | ζεύγη runs | Wilcoxon, rank-biserial r, bootstrap 95% CI, Holm (κύριες: στις 3 υποθέσεις· δευτερεύουσες: ανά μετρική) | — | ✅ |

`<group>` = `far` για τον dynamic, `near` και `far` για τον fixed. `<scope>` = `all_frames` ή `iwalk_feasible_frames` (frames όπου το iWalk χωρά στη διαδρομή του Jackal).

Η πρόβλεψη πρόθεσης (`dwal_intent_estimator.hpp`) **δεν αξιολογείται**: δεν είναι συνεισφορά της εργασίας και δεν ισχυριζόμαστε βελτίωσή της. Ο estimator είναι **ο αρχικός του upstream, ίδιος σε όλες τις μεθόδους**: το `intent.split_by_overlap` (μοίρασμα πίστης σε split, που ήταν ενεργό μόνο στο tracking) αφαιρέθηκε 2026-11-10. Έτσι οι μέθοδοι διαφέρουν **μόνο** στο clustering και την αντιστοίχιση IDs. Τα runs tracking πριν από αυτή την αλλαγή (2026-11-04 και τα δοκιμαστικά 2026-11-10) έτρεξαν με το μοίρασμα ενεργό. Γι' αυτό δεν υπάρχουν μετρικές πιθανοτήτων (hit rate, Brier, log-loss). Στο SCAND αξιολογείται μόνο το **path** που διαλέγει ο controller (Β5) και το αν κάποιο cluster περιέχει την κατεύθυνση του οδηγού (`phi_true_inside_any_cluster_fraction`, μετρική clustering).

---

## 5. Εντολές συνοπτικά

SCENARIO=dense_crowd | cafe | dense_static | junction | crossing | doorway |  narrow_corridor | 
dense_crowd | group_blocking | open_area 
DENSITY=sparse | medium | dense
PATH_SHAPE=line | s_curve | hard_curve
REFERENCE_MODE=autonomous | teleop
run dynamic_dwal | fixed_dwal
off | on



```bash
# ένα run για να το δεις
bash ~/crowd_aware_ws/scripts/run_dwal_cafe.sh build

SCENARIO=open_area DENSITY=medium PATH_SHAPE=hard_curve REFERENCE_MODE=autonomous RVIZ=true bash ~/crowd_aware_ws/scripts/run_dwal_cafe.sh run dynamic_dwal on 2
# βήματα
bash ~/crowd_aware_ws/scripts/run_experiments.sh deviation
bash ~/crowd_aware_ws/scripts/run_experiments.sh jump
JUMP=<τ> VIEW=1 bash ~/crowd_aware_ws/scripts/run_experiments.sh compare
JUMP=<τ> bash ~/crowd_aware_ws/scripts/run_experiments.sh scand
JUMP=<τ> bash ~/crowd_aware_ws/scripts/run_experiments.sh broad
# μετρικές
python3 ~/crowd_aware_ws/scripts/metrics_summary.py --metrics path_deviation --evaluate   # μόνο απόκλιση
python3 ~/crowd_aware_ws/scripts/metrics_summary.py ~/crowd_aware_ws/rosbags ~/scand_ws/results --evaluate   # όλες
# replay
bash ~/crowd_aware_ws/scripts/replay_rviz.sh <run>
```
Κάθε mode δέχεται τα δικά του seeds στο τέλος και τα `SCENARIOS`, `DENSITIES`, `PATHS`, `JUMPS`, `METHODS` (`fixed`, `upstream`, `tracking`) για να περιοριστεί.

---

## 6. Αποφάσεις (οριστικές, 2026-11-10)

|         | Απόφαση                                | Τι ισχύει                                                                                                                                                                                                                             | Πού στον κώδικα                                      |
| ------- | -------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------- |
| **Α**   | Seed variation                         | dense_static: μόνο θέση εμποδίων (±0.2 m κατά μήκος, ±0.15 m πλευρικά). dense_crowd: αρχική θέση πεζών, τυχαίες δυνάμεις HuNav, ταχύτητα ×U(0.85, 1.15), στόχοι ±0.3 m. Seed 1 = χωρίς μεταβολές.                                     | `seed_variation` στα `dense_*.yaml`, `layout_agents` |
| **Β**   | Κριτήριο αποδοχής                      | κάθε run: στόχος εντός 0.2 m, 0 συγκρούσεις, cross-track p95 ≤ 0.30 m (βήμα 1). Στο βήμα 2: από τα **αποδεκτά** τ (όλα τα runs χωρίς σύγκρουση και με στόχο) επιλέγεται αυτό με το καλύτερο Loss.                                     | `metrics_summary.py: jump_selection`                 |
| **Γ**   | Loss(τ)                                | λ = (1, 1, 1). Ισοπαλία (< 0.02) → μικρότερο S (ID switches / λεπτό).                                                                                                                                                                 | `jump_selection`, `--lambdas`                        |
| **Δ**   | Κύριες μετρικές                        | H1 standstill, H2 mixed, H3 ID switches (§3 βήμα 3). Οι υπόλοιπες δευτερεύουσες.                                                                                                                                                      | `primary_hypotheses`                                 |
| **Ε**   | Βήμα 3                                 | και οι 3 διαδρομές, **μόνο fixed και dynamic tracking** (360 runs). Το H3 κρίνεται offline στα ίδια clusters.                                                                                                                         | `run_experiments.sh` (`METHODS="fixed tracking"`)    |
| **ΣΤ**  | Ανάλυση ανά σενάριο                    | δευτερεύουσα. Εξηγεί από πού έρχεται το αποτέλεσμα, δεν το αλλάζει (§3 βήμα 3).                                                                                                                                                       | `primary_hypotheses` (πίνακες «ανά σενάριο»)         |
| **Ζ**   | Εμφανής βελτίωση                       | Holm p < 0.05, CI χωρίς 0, ασφάλεια όχι χειρότερη (§3 βήμα 5).                                                                                                                                                                        | `primary_hypotheses`                                 |
| **Η**   | Βήμα 6                                 | 9 σενάρια, 50 seeds, fixed και tracking (900 runs ≈ 39 h), μόνο αν το βήμα 5 είναι θετικό. Laptop ή server: §3 βήμα 6.                                                                                                                | `run_experiments.sh broad`                           |
| **Θ**   | Καμπύλες                               | Βήμα 1 στο open_area (χωρίς εμπόδια). Στο dense_static εμπόδια στα 1.3 m στις καμπύλες, 1.0 m στην ευθεία.                                                                                                                            | `lateral_m` στο `dense_static.yaml`                  |
| **Ι** | Ακινησία όταν κολλά το robot | Ο reference δεν εγκαταλείπει ποτέ (χωρίς collision check, χωρίς progress timeout). Το H1 μένει `standstill_while_requested_s`, όπως θα μετριόταν και σε πραγματικά πειράματα χωρίς reference path. | `controllers/reference.yaml`, `prepare_scene.py` |
| **Κ** (2026-11-11, απόκλιση) | Φίλτρο ορισμού στην επιλογή τ | Under-split = **παραβίαση ορισμού** (`choice_violation_fraction`): κάθε διαδρομή παίρνει επιλογή κίνησης, *brake* (τερματίζει σε οποιοδήποτε εμπόδιο) ή *pass k* (k-οστό συνεχόμενο ελεύθερο τόξο, σε σειρά γωνίας πάνω σε **όλες** τις διαδρομές)· cluster σε ≥ 2 επιλογές = παραβίαση. Αντικαθιστά το `mixed_cluster_fraction` στο Loss (το mixed μετρούσε και «προς Α + προς Β», που είναι μία επιλογή). **Αποδεκτό τ** = 0 συγκρούσεις + στόχος σε κάθε run **και** παραβίαση ≤ **0.10 σε κάθε κελί** (σενάριο × πυκνότητα). Το όριο ορίστηκε πριν τον υπολογισμό στα 180 runs, έχοντας δει δοκιμή σε 4 (dense, seed 101: τ 0.1 ≈ 0.03, τ 1.0 = 0.33–0.66). Αιτία: ο αρχικός κανόνας επέλεγε τ = 1.0, που στο dense_crowd/dense αφήνει τα 2/3 των clusters σε ≥ 2 επιλογές· το Loss ευνοεί το «κανένα split» (2 από 3 όροι πέφτουν με μεγάλο τ). Αν κανένα τ δεν περάσει, γράφεται ως αποτέλεσμα, το όριο δεν χαλαρώνει. | `offline_metrics.py: clustering_metrics`, `metrics_summary.py: jump_selection` |
| **Λ** (2026-11-11, απόκλιση μετά τα αποτελέσματα) | Όριο φίλτρου ορισμού 0.10 → **0.25** | Με το όριο της απόφασης Κ (≤ 0.10 σε κάθε κελί) πέρασε μόνο το τ = 0.1· το 0.2 κόπηκε οριακά (χειρότερο κελί 0.101, dense_crowd/medium). Κρίθηκε πολύ αυστηρό, γιατί άφηνε στο Loss μία μόνο υποψήφια τιμή, και το όριο χαλαρώθηκε σε 0.25, **αφού είδαμε τα αποτελέσματα**. Με 0.25 περνούν τα τ = 0.1, 0.2, 0.3 (χειρότερο κελί 0.051 / 0.101 / 0.176)· κόβονται τα 0.5 (0.260), 0.7 (0.351), 1.0 (0.559). Το Loss επιλέγει πάλι **τ = 0.1** (1.493, έναντι 1.617 για 0.3 και 1.686 για 0.2). **Η επιλογή του τ δεν αλλάζει** μεταξύ των δύο ορίων, άρα δεν εξαρτάται από την ακριβή τιμή του ορίου. Μετρικές αμετάβλητες (μόνο ο κανόνας επιλογής)· η έκδοση με 0.10: `rosbags/results/summary_jump_v010.*`. | `metrics_summary.py: VIOLATION_MAX` |
| **Μ** (2026-11-11, πριν το βήμα 3) | Βήμα 3 σε όλα τα σενάρια | Αντικαθιστά το εύρος της απόφασης **Ε**. Βήμα 3 = {fixed, tracking} × seeds 1–10 × (dense_crowd, dense_static: 3 πυκνότητες × 3 διαδρομές = 360 runs· τα 7 χειρόγραφα σενάρια: medium × line = 140 runs) = **500 runs ≈ 22 h**, τ = 0.1 (βήματα 2, Κ, Λ). Κύρια ανάλυση: όλα τα ζεύγη· δευτερεύουσα: ανά σενάριο (στα χειρόγραφα 10 ζεύγη ανά σενάριο). Αποφασίστηκε **πριν** τρέξει το βήμα 3. Για να μείνει ανεξάρτητο το βήμα 6, το `broad` τρέχει πια με **seeds 11–60** (τα 1–10 είναι ήδη στο βήμα 3). Το image ξαναχτίστηκε 2026-11-11 15:50 (`3135febe…`) μόνο για τον κώδικα αξιολόγησης· τα layers DWAL / controller / patches είναι ίδια με το βήμα 2 (`dd80…`). | `run_experiments.sh` (`compare`, `broad`, `LAYOUT_SCENARIOS`) |


----

