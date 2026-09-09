"""Use the existing ROS CLI to wait until DWAL's parameter server is responsive."""
import subprocess,time,sys
end=time.monotonic()+90
while time.monotonic()<end:
    try:
        result=subprocess.run(['ros2','param','get','/dwal_planner/dwal_clustering','dwal_clustering/Marker_num'],capture_output=True,timeout=5)
        if result.returncode==0 and b'Integer value' in result.stdout:
            sys.exit(0)
    except subprocess.TimeoutExpired:
        pass
    time.sleep(0.2)
print('DWAL clustering parameter service did not become ready',file=sys.stderr)
sys.exit(1)
