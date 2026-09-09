"""Wait for the upstream asynchronous world generator; never launch a stale world."""
import sys,time,xml.etree.ElementTree as E
from pathlib import Path
p=Path(sys.argv[1]); deadline=time.monotonic()+120
while time.monotonic()<deadline:
    try:
        r=E.parse(p).getroot();w=r.find('world')
        if w is not None and w.findall('actor') and any(x.get('filename')=='libHuNavPlugin.so' for x in w.findall('plugin')):
            print('HuNav world ready:',p,flush=True);sys.exit(0)
    except (OSError,E.ParseError):
        pass
    time.sleep(0.2)
print('HuNav world generation timed out. Inspect hunav_loader and world_generator logs.',file=sys.stderr)
sys.exit(1)
