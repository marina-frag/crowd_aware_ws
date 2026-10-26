#!/usr/bin/env python3
"""Regression check that evaluation is read-only for an existing run."""
import hashlib
import os
from pathlib import Path
import subprocess
import sys
import unittest


ROOT = Path(__file__).resolve().parents[3]


def digest_tree(root):
    digest = hashlib.sha256()
    for path in sorted(p for p in root.rglob('*') if p.is_file() and 'analysis_v3' not in p.parts):
        digest.update(str(path.relative_to(root)).encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


class EvaluationRegression(unittest.TestCase):
    def test_existing_run_is_not_changed(self):
        run = os.environ.get('DWAL_REGRESSION_RUN')
        if not run:
            self.skipTest('set DWAL_REGRESSION_RUN to run against an existing rosbag')
        run = Path(run).resolve()
        self.assertTrue((run / 'bag').is_dir())
        self.assertTrue((run / 'evaluation.json').is_file())
        if not os.access(run, os.W_OK):
            self.skipTest(f'run directory is not writable: {run}')
        before = digest_tree(run)
        subprocess.run([sys.executable, str(ROOT / 'src/crowd_aware_simulation/scripts/evaluation.py'), 'evaluate', str(run)],
                       cwd=ROOT, check=True)
        self.assertEqual(before, digest_tree(run))
        analysis = run / 'analysis_v3'
        self.assertTrue(analysis.is_dir())
        self.assertTrue(
            {'metrics.json', 'additional_timeline.csv', 'samples.csv', 'clusters.jsonl'}
            <= {p.name for p in analysis.iterdir() if p.is_file()}
        )


if __name__ == '__main__':
    unittest.main()
