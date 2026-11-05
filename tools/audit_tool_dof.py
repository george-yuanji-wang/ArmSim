"""Audit all loaded scenes: python -m tools.audit_tool_dof."""

import hashlib
import json
from collections import Counter

import numpy as np

from armsim.catalog import LAYOUTS, MODEL_DIR, WRIST_PROFILES
from armsim.kinematics import tool_mobility
from armsim.simulation import Simulation


def main():
    seed, count = 20261031, 500
    entries = []
    for layout in LAYOUTS:
        for profile in WRIST_PROFILES:
            sim = Simulation(layout, profile)
            bent = tool_mobility(sim.model, sim.data)
            sim.reset(np.zeros(6))
            straight = tool_mobility(sim.model, sim.data)
            # Identical samples for comparable layouts, within 80% of limits.
            samples = np.random.default_rng(seed).uniform(
                sim.limits[:, 0]*0.8, sim.limits[:, 1]*0.8, (count, 6))
            ranks = Counter()
            weakest = 1.
            for pose in samples:
                sim.reset(pose)
                mobility = tool_mobility(sim.model, sim.data)
                ranks[mobility['rank']] += 1
                weakest = min(weakest, mobility['reciprocal_condition'])
            if bent['rank'] != 6 or ranks[6] != count:
                raise ValueError(f'Mobility check failed: {layout.number} / {profile}: {ranks}')
            path = layout.path_for(profile)
            entries.append({'file': path.name, 'title': layout.title, 'wrist': profile,
                            'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                            'bent_pose_radians': layout.bent_pose.tolist(), 'bent_pose': bent,
                            'straight_pose': straight, 'sample_rank_counts': dict(ranks),
                            'weakest_sample_reciprocal_condition': weakest})
            print(f'{path.name}: bent 6/6; samples {ranks[6]}/{count} full rank; straight {straight["rank"]}/6')
    report = {'method': 'Full 6x6 tool-site geometric Jacobian, world-frame linear and angular velocity; translation scaled by L1=0.32 m.',
              'scope': 'Local kinematic mobility; sampling does not certify collision-free reachability or exclude all singular poses.',
              'seed': seed, 'samples_per_model': count, 'models': entries}
    path = MODEL_DIR/'tool-dof-audit.json'
    path.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(f'Saved {path}')


if __name__ == '__main__':
    main()
