#!/usr/bin/env python3
"""Check that Gazebo delivered contact observations and summarize the run."""
from collections import Counter
import json
from pathlib import Path
import re
import sys


def main():
    evidence = Path(sys.argv[1])
    contacts = (evidence / 'gazebo_contacts.log').read_text()
    frames = len(re.findall(r'^time \{', contacts, re.MULTILINE))
    pairs = [tuple(sorted(pair)) for pair in re.findall(
        r'collision1:\s*"([^"]+)"\s*collision2:\s*"([^"]+)"', contacts)]
    assert frames >= 100, f'Only {frames} Gazebo contact samples'
    assert contacts.count('collision1:') == len(pairs), 'Could not parse all contact pairs'
    self_pairs = Counter((a, b) for a, b in pairs
                         if a.startswith('tomato_picker::') and b.startswith('tomato_picker::'))
    result = json.loads((evidence / 'motion_results.json').read_text())
    result['gazebo_contacts'] = {
        'samples': frames, 'contact_records': len(pairs),
        'robot_self_contact_pairs': [{'a': a, 'b': b, 'records': count}
                                     for (a, b), count in sorted(self_pairs.items())],
    }
    (evidence / 'phase1_summary.json').write_text(json.dumps(result, indent=2) + '\n')
    print(f'Gazebo reported {frames} contact samples and {sum(self_pairs.values())} robot self-contact records.')


if __name__ == '__main__':
    main()
