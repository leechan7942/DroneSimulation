#!/usr/bin/env python3

import os
import subprocess
import sys

scripts_dir = os.path.dirname(os.path.abspath(__file__))
scripts = ["dodge.py", "land.py","path.py"]
processes = []

for script in scripts:
    path = os.path.join(scripts_dir, script)
    processes.append(subprocess.Popen([sys.executable, path]))

for process in processes:
    process.wait()
