"""Host static RNEA vs independent double-precision potential-energy gradient.

Requires Python + numpy and a native g++ (not arm-none-eabi-g++).
Run: python validate_rnea.py --compiler /path/to/g++
"""

import argparse
import io
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

import numpy as np


def potential(q, mode):
    """Use world-frame potential energy and MATLAB's fixed base rotation."""
    dh = [(0.12, -np.pi / 2, 0.0, 0.0), (0.30, 0.0, 0.10, 0.0),
          (0.0, np.pi / 2, 0.0, np.pi / 2), (0.0, 0.0, 0.14, 0.0)]
    masses = [0.8, 0.6, 0.4, 0.2]
    centers = np.array([[-0.06, 0.01, 0.015], [-0.15, 0.02, -0.01],
                        [0.03, -0.02, 0.04], [0.01, 0.02, 0.05]])
    if mode == 0:
        centers[:] = 0
    gravity = np.array([0.0, 0.0, -9.81])
    if mode == 2:
        gravity[:] = 0
    elif mode == 3:
        gravity = np.array([1.2, 2.3, -8.7])

    transform = np.array([[1., 0., 0., 0.], [0., 0., -1., 0.],
                          [0., 1., 0., 0.], [0., 0., 0., 1.]])
    energy = 0.0
    for k, (a, alpha, d, offset) in enumerate(dh):
        ct, st = np.cos(q[k] + offset), np.sin(q[k] + offset)
        ca, sa = np.cos(alpha), np.sin(alpha)
        transform = transform @ np.array([
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0, sa, ca, d], [0, 0, 0, 1]])
        center = transform[:3, 3] + transform[:3, :3] @ centers[k]
        energy -= masses[k] * gravity @ center
    return energy


def gravity_from_potential(q, mode):
    torque = np.zeros(4)
    step = 1e-5
    for k in range(4):
        delta = np.zeros(4)
        delta[k] = step
        torque[k] = (potential(q + delta, mode) - potential(q - delta, mode)) / (2 * step)
    return torque


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--compiler', default='g++')
    args = parser.parse_args()
    compiler = shutil.which(args.compiler)
    if compiler is None:
        raise FileNotFoundError(args.compiler)
    environment = os.environ.copy()
    environment['PATH'] = str(Path(compiler).parent) + os.pathsep + environment.get('PATH', '')
    root = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix='centaur_rnea_') as temporary:
        executable = Path(temporary) / ('test_rnea.exe' if os.name == 'nt' else 'test_rnea')
        command = [compiler, '-std=c++11', '-O2', '-Wall', '-Wextra', '-Werror', '-pedantic',
                   '-I', str(root / 'Inc'), str(root / 'Src' / 'alg_rnea.cpp'),
                   str(root / 'Tests' / 'test_alg_rnea.cpp'), '-o', str(executable)]
        if os.name == 'nt':
            command.append('-static')
        subprocess.run(command, check=True, env=environment)
        result = subprocess.run([str(executable)], check=True, capture_output=True, text=True, env=environment)
        print(result.stderr.strip())
        samples = np.loadtxt(io.StringIO(result.stdout), delimiter=',')
    errors = []
    for row in samples:
        expected = gravity_from_potential(row[1:5], int(row[0]))
        errors.append(np.abs(expected - row[5:9]))
    errors = np.array(errors)
    maximum = float(errors.max())
    print(f'{len(samples)} cases / {errors.size} torques; max error = {maximum:.9g} N*m')
    if maximum >= 2e-5:
        raise AssertionError('Static RNEA does not match the potential-energy reference')
    print('PASS: zero/nonzero first moments, zero/tilted gravity and MATLAB base orientation.')


if __name__ == '__main__':
    main()
