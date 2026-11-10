"""Test the joint calibration FSM with simulated motor feedback.

Run with a native g++, not arm-none-eabi-g++:
  python validate_joint_calibration.py --compiler /path/to/g++
"""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--compiler', default='g++')
    args = parser.parse_args()
    compiler = shutil.which(args.compiler)
    if compiler is None:
        raise FileNotFoundError(args.compiler)
    project = Path(__file__).resolve().parents[3]
    environment = os.environ.copy()
    environment['PATH'] = str(Path(compiler).parent) + os.pathsep + environment.get('PATH', '')
    with tempfile.TemporaryDirectory(prefix='centaur_feedforward_') as temporary:
        temp = Path(temporary)
        stubs = {
            'drv_math.h': '#pragma once\n#include <stdint.h>\n',
            'drv_can.h': '#pragma once\nstruct FDCAN_HandleTypeDef {};\nstruct Struct_CAN_Manage_Object {};\n',
            'alg_pid.h': '#pragma once\nclass Class_PID {};\n',
            'main.h': '#pragma once\n',
        }
        for name, contents in stubs.items():
            (temp / name).write_text(contents, encoding='utf-8')
        executable = temp / ('joint_calibration_test.exe' if os.name == 'nt' else 'joint_calibration_test')
        command = [compiler, '-std=c++11', '-O2', '-Wall', '-Wextra', '-Werror',
                   '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections',
                   '-I', str(temp), '-I', str(project / 'User/Device/Inc'),
                   '-I', str(project / 'RobotArm/Inc'),
                   '-I', str(project / 'User/Algorithm/Inc'),
                   str(project / 'RobotArm/Src/rbt_Joint.cpp'),
                   str(project / 'User/Algorithm/Src/alg_fsm.cpp'),
                   str(Path(__file__).with_name('test_joint_calibration.cpp')), '-o', str(executable)]
        if os.name == 'nt':
            command.append('-static')
        subprocess.run(command, check=True, env=environment)
        subprocess.run([str(executable)], check=True, env=environment)


if __name__ == '__main__':
    main()
