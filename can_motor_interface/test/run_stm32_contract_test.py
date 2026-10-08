#!/usr/bin/env python3
"""Compile the real ROS encoder and STM32 parser/control with firmware HAL mocks."""
import argparse
from pathlib import Path
import subprocess
import tempfile


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--firmware', type=Path, default=root.parent / 'STM32_Motor_Controller')
    firmware = parser.parse_args().firmware.resolve()
    if not (firmware / 'tests/can_regression.c').is_file():
        parser.error('provide --firmware pointing to the STM32_Motor_Controller source tree')
    with tempfile.TemporaryDirectory(prefix='ros-stm32-contract-') as directory:
        output = Path(directory)
        includes = ['-include', str(firmware / 'tests/support/hal_stub.h'),
                    '-I', str(firmware / 'tests/support'), '-I', str(firmware / 'Core/Inc')]
        sources = [firmware / 'Core/Src' / (name + '.c') for name in
                   ('can_transport', 'zdt_can_driver', 'motor_control', 'can_protocol', 'zdt_status', 'stm32f4xx_it')]
        sources.append(root / 'test/stm32_contract.c')
        objects = []
        for source in sources:
            obj = output / (source.stem + '.o')
            subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-O2'] + includes +
                           ['-DCAN_FIRMWARE_REGRESSION="' + str(firmware / 'tests/can_regression.c') + '"',
                            '-c', str(source), '-o', str(obj)], check=True)
            objects.append(str(obj))
        encoder = output / 'ros_encoder.o'
        subprocess.run(['g++', '-std=c++14', '-Wall', '-Wextra', '-Werror', '-O2',
                        '-I', str(root / 'include'), '-c', str(root / 'test/ros_encoder_contract.cpp'),
                        '-o', str(encoder)], check=True)
        executable = output / 'contract'
        subprocess.run(['g++'] + objects + [str(encoder), '-o', str(executable)], check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == '__main__':
    main()
