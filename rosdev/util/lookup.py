from frozendict import frozendict

from rosdev.util.subprocess import get_shell_lines


def get_build_num(architecture: str, release: str) -> int:
    return {
        'amd64': {
            'crystal': 1289,
        },
        'arm32v7': {

        },
        'arm64v8': {
            'crystal': 651,
        },
    }[architecture][release]


def get_machine(architecture: str) -> str:
    return {
        'amd64': 'x86_64',
        'arm32v7': 'arm',
        'arm64v8': 'aarch64'
    }[architecture]


def get_operating_system(architecture: str) -> str:
    return {
        'amd64': 'linux',
        'arm64v8': f'linux-{get_machine(architecture)}',
    }[architecture]


async def get_environ() -> frozendict:
    lines = await get_shell_lines(f'env -i bash -c \'source install/setup.bash && env\'')
    environ = {}
    for line in lines:
        if line:
            k, v = line.split('=', 1)
            if k in {
                'AMENT_PREFIX_PATH',
                'CMAKE_PREFIX_PATH',
                'COLCON_PREFIX_PATH',
                'LD_LIBRARY_PATH',
                'PATH',
                'PYTHONPATH',
                'ROS_DISTRO',
                'ROS_PYTHON_VERSION',
                'ROS_VERSION',
            }:
                environ[k] = v

    return frozendict(environ)
