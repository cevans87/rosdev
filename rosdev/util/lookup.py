def get_build_num(architecture: str, release: str) -> int:
    return {
        'amd64': {
            'dashing': 1482,
            'crystal': 1289,
        },
        'arm32v7': {
            'dashing': 16,
        },
        'arm64v8': {
            'dashing': 825,
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
