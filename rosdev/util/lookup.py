# FIXME replace with options equivalent
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
