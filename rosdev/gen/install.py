from asyncio import get_event_loop
from asyncio.subprocess import create_subprocess_exec
from atools import memoize
from functools import partial
from logging import getLogger
import os
import shutil
from tempfile import TemporaryDirectory
from typing import Optional


log = getLogger(__package__)


def machine(architecture: str) -> str:
    return {
        'amd64': 'x86_64',
        'arm32v7': 'arm',
        'arm64v8': 'aarch64'
    }[architecture]


async def _exec(command: str) -> None:
    log.debug(command)
    proc = await create_subprocess_exec(*command.split())
    await proc.wait()


@memoize
async def install(
        *,
        architecture: str,
        build_num: Optional[str],
) -> None:
    path_base = f'.rosdev/{architecture}'
    path = f'{path_base}/{build_num}'
    cache_path_base = f'~/{path_base}'
    cache_path = f'~/{path}'
    install_path_base = f'./{path_base}'
    install_path = f'./{path}'

    log.info(f'Installing build artifacts for architecture: {architecture}, build: {build_num}')
    os.makedirs(cache_path_base, exist_ok=True)

    if os.path.exists(cache_path):
        log.info(f'Found cached artifacts at {cache_path}')
    else:
        with TemporaryDirectory() as temp_dir:
            artifacts_path = f'{temp_dir}/artifacts.tar.bz2'
            temp_path = f'{temp_dir}/{path}'

            log.info('Downloading build artifacts')
            await _exec(
                f'wget https://ci.ros2.org/view/packaging/job/packaging_linux/'
                f'{"lastSuccessfulBuild" if build_num is None else build_num}'
                f'/artifact/ws/ros2-package-linux-{machine(architecture)}.tar.bz2 '
                f'-O {artifacts_path}'
            )

            log.info(f'Extracting build artifacts')
            os.makedirs(temp_path, exist_ok=True)
            await _exec(f'tar -xf {artifacts_path} -C {temp_path} --strip-components 1')

            log.info(f'Caching artifacts to {cache_path}')
            await get_event_loop().run_in_executor(
                None, partial(shutil.move, temp_path, cache_path))

    if os.path.exists(install_path):
        log.info(f'Removing previous install at {install_path}')
        await get_event_loop().run_in_executor(
            None, partial(shutil.rmtree, install_path, ignore_errors=True))

    log.info(f'Installing artifacts to {install_path}')
    os.makedirs(install_path_base, exist_ok=True)
    await get_event_loop().run_in_executor(
        None, partial(shutil.copytree, cache_path, install_path))
