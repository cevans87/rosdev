from dataclasses import dataclass
import json
from logging import getLogger
import re

from rosdev.gen.docker.image_base import GenDockerImageBase
from rosdev.gen.host import GenHost
from rosdev.util.atools import memoize, memoize_db
from rosdev.util.frozendict import frozendict, FrozenDict
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.path import Path

log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerContainerBase(Handler):

    # noinspection PyShadowingBuiltins
    @staticmethod
    @memoize
    async def get_id(options: Options) -> int:
        @memoize_db
        def get_id_inner() -> int:
            return 0

        if (
                options.docker_container_replace or
                not (await GenDockerContainerBase._get_inspect_json(options))  # container is gone
        ):
            get_id_inner.memoize.update()(get_id_inner() + 1)

        id = get_id_inner() + await GenDockerImageBase.get_id(options)

        log.debug(f'{__class__.__name__} {id = }')

        return id

    @staticmethod
    @memoize_db(keygen=lambda options: GenDockerContainerBase.get_name(options))
    async def _get_inspect_json(options: Options) -> FrozenDict:
        lines = await GenHost.execute_shell_and_get_lines(
            command=(
                f'docker container inspect {await GenDockerContainerBase.get_name(options)} 2>'
                f' /dev/null'
            ),
            options=options,
            err_ok=True,
        )
        inspect_json = frozendict(array[0] if (array := json.loads('\n'.join(lines))) else {})

        log.debug(f'{__class__.__name__} {inspect_json = }')

        return inspect_json

    @staticmethod
    @memoize
    async def get_name(options: Options) -> str:
        name = re.sub(
            r'[^\w.]',
            '_',
            (
                f'rosdev_{Path.workspace().relative_to(Path.home())}'
                f'_{options.release}'
                f'_{options.architecture}'
            ),
        )

        log.debug(f'{__class__.__name__} {name = }')

        return name

    @staticmethod
    @memoize
    async def get_ssh_path(options: Options) -> Path:
        ssh_path = Path.home() / '.ssh'

        log.debug(f'{__class__.__name__} {ssh_path = }')

        return ssh_path
