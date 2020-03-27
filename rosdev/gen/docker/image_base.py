from dataclasses import dataclass
import json
from logging import getLogger

from rosdev.gen.host import GenHost
from rosdev.util.atools import memoize, memoize_db
from rosdev.util.frozendict import frozendict, FrozenDict
from rosdev.util.handler import Handler
from rosdev.util.options import Options

log = getLogger(__name__)


@dataclass(frozen=True)
class GenDockerImageBase(Handler):

    @staticmethod
    @memoize
    async def get_tag(options: Options) -> str:
        tag = f'rosdev:{options.release}_{options.architecture}'

        log.debug(f'{__class__.__name__} {tag = }')

        return tag

    # noinspection PyShadowingBuiltins
    @staticmethod
    @memoize
    async def get_id(options: Options) -> int:
        @memoize_db
        def get_id_inner() -> int:
            return 0

        if options.docker_image_pull or options.docker_image_replace:
            get_id_inner.memoize.update()(get_id_inner() + 1)

        id = get_id_inner()

        log.debug(f'{__class__.__name__} {id = }')

        return id

    @staticmethod
    @memoize_db(keygen=lambda options: GenDockerImageBase.get_tag(options))
    async def _get_inspect_json(options: Options) -> FrozenDict:
        lines = await GenHost.execute_shell_and_get_lines(
            command=(
                f'docker image inspect {await GenDockerImageBase.get_tag(options)} 2> /dev/null'
            ),
            options=options,
            err_ok=True,
        )
        inspect_json = frozendict(array[0] if (array := json.loads('\n'.join(lines))) else {})

        log.debug(f'{__class__.__name__} {inspect_json = }')

        return inspect_json
