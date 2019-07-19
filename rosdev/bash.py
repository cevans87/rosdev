from asyncio import gather
from atools import memoize
from dataclasses import dataclass, replace
from frozendict import frozendict
from logging import getLogger
from pathlib import Path

from rosdev.gen.install import Install
from rosdev.gen.rosdev.config import Config as RosdevConfig
from rosdev.gen.src import Src
from rosdev.gen.docker.container import Container
from rosdev.util.handler import Handler


log = getLogger(__name__)


@memoize
@dataclass(frozen=True)
class Bash(Handler):
    
    @memoize
    async def _main(self) -> None:
        build_num = await RosdevConfig(self.options).get_build_num()
        options = replace(
            self.options,
            build_num=build_num,
            command='/bin/bash',
            interactive=True,
            volumes=frozendict({
                **self.options.volumes,
                f'{Path.home()}/.bashrc': f'{Path.home()}/.bashrc',
                f'{Path.home()}/.bash_history': f'{Path.home()}/.bash_history',
                f'{Path.home()}/.profile': f'{Path.home()}/.profile',
                **Install(replace(self.options, build_num=build_num)).options.volumes,
                **Src(replace(self.options, build_num=build_num)).options.volumes,
            })
        )
        await gather(
            Install(options),
            Src(options),
        )
        await Container(options)
