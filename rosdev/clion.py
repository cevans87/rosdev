from dataclasses import dataclass, field, replace
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.idea.clion.core import GenIdeaClionCore
from rosdev.gen.idea.core import GenIdeaCore
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class Clion(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenIdeaClionCore,
    ))

    post_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenIdeaCore,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        return replace(
            options,
            docker_entrypoint_sh_setup_overlay=True,
            docker_entrypoint_sh_setup_underlay=True,
            idea_ide_name='CLion',
        )
