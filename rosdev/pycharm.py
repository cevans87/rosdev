from dataclasses import dataclass, field, replace
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.idea.pycharm.core import GenIdeaPycharmCore
from rosdev.gen.idea.core import GenIdeaCore
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class Pycharm(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenIdeaPycharmCore,
    ))

    post_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenIdeaCore,
    ))

    @classmethod
    async def resolve_options(cls, options: Options) -> Options:
        return replace(options, idea_ide_name='PyCharm')
