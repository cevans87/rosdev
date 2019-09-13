from dataclasses import asdict, dataclass, field
from pathlib import Path
from pprint import pformat
from typing import Tuple, Type

from rosdev.gen.container import GenContainer
from rosdev.gen.universal import GenUniversal
from rosdev.gen.workspace import GenWorkspace
from rosdev.util.handler import Handler
from rosdev.util.options import Options


@dataclass(frozen=True)
class GenOverrides(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenContainer,
        GenUniversal,
        GenWorkspace,
    ))

    @classmethod
    async def main(cls, options: Options) -> None:
        commit = {}
        default_options = Options()

        for k, v in asdict(options).items():
            if (not isinstance(v, Path)) and getattr(default_options, k) != v:
                commit[k] = v

        if commit:
            options.write_text(
                path=options.overrides_path,
                text=f'{pformat(commit)}\n'
            )
