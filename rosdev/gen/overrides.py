from dataclasses import asdict, dataclass, field
from frozendict import frozendict
from pathlib import Path
from pprint import pformat
from typing import Tuple, Type

from rosdev.gen.host import GenHost
from rosdev.gen.workspace import GenWorkspace
from rosdev.util.handler import Handler
from rosdev.util.options import Options


@dataclass(frozen=True)
class GenOverrides(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenHost,
        GenWorkspace,
    ))

    @classmethod
    async def main(cls, options: Options) -> None:
        commit = {}
        default_options = Options()

        for k, v in asdict(options).items():
            if (k == 'stage') or (getattr(default_options, k) == v):
                continue

            if isinstance(v, Path):
                v = str(v)
            elif isinstance(v, frozendict):
                v_mod = {}
                for k_inner, v_inner in v.items():
                    if isinstance(k_inner, Path):
                        k_inner = str(k_inner)
                    if isinstance(v_inner, Path):
                        v_inner = str(v_inner)
                    v_mod[k_inner] = str(v_inner)
                v = v_mod
            commit[k] = v

        if commit:
            GenHost.write_text(
                data=f'{pformat(commit)}\n',
                options=options,
                path=options.overrides_path,
            )
