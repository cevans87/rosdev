from atools import memoize
from dataclasses import asdict, dataclass
from frozendict import frozendict
from logging import getLogger
from pathlib import Path
from pprint import pformat

from rosdev.gen.host import GenHost
from rosdev.gen.workspace import GenWorkspace
from rosdev.util.handler import Handler
from rosdev.util.options import Options


log = getLogger(__name__)


@dataclass(frozen=True)
class GenOverrides(Handler):

    @classmethod
    @memoize
    async def get_path(cls, options: Options) -> Path:
        path = await GenWorkspace.get_path(options) / '.rosdev' / 'overrides'

        log.debug(f'{cls.__name__} {path = }')

        return path

    @classmethod
    @memoize
    async def get_text(cls, options: Options) -> str:
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
        text = f'{pformat(commit)}\n' if commit else ''
        
        log.debug(f'{cls.__name__} {text = }')
        
        return text

    @classmethod
    async def main(cls, options: Options) -> None:
        GenHost.write_text(
            data=await cls.get_text(options),
            options=options,
            path=await cls.get_path(options),
        )
