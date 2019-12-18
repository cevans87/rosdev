from atools import memoize
from dataclasses import asdict, dataclass
from frozendict import frozendict
from logging import getLogger
from pprint import pformat

from rosdev.gen.host import GenHost
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.path import Path


log = getLogger(__name__)


@dataclass(frozen=True)
class GenOverrides(Handler):

    @staticmethod
    @memoize
    async def get_path(options: Options) -> Path:
        # TODO make only the architecture and release live at Path.rosdev(). The rest should live in
        #  Path.store()
        path = Path.rosdev() / 'overrides'

        log.debug(f'{GenOverrides.__name__} {path = }')

        return path

    @staticmethod
    @memoize
    async def get_text(options: Options) -> str:
        commit = {}
        default_options = Options()

        for k, v in asdict(options).items():
            if (k == 'stage') or (getattr(default_options, k) == v):
                continue

            if isinstance(v, frozendict):
                v = dict(v)
            commit[k] = v
        text = f'{pformat(commit)}\n' if commit else ''
        
        log.debug(f'{GenOverrides.__name__} {text = }')
        
        return text

    @staticmethod
    async def main(options: Options) -> None:
        GenHost.write_text(
            data=await GenOverrides.get_text(options),
            options=options,
            path=await GenOverrides.get_path(options),
        )
