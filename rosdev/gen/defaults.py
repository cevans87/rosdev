from atools import memoize
from pprint import pformat

from rosdev.gen.rosdev.config import Config
from rosdev.util.handler import Handler
from rosdev.util.parser import Defaults as ParserDefaults
from rosdev.util.subprocess import exec


@memoize
class Defaults(Handler):

    @property
    def local_path_base(self) -> str:
        return Config(self.options).local_path
    
    @property
    def local_path(self) -> str:
        return f'{self.local_path_base}/defaults'

    @memoize
    async def _main(self) -> None:

        commit = {}
        parser_defaults = ParserDefaults.with_global_overrides()

        for k, v in self.options.__dict__.items():
            if getattr(parser_defaults, k) != v:
                commit[k] = v

        if commit:
            await exec(f'mkdir -p {self.local_path_base}')
            with open(self.local_path, 'w') as defaults_f_out:
                defaults_f_out.write(f'{pformat(commit)}\n')
