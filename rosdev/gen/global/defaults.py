from __future__ import annotations
from atools import memoize
from dataclasses import asdict, dataclass
from pathlib import Path
from pprint import pformat

from rosdev.util.handler import Handler
from rosdev.util.parser import Defaults as ParserDefaults
from rosdev.util.subprocess import exec


@memoize
@dataclass(frozen=True)
class Defaults(Handler):

    @memoize
    async def _main(self) -> None:

        commit = {}
        parser_defaults = ParserDefaults()

        for k, v in asdict(self.options).items():
            if getattr(parser_defaults, k) != v:
                commit[k] = v

        if commit:
            await exec(f'mkdir -p {Path.home()}/.rosdev')
            with open(f'{Path.home()}/.rosdev/defaults', 'w') as defaults_f_out:
                defaults_f_out.write(f'{pformat(commit)}\n')
