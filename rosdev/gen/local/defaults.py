from __future__ import annotations
from atools import memoize
import os
from pprint import pformat

from rosdev.util.handler import Handler
from rosdev.util.parser import Defaults as ParserDefaults
from rosdev.util.subprocess import exec


@memoize
class Defaults(Handler):

    @memoize
    async def _main(self) -> None:

        commit = {}
        parser_defaults = ParserDefaults.with_global_overrides()

        for k, v in self.options.__dict__.items():
            if getattr(parser_defaults, k) != v:
                commit[k] = v

        if commit:
            await exec(f'mkdir -p {os.getcwd()}/.rosdev')
            with open(f'{os.getcwd()}/.rosdev/defaults', 'w') as defaults_f_out:
                defaults_f_out.write(f'{pformat(commit)}\n')
