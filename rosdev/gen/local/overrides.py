from __future__ import annotations
from atools import memoize
import os
from pprint import pformat

from rosdev.util.handler import Handler
from rosdev.util.parser import Defaults
from rosdev.util.subprocess import exec


@memoize
class Overrides(Handler):

    @memoize
    async def _main(self) -> None:

        commit = {}
        defaults = Defaults()

        for k, v in self.options.__dict__.items():
            if getattr(defaults, k) != v:
                commit[k] = v

        if commit:
            await exec(f'mkdir -p {os.getcwd()}/.rosdev')
            with open(f'{os.getcwd()}/.rosdev/overrides', 'w') as overrides_f_out:
                overrides_f_out.write(f'{pformat(commit)}\n')
