from __future__ import annotations
from atools import memoize
import os
from pprint import pformat

from rosdev.util.handler import Handler
from rosdev.util.parser import Defaults


@memoize
class Workspace(Handler):

    @memoize
    async def _main(self) -> None:

        commit = {}
        defaults = Defaults()

        for k, v in self.__dict__.items():
            if getattr(defaults, k) != v:
                commit[k] = v

        if commit:
            with open(f'{os.getcwd()}/.rosdev/workspace', 'w') as workspace_f_out:
                workspace_f_out.write(f'{pformat(commit)}\n')
