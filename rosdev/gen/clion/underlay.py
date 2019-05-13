from __future__ import annotations
from asyncio import gather, get_event_loop
from atools import memoize
from dataclasses import dataclass
from frozendict import frozendict
#import keyring
from logging import getLogger
from lxml import etree
# noinspection PyProtectedMember
from lxml.etree import Element, _Element
import os
from pathlib import Path
import re
import sys
from textwrap import dedent
from typing import Dict, Mapping, Optional
from uuid import UUID, uuid4

from rosdev.util.handler import Handler
from rosdev.util.subprocess import exec, shell


log = getLogger(__name__)


@memoize
class Underlay(Handler):

    @property
    def path_base(self) -> str:
        # FIXME find a way to determine installed clion version.
        if sys.platform == 'darwin':
            return f'{Path.home()}/Library/Preferences/.CLion2019.1'
        return f'{Path.home()}/.CLion2019.1'

    @property
    def config_path(self) -> str:
        return f'{self.path_base}/config'

    @property
    def options_path(self) -> str:
        return f'{self.config_path}/options'

    @memoize
    async def _main(self) -> None:
        pass
