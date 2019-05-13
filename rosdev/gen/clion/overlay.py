from __future__ import annotations
import asyncio
from atools import memoize
import docker
from logging import getLogger
import os
from pathlib import Path
import platform
from tempfile import TemporaryDirectory
from textwrap import dedent

from rosdev.util.handler import Handler
from rosdev.util.lookup import get_machine


log = getLogger(__name__)


@memoize
class Overlay(Handler):

    @memoize
    async def _main(self) -> None:
        pass
