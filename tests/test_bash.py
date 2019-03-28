from atools import async_test_case
import unittest
from unittest.mock import MagicMock, patch

from rosdev.bash import Bash
from tests.util.test_handler import TestHandler


@async_test_case
class TestBash(TestHandler):

    @async_test_case
    async def test_await_creates_interactive_bash_container(self) -> None:
        await Bash(self.default_options)


if __name__ == '__main__':
    unittest.main(verbosity=2)
