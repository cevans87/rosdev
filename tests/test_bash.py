from atools import async_test_case
import unittest
from unittest.mock import MagicMock, patch


@async_test_case
class TestBash(unittest.TestCase):

    def setUp(self) -> None:
        pass

    def tearDown(self) -> None:
        pass

    async def test_foo(self) -> None:
        raise Exception()


if __name__ == '__main__':
    unittest.main(verbosity=2)
