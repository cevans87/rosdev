#from atools import async_test_case, memoize
#from contextlib import ExitStack
#from importlib import import_module
#import unittest
#from unittest.mock import MagicMock, patch
#
#from rosdev.util.handler import Handler
#from rosdev.util.options import Options
#from rosdev.util.parser import Defaults
#
#from mocks.mock_docker import MockDocker


#@async_test_case
#class TestHandler(unittest.TestCase):
#
#    def setUp(self) -> None:
#        self.default_options = Options(**Defaults().__dict__)
#        self._stack = ExitStack()
#
#        self.m_docker = self._stack.enter_context(
#            patch.object(
#                import_module('rosdev.gen.docker.images'),
#                'docker',
#                MockDocker,
#            )
#        )
#
#    def tearDown(self) -> None:
#        memoize.reset_all()
#        self._stack.close()
#
#    async def test_not_instantiable(self) -> None:
#        with self.assertRaises(TypeError):
#            Handler(self.default_options)
#
#    async def test_await_calls_subclass_main(self) -> None:
#        body = MagicMock()
#
#        class Foo(Handler):
#            async def _main(self) -> None:
#                body()
#
#        await Foo(self.default_options)
#
#        body.assert_called_once()
#
#
#if __name__ == '__main__':
#    unittest.main(verbosity=2)
