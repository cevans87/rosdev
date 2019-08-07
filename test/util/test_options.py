from dataclasses import asdict, astuple, replace
from pathlib import Path
import pytest
from rosdev.util.options import Options

pytestmark = pytest.mark.asyncio

#@pytest.mark.parametrize('template_path,expected_path', [
#    Path('{architecture}')
#])
#async def test_paths_resolves(path: Path) -> None:
#    for k, v in asdict(Options()).items():
#        print(k, Options.__annotations__[k])
