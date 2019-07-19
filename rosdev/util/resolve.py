from atools import memoize
from dataclasses import replace
from pathlib import Path

from rosdev.gen.rosdev.config import Config as RosdevConfig
from rosdev.util.build_farm import get_build_num
from rosdev.util.options import Options


# TODO make it more obvious that these are post-processors for the parser. Maybe even move it there.

@memoize
async def build_num(options: Options) -> Options:
    build_num = options.build_num

    if (build_num is None) and (not options.pull_install) and (not options.pull_src):
        try:
            paths = sorted(Path(RosdevConfig(options).global_path_base).iterdir())
        except FileNotFoundError:
            pass
        else:
            try:
                build_num = int(str(paths[-1].parts[-1]))
            except (IndexError, ValueError):
                pass

    if build_num is None:
        build_num = await get_build_num(
            architecture=options.architecture, release=options.release
        )

    return replace(options, build_num=build_num)
