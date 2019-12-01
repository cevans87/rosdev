from atools import memoize
from dataclasses import dataclass
import getpass
from logging import getLogger
from pykeepass import PyKeePass

from rosdev.gen.host import GenHost
from rosdev.gen.idea.c_pwd import GenIdeaCPwd
from rosdev.gen.idea.home import GenIdeaHome
from rosdev.gen.idea.workspace import GenIdeaWorkspace
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.path import Path

log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaCKdbx(Handler):

    @classmethod
    @memoize
    async def get_path(cls, options: Options) -> Path:
        path = await GenIdeaHome.get_path(options) / 'c.kdbx'

        log.debug(f'{cls.__name__} {path = }')

        return path
    
    @classmethod
    async def main(cls, options: Options) -> None:
        if not (await cls.get_path(options)).is_file():
            await GenHost.execute(
                command=f'cp {Path(__file__).parent}/c.kdbx {await cls.get_path(options)}',
                options=options,
            )

        with PyKeePass(
                filename=str(await cls.get_path(options)),
                password=await GenIdeaCPwd.get_password(options),
        ) as db:
            # TODO remove stale entries
            group = db.find_groups(first=True, name='IntelliJ Platform')
            if group is None:
                group = db.add_group(
                    destination_group=db.root_group,
                    group_name='IntelliJ Platform'
                )

            entry = db.find_entries_by_title(
                first=True,
                group=group,
                title=f'IntelliJ Platform Deployment — {await GenIdeaWorkspace.get_uuid(options)}',
            )
            if entry is None:
                db.add_entry(
                    destination_group=group,
                    password=await GenIdeaCPwd.get_password(options),
                    title=(
                        f'IntelliJ Platform Deployment —'
                        f' {await GenIdeaWorkspace.get_uuid(options)}'
                    ),
                    username=getpass.getuser(),
                )

            db.save(str(await cls.get_path(options)))
