from dataclasses import dataclass, field
from base64 import b64decode, b64encode
# noinspection PyPackageRequirements
from Crypto import Random
# noinspection PyPackageRequirements
from Crypto.Cipher import AES
# noinspection PyPackageRequirements
from Crypto.Util import Padding
from logging import getLogger
import os
from pathlib import Path
from pykeepass import PyKeePass
from textwrap import dedent
from typing import Tuple, Type

from rosdev.gen.idea.ide.name import GenIdeaIdeName
from rosdev.gen.idea.universal import GenIdeaUniversal
from rosdev.gen.idea.uuid import GenIdeaUuid
from rosdev.util.handler import Handler
from rosdev.util.options import Options
from rosdev.util.subprocess import execute_command


log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaKeepass(Handler):

    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenIdeaIdeName,
        GenIdeaUniversal,
        GenIdeaUuid,
    ))

    @classmethod
    async def validate_options(cls, options: Options) -> None:
        # TODO py38 debug print
        log.debug(f'idea_c_pwd_universal_path: {options.idea_c_pwd_universal_path}')
        log.debug(f'idea_c_kdbx_universal_path: {options.idea_c_kdbx_universal_path}')

    @classmethod
    async def main(cls, options: Options) -> None:
        try:
            idea_c_kdbx_decoded_data = b64decode(
                options.read_bytes(path=options.idea_c_pwd_universal_path).strip().split()[-1]
            )
        except FileNotFoundError:
            iv = Random.new().read(AES.block_size)
            cipher = AES.new(key=b'Proxy Config Sec', mode=AES.MODE_CBC, iv=iv)
            idea_c_kdbx_decoded_data = b64decode(b64encode(bytes([
                0, 0, 0, 16,
                *iv,
                *cipher.encrypt(Padding.pad(b'rosdev', block_size=16))
            ])))
            assert iv == idea_c_kdbx_decoded_data[4:20]

        assert (
                (
                        (idea_c_kdbx_decoded_data[0] << 24) +
                        (idea_c_kdbx_decoded_data[1] << 16) +
                        (idea_c_kdbx_decoded_data[2] << 8) +
                        (idea_c_kdbx_decoded_data[3] << 0)
                ) == 16
        ), 'Expected decoded data to have iv_len field with value 16'

        iv = idea_c_kdbx_decoded_data[4:20]
        cipher = AES.new(key=b'Proxy Config Sec', mode=AES.MODE_CBC, iv=iv)
        password = Padding.unpad(
            cipher.decrypt(idea_c_kdbx_decoded_data[20:]),
            block_size=16,
        ).decode()

        if not options.idea_c_kdbx_universal_path.is_file():
            await execute_command(f'cp {Path(__file__).parent}/c.kdbx {options.idea_c_kdbx_universal_path}')
        with PyKeePass(str(options.idea_c_kdbx_universal_path), password=password) as db:
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
                title=f'IntelliJ Platform Deployment — {options.idea_uuid}',
            )
            if entry is None:
                db.add_entry(
                    destination_group=group,
                    password=password,
                    title=f'IntelliJ Platform Deployment — {options.idea_uuid}',
                    username=os.getlogin(),
                )

            db.save(str(options.idea_c_kdbx_universal_path))
            
        options.idea_c_pwd_universal_path.write_bytes(
            data=dedent(f'''
                encryption: BUILT_IN
                isAutoGenerated: false
                value: !!binary {b64encode(idea_c_kdbx_decoded_data).decode()}
            ''').strip().encode()
        )
