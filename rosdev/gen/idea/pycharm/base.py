from dataclasses import dataclass, field
from logging import getLogger
from typing import Tuple, Type

from rosdev.gen.idea.pycharm.deployment_xml import GenIdeaPycharmDeploymentXml
from rosdev.gen.idea.pycharm.jdk_table_xml import GenIdeaPycharmJdkTableXml
from rosdev.gen.idea.pycharm.misc_xml import GenIdeaPycharmMiscXml
from rosdev.gen.idea.pycharm.webservers_xml import GenIdeaPycharmWebserversXml
from rosdev.util.handler import Handler

log = getLogger(__name__)


@dataclass(frozen=True)
class GenIdeaPycharmBase(Handler):
    pre_dependencies: Tuple[Type[Handler], ...] = field(init=False, default=(
        GenIdeaPycharmDeploymentXml,
        GenIdeaPycharmJdkTableXml,
        GenIdeaPycharmMiscXml,
        GenIdeaPycharmWebserversXml,
    ))
