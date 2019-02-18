from argparse import ArgumentParser
import logging

architecture_default = 'amd64'
architecture_choices = sorted({architecture_default, 'arm32v7', 'arm64v8'})
architecture_parser = ArgumentParser(add_help=False)
architecture_parser.add_argument(
    '--architecture', '-a',
    default=architecture_default,
    choices=architecture_choices,
    help=f'Architecture to build. Default: {architecture_default}',
)

architectures_default = [architecture_default]
architectures_parser = ArgumentParser(add_help=False)
architectures_parser.add_argument(
    '--architectures', '-a',
    default=[architecture_default],
    nargs='+',
    choices=architecture_choices,
    help=f'List of architectures to build. Default: {architectures_default}',
)

log_level_default = 'INFO'
log_level_parser = ArgumentParser(add_help=False)
log_level_parser.add_argument(
    '--log-level',
    default=log_level_default,
    choices=[name for _, name in sorted(logging._levelToName.items())],
    help=f'Default: {log_level_default}',
)

release_default = 'crystal'
release_choices = sorted({'ardent', 'bionic', release_default, 'kinetic', 'melodic'}),
release_parser = ArgumentParser(add_help=False)
release_parser.add_argument(
    '--release', '-r',
    default=release_default,
    choices=release_choices,
    help=f'ROS release to build. Default: {release_default}',
)

releases_default = [release_default]
releases_parser = ArgumentParser(add_help=False)
releases_parser.add_argument(
    '--release', '-r',
    default=releases_default,
    nargs='+',
    choices=release_choices,
    help=f'List of ROS releases to build. Default: {releases_default}',
)

interactive_parser = ArgumentParser(add_help=False)
interactive_parser.add_argument('--interactive', '-i', action='store_true')

port_parser = ArgumentParser(add_help=False)
port_parser.add_argument('--port', '-p', type=int)

ports_parser = ArgumentParser(add_help=False)
ports_parser.add_argument('--ports', '-p', nargs='*', default=[], type=int)

