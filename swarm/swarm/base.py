import logging, sys
from configparser import ConfigParser

import casperfpga
from katcp import Message

from .defines import *


module_logger = logging.getLogger(__name__)


class Interface(object):

    def __init__(self, mac, ip, port):
        self.logger = logging.getLogger(self.__class__.__name__)
        self.port = port
        self.mac = mac
        self.arp = []
        self.ip = ip


class SwarmROACH(object):

    def __init__(self, roach2_host, parent_logger=module_logger):

        # Set all initial members
        self.roach2_host = roach2_host
        self.logger = parent_logger.getChild(
            '{name}[host={host!r}]'.format(
                name=self.__class__.__name__,
                host=self.roach2_host,
                )
            )

        # Connect to our ROACH2
        if self.roach2_host:
            self._connect(roach2_host)

    def __eq__(self, other):
        if other is not None:
            return self.roach2_host == other.roach2_host
        else:
            return not self.is_valid()

    def __ne__(self, other):
        return not self.__eq__(other)

    def is_valid(self):
        return self.roach2_host is not None

    def _connect(self, roach2_host):

        # Connect and wait until ready
        self.roach2 = casperfpga.CasperFpga(roach2_host)
        if roach2_host:
            if not self.roach2.wait_connected(timeout=5):
                raise RuntimeError('Timeout trying to connect to {0}.'
                                   'Is it up and running the swarm sever?'.format(self.roach2.host))

    def _program(self, bitcode):

        # Program with the bitcode
        self._bitcode = bitcode
        self.roach2.progdev(self._bitcode)

    def idle(self, bitcode=SWARM_IDLE_BITCODE):

        # Unload plugins and program with idle code
        self.unload_plugins()
        self.roach2.progdev(bitcode)
        self.logger.info('Idled {0} with {1}'.format(self.roach2_host, bitcode))

    def send_katcp_cmd(self, cmd, *args):

        # Create the message object
        message = Message.request(cmd, *args)

        # Send the request, and block for 60 seconds
        reply, informs = self.roach2.blocking_request(message, timeout=60)

        # Check for error, and raise one if present
        if not reply.reply_ok():
            raise RuntimeError(reply)

        # Otherwise return what we got
        return reply, informs

    def plugin_list(self):

        # Send plugin-list command
        reply, informs = self.send_katcp_cmd('plugin-list')

        # Return the list of loaded plugin names
        return list(inform.arguments[0] for inform in informs)

    def unload_plugins(self):

        # Unload all currently loaded plugins
        for plugin in reversed(self.plugin_list()):
            self.send_katcp_cmd('plugin-unload', plugin)

    def reload_plugins(self, plugins_config=SWARM_PLUGINS_CONFIG):

        # Unload all currently loaded plugins
        self.unload_plugins()

        # Read the default plugins file
        cfg = ConfigParser({'init': ''})
        cfg.read(plugins_config)

        # Get the names of all default plugins
        default_plugins = cfg.sections()

        # Cycle through defaults
        for plugin in default_plugins:

            # First, load the plugin
            path = cfg.get(plugin, 'file')
            self.send_katcp_cmd('plugin-load', path)

            # Then, run user init commands (if requested)
            for cmdstr in cfg.get(plugin, 'init').splitlines():
                cmd, sep, args = cmdstr.partition(' ')

                # If failure: catch, log, and proceed
                try:
                    self.send_katcp_cmd(cmd, *args.split())
                except RuntimeError as err:
                    self.logger.error("Plugin init failure: {0}".format(err))
